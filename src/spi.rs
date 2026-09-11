//! Blocking 8-bit SPI master driver.

use embedded_hal::spi::{self, ErrorType, SpiBus};
use mik32_pac::spi_0::RegisterBlock;
use mik32_pac::{Peripherals, Spi0 as PacSpi0, Spi1 as PacSpi1};

const BAUD_DIVIDERS: [u32; 7] = [4, 8, 16, 32, 64, 128, 256];
const DEFAULT_TIMEOUT: u32 = 100_000;
const DEFAULT_THRESHOLD: u32 = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Delays {
    pub init: u8,
    pub after: u8,
    pub between: u8,
}

impl Default for Delays {
    fn default() -> Self {
        Self {
            init: 0,
            after: 0,
            between: 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    pub frequency: crate::clock::Hertz,
    pub mode: embedded_hal::spi::Mode,
    pub timeout: u32,
    pub threshold: u32,
    pub delays: Delays,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            frequency: crate::clock::Hertz::mhz(8),
            mode: embedded_hal::spi::MODE_0,
            timeout: DEFAULT_TIMEOUT,
            threshold: DEFAULT_THRESHOLD,
            delays: Delays::default(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    InvalidFrequency,
    InvalidTimeout,
    InvalidThreshold,
    Timeout,
    ModeFault,
    RxOverflow,
}

impl spi::Error for Error {
    fn kind(&self) -> spi::ErrorKind {
        match self {
            Self::Timeout => spi::ErrorKind::Other,
            Self::ModeFault => spi::ErrorKind::ModeFault,
            Self::RxOverflow => spi::ErrorKind::Overrun,
            Self::InvalidFrequency | Self::InvalidTimeout | Self::InvalidThreshold => {
                spi::ErrorKind::Other
            }
        }
    }
}

pub struct Spi<SPI: Instance> {
    spi: SPI,
    timeout: u32,
}

pub type Spi0Bus = Spi<PacSpi0>;
pub type Spi1Bus = Spi<PacSpi1>;
pub type Spi0 = Spi0Bus;
pub type Spi1 = Spi1Bus;

mod sealed {
    pub trait Sealed {}

    impl Sealed for mik32_pac::Spi0 {}
    impl Sealed for mik32_pac::Spi1 {}
}

pub trait Instance: sealed::Sealed {
    fn ptr() -> *const RegisterBlock;
    fn enable_clock();
}

fn select_divider(apb_frequency: u32, requested_frequency: u32) -> Option<u32> {
    BAUD_DIVIDERS
        .iter()
        .find(|&&divider| apb_frequency / divider <= requested_frequency)
        .copied()
}

impl Instance for PacSpi0 {
    fn ptr() -> *const RegisterBlock {
        PacSpi0::ptr()
    }

    fn enable_clock() {
        let peripherals = unsafe { Peripherals::steal() };
        peripherals
            .pm
            .clk_apb_p_set()
            .modify(|_, w| w.spi_0().enable());
    }
}

impl Instance for PacSpi1 {
    fn ptr() -> *const RegisterBlock {
        PacSpi1::ptr() as *const RegisterBlock
    }

    fn enable_clock() {
        let peripherals = unsafe { Peripherals::steal() };
        peripherals
            .pm
            .clk_apb_p_set()
            .modify(|_, w| w.spi_1().enable());
    }
}

impl<SPI: Instance> Spi<SPI> {
    pub fn new(spi: SPI, clocks: &crate::rcc::Clocks, config: Config) -> Result<Self, Error> {
        if config.frequency.0 == 0 {
            return Err(Error::InvalidFrequency);
        }
        if config.timeout == 0 {
            return Err(Error::InvalidTimeout);
        }
        if config.threshold == 0 {
            return Err(Error::InvalidThreshold);
        }

        let divider = select_divider(clocks.apb_p_clk().0, config.frequency.0)
            .ok_or(Error::InvalidFrequency)?;

        SPI::enable_clock();
        let registers = unsafe { &*SPI::ptr() };

        registers.enable().write(|w| w.spi_en().disable());
        registers
            .enable()
            .write(|w| w.clear_tx_fifo().set_bit().clear_px_fifo().set_bit());
        registers.config().write(|w| {
            let w = w
                .mode_sel()
                .master()
                .ref_clk()
                .apb_p_clk()
                .baud_rate_div()
                .variant(match divider {
                    4 => mik32_pac::spi_0::config::BaudRateDiv::Div4,
                    8 => mik32_pac::spi_0::config::BaudRateDiv::Div8,
                    16 => mik32_pac::spi_0::config::BaudRateDiv::Div16,
                    32 => mik32_pac::spi_0::config::BaudRateDiv::Div32,
                    64 => mik32_pac::spi_0::config::BaudRateDiv::Div64,
                    128 => mik32_pac::spi_0::config::BaudRateDiv::Div128,
                    _ => mik32_pac::spi_0::config::BaudRateDiv::Div256,
                });
            let w = match (config.mode.polarity, config.mode.phase) {
                (
                    embedded_hal::spi::Polarity::IdleLow,
                    embedded_hal::spi::Phase::CaptureOnFirstTransition,
                ) => w.clk_pol()._0().clk_ph()._0(),
                (
                    embedded_hal::spi::Polarity::IdleLow,
                    embedded_hal::spi::Phase::CaptureOnSecondTransition,
                ) => w.clk_pol()._0().clk_ph()._1(),
                (
                    embedded_hal::spi::Polarity::IdleHigh,
                    embedded_hal::spi::Phase::CaptureOnFirstTransition,
                ) => w.clk_pol()._1().clk_ph()._0(),
                (
                    embedded_hal::spi::Polarity::IdleHigh,
                    embedded_hal::spi::Phase::CaptureOnSecondTransition,
                ) => w.clk_pol()._1().clk_ph()._1(),
            };
            w.manual_cs().automatic().cs().not_selected()
        });
        registers.delay().write(|w| unsafe {
            w.d_int()
                .bits(config.delays.init)
                .d_after()
                .bits(config.delays.after)
                .d_btwn()
                .bits(config.delays.between)
        });
        registers
            .tx_thr()
            .write(|w| unsafe { w.threshold_of_tx_fifo().bits(config.threshold) });
        registers.int_disable().write(|w| {
            w.rx_overflow()
                .set_bit()
                .mode_fail()
                .set_bit()
                .tx_fifo_not_full()
                .set_bit()
                .tx_fifo_full()
                .set_bit()
                .rx_fifo_not_empty()
                .set_bit()
                .px_fifo_full()
                .set_bit()
                .tx_fifo_underflow()
                .set_bit()
        });
        registers.enable().write(|w| w.spi_en().enable());

        Ok(Self {
            spi,
            timeout: config.timeout,
        })
    }

    pub fn free(self) -> SPI {
        self.spi
    }

    fn registers(&self) -> &RegisterBlock {
        unsafe { &*SPI::ptr() }
    }

    fn wait(&self, mut condition: impl FnMut(&RegisterBlock) -> bool) -> Result<(), Error> {
        let registers = self.registers();
        for _ in 0..self.timeout {
            let status = registers.status().read();
            if status.mode_fail().is_fail() {
                return Err(Error::ModeFault);
            }
            if status.rx_overflow().is_overflow() {
                return Err(Error::RxOverflow);
            }
            if condition(registers) {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(Error::Timeout)
    }

    fn transfer_byte(&mut self, byte: u8) -> Result<u8, Error> {
        self.wait(|registers| {
            registers
                .status()
                .read()
                .tx_fifo_not_full()
                .is_more_than_the_threshold()
        })?;
        self.registers()
            .txdata()
            .write(|w| unsafe { w.tx_fifo_data().bits(byte) });
        self.wait(|registers| registers.status().read().rx_fifo_not_empty().is_not_empty())?;
        Ok(self.registers().rxdata().read().rx_fifo_data().bits())
    }
}

impl<SPI: Instance> ErrorType for Spi<SPI> {
    type Error = Error;
}

impl<SPI: Instance> SpiBus<u8> for Spi<SPI> {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words {
            *word = self.transfer_byte(0xff)?;
        }
        self.flush()
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for &word in words {
            self.transfer_byte(word)?;
        }
        self.flush()
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let common = read.len().min(write.len());
        for (read, &write) in read[..common].iter_mut().zip(&write[..common]) {
            *read = self.transfer_byte(write)?;
        }
        if read.len() > common {
            for read in &mut read[common..] {
                *read = self.transfer_byte(0xff)?;
            }
        } else {
            for &write in &write[common..] {
                self.transfer_byte(write)?;
            }
        }
        self.flush()
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words {
            *word = self.transfer_byte(*word)?;
        }
        self.flush()
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.wait(|registers| !registers.status().read().spi_active().is_busy())
    }
}

#[cfg(test)]
mod tests {
    use super::select_divider;

    #[test]
    fn selects_supported_divider_not_above_requested_frequency() {
        assert_eq!(select_divider(32_000_000, 8_000_000), Some(4));
        assert_eq!(select_divider(32_000_000, 1_000_000), Some(32));
        assert_eq!(select_divider(32_000_000, 100_000), Some(256));
    }

    #[test]
    fn rejects_frequency_below_hardware_minimum() {
        assert_eq!(select_divider(32_000_000, 125_000), Some(256));
        assert_eq!(select_divider(32_000_000, 124_999), None);
    }
}
