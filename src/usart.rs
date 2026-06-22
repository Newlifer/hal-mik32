//! USART

use core::fmt;
use core::marker::PhantomData;

use embedded_hal_nb::nb::{Error as NbError, Result as NbResult};
use embedded_hal_nb::serial::{ErrorKind, ErrorType, Read, Write};
use mik32_pac::usart_0::RegisterBlock;
use mik32_pac::{Peripherals, Usart0, Usart1};

use crate::gpio::{Func2Mode, Pin};
use crate::rcc::system_clock;

const DEFAULT_INIT_TIMEOUT: u32 = 100_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WordLength {
    DataBits7,
    DataBits8,
    DataBits9,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DuplexMode {
    Half,
    Full,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyncMode {
    Async,
    Sync,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Parity {
    None,
    Even,
    Odd,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopBits {
    Stop1,
    Stop2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaConfig {
    None,
    Tx,
    Rx,
    TxRx,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    pub baudrate: u32,
    pub word_length: WordLength,
    pub parity: Parity,
    pub stop_bits: StopBits,
    pub duplex_mode: DuplexMode,
    pub sync_mode: SyncMode,
    pub dma: DmaConfig,
    pub init_timeout: u32,
}

impl Config {
    pub const fn default() -> Self {
        Self {
            baudrate: 115_200,
            word_length: WordLength::DataBits8,
            parity: Parity::None,
            stop_bits: StopBits::Stop1,
            duplex_mode: DuplexMode::Full,
            sync_mode: SyncMode::Async,
            dma: DmaConfig::None,
            init_timeout: DEFAULT_INIT_TIMEOUT,
        }
    }

    pub const fn baudrate(mut self, baudrate: u32) -> Self {
        self.baudrate = baudrate;
        self
    }

    pub const fn word_length(mut self, word_length: WordLength) -> Self {
        self.word_length = word_length;
        self
    }

    pub const fn parity(mut self, parity: Parity) -> Self {
        self.parity = parity;
        self
    }

    pub const fn stop_bits(mut self, stop_bits: StopBits) -> Self {
        self.stop_bits = stop_bits;
        self
    }

    pub const fn duplex_mode(mut self, duplex_mode: DuplexMode) -> Self {
        self.duplex_mode = duplex_mode;
        self
    }

    pub const fn sync_mode(mut self, sync_mode: SyncMode) -> Self {
        self.sync_mode = sync_mode;
        self
    }

    pub const fn dma(mut self, dma: DmaConfig) -> Self {
        self.dma = dma;
        self
    }

    pub const fn init_timeout(mut self, timeout: u32) -> Self {
        self.init_timeout = timeout;
        self
    }

    pub const fn validate(&self) -> Result<(), ConfigError> {
        if self.baudrate == 0 {
            return Err(ConfigError::ZeroBaudrate);
        }
        if self.init_timeout == 0 {
            return Err(ConfigError::ZeroInitTimeout);
        }

        Ok(())
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::default()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    ZeroBaudrate,
    ZeroInitTimeout,
    BaudrateTooHigh,
    BaudrateTooLow,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InitErrorKind {
    InvalidConfig(ConfigError),
    PeripheralNotReady,
}

pub struct InitError<UART, TXPIN, RXPIN> {
    pub uart: UART,
    pub pins: (TXPIN, RXPIN),
    pub error: InitErrorKind,
}

impl<UART, TXPIN, RXPIN> InitError<UART, TXPIN, RXPIN> {
    pub fn into_parts(self) -> (UART, (TXPIN, RXPIN), InitErrorKind) {
        (self.uart, self.pins, self.error)
    }
}

impl<UART, TXPIN, RXPIN> fmt::Debug for InitError<UART, TXPIN, RXPIN> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("InitError")
            .field("error", &self.error)
            .finish_non_exhaustive()
    }
}

pub trait TxPin<UART> {}
pub trait RxPin<UART> {}

impl TxPin<Usart0> for Pin<0, 6, Func2Mode> {}
impl RxPin<Usart0> for Pin<0, 5, Func2Mode> {}

impl TxPin<Usart1> for Pin<1, 9, Func2Mode> {}
impl RxPin<Usart1> for Pin<1, 8, Func2Mode> {}

mod sealed {
    pub trait Sealed {}

    impl Sealed for mik32_pac::Usart0 {}
    impl Sealed for mik32_pac::Usart1 {}
}

pub trait Instance: sealed::Sealed {
    fn ptr() -> *const RegisterBlock;
    fn enable_clock();
}

impl Instance for Usart0 {
    #[inline(always)]
    fn ptr() -> *const RegisterBlock {
        Usart0::ptr() as *const RegisterBlock
    }

    #[inline(always)]
    fn enable_clock() {
        let p = unsafe { Peripherals::steal() };
        p.pm.clk_apb_p_set().modify(|_, w| w.uart_0().enable());
    }
}

impl Instance for Usart1 {
    #[inline(always)]
    fn ptr() -> *const RegisterBlock {
        Usart1::ptr() as *const RegisterBlock
    }

    #[inline(always)]
    fn enable_clock() {
        let p = unsafe { Peripherals::steal() };
        p.pm.clk_apb_p_set().modify(|_, w| w.uart_1().enable());
    }
}

pub struct Serial<UART, TXPIN, RXPIN>
where
    UART: Instance,
    TXPIN: TxPin<UART>,
    RXPIN: RxPin<UART>,
{
    uart: UART,
    pins: (TXPIN, RXPIN),
}

pub struct Tx<UART: Instance> {
    _uart: PhantomData<UART>,
}

pub struct Rx<UART: Instance> {
    _uart: PhantomData<UART>,
}

impl<UART, TXPIN, RXPIN> Serial<UART, TXPIN, RXPIN>
where
    UART: Instance,
    TXPIN: TxPin<UART>,
    RXPIN: RxPin<UART>,
{
    pub fn new(
        uart: UART,
        pins: (TXPIN, RXPIN),
        config: Config,
    ) -> Result<Self, InitError<UART, TXPIN, RXPIN>> {
        if let Err(error) = config.validate() {
            return Err(InitError {
                uart,
                pins,
                error: InitErrorKind::InvalidConfig(error),
            });
        }

        let baudrate_divisor = match calc_baudrate_divisor(config.baudrate) {
            Ok(divisor) => divisor,
            Err(error) => {
                return Err(InitError {
                    uart,
                    pins,
                    error: InitErrorKind::InvalidConfig(error),
                });
            }
        };

        UART::enable_clock();

        let serial = Self { uart, pins };
        if let Err(error) = serial.configure(config, baudrate_divisor) {
            serial.regs().control1().modify(|_, w| w.ue().disable());
            let Self { uart, pins } = serial;
            return Err(InitError { uart, pins, error });
        }

        Ok(serial)
    }

    pub fn split(self) -> (Tx<UART>, Rx<UART>) {
        let _ = self.uart;
        let _ = self.pins;

        (Tx { _uart: PhantomData }, Rx { _uart: PhantomData })
    }

    #[inline(always)]
    fn regs(&self) -> &RegisterBlock {
        unsafe { &*UART::ptr() }
    }

    fn configure(&self, config: Config, baudrate_divisor: u16) -> Result<(), InitErrorKind> {
        let regs = self.regs();

        regs.control1().modify(|_, w| w.ue().disable());

        regs.divider()
            .write(|w| unsafe { w.brr().bits(baudrate_divisor) });

        regs.control1().write(|w| {
            w.idleie()
                .disable()
                .peie()
                .disable()
                .rxneie()
                .disable()
                .tcie()
                .disable()
                .txeie()
                .disable()
        });

        regs.control2().write(|w| {
            w.lbdie()
                .disable()
                .lbm()
                .normal()
                .swap()
                .normal()
                .rxinv()
                .direct()
                .txinv()
                .direct()
                .datainv()
                .direct()
                .msbfirst()
                .lsb()
        });

        regs.control3().write(|w| {
            w.eie()
                .disable()
                .ctsie()
                .disable()
                .ctse()
                .ignored()
                .rtse()
                ._0()
        });

        match config.word_length {
            WordLength::DataBits7 => regs.control1().modify(|_, w| w.m()._7bits()),
            WordLength::DataBits8 => regs.control1().modify(|_, w| w.m()._8bits()),
            WordLength::DataBits9 => regs.control1().modify(|_, w| w.m()._9bits()),
        };

        match config.parity {
            Parity::None => regs.control1().modify(|_, w| w.pce().disable()),
            Parity::Even => regs
                .control1()
                .modify(|_, w| w.pce().enable().ps().parity()),
            Parity::Odd => regs.control1().modify(|_, w| w.pce().enable().ps().odd()),
        };

        match config.stop_bits {
            StopBits::Stop1 => regs.control2().modify(|_, w| w.stop_1()._1bit()),
            StopBits::Stop2 => regs.control2().modify(|_, w| w.stop_1()._2bits()),
        };

        match config.duplex_mode {
            DuplexMode::Full => regs.control3().modify(|_, w| w.hdsel().duplex()),
            DuplexMode::Half => regs.control3().modify(|_, w| w.hdsel().half_duplex()),
        };

        match config.sync_mode {
            SyncMode::Async => regs.control2().modify(|_, w| w.clken().asynchronous()),
            SyncMode::Sync => regs.control2().modify(|_, w| w.clken().synchronous()),
        };

        match config.dma {
            DmaConfig::None => regs
                .control3()
                .modify(|_, w| w.dmat().disable().dmar().disable()),
            DmaConfig::Tx => regs
                .control3()
                .modify(|_, w| w.dmat().enable().dmar().disable()),
            DmaConfig::Rx => regs
                .control3()
                .modify(|_, w| w.dmat().disable().dmar().enable()),
            DmaConfig::TxRx => regs
                .control3()
                .modify(|_, w| w.dmat().enable().dmar().enable()),
        };

        regs.flags().write(|w| unsafe { w.bits(0x03ff) });

        regs.control1()
            .modify(|_, w| w.te().enable().re().enable().ue().enable());

        for _ in 0..config.init_timeout {
            let flags = regs.flags().read();
            if flags.teack().bit_is_set() && flags.reack().bit_is_set() {
                return Ok(());
            }
            core::hint::spin_loop();
        }

        Err(InitErrorKind::PeripheralNotReady)
    }
}

impl<UART, TXPIN, RXPIN> ErrorType for Serial<UART, TXPIN, RXPIN>
where
    UART: Instance,
    TXPIN: TxPin<UART>,
    RXPIN: RxPin<UART>,
{
    type Error = ErrorKind;
}

impl<UART: Instance> ErrorType for Tx<UART> {
    type Error = ErrorKind;
}

impl<UART: Instance> ErrorType for Rx<UART> {
    type Error = ErrorKind;
}

impl<UART: Instance> Write for Tx<UART> {
    fn write(&mut self, byte: u8) -> NbResult<(), Self::Error> {
        let regs = regs::<UART>();

        if regs.flags().read().txe().bit_is_clear() {
            return Err(NbError::WouldBlock);
        }

        regs.txdata()
            .write(|w| unsafe { w.tdr().bits(byte as u16) });

        Ok(())
    }

    fn flush(&mut self) -> NbResult<(), Self::Error> {
        if regs::<UART>().flags().read().tc().bit_is_clear() {
            return Err(NbError::WouldBlock);
        }

        Ok(())
    }
}

impl<UART: Instance> fmt::Write for Tx<UART> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for byte in s.bytes() {
            nb_block(|| self.write(byte)).map_err(|_| fmt::Error)?;
        }

        nb_block(|| self.flush()).map_err(|_| fmt::Error)
    }
}

impl<UART: Instance> Read for Rx<UART> {
    fn read(&mut self) -> NbResult<u8, Self::Error> {
        let regs = regs::<UART>();
        let flags = regs.flags().read();

        if flags.pe().bit_is_set() {
            regs.flags().write(|w| w.pe().clear_bit_by_one());
            return Err(NbError::Other(ErrorKind::Parity));
        }

        if flags.fe().bit_is_set() {
            regs.flags().write(|w| w.fe().clear_bit_by_one());
            return Err(NbError::Other(ErrorKind::FrameFormat));
        }

        if flags.nf().bit_is_set() {
            regs.flags().write(|w| w.nf().clear_bit_by_one());
            return Err(NbError::Other(ErrorKind::Noise));
        }

        if flags.ore().bit_is_set() {
            regs.flags().write(|w| w.ore().clear_bit_by_one());
            return Err(NbError::Other(ErrorKind::Overrun));
        }

        if flags.rxne().bit_is_clear() {
            return Err(NbError::WouldBlock);
        }

        Ok(regs.rxdata().read().rdr().bits() as u8)
    }
}

#[inline(always)]
fn regs<UART: Instance>() -> &'static RegisterBlock {
    unsafe { &*UART::ptr() }
}

#[inline(always)]
fn calc_baudrate_divisor(baudrate: u32) -> Result<u16, ConfigError> {
    if baudrate == 0 {
        return Err(ConfigError::ZeroBaudrate);
    }

    let sys_clock: u32 = system_clock().into();
    let p = unsafe { Peripherals::steal() };
    let ahb_divisor = p.pm.div_ahb().read().bits().saturating_add(1);
    let apb_p_divisor = p.pm.div_apb_p().read().bits().saturating_add(1);
    let clock = sys_clock / ahb_divisor / apb_p_divisor;
    let divisor = (u64::from(clock) + u64::from(baudrate) / 2) / u64::from(baudrate);

    if divisor < 16 {
        return Err(ConfigError::BaudrateTooHigh);
    }
    if divisor > u64::from(u16::MAX) {
        return Err(ConfigError::BaudrateTooLow);
    }

    Ok(divisor as u16)
}

fn nb_block<T, E>(mut f: impl FnMut() -> NbResult<T, E>) -> Result<T, E> {
    loop {
        match f() {
            Ok(value) => return Ok(value),
            Err(NbError::WouldBlock) => core::hint::spin_loop(),
            Err(NbError::Other(error)) => return Err(error),
        }
    }
}
