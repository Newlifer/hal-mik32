//! Driver for the SPIFI controller.
//!
//! [`SpifiSpiDevice`] lets SPI NOR flash drivers use SPIFI through the usual
//! [`embedded_hal::spi::SpiDevice`] interface.
//!
//! This driver is synchronous. It waits for every command in a polling loop.
//! There is no async version yet because it would need interrupt support.

use embedded_hal::spi::{ErrorKind, ErrorType, Operation, SpiDevice};
use mik32_pac::SpifiConfig;

// DATALEN is a 14-bit field in the CMD register, so its largest value is
// 0b11_1111_1111_1111 = 0x3fff = 16383 bytes.
const MAX_DATA_LENGTH: usize = 0x3fff;

/// SPIFI settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    /// Maximum idle time in memory mode, in SCK periods.
    pub timeout: u16,
    /// Time between commands, in SCK periods minus one.
    pub cs_high: u8,
    /// SCK divider: `F_SCK = F_HCLK / 2^(sck_div + 1)`.
    pub sck_div: u8,
    /// Use SPI mode 3. If false, mode 0 is used.
    pub mode3: bool,
    /// How many times to check whether an operation has finished.
    pub poll_limit: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            timeout: u16::MAX,
            cs_high: 0x0f,
            sck_div: 3,
            mode3: false,
            poll_limit: 100_000,
        }
    }
}

/// Error returned by SPIFI.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The operation took too long.
    Timeout,
    /// SPIFI cannot run this kind of SPI transaction.
    UnsupportedTransaction,
    /// The command has a wrong format.
    InvalidCommand,
    /// The transfer is longer than 16383 bytes.
    TransferTooLong,
}

impl embedded_hal::spi::Error for Error {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

/// Synchronous SPI device built on the MIK32 SPIFI controller.
///
/// This is not a general-purpose SPI driver. It supports the transaction
/// forms normally used by SPI NOR flash: a one-byte command, a command with
/// a 24-bit address, reading data, and writing data.
///
/// Every call waits until SPIFI finishes or [`Config::poll_limit`] is reached.
/// This type has no async implementation.
///
/// Set up the SPIFI clock and pins before calling [`SpifiSpiDevice::new`].
pub struct SpifiSpiDevice {
    peripheral: SpifiConfig,
    poll_limit: u32,
}

impl SpifiSpiDevice {
    /// Resets SPIFI and applies the settings.
    pub fn new(peripheral: SpifiConfig, config: Config) -> Result<Self, Error> {
        if config.cs_high > 0x0f || config.sck_div > 0x07 {
            return Err(Error::InvalidCommand);
        }

        peripheral.stat().write(|w| w.reset().reset());
        wait_until(config.poll_limit, || {
            peripheral.stat().read().reset().is_ready()
        })?;

        peripheral.ctrl().write(|w| unsafe {
            w.timeout()
                .bits(config.timeout)
                .cshigh()
                .bits(config.cs_high)
                .mode3()
                .bit(config.mode3)
                .sck_div()
                .bits(config.sck_div)
        });

        Ok(Self {
            peripheral,
            poll_limit: config.poll_limit,
        })
    }

    /// Gives the PAC peripheral back.
    pub fn release(self) -> SpifiConfig {
        self.peripheral
    }

    fn begin(
        &self,
        opcode: u8,
        address: u32,
        frame: u32,
        length: usize,
        output: bool,
    ) -> Result<(), Error> {
        if length > MAX_DATA_LENGTH {
            return Err(Error::TransferTooLong);
        }

        self.peripheral
            .stat()
            .write(|w| w.intrq().clear_interrupt());
        self.peripheral
            .address()
            .write(|w| unsafe { w.address().bits(address) });
        self.peripheral
            .idata()
            .write(|w| unsafe { w.idata().bits(0) });

        let direction = if output { 1 << 15 } else { 0 };
        let command = (opcode as u32) << 24 | frame << 21 | direction | length as u32;
        self.peripheral.cmd().write(|w| unsafe { w.bits(command) });
        Ok(())
    }

    fn finish(&self) -> Result<(), Error> {
        wait_until(self.poll_limit, || {
            self.peripheral.stat().read().intrq().bit_is_set()
        })
    }

    fn command(&self, opcode: u8, address: Option<u32>) -> Result<(), Error> {
        let (address, frame) = address.map_or((0, 1), |address| (address, 4));
        self.begin(opcode, address, frame, 0, false)?;
        self.finish()
    }

    fn hw_read(&self, opcode: u8, address: Option<u32>, bytes: &mut [u8]) -> Result<(), Error> {
        let (address, frame) = address.map_or((0, 1), |address| (address, 4));
        self.begin(opcode, address, frame, bytes.len(), false)?;

        for byte in bytes {
            *byte = self.peripheral.data8().read().data().bits();
        }
        self.finish()
    }

    fn hw_write(&self, opcode: u8, address: u32, bytes: &[u8]) -> Result<(), Error> {
        self.begin(opcode, address, 4, bytes.len(), true)?;

        for &byte in bytes {
            self.peripheral
                .data8()
                .write(|w| unsafe { w.data().bits(byte) });
        }
        self.finish()
    }
}

impl ErrorType for SpifiSpiDevice {
    type Error = Error;
}

impl SpiDevice for SpifiSpiDevice {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        match operations {
            [Operation::Write(command)] => match command.len() {
                1 => self.command(command[0], None),
                4 => self.command(command[0], Some(parse_address(command)?)),
                _ => Err(Error::InvalidCommand),
            },
            [Operation::TransferInPlace(buffer)] if !buffer.is_empty() => {
                self.hw_read(buffer[0], None, &mut buffer[1..])
            }
            [Operation::Write(command), Operation::Read(bytes)] => self.hw_read(
                command_opcode(command)?,
                Some(parse_address(command)?),
                bytes,
            ),
            [Operation::Write(command), Operation::Write(bytes)] => {
                self.hw_write(command_opcode(command)?, parse_address(command)?, bytes)
            }
            _ => Err(Error::UnsupportedTransaction),
        }
    }
}

fn command_opcode(command: &[u8]) -> Result<u8, Error> {
    command.first().copied().ok_or(Error::InvalidCommand)
}

fn parse_address(command: &[u8]) -> Result<u32, Error> {
    let [_, a2, a1, a0] = command else {
        return Err(Error::InvalidCommand);
    };
    Ok(u32::from_be_bytes([0, *a2, *a1, *a0]))
}

fn wait_until(mut remaining: u32, mut condition: impl FnMut() -> bool) -> Result<(), Error> {
    while remaining != 0 {
        if condition() {
            return Ok(());
        }
        remaining -= 1;
        core::hint::spin_loop();
    }
    Err(Error::Timeout)
}
