use core::fmt;
use embedded_hal::i2c::{
    self, ErrorType, I2c as HalI2c, NoAcknowledgeSource, Operation, SevenBitAddress,
};
use mik32_pac::i2c_0::RegisterBlock;
use mik32_pac::{I2c0, I2c1, Peripherals};

const I2C_ADDRESS_7BIT_MAX: u16 = 0x7f;
const I2C_ADDRESS_10BIT_MAX: u16 = 0x03ff;
const I2C_NBYTE_MAX: usize = 255;
const TIMING_4BIT_MAX: u8 = 0x0f;
const DEFAULT_TIMEOUT: u32 = 1_000;
const DEFAULT_TIMING: Timing = Timing {
    prescaler: 3,
    scl_delay: 4,
    sda_delay: 2,
    scl_high: 39,
    scl_low: 39,
};
const DEFAULT_CONFIG: Config = Config {
    mode: Mode::Master,
    address_primary: 0,
    address_secondary: None,
    general_call: false,
    sbc_mode: false,
    underflow_fill: 0xff,
    timing: DEFAULT_TIMING,
    timeout: DEFAULT_TIMEOUT,
};

/// Raw values for the I2C `TIMINGR` register.
///
/// The default values configure approximately 100 kHz SCL when I2CCLK is
/// 32 MHz. Applications using another peripheral clock should provide values
/// calculated for that clock.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Timing {
    pub prescaler: u8,
    pub scl_delay: u8,
    pub sda_delay: u8,
    pub scl_high: u8,
    pub scl_low: u8,
}

impl Timing {
    pub const fn default_100khz_32mhz() -> Self {
        DEFAULT_TIMING
    }
}

impl Default for Timing {
    fn default() -> Self {
        DEFAULT_TIMING
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Mode {
    Master,
    Slave,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[repr(u8)]
pub enum SecondaryAddressMask {
    Exact = 0,
    IgnoreOneBit = 1,
    IgnoreTwoBits = 2,
    IgnoreThreeBits = 3,
    IgnoreFourBits = 4,
    IgnoreFiveBits = 5,
    IgnoreSixBits = 6,
    AllNonReserved = 7,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct SecondaryAddress {
    pub address: u8,
    pub mask: SecondaryAddressMask,
}

impl SecondaryAddress {
    pub const fn new(address: u8, mask: SecondaryAddressMask) -> Self {
        Self { address, mask }
    }
}

impl Default for Mode {
    fn default() -> Self {
        Self::Master
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Config {
    pub mode: Mode,
    pub address_primary: u16,
    pub address_secondary: Option<SecondaryAddress>,
    pub general_call: bool,
    pub sbc_mode: bool,
    /// Byte sent when the master reads beyond the supplied slave TX buffer.
    pub underflow_fill: u8,
    pub timing: Timing,
    pub timeout: u32,
}

impl Config {
    pub const fn default() -> Self {
        DEFAULT_CONFIG
    }

    pub const fn as_master(mut self) -> Self {
        self.mode = Mode::Master;
        self
    }

    pub const fn as_slave(mut self) -> Self {
        self.mode = Mode::Slave;
        self
    }

    pub const fn timeout(mut self, timeout: u32) -> Self {
        self.timeout = timeout;
        self
    }

    pub const fn timing(mut self, timing: Timing) -> Self {
        self.timing = timing;
        self
    }

    pub const fn primary_address(mut self, address: u16) -> Self {
        self.address_primary = address;
        self
    }

    pub const fn secondary_address(mut self, address: SecondaryAddress) -> Self {
        self.address_secondary = Some(address);
        self
    }

    pub const fn without_secondary_address(mut self) -> Self {
        self.address_secondary = None;
        self
    }

    pub const fn general_call(mut self, enabled: bool) -> Self {
        self.general_call = enabled;
        self
    }

    pub const fn underflow_fill(mut self, byte: u8) -> Self {
        self.underflow_fill = byte;
        self
    }

    pub const fn validate(&self) -> Result<(), ConfigError> {
        if self.timeout == 0 {
            return Err(ConfigError::ZeroTimeout);
        }
        if self.timing.prescaler > TIMING_4BIT_MAX {
            return Err(ConfigError::TimingPrescalerOutOfRange);
        }
        if self.timing.scl_delay > TIMING_4BIT_MAX {
            return Err(ConfigError::TimingSclDelayOutOfRange);
        }
        if self.timing.sda_delay > TIMING_4BIT_MAX {
            return Err(ConfigError::TimingSdaDelayOutOfRange);
        }
        match self.mode {
            Mode::Master => {}
            Mode::Slave => {
                if self.address_primary > I2C_ADDRESS_10BIT_MAX {
                    return Err(ConfigError::PrimaryAddressOutOfRange);
                }
                if self.address_primary > I2C_ADDRESS_7BIT_MAX {
                    return Err(ConfigError::SlaveTenBitAddressUnsupported);
                }
                if let Some(address) = self.address_secondary {
                    if address.address as u16 > I2C_ADDRESS_7BIT_MAX {
                        return Err(ConfigError::SecondaryAddressOutOfRange);
                    }
                }
                if self.sbc_mode {
                    return Err(ConfigError::SlaveSbcUnsupported);
                }
            }
        }

        Ok(())
    }
}

impl Default for Config {
    fn default() -> Self {
        DEFAULT_CONFIG
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum ConfigError {
    ZeroTimeout,
    TimingPrescalerOutOfRange,
    TimingSclDelayOutOfRange,
    TimingSdaDelayOutOfRange,
    PrimaryAddressOutOfRange,
    SlaveTenBitAddressUnsupported,
    SecondaryAddressOutOfRange,
    SlaveSbcUnsupported,
}

pub struct InitError<I2C> {
    pub i2c: I2C,
    pub error: ConfigError,
}

impl<I2C> InitError<I2C> {
    pub fn into_parts(self) -> (I2C, ConfigError) {
        (self.i2c, self.error)
    }
}

impl<I2C> fmt::Debug for InitError<I2C> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("InitError")
            .field("error", &self.error)
            .finish_non_exhaustive()
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Error {
    BusError,
    ArbitrationLoss,
    Nack,
    Overrun,
    Timeout,
    InvalidMode,
    InvalidAddress,
    InvalidDirection,
    SlaveTimeout(SlaveTimeout),
}

impl i2c::Error for Error {
    fn kind(&self) -> i2c::ErrorKind {
        match *self {
            Error::BusError => i2c::ErrorKind::Bus,
            Error::ArbitrationLoss => i2c::ErrorKind::ArbitrationLoss,
            Error::Nack => i2c::ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown),
            Error::Overrun => i2c::ErrorKind::Overrun,
            Error::Timeout
            | Error::InvalidMode
            | Error::InvalidAddress
            | Error::InvalidDirection
            | Error::SlaveTimeout(_) => i2c::ErrorKind::Other,
        }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum SlaveDirection {
    /// The master writes and the slave receives.
    Receive,
    /// The master reads and the slave transmits.
    Transmit,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct SlaveTimeout {
    pub direction: SlaveDirection,
    pub count: usize,
    pub buffer_status: SlaveBufferStatus,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum SlaveAcknowledge {
    Ack,
    Nack,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum AddressMatchSource {
    Primary,
    Secondary,
    GeneralCall,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct AddressMatch {
    pub address: u8,
    pub source: AddressMatchSource,
    pub direction: SlaveDirection,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum SlaveTransferEnd {
    Stop,
    Nack,
    RepeatedStart,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum SlaveBufferStatus {
    Complete,
    Overflow,
    Underflow,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct SlaveTransfer {
    pub count: usize,
    pub end: SlaveTransferEnd,
    pub buffer_status: SlaveBufferStatus,
}

#[derive(Debug)]
pub struct I2c<I2C: Instance> {
    i2c: I2C,
    config: Config,
}

pub type I2c0Bus = I2c<I2c0>;
pub type I2c1Bus = I2c<I2c1>;
pub type I2CImpl = I2c0Bus;

mod sealed {
    pub trait Sealed {}

    impl Sealed for mik32_pac::I2c0 {}
    impl Sealed for mik32_pac::I2c1 {}
}

pub trait Instance: sealed::Sealed {
    fn ptr() -> *const RegisterBlock;
    fn enable_clock();
}

impl Instance for I2c0 {
    #[inline(always)]
    fn ptr() -> *const RegisterBlock {
        I2c0::ptr()
    }

    #[inline(always)]
    fn enable_clock() {
        let p = unsafe { Peripherals::steal() };
        p.pm.clk_apb_p_set().modify(|_, w| w.i2c_0().enable());
    }
}

impl Instance for I2c1 {
    #[inline(always)]
    fn ptr() -> *const RegisterBlock {
        I2c1::ptr() as *const RegisterBlock
    }

    #[inline(always)]
    fn enable_clock() {
        let p = unsafe { Peripherals::steal() };
        p.pm.clk_apb_p_set().modify(|_, w| w.i2c_1().enable());
    }
}

impl<I2C: Instance> I2c<I2C> {
    pub fn new(i2c: I2C, config: Config) -> Result<Self, InitError<I2C>> {
        if let Err(error) = config.validate() {
            return Err(InitError { i2c, error });
        }
        I2C::enable_clock();

        let regs = regs::<I2C>();

        Self::disable(regs);
        Self::configure_filters(regs);
        Self::configure_timing(regs, config.timing);
        Self::configure_stretching(regs);
        Self::enable(regs);

        if config.mode == Mode::Slave {
            Self::configure_slave(regs, config);
        }

        Ok(Self { i2c, config })
    }

    pub fn free(self) -> I2C {
        self.i2c
    }

    /// Selects whether the slave acknowledges the next received byte.
    ///
    /// This is primarily useful after [`wait_address`](Self::wait_address)
    /// and before [`slave_receive`](Self::slave_receive). The peripheral
    /// clears the NACK request automatically on STOP or a new address match.
    pub fn set_slave_acknowledge(&mut self, acknowledge: SlaveAcknowledge) -> Result<(), Error> {
        if self.config.mode != Mode::Slave {
            return Err(Error::InvalidMode);
        }

        let regs = regs::<I2C>();
        regs.cr2().modify(|_, w| match acknowledge {
            SlaveAcknowledge::Ack => w.nack().clear_bit(),
            SlaveAcknowledge::Nack => w.nack().set_bit(),
        });
        Ok(())
    }

    pub fn slave_ack(&mut self) -> Result<(), Error> {
        self.set_slave_acknowledge(SlaveAcknowledge::Ack)
    }

    pub fn slave_nack(&mut self) -> Result<(), Error> {
        self.set_slave_acknowledge(SlaveAcknowledge::Nack)
    }

    /// Waits until this slave address is matched.
    ///
    /// With clock stretching enabled, SCL remains stretched until
    /// [`slave_receive`](Self::slave_receive) or
    /// [`slave_transmit`](Self::slave_transmit) clears the `ADDR` flag.
    pub fn wait_address(&mut self) -> Result<AddressMatch, Error> {
        if self.config.mode != Mode::Slave {
            return Err(Error::InvalidMode);
        }

        let regs = regs::<I2C>();

        for _ in 0..self.config.timeout {
            Self::check_slave_bus_errors(regs)?;
            let isr = regs.isr().read();

            if isr.addr().bit_is_set() {
                let address = isr.addcode().bits();
                let direction = if isr.dir().bit_is_set() {
                    SlaveDirection::Transmit
                } else {
                    SlaveDirection::Receive
                };

                return Ok(AddressMatch {
                    address,
                    source: self.address_match_source(address),
                    direction,
                });
            }

            if isr.stopf().bit_is_set() || isr.nackf().bit_is_set() {
                Self::clear_slave_end_flags(regs);
            }
        }

        Err(Error::Timeout)
    }

    fn address_match_source(&self, address: u8) -> AddressMatchSource {
        if self.config.general_call && address == 0 {
            AddressMatchSource::GeneralCall
        } else if self.config.address_primary as u8 == address {
            AddressMatchSource::Primary
        } else {
            // ADDR can only be raised for an enabled own address or general
            // call. With OA1 and general call ruled out above, this is OA2,
            // including matches accepted through OA2MSK.
            AddressMatchSource::Secondary
        }
    }

    /// Receives bytes written by the master until STOP or repeated START.
    /// If more bytes arrive than fit in `buffer`, the extra byte is discarded,
    /// NACK is requested, and [`SlaveBufferStatus::Overflow`] is reported.
    pub fn slave_receive(&mut self, buffer: &mut [u8]) -> Result<SlaveTransfer, Error> {
        self.ensure_slave_direction(SlaveDirection::Receive)?;
        let regs = regs::<I2C>();
        let result = self.slave_receive_inner(regs, buffer);

        if result.is_err() {
            self.recover_slave(regs);
        }

        result
    }

    /// Transmits bytes requested by the master until NACK, STOP, or repeated
    /// START. If the master requests more bytes than supplied,
    /// [`Config::underflow_fill`] is sent and
    /// [`SlaveBufferStatus::Underflow`] is reported.
    pub fn slave_transmit(&mut self, buffer: &[u8]) -> Result<SlaveTransfer, Error> {
        self.ensure_slave_direction(SlaveDirection::Transmit)?;
        let regs = regs::<I2C>();
        let result = self.slave_transmit_inner(regs, buffer);

        if result.is_err() {
            self.recover_slave(regs);
        }

        result
    }

    fn ensure_slave_direction(&self, expected: SlaveDirection) -> Result<(), Error> {
        if self.config.mode != Mode::Slave {
            return Err(Error::InvalidMode);
        }

        let isr = regs::<I2C>().isr().read();
        if isr.addr().bit_is_clear() {
            return Err(Error::InvalidDirection);
        }

        let actual = if isr.dir().bit_is_set() {
            SlaveDirection::Transmit
        } else {
            SlaveDirection::Receive
        };

        if actual != expected {
            return Err(Error::InvalidDirection);
        }

        Ok(())
    }

    fn slave_receive_inner(
        &self,
        i2c: &RegisterBlock,
        buffer: &mut [u8],
    ) -> Result<SlaveTransfer, Error> {
        Self::clear_slave_end_flags(i2c);
        Self::clear_address(i2c);

        let mut count = 0;
        let mut overflow = false;
        let mut remaining = self.config.timeout;

        loop {
            if remaining == 0 {
                return Err(Error::SlaveTimeout(SlaveTimeout {
                    direction: SlaveDirection::Receive,
                    count,
                    buffer_status: if overflow {
                        SlaveBufferStatus::Overflow
                    } else {
                        SlaveBufferStatus::Complete
                    },
                }));
            }
            remaining -= 1;

            Self::check_slave_bus_errors(i2c)?;
            let isr = i2c.isr().read();

            if isr.rxne().bit_is_set() {
                let byte = Self::read_byte(i2c);
                if let Some(slot) = buffer.get_mut(count) {
                    *slot = byte;
                    count += 1;
                } else {
                    overflow = true;
                    i2c.cr2().modify(|_, w| w.nack().set_bit());
                }
                remaining = self.config.timeout;
                continue;
            }

            if isr.addr().bit_is_set() {
                return Ok(SlaveTransfer {
                    count,
                    end: SlaveTransferEnd::RepeatedStart,
                    buffer_status: if overflow {
                        SlaveBufferStatus::Overflow
                    } else {
                        SlaveBufferStatus::Complete
                    },
                });
            }

            if isr.stopf().bit_is_set() {
                Self::finish_slave_transfer(i2c);
                return Ok(SlaveTransfer {
                    count,
                    end: SlaveTransferEnd::Stop,
                    buffer_status: if overflow {
                        SlaveBufferStatus::Overflow
                    } else {
                        SlaveBufferStatus::Complete
                    },
                });
            }
        }
    }

    fn slave_transmit_inner(
        &self,
        i2c: &RegisterBlock,
        buffer: &[u8],
    ) -> Result<SlaveTransfer, Error> {
        Self::clear_slave_end_flags(i2c);
        Self::flush_txdr(i2c);
        Self::clear_address(i2c);

        let mut count = 0;
        let mut underflow = false;
        let mut saw_nack = false;
        let mut remaining = self.config.timeout;

        if let Some(byte) = buffer.get(count) {
            Self::write_byte(i2c, *byte);
            count += 1;
        } else {
            Self::write_byte(i2c, self.config.underflow_fill);
            underflow = true;
        }

        loop {
            if remaining == 0 {
                return Err(Error::SlaveTimeout(SlaveTimeout {
                    direction: SlaveDirection::Transmit,
                    count,
                    buffer_status: if underflow {
                        SlaveBufferStatus::Underflow
                    } else {
                        SlaveBufferStatus::Complete
                    },
                }));
            }
            remaining -= 1;

            Self::check_slave_bus_errors(i2c)?;
            let isr = i2c.isr().read();

            if isr.nackf().bit_is_set() {
                i2c.icr().write(|w| w.nackcf().set_bit());
                saw_nack = true;
                remaining = self.config.timeout;
            }

            if isr.addr().bit_is_set() {
                Self::flush_txdr(i2c);
                return Ok(SlaveTransfer {
                    count,
                    end: SlaveTransferEnd::RepeatedStart,
                    buffer_status: if underflow {
                        SlaveBufferStatus::Underflow
                    } else {
                        SlaveBufferStatus::Complete
                    },
                });
            }

            if isr.stopf().bit_is_set() {
                Self::finish_slave_transfer(i2c);
                return Ok(SlaveTransfer {
                    count,
                    end: if saw_nack {
                        SlaveTransferEnd::Nack
                    } else {
                        SlaveTransferEnd::Stop
                    },
                    buffer_status: if underflow {
                        SlaveBufferStatus::Underflow
                    } else {
                        SlaveBufferStatus::Complete
                    },
                });
            }

            if saw_nack && isr.busy().bit_is_clear() {
                Self::finish_slave_transfer(i2c);
                return Ok(SlaveTransfer {
                    count,
                    end: SlaveTransferEnd::Nack,
                    buffer_status: if underflow {
                        SlaveBufferStatus::Underflow
                    } else {
                        SlaveBufferStatus::Complete
                    },
                });
            }

            if !saw_nack && isr.txis().bit_is_set() {
                if let Some(byte) = buffer.get(count) {
                    Self::write_byte(i2c, *byte);
                    count += 1;
                } else {
                    Self::write_byte(i2c, self.config.underflow_fill);
                    underflow = true;
                }
                remaining = self.config.timeout;
            }
        }
    }

    fn disable(i2c: &RegisterBlock) {
        i2c.cr1().modify(|_, w| w.pe().clear_bit());
    }

    fn enable(i2c: &RegisterBlock) {
        i2c.cr1().modify(|_, w| w.pe().set_bit());
    }

    fn configure_filters(i2c: &RegisterBlock) {
        i2c.cr1()
            .write(|w| unsafe { w.pe().clear_bit().anfoff().clear_bit().dnf().bits(0) });
    }

    fn configure_timing(i2c: &RegisterBlock, timing: Timing) {
        i2c.timingr().write(|w| unsafe {
            w.presc()
                .bits(timing.prescaler)
                .scldel()
                .bits(timing.scl_delay)
                .sdadel()
                .bits(timing.sda_delay)
                .sclh()
                .bits(timing.scl_high)
                .scll()
                .bits(timing.scl_low)
        });
    }

    fn configure_stretching(i2c: &RegisterBlock) {
        i2c.cr1().modify(|_, w| w.nostretch().clear_bit());
    }

    fn configure_slave(i2c: &RegisterBlock, config: Config) {
        Self::configure_primary_address(i2c, config.address_primary);
        Self::configure_secondary_address(i2c, config.address_secondary);

        if config.general_call {
            i2c.cr1().modify(|_, w| w.gcen().set_bit());
        } else {
            i2c.cr1().modify(|_, w| w.gcen().clear_bit());
        }

        if config.sbc_mode {
            i2c.cr1().modify(|_, w| w.sbc().set_bit());
        } else {
            i2c.cr1().modify(|_, w| w.sbc().clear_bit());
        }
    }

    fn configure_primary_address(i2c: &RegisterBlock, address: u16) {
        i2c.oar1().write(|w| w.oa1en().clear_bit());

        if address <= I2C_ADDRESS_7BIT_MAX {
            i2c.oar1()
                .modify(|_, w| unsafe { w.oa1mode()._7bit().oa1_7bit().bits(address as u8) });
        } else {
            i2c.oar1()
                .modify(|_, w| unsafe { w.oa1mode()._10bit().oa1_10bit().bits(address) });
        }

        i2c.oar1().modify(|_, w| w.oa1en().set_bit());
    }

    fn configure_secondary_address(i2c: &RegisterBlock, address: Option<SecondaryAddress>) {
        i2c.oar2().write(|w| w.oa2en().nack());

        if let Some(address) = address {
            i2c.oar2().write(|w| {
                let w = unsafe { w.oa2().bits(address.address) };
                let w = match address.mask {
                    SecondaryAddressMask::Exact => w.oa2msk().no_mask(),
                    SecondaryAddressMask::IgnoreOneBit => w.oa2msk()._1_1_masked(),
                    SecondaryAddressMask::IgnoreTwoBits => w.oa2msk()._2_1_masked(),
                    SecondaryAddressMask::IgnoreThreeBits => w.oa2msk()._3_1_masked(),
                    SecondaryAddressMask::IgnoreFourBits => w.oa2msk()._4_1_masked(),
                    SecondaryAddressMask::IgnoreFiveBits => w.oa2msk()._5_1_masked(),
                    SecondaryAddressMask::IgnoreSixBits => w.oa2msk()._6_1_masked(),
                    SecondaryAddressMask::AllNonReserved => w.oa2msk()._7_1_masked(),
                };
                w.oa2en().ack()
            });
        }
    }
}

impl<I2C: Instance> ErrorType for I2c<I2C> {
    type Error = Error;
}

impl<I2C: Instance> HalI2c<SevenBitAddress> for I2c<I2C> {
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        if self.config.mode != Mode::Master {
            return Err(Error::InvalidMode);
        }
        if address as u16 > I2C_ADDRESS_7BIT_MAX {
            return Err(Error::InvalidAddress);
        }

        let regs = regs::<I2C>();
        Self::clear_flags(regs);
        if let Err(error) = self.wait_bus_idle(regs) {
            Self::flush_txdr(regs);
            Self::clear_flags(regs);
            return Err(error);
        }

        let result = (|| {
            let mut previous_direction = None;

            for index in 0..operations.len() {
                let direction = match &operations[index] {
                    Operation::Read(_) => Direction::Read,
                    Operation::Write(_) => Direction::Write,
                };
                let next_direction = operations.get(index + 1).map(|operation| match operation {
                    Operation::Read(_) => Direction::Read,
                    Operation::Write(_) => Direction::Write,
                });
                let send_start = previous_direction != Some(direction);
                let ends_direction = next_direction != Some(direction);
                let send_stop = next_direction.is_none();

                match &mut operations[index] {
                    Operation::Read(buffer) => self.transaction_read(
                        address,
                        buffer,
                        send_start,
                        ends_direction,
                        send_stop,
                    )?,
                    Operation::Write(bytes) => self.transaction_write(
                        address,
                        bytes,
                        send_start,
                        ends_direction,
                        send_stop,
                    )?,
                }

                previous_direction = Some(direction);
            }

            Ok(())
        })();

        if let Err(error) = result {
            self.recover(regs, error);
        }

        result
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
enum Direction {
    Read,
    Write,
}

impl<I2C: Instance> I2c<I2C> {
    fn transaction_read(
        &mut self,
        address: SevenBitAddress,
        buffer: &mut [u8],
        send_start: bool,
        ends_direction: bool,
        send_stop: bool,
    ) -> Result<(), Error> {
        let regs = regs::<I2C>();

        if buffer.is_empty() {
            return self.transaction_empty(
                regs,
                address,
                Direction::Read,
                send_start,
                ends_direction,
                send_stop,
            );
        }

        let buffer_start = buffer.as_ptr();
        let mut chunks = buffer.chunks_mut(I2C_NBYTE_MAX).peekable();

        while let Some(chunk) = chunks.next() {
            let is_first_chunk = chunk.as_ptr() == buffer_start;
            let is_last_chunk = chunks.peek().is_none();
            let reload = !is_last_chunk || !ends_direction;
            let autoend = is_last_chunk && send_stop;

            Self::configure_transfer_size(regs, chunk.len(), reload, autoend);

            if is_first_chunk && send_start {
                Self::configure_address(regs, address);
                Self::start(regs, Direction::Read);
            }

            for byte in chunk {
                self.wait_rxne(regs)?;
                *byte = Self::read_byte(regs);
            }

            self.wait_chunk_end(regs, reload, autoend)?;
        }

        Ok(())
    }

    fn transaction_write(
        &mut self,
        address: SevenBitAddress,
        bytes: &[u8],
        send_start: bool,
        ends_direction: bool,
        send_stop: bool,
    ) -> Result<(), Error> {
        let regs = regs::<I2C>();

        if bytes.is_empty() {
            return self.transaction_empty(
                regs,
                address,
                Direction::Write,
                send_start,
                ends_direction,
                send_stop,
            );
        }

        let mut chunks = bytes.chunks(I2C_NBYTE_MAX).peekable();

        while let Some(chunk) = chunks.next() {
            let is_first_chunk = chunk.as_ptr() == bytes.as_ptr();
            let is_last_chunk = chunks.peek().is_none();
            let reload = !is_last_chunk || !ends_direction;
            let autoend = is_last_chunk && send_stop;

            Self::configure_transfer_size(regs, chunk.len(), reload, autoend);

            if is_first_chunk && send_start {
                Self::configure_address(regs, address);
                Self::start(regs, Direction::Write);
            }

            for byte in chunk {
                self.wait_txis(regs)?;
                Self::write_byte(regs, *byte);
            }

            self.wait_chunk_end(regs, reload, autoend)?;
        }

        Ok(())
    }

    fn transaction_empty(
        &self,
        i2c: &RegisterBlock,
        address: SevenBitAddress,
        direction: Direction,
        send_start: bool,
        ends_direction: bool,
        send_stop: bool,
    ) -> Result<(), Error> {
        let reload = !ends_direction;
        Self::configure_transfer_size(i2c, 0, reload, send_stop);

        if send_start {
            Self::configure_address(i2c, address);
            Self::start(i2c, direction);
        }

        self.wait_chunk_end(i2c, reload, send_stop)
    }

    fn configure_transfer_size(i2c: &RegisterBlock, len: usize, reload: bool, autoend: bool) {
        i2c.cr2().modify(|_, w| unsafe {
            w.nbytes()
                .bits(len as u8)
                .reload()
                .bit(reload)
                .autoend()
                .bit(autoend)
        });
    }

    fn configure_address(i2c: &RegisterBlock, address: SevenBitAddress) {
        i2c.cr2().modify(|_, w| unsafe {
            w.add10()
                .clear_bit()
                .sadd_7bit()
                .bits(address & I2C_ADDRESS_7BIT_MAX as u8)
        });
    }

    fn start(i2c: &RegisterBlock, direction: Direction) {
        i2c.cr2().modify(|_, w| {
            let w = match direction {
                Direction::Read => w.rd_wrn().set_bit(),
                Direction::Write => w.rd_wrn().clear_bit(),
            };
            w.start().set_bit()
        });
    }

    fn write_byte(i2c: &RegisterBlock, byte: u8) {
        i2c.txdr().write(|w| unsafe { w.txdata().bits(byte) });
    }

    fn read_byte(i2c: &RegisterBlock) -> u8 {
        i2c.rxdr().read().txdata().bits()
    }

    fn wait_chunk_end(
        &self,
        i2c: &RegisterBlock,
        reload: bool,
        autoend: bool,
    ) -> Result<(), Error> {
        if reload {
            self.wait_tcr(i2c)
        } else if autoend {
            self.wait_stop(i2c)
        } else {
            self.wait_tc(i2c)
        }
    }

    fn check_errors(i2c: &RegisterBlock) -> Result<(), Error> {
        let isr = i2c.isr().read();

        if isr.nackf().bit_is_set() {
            i2c.icr().write(|w| w.nackcf().set_bit());
            return Err(Error::Nack);
        }

        Self::check_slave_bus_errors(i2c)
    }

    fn check_slave_bus_errors(i2c: &RegisterBlock) -> Result<(), Error> {
        let isr = i2c.isr().read();

        if isr.berr().bit_is_set() {
            i2c.icr().write(|w| w.berrcf().set_bit());
            return Err(Error::BusError);
        }
        if isr.arlo().bit_is_set() {
            i2c.icr().write(|w| w.arlocf().set_bit());
            return Err(Error::ArbitrationLoss);
        }
        if isr.ovr().bit_is_set() {
            i2c.icr().write(|w| w.ovrcf().set_bit());
            return Err(Error::Overrun);
        }

        Ok(())
    }

    fn clear_address(i2c: &RegisterBlock) {
        i2c.icr().write(|w| w.addrcf().set_bit());
    }

    fn clear_slave_end_flags(i2c: &RegisterBlock) {
        i2c.icr().write(|w| w.nackcf().set_bit().stopcf().set_bit());
    }

    fn finish_slave_transfer(i2c: &RegisterBlock) {
        Self::flush_txdr(i2c);
        Self::clear_slave_end_flags(i2c);
        i2c.cr2().modify(|_, w| w.nack().clear_bit());
    }

    fn clear_flags(i2c: &RegisterBlock) {
        i2c.icr().write(|w| {
            w.nackcf()
                .set_bit()
                .stopcf()
                .set_bit()
                .berrcf()
                .set_bit()
                .arlocf()
                .set_bit()
                .ovrcf()
                .set_bit()
        });
    }

    fn wait_until(
        &self,
        i2c: &RegisterBlock,
        ready: impl Fn(&RegisterBlock) -> bool,
    ) -> Result<(), Error> {
        for _ in 0..self.config.timeout {
            Self::check_errors(i2c)?;
            if ready(i2c) {
                return Ok(());
            }
        }

        Err(Error::Timeout)
    }

    /// Waits until `TXDR` is ready for the next byte in master transmit mode.
    ///
    /// Polls the `ISR` register until the `TXIS` flag is set.
    ///
    /// # Errors
    ///
    /// Returns:
    /// - [`Error::Nack`] if the `NACKF` flag is set.
    /// - [`Error::BusError`] if the `BERR` flag is set.
    /// - [`Error::ArbitrationLoss`] if the `ARLO` flag is set.
    /// - [`Error::Timeout`] if `TXIS` is not set before `Config::timeout`
    ///   polling attempts are exhausted.
    fn wait_txis(&self, i2c: &RegisterBlock) -> Result<(), Error> {
        self.wait_until(i2c, |i2c| i2c.isr().read().txis().bit_is_set())
    }

    fn wait_rxne(&self, i2c: &RegisterBlock) -> Result<(), Error> {
        self.wait_until(i2c, |i2c| i2c.isr().read().rxne().bit_is_set())
    }

    /// Waits until the I2C bus is no longer busy.
    ///
    /// Polls the `ISR` register until the `BUSY` flag is cleared.
    /// # Errors
    ///
    /// Returns [`Error::Timeout`] if the `BUSY` flag is still set after
    /// `Config::timeout` polling attempts are exhausted.
    fn wait_bus_idle(&self, i2c: &RegisterBlock) -> Result<(), Error> {
        for _ in 0..self.config.timeout {
            if i2c.isr().read().busy().bit_is_clear() {
                return Ok(());
            }
        }

        Err(Error::Timeout)
    }

    /// Waits until the current master transfer is complete.
    ///
    /// Polls the `ISR` register until the `TC` flag is set.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Timeout`] if the `TC` flag is not set before
    /// `Config::timeout` polling attempts are exhausted.
    fn wait_tc(&self, i2c: &RegisterBlock) -> Result<(), Error> {
        self.wait_until(i2c, |i2c| i2c.isr().read().tc().bit_is_set())
    }

    fn wait_tcr(&self, i2c: &RegisterBlock) -> Result<(), Error> {
        self.wait_until(i2c, |i2c| i2c.isr().read().tcr().bit_is_set())
    }

    fn wait_stop(&self, i2c: &RegisterBlock) -> Result<(), Error> {
        self.wait_until(i2c, |i2c| i2c.isr().read().stopf().bit_is_set())?;
        Self::flush_txdr(i2c);
        i2c.icr().write(|w| w.stopcf().set_bit());
        Ok(())
    }

    fn flush_txdr(i2c: &RegisterBlock) {
        // TXE is software-settable to discard pending TXDR data, but the PAC
        // does not currently expose a typed writer for this bit.
        i2c.isr().write(|w| unsafe { w.bits(1) });
    }

    fn recover(&self, i2c: &RegisterBlock, error: Error) {
        Self::flush_txdr(i2c);

        // After losing arbitration another master owns the bus. Generating
        // STOP or toggling PE here would interfere with its transaction.
        if error == Error::ArbitrationLoss {
            Self::clear_flags(i2c);
            return;
        }

        if i2c.isr().read().busy().bit_is_set() {
            i2c.cr2().modify(|_, w| w.stop().set_bit());

            for _ in 0..self.config.timeout {
                if i2c.isr().read().stopf().bit_is_set() || i2c.isr().read().busy().bit_is_clear() {
                    break;
                }
            }
        }

        Self::flush_txdr(i2c);
        Self::clear_flags(i2c);

        if i2c.isr().read().busy().bit_is_set() {
            Self::disable(i2c);
            Self::enable(i2c);
        }
    }

    fn recover_slave(&self, i2c: &RegisterBlock) {
        Self::flush_txdr(i2c);
        i2c.cr2().modify(|_, w| w.nack().clear_bit());
        if i2c.isr().read().addr().bit_is_set() {
            Self::clear_address(i2c);
        }
        Self::clear_flags(i2c);
        Self::disable(i2c);
        Self::enable(i2c);
    }
}

fn regs<I2C: Instance>() -> &'static RegisterBlock {
    unsafe { &*I2C::ptr() }
}
