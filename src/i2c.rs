use embedded_hal::i2c::{
    self, ErrorType, I2c as HalI2c, NoAcknowledgeSource, Operation, SevenBitAddress,
};
use mik32_pac::i2c_0::RegisterBlock;
use mik32_pac::{I2c0, I2c1, Peripherals};

const I2C_ADDRESS_7BIT_MAX: u16 = 0x7f;
const I2C_NBYTE_MAX: usize = 255;
const DEFAULT_TIMEOUT: u32 = 1_000;
const DEFAULT_CONFIG: Config = Config {
    mode: Mode::Master,
    address_primary: 0,
    address_secondary: 0,
    general_call: true,
    sbc_mode: false,
    timeout: DEFAULT_TIMEOUT,
};

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Mode {
    Master,
    Slave,
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
    pub address_secondary: u16,
    pub general_call: bool,
    pub sbc_mode: bool,
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
}

impl Default for Config {
    fn default() -> Self {
        DEFAULT_CONFIG
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Error {
    BusError,
    ArbitrationLoss,
    Nack,
    Timeout,
}

impl i2c::Error for Error {
    fn kind(&self) -> i2c::ErrorKind {
        match *self {
            Error::BusError => i2c::ErrorKind::Bus,
            Error::ArbitrationLoss => i2c::ErrorKind::ArbitrationLoss,
            Error::Nack => i2c::ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown),
            Error::Timeout => i2c::ErrorKind::Other,
        }
    }
}

#[derive(Debug)]
pub struct I2c<I2C: Instance> {
    i2c: I2C,
    config: Config,
}

pub type I2c0Bus = I2c<I2c0>;
pub type I2c1Bus = I2c<I2c1>;
pub type I2CImpl = I2c0Bus;

pub trait Instance {
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
    pub fn new(i2c: I2C, config: Config) -> Self {
        I2C::enable_clock();

        let regs = regs::<I2C>();

        Self::disable(regs);
        Self::configure_filters(regs);
        Self::configure_stretching(regs, config.mode);
        Self::enable(regs);

        if config.mode == Mode::Slave {
            Self::configure_slave(regs, config);
        }

        Self { i2c, config }
    }

    pub fn free(self) -> I2C {
        self.i2c
    }

    fn disable(i2c: &RegisterBlock) {
        i2c.cr1().write(|w| w.pe().clear_bit());
    }

    fn enable(i2c: &RegisterBlock) {
        i2c.cr1().modify(|_, w| w.pe().set_bit());
    }

    fn configure_filters(i2c: &RegisterBlock) {
        i2c.cr1()
            .write(|w| unsafe { w.pe().clear_bit().anfoff().clear_bit().dnf().bits(0) });
    }

    fn configure_stretching(i2c: &RegisterBlock, mode: Mode) {
        i2c.cr1().modify(|_, w| match mode {
            Mode::Master => w.nostretch().clear_bit(),
            Mode::Slave => w.nostretch().set_bit(),
        });
    }

    fn configure_slave(i2c: &RegisterBlock, config: Config) {
        Self::configure_primary_address(i2c, config.address_primary);

        if config.general_call {
            i2c.cr1().modify(|_, w| w.gcen().set_bit());
        } else {
            i2c.cr1().modify(|_, w| w.gcen().clear_bit());
        }

        if config.sbc_mode {
            i2c.cr1().modify(|_, w| w.sbc().clear_bit());
        } else {
            i2c.cr1().modify(|_, w| w.sbc().set_bit());
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
        let operations_len = operations.len();

        for (index, operation) in operations.iter_mut().enumerate() {
            let is_first = index == 0;
            let is_last = index + 1 == operations_len;

            match operation {
                Operation::Read(buffer) => {
                    self.transaction_read(address, buffer, is_first, is_last)?;
                }
                Operation::Write(bytes) => {
                    self.transaction_write(address, bytes, is_first, is_last)?;
                }
            }
        }

        Ok(())
    }
}

impl<I2C: Instance> I2c<I2C> {
    fn transaction_read(
        &mut self,
        address: SevenBitAddress,
        buffer: &mut [u8],
        is_first: bool,
        is_last: bool,
    ) -> Result<(), Error> {
        let _ = (address, buffer, is_first, is_last);

        // TODO: prepare START/repeated START, send address with read direction,
        // receive bytes into buffer, and send STOP when this is the last operation.
        Ok(())
    }

    fn transaction_write(
        &mut self,
        address: SevenBitAddress,
        bytes: &[u8],
        is_first: bool,
        is_last: bool,
    ) -> Result<(), Error> {
        let _ = (is_first, is_last);
        let regs = regs::<I2C>();

        let mut chunks = bytes.chunks(I2C_NBYTE_MAX).enumerate().peekable();

        while let Some((index, chunk)) = chunks.next() {
            let is_first_chunk = index == 0;
            let is_last_chunk = chunks.peek().is_none();

            if is_last_chunk {
                self.wait_for_last_chunk_slot(regs)?;
            }

            Self::configure_transfer_size(regs, chunk.len());

            if is_first_chunk {
                Self::configure_write_address(regs, address);
                Self::start_write(regs);
            }

            for byte in chunk {
                self.wait_txis(regs)?;
                Self::write_byte(regs, *byte);
            }
        }

        Ok(())
    }

    fn configure_transfer_size(i2c: &RegisterBlock, len: usize) {
        i2c.cr2().modify(|_, w| unsafe {
            w.nbytes()
                .bits(len as u8)
                .reload()
                .clear_bit()
                .autoend()
                .set_bit()
        });
    }

    fn configure_write_address(i2c: &RegisterBlock, address: SevenBitAddress) {
        i2c.cr2().modify(|_, w| unsafe {
            w.add10()
                .clear_bit()
                .sadd_7bit()
                .bits(address & I2C_ADDRESS_7BIT_MAX as u8)
        });
    }

    fn start_write(i2c: &RegisterBlock) {
        i2c.cr2()
            .modify(|_, w| w.rd_wrn().clear_bit().start().set_bit());
    }

    fn write_byte(i2c: &RegisterBlock, byte: u8) {
        i2c.txdr().write(|w| unsafe { w.txdata().bits(byte) });
    }

    fn wait_for_last_chunk_slot(&self, i2c: &RegisterBlock) -> Result<(), Error> {
        if i2c.isr().read().tc().bit_is_set() {
            self.wait_busy(i2c)
        } else {
            self.wait_tc(i2c)
        }
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
        for _ in 0..self.config.timeout {
            let isr = i2c.isr().read();

            if isr.nackf().bit_is_set() {
                return Err(Error::Nack);
            }

            if isr.berr().bit_is_set() {
                return Err(Error::BusError);
            }

            if isr.arlo().bit_is_set() {
                return Err(Error::ArbitrationLoss);
            }

            if isr.txis().bit_is_set() {
                return Ok(());
            }
        }

        Err(Error::Timeout)
    }

    /// Waits until the I2C bus is no longer busy.
    ///
    /// Polls the `ISR` register until the `BUSY` flag is cleared.
    /// # Errors
    ///
    /// Returns [`Error::Timeout`] if the `BUSY` flag is still set after
    /// `Config::timeout` polling attempts are exhausted.
    fn wait_busy(&self, i2c: &RegisterBlock) -> Result<(), Error> {
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
        for _ in 0..self.config.timeout {
            if i2c.isr().read().tc().bit_is_set() {
                return Ok(());
            }
        }

        Err(Error::Timeout)
    }
}

fn regs<I2C: Instance>() -> &'static RegisterBlock {
    unsafe { &*I2C::ptr() }
}
