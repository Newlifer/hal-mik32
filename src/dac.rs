//! Digital-to-analog converter (DAC).
//!
//! MIK32 has two independent 12-bit DAC channels. DAC0 is routed to P1.12 and
//! DAC1 is routed to P1.13. Each channel has a single-word input buffer. The
//! hardware exposes buffer-ready signals to EPIC and DMA. This module supports
//! polling and DMA writes and identifies the corresponding EPIC line.

use core::fmt;

use embedded_dma::ReadBuffer;
use embedded_hal_nb::nb::{Error as NbError, Result as NbResult};
use mik32_pac::{Dac0 as Dac0Peripheral, Dac1 as Dac1Peripheral, RefvConfig};

use crate::clock::Hertz;
use crate::dma::{Channel as DmaChannel, ChannelId as DmaChannelId, Error as DmaError};
use crate::epic::Interrupt;
use crate::gpio::{Analog, Pin};

/// DAC resolution in bits.
pub const RESOLUTION_BITS: u8 = 12;

/// Largest value accepted by a DAC channel.
pub const MAX_CODE: u16 = (1 << RESOLUTION_BITS) - 1;

/// Maximum documented DAC sample rate.
pub const MAX_FREQUENCY: Hertz = Hertz(1_000_000);

/// Divider suitable for a 32 MHz DAC input clock and the documented 1 MHz limit.
pub const DEFAULT_DIVIDER: u8 = 31;

/// Default number of readiness polls performed during initialization.
pub const DEFAULT_READY_TIMEOUT: u32 = 500_000;

const CFG_ENABLE: u32 = 1 << 0;
const CFG_RESET_RELEASE: u32 = 1 << 1;
const CFG_DIV_SHIFT: u32 = 2;
const CFG_DIV_MASK: u32 = 0xff << CFG_DIV_SHIFT;
const CFG_EXTERNAL_REFERENCE: u32 = 1 << 10;
const CFG_EXTERNAL_REFERENCE_PIN: u32 = 1 << 11;

/// A value which is guaranteed to fit into the 12-bit DAC data register.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Code(u16);

impl Code {
    pub const ZERO: Self = Self(0);
    pub const MAX: Self = Self(MAX_CODE);

    /// Creates a checked 12-bit DAC code.
    pub const fn new(value: u16) -> Option<Self> {
        if value <= MAX_CODE {
            Some(Self(value))
        } else {
            None
        }
    }

    /// Creates a DAC code, clamping values above [`MAX_CODE`].
    pub const fn saturating(value: u16) -> Self {
        Self(if value > MAX_CODE { MAX_CODE } else { value })
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

impl From<u8> for Code {
    fn from(value: u8) -> Self {
        Self(u16::from(value))
    }
}

impl TryFrom<u16> for Code {
    type Error = ValueOutOfRange;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        Self::new(value).ok_or(ValueOutOfRange(value))
    }
}

impl From<Code> for u16 {
    fn from(value: Code) -> Self {
        value.get()
    }
}

/// Error returned when a value does not fit into 12 bits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ValueOutOfRange(pub u16);

/// A four-bit calibration coefficient.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CalibrationCoefficient(u8);

impl CalibrationCoefficient {
    pub const DEFAULT: Self = Self(8);

    pub const fn new(value: u8) -> Option<Self> {
        if value <= 0x0f {
            Some(Self(value))
        } else {
            None
        }
    }

    pub const fn get(self) -> u8 {
        self.0
    }
}

impl Default for CalibrationCoefficient {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl TryFrom<u8> for CalibrationCoefficient {
    type Error = CalibrationOutOfRange;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Self::new(value).ok_or(CalibrationOutOfRange(value))
    }
}

/// Error returned when a reference calibration coefficient does not fit into four bits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CalibrationOutOfRange(pub u8);

/// Error returned when a DAC clock cannot produce the requested sample rate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrequencyError {
    /// The supplied `FIN` clock is zero.
    InputClockZero,
    /// The requested rate exceeds the documented hardware limit.
    FrequencyTooHigh { requested: Hertz, maximum: Hertz },
    /// The requested rate needs a divider larger than the eight-bit field.
    FrequencyTooLow { requested: Hertz, minimum: Hertz },
}

/// DAC initialization error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InitError {
    /// The input register did not become ready after reset was released.
    ReadyTimeout,
}

/// DAC operation error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// An output value was requested while the DAC was disabled.
    Disabled,
}

/// Error returned by [`Dac::blocking_flush_timeout`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlushError {
    /// The DAC is disabled.
    Disabled,
    /// The input value was not transferred into the DAC before timeout.
    Timeout,
}

/// Error returned by DAC DMA writes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DmaWriteError {
    /// The DAC is disabled.
    Disabled,
    /// A buffer element does not fit in the 12-bit DAC input register.
    ValueOutOfRange { index: usize, value: u16 },
    /// The DMA buffer is empty.
    EmptyBuffer,
    /// The DMA transfer length does not fit in the hardware counter.
    TransferTooLong,
    /// The DMA channel did not complete within the requested timeout.
    DmaTimeout,
    /// DMA reported a bus error.
    Bus,
    /// The DMA channel is not supported by the controller.
    InvalidChannel,
    /// DMA completed, but the final value was not transferred into the DAC
    /// within the requested timeout.
    ReadyTimeout,
}

impl From<DmaError> for DmaWriteError {
    fn from(error: DmaError) -> Self {
        match error {
            DmaError::EmptyBuffer => Self::EmptyBuffer,
            DmaError::TransferTooLong => Self::TransferTooLong,
            DmaError::Timeout => Self::DmaTimeout,
            DmaError::Bus => Self::Bus,
            DmaError::InvalidChannel => Self::InvalidChannel,
        }
    }
}

/// Coefficients of the shared calibrated voltage and current references.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Calibration {
    pub voltage: CalibrationCoefficient,
    pub current: CalibrationCoefficient,
}

impl Default for Calibration {
    fn default() -> Self {
        Self {
            voltage: CalibrationCoefficient::DEFAULT,
            current: CalibrationCoefficient::DEFAULT,
        }
    }
}

/// Fixed internal 1.2 V reference.
#[derive(Clone, Copy, Debug, Default)]
pub struct InternalReference;

/// Shared configurable reference backed by `REFV_CONFIG`.
///
/// Pass `&CalibratedReference` to [`Dac::new_with_reference`]. Multiple DAC
/// channels may borrow the same reference. Its calibration cannot be changed
/// while those borrows are alive.
pub struct CalibratedReference {
    peripheral: RefvConfig,
}

impl CalibratedReference {
    /// Enables and configures the shared calibrated reference.
    pub fn new(peripheral: RefvConfig, calibration: Calibration) -> Self {
        enable_analog_register_clock();
        let reference = Self { peripheral };
        reference.configure(calibration, true);
        reference
    }

    pub fn calibration(&self) -> Calibration {
        let register = self.peripheral.ref_clb().read();
        Calibration {
            voltage: CalibrationCoefficient(register.coef_refvclb().bits()),
            current: CalibrationCoefficient(register.coef_reficlb().bits()),
        }
    }

    pub fn set_calibration(&mut self, calibration: Calibration) {
        self.configure(calibration, self.is_enabled());
    }

    pub fn is_enabled(&self) -> bool {
        self.peripheral.ref_clb().read().clb_en().is_enable()
    }

    pub fn enable(&mut self) {
        self.peripheral.ref_clb().modify(|_, w| w.clb_en().enable());
    }

    pub fn disable(&mut self) {
        self.peripheral
            .ref_clb()
            .modify(|_, w| w.clb_en().disable());
    }

    pub fn release(self) -> RefvConfig {
        self.peripheral
    }

    fn configure(&self, calibration: Calibration, enabled: bool) {
        self.peripheral.ref_clb().modify(|_, w| unsafe {
            w.coef_refvclb()
                .bits(calibration.voltage.get())
                .coef_reficlb()
                .bits(calibration.current.get())
                .clb_en()
                .bit(enabled)
        });
    }
}

/// External reference voltage connected to P1.11.
///
/// Owning the pin prevents safe code from repurposing it while a DAC borrows
/// this reference.
pub struct ExternalReference {
    pin: Pin<1, 11, Analog>,
}

impl ExternalReference {
    pub const fn new(pin: Pin<1, 11, Analog>) -> Self {
        Self { pin }
    }

    pub fn release(self) -> Pin<1, 11, Analog> {
        self.pin
    }
}

mod reference_sealed {
    pub trait Sealed {}
}

/// A reference source accepted by a DAC channel.
pub trait ReferenceSource: reference_sealed::Sealed {
    #[doc(hidden)]
    const CONFIG_BITS: u32;
}

impl reference_sealed::Sealed for InternalReference {}
impl ReferenceSource for InternalReference {
    const CONFIG_BITS: u32 = 0;
}

impl reference_sealed::Sealed for CalibratedReference {}
impl ReferenceSource for CalibratedReference {
    const CONFIG_BITS: u32 = CFG_EXTERNAL_REFERENCE;
}

impl reference_sealed::Sealed for &CalibratedReference {}
impl ReferenceSource for &CalibratedReference {
    const CONFIG_BITS: u32 = CFG_EXTERNAL_REFERENCE;
}

impl reference_sealed::Sealed for ExternalReference {}
impl ReferenceSource for ExternalReference {
    const CONFIG_BITS: u32 = CFG_EXTERNAL_REFERENCE | CFG_EXTERNAL_REFERENCE_PIN;
}

impl reference_sealed::Sealed for &ExternalReference {}
impl ReferenceSource for &ExternalReference {
    const CONFIG_BITS: u32 = CFG_EXTERNAL_REFERENCE | CFG_EXTERNAL_REFERENCE_PIN;
}

/// Initial DAC channel configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    /// Raw divider value: the DAC clock is `FIN / (divider + 1)`.
    ///
    /// The chip documentation does not identify the clock domain supplying
    /// `FIN`, so the caller must derive this value from a known DAC input clock.
    /// The resulting frequency must not exceed [`MAX_FREQUENCY`].
    pub divider: u8,
    /// Value queued during initialization.
    pub initial_value: Code,
    /// Maximum number of unsuccessful readiness polls during initialization.
    pub ready_timeout: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            divider: DEFAULT_DIVIDER,
            initial_value: Code::ZERO,
            ready_timeout: DEFAULT_READY_TIMEOUT,
        }
    }
}

impl Config {
    /// Selects the smallest divider that does not exceed `requested`.
    ///
    /// `input_clock` is the DAC `FIN` clock. The reference documentation does
    /// not identify its RCC domain, so callers must supply a frequency whose
    /// origin is known for their hardware configuration.
    pub const fn frequency(
        mut self,
        input_clock: Hertz,
        requested: Hertz,
    ) -> Result<Self, FrequencyError> {
        match calculate_divider(input_clock, requested) {
            Ok(divider) => {
                self.divider = divider;
                Ok(self)
            }
            Err(error) => Err(error),
        }
    }

    pub const fn divider(mut self, divider: u8) -> Self {
        self.divider = divider;
        self
    }

    pub const fn initial_value(mut self, value: Code) -> Self {
        self.initial_value = value;
        self
    }

    /// Sets the initialization readiness timeout in polling iterations.
    ///
    /// A value of zero still performs one readiness check.
    pub const fn ready_timeout(mut self, polls: u32) -> Self {
        self.ready_timeout = polls;
        self
    }

    /// Validates the raw divider and returns the resulting DAC frequency.
    pub const fn validate_frequency(&self, input_clock: Hertz) -> Result<Hertz, FrequencyError> {
        validate_divider(input_clock, self.divider)
    }
}

/// An initialized DAC channel which owns its peripheral and output pin.
pub struct Dac<DAC: Instance, REF: ReferenceSource = InternalReference> {
    peripheral: DAC,
    output: DAC::OutputPin,
    reference: REF,
    divider: u8,
    ready_timeout: u32,
}

/// Active non-blocking DAC DMA transfer.
///
/// Dropping this guard stops the DMA channel before releasing the buffer.
pub struct DacDmaTransfer<DAC: Instance, REF: ReferenceSource, CHANNEL: DmaChannelId, BUFFER> {
    dac: Option<Dac<DAC, REF>>,
    channel: Option<DmaChannel<CHANNEL>>,
    buffer: Option<BUFFER>,
    dma_done: bool,
}

/// Failed DAC DMA operation with all consumed resources returned.
pub struct DmaTransferFailure<DAC: Instance, REF: ReferenceSource, CHANNEL: DmaChannelId, BUFFER> {
    pub error: DmaWriteError,
    pub dac: Dac<DAC, REF>,
    pub channel: DmaChannel<CHANNEL>,
    pub buffer: BUFFER,
}

impl<DAC: Instance, REF: ReferenceSource, CHANNEL: DmaChannelId, BUFFER> fmt::Debug
    for DmaTransferFailure<DAC, REF, CHANNEL, BUFFER>
{
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("DmaTransferFailure")
            .field("error", &self.error)
            .finish_non_exhaustive()
    }
}

pub type Dac0<REF = InternalReference> = Dac<Dac0Peripheral, REF>;
pub type Dac1<REF = InternalReference> = Dac<Dac1Peripheral, REF>;

/// Failed initialization with all consumed resources returned to the caller.
pub struct InitFailure<DAC: Instance, REF: ReferenceSource> {
    pub error: InitError,
    pub peripheral: DAC,
    pub output: DAC::OutputPin,
    pub reference: REF,
}

impl<DAC: Instance, REF: ReferenceSource> fmt::Debug for InitFailure<DAC, REF> {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("InitFailure")
            .field("error", &self.error)
            .finish_non_exhaustive()
    }
}

impl<DAC: Instance> Dac<DAC, InternalReference> {
    /// Creates and enables a channel using the fixed internal 1.2 V reference.
    ///
    /// Returns all supplied resources if the input register does not become
    /// ready within [`Config::ready_timeout`].
    pub fn new(
        peripheral: DAC,
        output: DAC::OutputPin,
        config: Config,
    ) -> Result<Self, InitFailure<DAC, InternalReference>> {
        Self::new_with_reference(peripheral, output, InternalReference, config)
    }
}

impl<DAC: Instance, REF: ReferenceSource> Dac<DAC, REF> {
    /// Creates and enables a channel using an explicit reference source.
    ///
    /// Returns all supplied resources if the input register does not become
    /// ready within [`Config::ready_timeout`].
    pub fn new_with_reference(
        peripheral: DAC,
        output: DAC::OutputPin,
        reference: REF,
        config: Config,
    ) -> Result<Self, InitFailure<DAC, REF>> {
        enable_analog_register_clock();

        // Match the vendor initialization order: clear CFG, power up and
        // release reset, then configure the clock and reference mux.
        peripheral.reset_config();
        peripheral.enable();
        peripheral.configure(config.divider, REF::CONFIG_BITS);

        if !wait_until_ready(&peripheral, config.ready_timeout) {
            peripheral.disable();
            return Err(InitFailure {
                error: InitError::ReadyTimeout,
                peripheral,
                output,
                reference,
            });
        }

        peripheral.write(config.initial_value);

        Ok(Self {
            peripheral,
            output,
            reference,
            divider: config.divider,
            ready_timeout: config.ready_timeout,
        })
    }

    /// Returns whether the one-word input buffer can accept a new value.
    ///
    /// `true` corresponds to `EMPTY_READ = 1`: the previous value has been
    /// transferred from `DAC_VALUE` into the converter. Writing `DAC_VALUE`
    /// clears the flag until that transfer occurs.
    #[inline(always)]
    pub fn is_ready(&self) -> bool {
        self.peripheral.is_ready()
    }

    /// Queues a value if the DAC input buffer is ready.
    #[inline(always)]
    pub fn try_write(&mut self, value: Code) -> NbResult<(), Error> {
        if !self.is_enabled() {
            return Err(NbError::Other(Error::Disabled));
        }
        if !self.is_ready() {
            return Err(NbError::WouldBlock);
        }
        self.peripheral.write(value);
        Ok(())
    }

    /// Waits until the input buffer is ready, then queues a value.
    pub fn blocking_write(&mut self, value: Code) -> Result<(), Error> {
        if !self.is_enabled() {
            return Err(Error::Disabled);
        }
        while !self.is_ready() {
            core::hint::spin_loop();
        }
        self.peripheral.write(value);
        Ok(())
    }

    /// Writes `VALUE` without checking whether an older value is still pending.
    ///
    /// This is useful for setting a static output, but can overwrite an
    /// unprocessed sample in a streamed waveform.
    #[inline(always)]
    pub fn force_write(&mut self, value: Code) -> Result<(), Error> {
        if !self.is_enabled() {
            return Err(Error::Disabled);
        }
        self.peripheral.write(value);
        Ok(())
    }

    /// Checks whether the last queued value was transferred into the DAC.
    pub fn flush(&mut self) -> NbResult<(), Error> {
        if !self.is_enabled() {
            return Err(NbError::Other(Error::Disabled));
        }
        if !self.is_ready() {
            return Err(NbError::WouldBlock);
        }
        Ok(())
    }

    /// Waits until the last queued value is transferred into the DAC.
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        if !self.is_enabled() {
            return Err(Error::Disabled);
        }
        while !self.is_ready() {
            core::hint::spin_loop();
        }
        Ok(())
    }

    /// Waits for the last queued value with a bounded number of polls.
    pub fn blocking_flush_timeout(&mut self, timeout: u32) -> Result<(), FlushError> {
        if !self.is_enabled() {
            return Err(FlushError::Disabled);
        }
        if wait_until_ready(&self.peripheral, timeout) {
            Ok(())
        } else {
            Err(FlushError::Timeout)
        }
    }

    /// Writes a slice of 12-bit samples through DMA and waits for completion.
    ///
    /// The DMA channel and buffer are borrowed for the whole operation, so they
    /// remain available to the caller after success or failure. Every sample is
    /// checked before DMA starts. `timeout` is used independently for DMA
    /// completion and for the final `EMPTY_READ` wait.
    pub fn blocking_write_dma<CHANNEL: DmaChannelId>(
        &mut self,
        channel: &mut DmaChannel<CHANNEL>,
        buffer: &[u16],
        timeout: u32,
    ) -> Result<(), DmaWriteError> {
        if !self.is_enabled() {
            return Err(DmaWriteError::Disabled);
        }
        let length = validate_dma_buffer(buffer)?;
        channel
            .transfer(
                buffer.as_ptr().cast(),
                self.peripheral.value_ptr(),
                length,
                dma_write_config::<DAC>(),
                timeout,
            )
            .map_err(DmaWriteError::from)?;

        if !wait_until_ready(&self.peripheral, timeout) {
            return Err(DmaWriteError::ReadyTimeout);
        }
        Ok(())
    }

    /// Starts a non-blocking DMA write of 12-bit samples.
    ///
    /// The DAC, channel and DMA-safe buffer are returned by
    /// [`DacDmaTransfer::wait`], [`DacDmaTransfer::wait_timeout`] or
    /// [`DacDmaTransfer::abort`]. Dropping the guard aborts the transfer.
    /// Every sample is validated before DMA starts.
    pub fn write_dma<CHANNEL: DmaChannelId, BUFFER>(
        self,
        mut channel: DmaChannel<CHANNEL>,
        buffer: BUFFER,
    ) -> Result<
        DacDmaTransfer<DAC, REF, CHANNEL, BUFFER>,
        DmaTransferFailure<DAC, REF, CHANNEL, BUFFER>,
    >
    where
        BUFFER: ReadBuffer<Word = u16>,
    {
        if !self.is_enabled() {
            return Err(DmaTransferFailure {
                error: DmaWriteError::Disabled,
                dac: self,
                channel,
                buffer,
            });
        }

        let (source, words) = unsafe { buffer.read_buffer() };
        let values = unsafe { core::slice::from_raw_parts(source, words) };
        let length = match validate_dma_buffer(values) {
            Ok(length) => length,
            Err(error) => {
                return Err(DmaTransferFailure {
                    error,
                    dac: self,
                    channel,
                    buffer,
                });
            }
        };

        if let Err(error) = channel.start(
            source.cast(),
            self.peripheral.value_ptr(),
            length,
            dma_write_config::<DAC>(),
        ) {
            return Err(DmaTransferFailure {
                error: error.into(),
                dac: self,
                channel,
                buffer,
            });
        }

        Ok(DacDmaTransfer {
            dac: Some(self),
            channel: Some(channel),
            buffer: Some(buffer),
            dma_done: false,
        })
    }

    pub fn divider(&self) -> u8 {
        self.divider
    }

    /// Sets the raw divider without validating the resulting frequency.
    ///
    /// Use [`Dac::set_frequency`] when `FIN` is known and the documented
    /// 1 MHz limit must be enforced.
    pub fn set_divider(&mut self, divider: u8) {
        self.peripheral.set_divider(divider);
        self.divider = divider;
    }

    /// Configures the closest DAC frequency not exceeding `requested`.
    ///
    /// Returns the frequency produced by the selected integer divider.
    pub fn set_frequency(
        &mut self,
        input_clock: Hertz,
        requested: Hertz,
    ) -> Result<Hertz, FrequencyError> {
        let divider = calculate_divider(input_clock, requested)?;
        self.set_divider(divider);
        Ok(clock_from_divider(input_clock, divider))
    }

    /// Returns the DAC clock derived from a caller-provided `F_IN` clock.
    ///
    /// The SVD specifies the divider formula but does not identify `F_IN`, so
    /// this method deliberately does not assume a particular RCC clock.
    pub fn clock_frequency(&self, input_clock: Hertz) -> Hertz {
        clock_from_divider(input_clock, self.divider)
    }

    pub fn is_enabled(&self) -> bool {
        self.peripheral.is_enabled()
    }

    /// Powers up the DAC, releases its active-low reset and waits for readiness.
    ///
    /// If the input register does not become ready within the timeout configured
    /// at construction, the channel is disabled again.
    pub fn enable(&mut self) -> Result<(), InitError> {
        self.peripheral.enable();
        if !wait_until_ready(&self.peripheral, self.ready_timeout) {
            self.peripheral.disable();
            return Err(InitError::ReadyTimeout);
        }
        Ok(())
    }

    /// Powers down the DAC and asserts its active-low reset.
    ///
    /// The reference documentation does not specify the electrical state of
    /// the output pin while the DAC is disabled.
    pub fn disable(&mut self) {
        self.peripheral.disable();
    }

    pub const fn interrupt(&self) -> Interrupt {
        DAC::INTERRUPT
    }

    /// Returns the DMA request number driven by this DAC channel.
    ///
    /// DAC0 uses request 10 and DAC1 uses request 11. DAC transfers are not yet
    /// implemented by this module; the value is exposed for diagnostics and
    /// integration with the DMA driver.
    pub const fn dma_request(&self) -> u8 {
        DAC::DMA_REQUEST
    }

    /// Disables the DAC and returns its peripheral, output pin and reference.
    pub fn release(mut self) -> (DAC, DAC::OutputPin, REF) {
        self.disable();
        (self.peripheral, self.output, self.reference)
    }

    /// Returns all owned resources without changing the DAC state.
    ///
    /// Use this when low-level code must take over an already configured DAC.
    pub fn release_enabled(self) -> (DAC, DAC::OutputPin, REF) {
        (self.peripheral, self.output, self.reference)
    }
}

impl<DAC: Instance, REF: ReferenceSource, CHANNEL: DmaChannelId, BUFFER>
    DacDmaTransfer<DAC, REF, CHANNEL, BUFFER>
{
    /// Returns `true` after DMA and the final transfer into the DAC complete.
    pub fn is_done(&mut self) -> Result<bool, DmaWriteError> {
        if !self.dma_done {
            self.dma_done = self
                .channel
                .as_mut()
                .expect("DAC DMA transfer channel missing")
                .poll()
                .map_err(DmaWriteError::from)?;
            if !self.dma_done {
                return Ok(false);
            }
        }
        Ok(self
            .dac
            .as_ref()
            .expect("DAC DMA transfer peripheral missing")
            .is_ready())
    }

    /// Waits without a timeout and returns all transfer resources.
    pub fn wait(
        mut self,
    ) -> Result<
        (Dac<DAC, REF>, DmaChannel<CHANNEL>, BUFFER),
        DmaTransferFailure<DAC, REF, CHANNEL, BUFFER>,
    > {
        loop {
            match self.is_done() {
                Ok(true) => return Ok(self.take_parts()),
                Ok(false) => core::hint::spin_loop(),
                Err(error) => return Err(self.take_failure(error)),
            }
        }
    }

    /// Waits for at most `timeout` polling iterations.
    pub fn wait_timeout(
        mut self,
        timeout: u32,
    ) -> Result<
        (Dac<DAC, REF>, DmaChannel<CHANNEL>, BUFFER),
        DmaTransferFailure<DAC, REF, CHANNEL, BUFFER>,
    > {
        for _ in 0..timeout {
            match self.is_done() {
                Ok(true) => return Ok(self.take_parts()),
                Ok(false) => core::hint::spin_loop(),
                Err(error) => return Err(self.take_failure(error)),
            }
        }
        let error = if self.dma_done {
            DmaWriteError::ReadyTimeout
        } else {
            DmaWriteError::DmaTimeout
        };
        Err(self.take_failure(error))
    }

    /// Stops DMA and returns the DAC, channel and buffer.
    pub fn abort(mut self) -> (Dac<DAC, REF>, DmaChannel<CHANNEL>, BUFFER) {
        self.take_parts()
    }

    fn stop(&mut self) {
        if let Some(channel) = self.channel.as_mut() {
            channel.stop();
        }
    }

    fn take_parts(&mut self) -> (Dac<DAC, REF>, DmaChannel<CHANNEL>, BUFFER) {
        self.stop();
        (
            self.dac
                .take()
                .expect("DAC DMA transfer peripheral missing"),
            self.channel
                .take()
                .expect("DAC DMA transfer channel missing"),
            self.buffer.take().expect("DAC DMA transfer buffer missing"),
        )
    }

    fn take_failure(
        &mut self,
        error: DmaWriteError,
    ) -> DmaTransferFailure<DAC, REF, CHANNEL, BUFFER> {
        let (dac, channel, buffer) = self.take_parts();
        DmaTransferFailure {
            error,
            dac,
            channel,
            buffer,
        }
    }
}

impl<DAC: Instance, REF: ReferenceSource, CHANNEL: DmaChannelId, BUFFER> Drop
    for DacDmaTransfer<DAC, REF, CHANNEL, BUFFER>
{
    fn drop(&mut self) {
        self.stop();
    }
}

mod instance_sealed {
    pub trait Sealed {}
}

/// PAC peripheral accepted by [`Dac`].
pub trait Instance: instance_sealed::Sealed {
    type OutputPin;

    #[doc(hidden)]
    const INTERRUPT: Interrupt;
    #[doc(hidden)]
    const DMA_REQUEST: u8;

    #[doc(hidden)]
    fn reset_config(&self);
    #[doc(hidden)]
    fn configure(&self, divider: u8, reference_bits: u32);
    #[doc(hidden)]
    fn set_divider(&self, divider: u8);
    #[doc(hidden)]
    fn is_ready(&self) -> bool;
    #[doc(hidden)]
    fn write(&self, value: Code);
    #[doc(hidden)]
    fn value_ptr(&self) -> *mut u8;
    #[doc(hidden)]
    fn is_enabled(&self) -> bool;
    #[doc(hidden)]
    fn enable(&self);
    #[doc(hidden)]
    fn disable(&self);
}

macro_rules! impl_instance {
    ($pac:ty, $pin:ty, $cfg:ident, $value:ident, $interrupt:ident, $dma_request:expr) => {
        impl instance_sealed::Sealed for $pac {}

        impl Instance for $pac {
            type OutputPin = $pin;

            const INTERRUPT: Interrupt = Interrupt::$interrupt;
            const DMA_REQUEST: u8 = $dma_request;

            #[inline(always)]
            fn reset_config(&self) {
                self.$cfg().reset();
            }

            #[inline(always)]
            fn configure(&self, divider: u8, reference_bits: u32) {
                self.$cfg().modify(|r, w| unsafe {
                    w.bits(
                        (r.bits() & !CFG_DIV_MASK)
                            | (u32::from(divider) << CFG_DIV_SHIFT)
                            | reference_bits,
                    )
                });
            }

            #[inline(always)]
            fn set_divider(&self, divider: u8) {
                self.$cfg().modify(|r, w| unsafe {
                    w.bits((r.bits() & !CFG_DIV_MASK) | (u32::from(divider) << CFG_DIV_SHIFT))
                });
            }

            #[inline(always)]
            fn is_ready(&self) -> bool {
                self.$cfg().read().empty_read().is_empty()
            }

            #[inline(always)]
            fn write(&self, value: Code) {
                self.$value()
                    .write(|w| unsafe { w.value().bits(value.get()) });
            }

            #[inline(always)]
            fn value_ptr(&self) -> *mut u8 {
                core::ptr::from_ref(self.$value()).cast_mut().cast()
            }

            #[inline(always)]
            fn is_enabled(&self) -> bool {
                self.$cfg().read().en().is_enable()
            }

            #[inline(always)]
            fn enable(&self) {
                self.$cfg()
                    .modify(|r, w| unsafe { w.bits(r.bits() | CFG_ENABLE | CFG_RESET_RELEASE) });
            }

            #[inline(always)]
            fn disable(&self) {
                self.$cfg()
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(CFG_ENABLE | CFG_RESET_RELEASE)) });
            }
        }
    };
}

impl_instance!(
    Dac0Peripheral,
    Pin<1, 12, Analog>,
    dac0_cfg,
    dac0_value,
    Dac0,
    10
);
impl_instance!(
    Dac1Peripheral,
    Pin<1, 13, Analog>,
    dac1_cfg,
    dac1_value,
    Dac1,
    11
);

#[inline(always)]
fn enable_analog_register_clock() {
    let pm = unsafe { mik32_pac::Pm::steal() };
    pm.clk_apb_p_set().write(|w| w.analog_regs().enable());
}

fn wait_until_ready<DAC: Instance>(peripheral: &DAC, timeout: u32) -> bool {
    let mut remaining = timeout;
    loop {
        if peripheral.is_ready() {
            return true;
        }
        if remaining == 0 {
            return false;
        }
        remaining -= 1;
        core::hint::spin_loop();
    }
}

const fn dma_write_config<DAC: Instance>() -> u32 {
    const SOURCE_MEMORY_MODE: u32 = 1 << 3;
    const SOURCE_INCREMENT: u32 = 1 << 5;
    const SOURCE_HALFWORD: u32 = 1 << 7;
    const DESTINATION_HALFWORD: u32 = 1 << 9;
    const SOURCE_BURST_HALFWORD: u32 = 1 << 11;
    const DESTINATION_BURST_HALFWORD: u32 = 1 << 14;
    const DESTINATION_REQUEST_SHIFT: u32 = 21;
    const DESTINATION_ACK: u32 = 1 << 26;

    SOURCE_MEMORY_MODE
        | SOURCE_INCREMENT
        | SOURCE_HALFWORD
        | DESTINATION_HALFWORD
        | SOURCE_BURST_HALFWORD
        | DESTINATION_BURST_HALFWORD
        | ((DAC::DMA_REQUEST as u32) << DESTINATION_REQUEST_SHIFT)
        | DESTINATION_ACK
}

fn validate_dma_buffer(buffer: &[u16]) -> Result<usize, DmaWriteError> {
    if buffer.is_empty() {
        return Err(DmaWriteError::EmptyBuffer);
    }
    for (index, value) in buffer.iter().copied().enumerate() {
        if value > MAX_CODE {
            return Err(DmaWriteError::ValueOutOfRange { index, value });
        }
    }
    let length = buffer
        .len()
        .checked_mul(core::mem::size_of::<u16>())
        .ok_or(DmaWriteError::TransferTooLong)?;
    if length > u32::MAX as usize {
        return Err(DmaWriteError::TransferTooLong);
    }
    Ok(length)
}

const fn calculate_divider(input_clock: Hertz, requested: Hertz) -> Result<u8, FrequencyError> {
    if input_clock.0 == 0 {
        return Err(FrequencyError::InputClockZero);
    }
    if requested.0 > MAX_FREQUENCY.0 {
        return Err(FrequencyError::FrequencyTooHigh {
            requested,
            maximum: MAX_FREQUENCY,
        });
    }

    let minimum = Hertz(input_clock.0.div_ceil(256));
    if requested.0 == 0 {
        return Err(FrequencyError::FrequencyTooLow { requested, minimum });
    }

    let divisor = input_clock.0.div_ceil(requested.0);
    if divisor > 256 {
        return Err(FrequencyError::FrequencyTooLow { requested, minimum });
    }

    Ok(divisor.saturating_sub(1) as u8)
}

const fn validate_divider(input_clock: Hertz, divider: u8) -> Result<Hertz, FrequencyError> {
    if input_clock.0 == 0 {
        return Err(FrequencyError::InputClockZero);
    }
    let frequency = clock_from_divider(input_clock, divider);
    if frequency.0 > MAX_FREQUENCY.0 {
        return Err(FrequencyError::FrequencyTooHigh {
            requested: frequency,
            maximum: MAX_FREQUENCY,
        });
    }
    Ok(frequency)
}

const fn clock_from_divider(input_clock: Hertz, divider: u8) -> Hertz {
    Hertz(input_clock.0 / (divider as u32 + 1))
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::Cell;

    struct MockDac {
        enabled: Cell<bool>,
        ready: Cell<bool>,
        writes: Cell<u32>,
    }

    impl MockDac {
        const fn new(enabled: bool, ready: bool) -> Self {
            Self {
                enabled: Cell::new(enabled),
                ready: Cell::new(ready),
                writes: Cell::new(0),
            }
        }
    }

    impl instance_sealed::Sealed for MockDac {}

    impl Instance for MockDac {
        type OutputPin = ();

        const INTERRUPT: Interrupt = Interrupt::Dac0;
        const DMA_REQUEST: u8 = 10;

        fn reset_config(&self) {}
        fn configure(&self, _divider: u8, _reference_bits: u32) {}
        fn set_divider(&self, _divider: u8) {}
        fn is_ready(&self) -> bool {
            self.ready.get()
        }
        fn write(&self, _value: Code) {
            self.writes.set(self.writes.get() + 1);
        }
        fn value_ptr(&self) -> *mut u8 {
            core::ptr::null_mut()
        }
        fn is_enabled(&self) -> bool {
            self.enabled.get()
        }
        fn enable(&self) {
            self.enabled.set(true);
        }
        fn disable(&self) {
            self.enabled.set(false);
        }
    }

    fn mock_dac(enabled: bool, ready: bool) -> Dac<MockDac> {
        Dac {
            peripheral: MockDac::new(enabled, ready),
            output: (),
            reference: InternalReference,
            divider: 0,
            ready_timeout: 0,
        }
    }

    #[test]
    fn code_checks_12_bit_range() {
        assert_eq!(Code::new(0), Some(Code::ZERO));
        assert_eq!(Code::new(MAX_CODE), Some(Code::MAX));
        assert_eq!(Code::new(MAX_CODE + 1), None);
        assert_eq!(Code::saturating(u16::MAX), Code::MAX);
    }

    #[test]
    fn calibration_coefficient_checks_four_bit_range() {
        assert_eq!(
            CalibrationCoefficient::new(0),
            Some(CalibrationCoefficient(0))
        );
        assert_eq!(
            CalibrationCoefficient::new(0x0f),
            Some(CalibrationCoefficient(0x0f))
        );
        assert_eq!(CalibrationCoefficient::new(0x10), None);
    }

    #[test]
    fn reference_bits_match_the_hardware_mux() {
        assert_eq!(InternalReference::CONFIG_BITS, 0);
        assert_eq!(CalibratedReference::CONFIG_BITS, CFG_EXTERNAL_REFERENCE);
        assert_eq!(
            ExternalReference::CONFIG_BITS,
            CFG_EXTERNAL_REFERENCE | CFG_EXTERNAL_REFERENCE_PIN
        );
    }

    #[test]
    fn dma_requests_match_the_hardware_map() {
        assert_eq!(<Dac0Peripheral as Instance>::DMA_REQUEST, 10);
        assert_eq!(<Dac1Peripheral as Instance>::DMA_REQUEST, 11);
    }

    #[test]
    fn default_divider_keeps_a_32_mhz_input_at_1_mhz() {
        let config = Config::default();
        assert_eq!(config.divider, DEFAULT_DIVIDER);
        assert_eq!(32_000_000 / (u32::from(config.divider) + 1), 1_000_000);
    }

    #[test]
    fn frequency_selects_a_rate_not_above_the_request() {
        let exact = Config::default()
            .frequency(Hertz::mhz(32), Hertz::mhz(1))
            .unwrap();
        assert_eq!(exact.divider, 31);
        assert_eq!(exact.validate_frequency(Hertz::mhz(32)), Ok(Hertz::mhz(1)));

        let rounded = Config::default()
            .frequency(Hertz::mhz(32), Hertz::khz(900))
            .unwrap();
        assert_eq!(rounded.divider, 35);
        assert_eq!(
            rounded.validate_frequency(Hertz::mhz(32)),
            Ok(Hertz(888_888))
        );
    }

    #[test]
    fn frequency_rejects_rates_outside_the_supported_range() {
        assert_eq!(
            Config::default().frequency(Hertz::mhz(32), Hertz(1_000_001)),
            Err(FrequencyError::FrequencyTooHigh {
                requested: Hertz(1_000_001),
                maximum: MAX_FREQUENCY,
            })
        );
        assert_eq!(
            Config::default().frequency(Hertz::mhz(32), Hertz(124_999)),
            Err(FrequencyError::FrequencyTooLow {
                requested: Hertz(124_999),
                minimum: Hertz::khz(125),
            })
        );
        assert_eq!(
            Config::default().frequency(Hertz(0), Hertz::khz(1)),
            Err(FrequencyError::InputClockZero)
        );
    }

    #[test]
    fn writes_are_rejected_while_disabled() {
        let mut dac = mock_dac(false, true);
        assert_eq!(
            dac.try_write(Code::ZERO),
            Err(NbError::Other(Error::Disabled))
        );
        assert_eq!(dac.blocking_write(Code::ZERO), Err(Error::Disabled));
        assert_eq!(dac.force_write(Code::ZERO), Err(Error::Disabled));
        assert_eq!(dac.peripheral.writes.get(), 0);
    }

    #[test]
    fn enable_rolls_back_when_ready_times_out() {
        let mut dac = mock_dac(false, false);
        assert_eq!(dac.enable(), Err(InitError::ReadyTimeout));
        assert!(!dac.is_enabled());
    }

    #[test]
    fn release_disables_the_channel() {
        let dac = mock_dac(true, true);
        let (peripheral, (), InternalReference) = dac.release();
        assert!(!peripheral.enabled.get());
    }

    #[test]
    fn dma_config_uses_halfwords_and_the_channel_request() {
        assert_eq!(
            dma_write_config::<Dac0Peripheral>(),
            (1 << 3)
                | (1 << 5)
                | (1 << 7)
                | (1 << 9)
                | (1 << 11)
                | (1 << 14)
                | (10 << 21)
                | (1 << 26)
        );
        assert_eq!(
            dma_write_config::<Dac1Peripheral>(),
            (1 << 3)
                | (1 << 5)
                | (1 << 7)
                | (1 << 9)
                | (1 << 11)
                | (1 << 14)
                | (11 << 21)
                | (1 << 26)
        );
    }

    #[test]
    fn dma_buffer_is_validated_before_transfer() {
        assert_eq!(validate_dma_buffer(&[]), Err(DmaWriteError::EmptyBuffer));
        assert_eq!(validate_dma_buffer(&[0, MAX_CODE]), Ok(4));
        assert_eq!(
            validate_dma_buffer(&[0, MAX_CODE + 1, 1]),
            Err(DmaWriteError::ValueOutOfRange {
                index: 1,
                value: MAX_CODE + 1,
            })
        );
    }
}
