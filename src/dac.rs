//! Digital-to-analog converter (DAC).
//!
//! MIK32 has two independent 12-bit DAC channels. DAC0 is routed to P1.12 and
//! DAC1 is routed to P1.13. Each channel has a single-word input buffer which
//! can be fed by polling, an EPIC interrupt, or DMA.

use core::convert::Infallible;

use embedded_hal_nb::nb::{Error as NbError, Result as NbResult};
use mik32_pac::{Dac0 as Dac0Peripheral, Dac1 as Dac1Peripheral, RefvConfig};

use crate::clock::Hertz;
use crate::epic::Interrupt;
use crate::gpio::{Analog, Pin};

/// DAC resolution in bits.
pub const RESOLUTION_BITS: u8 = 12;

/// Largest value accepted by a DAC channel.
pub const MAX_CODE: u16 = (1 << RESOLUTION_BITS) - 1;

/// Divider suitable for a 32 MHz DAC input clock and the documented 1 MHz limit.
pub const DEFAULT_DIVIDER: u8 = 31;

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
    /// Raw divider value: the DAC clock is `F_IN / (divider + 1)`.
    pub divider: u8,
    /// Value queued during initialization.
    pub initial_value: Code,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            divider: DEFAULT_DIVIDER,
            initial_value: Code::ZERO,
        }
    }
}

impl Config {
    pub const fn divider(mut self, divider: u8) -> Self {
        self.divider = divider;
        self
    }

    pub const fn initial_value(mut self, value: Code) -> Self {
        self.initial_value = value;
        self
    }
}

/// An initialized DAC channel which owns its peripheral and output pin.
pub struct Dac<DAC: Instance, REF: ReferenceSource = InternalReference> {
    peripheral: DAC,
    output: DAC::OutputPin,
    reference: REF,
    divider: u8,
}

pub type Dac0<REF = InternalReference> = Dac<Dac0Peripheral, REF>;
pub type Dac1<REF = InternalReference> = Dac<Dac1Peripheral, REF>;

impl<DAC: Instance> Dac<DAC, InternalReference> {
    /// Creates and enables a channel using the fixed internal 1.2 V reference.
    pub fn new(peripheral: DAC, output: DAC::OutputPin, config: Config) -> Self {
        Self::new_with_reference(peripheral, output, InternalReference, config)
    }
}

impl<DAC: Instance, REF: ReferenceSource> Dac<DAC, REF> {
    /// Creates and enables a channel using an explicit reference source.
    pub fn new_with_reference(
        peripheral: DAC,
        output: DAC::OutputPin,
        reference: REF,
        config: Config,
    ) -> Self {
        enable_analog_register_clock();

        // Match the vendor initialization order: clear CFG, power up and
        // release reset, then configure the clock and reference mux.
        peripheral.reset_config();
        peripheral.enable();
        peripheral.configure(config.divider, REF::CONFIG_BITS);
        peripheral.write(config.initial_value);

        Self {
            peripheral,
            output,
            reference,
            divider: config.divider,
        }
    }

    /// Returns whether the one-word input buffer can accept a new value.
    #[inline(always)]
    pub fn is_ready(&self) -> bool {
        self.peripheral.is_ready()
    }

    /// Queues a value if the DAC input buffer is ready.
    #[inline(always)]
    pub fn try_write(&mut self, value: Code) -> NbResult<(), Infallible> {
        if !self.is_ready() {
            return Err(NbError::WouldBlock);
        }
        self.peripheral.write(value);
        Ok(())
    }

    /// Waits until the input buffer is ready, then queues a value.
    pub fn blocking_write(&mut self, value: Code) {
        while !self.is_ready() {
            core::hint::spin_loop();
        }
        self.peripheral.write(value);
    }

    /// Writes `VALUE` without checking whether an older value is still pending.
    ///
    /// This is useful for setting a static output, but can overwrite an
    /// unprocessed sample in a streamed waveform.
    #[inline(always)]
    pub fn force_write(&mut self, value: Code) {
        self.peripheral.write(value);
    }

    pub fn divider(&self) -> u8 {
        self.divider
    }

    pub fn set_divider(&mut self, divider: u8) {
        self.peripheral.set_divider(divider);
        self.divider = divider;
    }

    /// Returns the DAC clock derived from a caller-provided `F_IN` clock.
    ///
    /// The SVD specifies the divider formula but does not identify `F_IN`, so
    /// this method deliberately does not assume a particular RCC clock.
    pub fn clock_frequency(&self, input_clock: Hertz) -> Hertz {
        Hertz(input_clock.0 / (u32::from(self.divider) + 1))
    }

    pub fn is_enabled(&self) -> bool {
        self.peripheral.is_enabled()
    }

    /// Powers up the DAC and releases its active-low reset.
    pub fn enable(&mut self) {
        self.peripheral.enable();
    }

    /// Powers down the DAC and asserts its active-low reset.
    pub fn disable(&mut self) {
        self.peripheral.disable();
    }

    pub const fn interrupt(&self) -> Interrupt {
        DAC::INTERRUPT
    }

    /// Returns the owned peripheral, output pin and reference object/borrow.
    pub fn release(self) -> (DAC, DAC::OutputPin, REF) {
        (self.peripheral, self.output, self.reference)
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

#[cfg(test)]
mod tests {
    use super::*;

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
    fn default_divider_keeps_a_32_mhz_input_at_1_mhz() {
        let config = Config::default();
        assert_eq!(config.divider, DEFAULT_DIVIDER);
        assert_eq!(32_000_000 / (u32::from(config.divider) + 1), 1_000_000);
    }
}
