//! HAL for the basic 32-bit timer (`Timer32_0`).

use crate::{
    clock::Hertz,
    rcc::{self, LSI32K_FREQ, OSC32K_FREQ, RCC},
};
use fugit::{HertzU32, MicrosDurationU32, NanosDurationU64};
use mik32_pac::Timer32_0 as Timer32Peripheral;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CountMode {
    #[default]
    Up,
    Down,
    UpDown,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ClockSource {
    /// HCLK divided by `prescaler + 1`.
    #[default]
    Prescaler,
    SystemClock,
    Hclk,
    Osc32k,
    Lsi32k,
    ExternalPin,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    Overflow,
    Underflow,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    ZeroPeriod,
    PeriodTooLong,
    FrequencyTooHigh,
    FrequencyTooLow,
    UnknownClockFrequency,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Period {
    pub prescaler: u32,
    pub top: u32,
    pub actual: NanosDurationU64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    pub top: u32,
    /// The divider is `prescaler + 1` when [`ClockSource::Prescaler`] is used.
    pub prescaler: u32,
    pub count_mode: CountMode,
    pub clock_source: ClockSource,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            top: u32::MAX,
            prescaler: 0,
            count_mode: CountMode::Up,
            clock_source: ClockSource::Prescaler,
        }
    }
}

/// Basic 32-bit counter without capture/compare/PWM channels.
pub struct Timer32_0 {
    peripheral: Timer32Peripheral,
    clock_frequency: Option<Hertz>,
}

impl Timer32_0 {
    /// Configures the timer and leaves it stopped with its counter cleared.
    pub fn new(peripheral: Timer32Peripheral, config: Config) -> Self {
        RCC::enable_timer32_0();

        let mut timer = Self {
            peripheral,
            clock_frequency: clock_frequency(config.clock_source),
        };
        timer.stop();
        timer.set_top(config.top);
        timer.set_prescaler(config.prescaler);
        timer.set_count_mode(config.count_mode);
        timer.set_clock_source(config.clock_source);
        timer.disable_all_interrupts();
        timer.clear_all_events();
        timer.reset_counter();
        timer
    }

    pub fn start(&mut self) {
        self.peripheral.enable().modify(|_, w| w.tim_en().enable());
    }

    pub fn stop(&mut self) {
        self.peripheral.enable().modify(|_, w| w.tim_en().disable());
    }

    pub fn is_running(&self) -> bool {
        self.peripheral.enable().read().tim_en().is_enable()
    }

    pub fn reset_counter(&mut self) {
        self.peripheral
            .enable()
            .modify(|_, w| w.tim_clr().set_bit());
    }

    pub fn counter(&self) -> u32 {
        self.peripheral.value().read().tim_val().bits()
    }

    pub fn top(&self) -> u32 {
        self.peripheral.top().read().tim_top().bits()
    }

    pub fn set_top(&mut self, top: u32) {
        self.peripheral
            .top()
            .write(|w| unsafe { w.tim_top().bits(top) });
    }

    pub fn prescaler(&self) -> u32 {
        self.peripheral.prescale().read().tim_prescale().bits()
    }

    pub fn set_prescaler(&mut self, prescaler: u32) {
        self.peripheral
            .prescale()
            .write(|w| unsafe { w.tim_prescale().bits(prescaler) });
    }

    pub fn set_count_mode(&mut self, mode: CountMode) {
        self.peripheral.control().modify(|_, w| match mode {
            CountMode::Up => w.count_mode().direct(),
            CountMode::Down => w.count_mode().reverse(),
            CountMode::UpDown => w.count_mode().bidirectional(),
        });
    }

    pub fn set_clock_source(&mut self, source: ClockSource) {
        RCC::configure_timer32_0_clock(source);
        let source_bits = match source {
            ClockSource::Prescaler => 0,
            ClockSource::SystemClock | ClockSource::Hclk => 1,
            ClockSource::ExternalPin => 2,
            ClockSource::Osc32k | ClockSource::Lsi32k => 3,
        };
        self.peripheral
            .control()
            .modify(|_, w| unsafe { w.source().bits(source_bits) });
        self.clock_frequency = clock_frequency(source);
    }

    pub fn set_duration(&mut self, duration: MicrosDurationU32) -> Result<Period, Error> {
        let frequency = self.clock_frequency.ok_or(Error::UnknownClockFrequency)?.0;
        calculate_timing(frequency, duration.as_ticks(), 1_000_000).map(|period| {
            self.set_prescaler(period.prescaler);
            self.set_top(period.top);
            period
        })
    }

    pub fn set_frequency(&mut self, frequency: HertzU32) -> Result<Period, Error> {
        let clock = self.clock_frequency.ok_or(Error::UnknownClockFrequency)?.0;
        let requested = frequency.to_raw();
        if requested == 0 {
            return Err(Error::FrequencyTooLow);
        }
        if requested > clock {
            return Err(Error::FrequencyTooHigh);
        }
        calculate_timing(clock, 1, requested).map(|period| {
            self.set_prescaler(period.prescaler);
            self.set_top(period.top);
            period
        })
    }

    pub fn start_periodic(&mut self, duration: MicrosDurationU32) -> Result<Period, Error> {
        let period = self.set_duration(duration)?;
        self.start();
        Ok(period)
    }

    pub fn enable_interrupt(&mut self, event: Event) {
        self.set_interrupt_enabled(event, true);
    }

    pub fn disable_interrupt(&mut self, event: Event) {
        self.set_interrupt_enabled(event, false);
    }

    pub fn event_pending(&self, event: Event) -> bool {
        let flags = self.peripheral.int_flag().read();
        match event {
            Event::Overflow => flags.ovf_int().bit_is_set(),
            Event::Underflow => flags.udf_int().bit_is_set(),
        }
    }

    pub fn clear_event(&mut self, event: Event) {
        self.clear_event_bits(event_mask(event));
    }

    pub fn clear_all_events(&mut self) {
        self.clear_event_bits(event_mask(Event::Overflow) | event_mask(Event::Underflow));
    }

    pub fn release(self) -> Timer32Peripheral {
        self.peripheral
    }

    fn set_interrupt_enabled(&self, event: Event, enabled: bool) {
        self.peripheral.int_mask().modify(|_, w| match event {
            Event::Overflow => w.ovf_int().bit(enabled),
            Event::Underflow => w.udf_int().bit(enabled),
        });
    }

    fn disable_all_interrupts(&self) {
        self.peripheral
            .int_mask()
            .modify(|_, w| w.ovf_int().clear_bit().udf_int().clear_bit());
    }

    fn clear_event_bits(&self, bits: u32) {
        self.peripheral
            .int_clear()
            .write(|w| unsafe { w.bits(bits) });
    }
}

pub(crate) fn clock_frequency(source: ClockSource) -> Option<Hertz> {
    match source {
        ClockSource::Prescaler | ClockSource::Hclk => Some(rcc::clocks().ahbclk()),
        ClockSource::SystemClock => Some(rcc::clocks().sysclk()),
        ClockSource::Osc32k => Some(OSC32K_FREQ),
        ClockSource::Lsi32k => Some(LSI32K_FREQ),
        ClockSource::ExternalPin => None,
    }
}

pub(crate) fn calculate_timing(clock: u32, units: u32, scale: u32) -> Result<Period, Error> {
    if units == 0 {
        return Err(Error::ZeroPeriod);
    }
    let source_ticks = ((clock as u128 * units as u128) + scale as u128 / 2) / scale as u128;
    if source_ticks == 0 {
        return Err(Error::ZeroPeriod);
    }
    let divisor = ((source_ticks + u32::MAX as u128) / (u32::MAX as u128 + 1)).max(1);
    if divisor > u32::MAX as u128 + 1 {
        return Err(Error::PeriodTooLong);
    }
    let counter_ticks = ((source_ticks + divisor / 2) / divisor).clamp(1, u32::MAX as u128 + 1);
    let actual_ticks = counter_ticks * divisor;
    let nanos = (actual_ticks * 1_000_000_000 + clock as u128 / 2) / clock as u128;
    if nanos > u64::MAX as u128 {
        return Err(Error::PeriodTooLong);
    }
    Ok(Period {
        prescaler: (divisor - 1) as u32,
        top: (counter_ticks - 1) as u32,
        actual: NanosDurationU64::from_ticks(nanos as u64),
    })
}

const fn event_mask(event: Event) -> u32 {
    match event {
        Event::Overflow => 1 << 0,
        Event::Underflow => 1 << 1,
    }
}
