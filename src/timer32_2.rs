//! HAL for the 32-bit timer with capture/compare channels (`Timer32_2`).

use crate::{
    clock::Hertz,
    rcc::RCC,
    timer32_0::{calculate_timing, clock_frequency},
};
use fugit::{HertzU32, MicrosDurationU32};
use mik32_pac::Timer32_2 as Timer32Peripheral;

pub use crate::timer32_0::{ClockSource, Config, CountMode, Error, Event, Period};
pub use crate::timer32_1::{Channel, PwmPolarity};

/// Timer32_2 basic counter and PWM channels.
pub struct Timer32_2 {
    peripheral: Timer32Peripheral,
    clock_frequency: Option<Hertz>,
}

impl Timer32_2 {
    /// Configures the timer and leaves it stopped with its counter cleared.
    pub fn new(peripheral: Timer32Peripheral, config: Config) -> Self {
        RCC::enable_timer32_2();
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
        RCC::configure_timer32_2_clock(source);
        self.peripheral.control().modify(|_, w| match source {
            ClockSource::Prescaler => w.source().prescaler(),
            ClockSource::SystemClock | ClockSource::Hclk => w.source().tim1(),
            ClockSource::ExternalPin => w.source().tx_pin(),
            ClockSource::Osc32k | ClockSource::Lsi32k => w.source().tim2(),
        });
        self.clock_frequency = clock_frequency(source);
    }

    pub fn set_duration(&mut self, duration: MicrosDurationU32) -> Result<Period, Error> {
        let clock = self.clock_frequency.ok_or(Error::UnknownClockFrequency)?.0;
        let period = calculate_timing(clock, duration.as_ticks(), 1_000_000)?;
        self.set_prescaler(period.prescaler);
        self.set_top(period.top);
        Ok(period)
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
        let period = calculate_timing(clock, 1, requested)?;
        self.set_prescaler(period.prescaler);
        self.set_top(period.top);
        Ok(period)
    }

    pub fn start_periodic(&mut self, duration: MicrosDurationU32) -> Result<Period, Error> {
        let period = self.set_duration(duration)?;
        self.start();
        Ok(period)
    }

    /// Configures a channel for PWM and leaves the channel disabled.
    pub fn configure_pwm(&mut self, channel: Channel, compare: u32, polarity: PwmPolarity) {
        macro_rules! configure {
            ($control:ident, $ocr:ident) => {{
                self.peripheral.$control().write(|w| {
                    let w = w.mode().pwm();
                    match polarity {
                        PwmPolarity::Direct => w.pwm_inv().direct(),
                        PwmPolarity::Inverted => w.pwm_inv().inverted(),
                    }
                });
                self.peripheral
                    .$ocr()
                    .write(|w| unsafe { w.ocr().bits(compare) });
            }};
        }
        match channel {
            Channel::Channel0 => configure!(ch1_cntr, ch1_ocr),
            Channel::Channel1 => configure!(ch2_cntr, ch2_ocr),
            Channel::Channel2 => configure!(ch3_cntr, ch3_ocr),
            Channel::Channel3 => configure!(ch4_cntr, ch4_ocr),
        }
    }

    pub fn set_compare(&mut self, channel: Channel, compare: u32) {
        macro_rules! set {
            ($ocr:ident) => {
                self.peripheral
                    .$ocr()
                    .write(|w| unsafe { w.ocr().bits(compare) })
            };
        }
        match channel {
            Channel::Channel0 => set!(ch1_ocr),
            Channel::Channel1 => set!(ch2_ocr),
            Channel::Channel2 => set!(ch3_ocr),
            Channel::Channel3 => set!(ch4_ocr),
        };
    }

    pub fn enable_channel(&mut self, channel: Channel) {
        match channel {
            Channel::Channel0 => self.peripheral.ch1_cntr().modify(|_, w| w.en().set_bit()),
            Channel::Channel1 => self.peripheral.ch2_cntr().modify(|_, w| w.en().set_bit()),
            Channel::Channel2 => self.peripheral.ch3_cntr().modify(|_, w| w.en().set_bit()),
            Channel::Channel3 => self.peripheral.ch4_cntr().modify(|_, w| w.en().set_bit()),
        };
    }

    pub fn disable_channel(&mut self, channel: Channel) {
        match channel {
            Channel::Channel0 => self.peripheral.ch1_cntr().modify(|_, w| w.en().clear_bit()),
            Channel::Channel1 => self.peripheral.ch2_cntr().modify(|_, w| w.en().clear_bit()),
            Channel::Channel2 => self.peripheral.ch3_cntr().modify(|_, w| w.en().clear_bit()),
            Channel::Channel3 => self.peripheral.ch4_cntr().modify(|_, w| w.en().clear_bit()),
        };
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

const fn event_mask(event: Event) -> u32 {
    match event {
        Event::Overflow => 1 << 0,
        Event::Underflow => 1 << 1,
    }
}
