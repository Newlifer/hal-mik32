//! HAL for the three 16-bit timers.

use crate::{
    clock::Hertz,
    rcc::{self, HSI32M_FREQ, LSI32K_FREQ, OSC32K_FREQ, OSC32M_FREQ},
};
use fugit::{HertzU32, MicrosDurationU32, NanosDurationU64};
use mik32_pac::{Timer16_0, Timer16_1, Timer16_2};

const MAX_COUNTER_TICKS: u64 = u16::MAX as u64 + 1;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TimerClock {
    #[default]
    SysClk,
    Hclk,
    Osc32m,
    Hsi32m,
    Osc32k,
    Lsi32k,
}

impl TimerClock {
    fn frequency(self) -> Hertz {
        match self {
            Self::SysClk => rcc::clocks().sysclk(),
            Self::Hclk => rcc::clocks().ahbclk(),
            Self::Osc32m => OSC32M_FREQ,
            Self::Hsi32m => HSI32M_FREQ,
            Self::Osc32k => OSC32K_FREQ,
            Self::Lsi32k => LSI32K_FREQ,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    ZeroPeriod,
    PeriodTooLong,
    FrequencyTooHigh,
    FrequencyTooLow,
    ClockDisabled,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Period {
    pub prescaler: Prescaler,
    pub auto_reload: u16,
    pub actual: NanosDurationU64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum Prescaler {
    #[default]
    Div1 = 0,
    Div2 = 1,
    Div4 = 2,
    Div8 = 3,
    Div16 = 4,
    Div32 = 5,
    Div64 = 6,
    Div128 = 7,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum OutputMode {
    #[default]
    NonInverted,
    Inverted,
    SetOnce,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    CompareMatch,
    AutoReloadMatch,
    ExternalTrigger,
    AutoReloadUpdateDone,
    CountUp,
    CountDown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    pub period: u16,
    pub compare: u16,
    pub prescaler: Prescaler,
    pub output: OutputMode,
    /// Apply new ARR/CMP values at the end of the current period.
    pub preload: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            period: u16::MAX,
            compare: 0,
            prescaler: Prescaler::Div1,
            output: OutputMode::NonInverted,
            preload: true,
        }
    }
}

/// A configured 16-bit timer which owns its PAC peripheral.
pub struct Timer16<T: Instance> {
    peripheral: T,
    clock_frequency: Hertz,
}

impl<T: Instance> Timer16<T> {
    /// Enable the timer clock, select its source and leave the timer stopped.
    pub fn new(peripheral: T, clock: TimerClock) -> Result<Self, Error> {
        Self::new_with_clock(
            peripheral,
            clock,
            Config {
                preload: false,
                ..Config::default()
            },
        )
    }

    /// Apply a raw configuration using the currently selected timer clock.
    pub fn new_raw(peripheral: T, config: Config) -> Self {
        let clock_frequency = T::current_clock().frequency();
        peripheral.stop();
        peripheral.configure(config);
        peripheral.set_period(config.period);
        peripheral.set_compare(config.compare);
        peripheral.clear_all_events();
        Self {
            peripheral,
            clock_frequency,
        }
    }

    /// Enable the timer clock, select its source and apply a raw configuration.
    pub fn new_with_clock(peripheral: T, clock: TimerClock, config: Config) -> Result<Self, Error> {
        if !clock_is_ready(clock) {
            return Err(Error::ClockDisabled);
        }

        T::enable_clock();
        T::select_clock(clock);
        Ok(Self::new_raw(peripheral, config))
    }

    pub fn start(&mut self) {
        self.peripheral.start_continuous();
    }

    pub fn start_one_shot(&mut self) {
        self.peripheral.start_one_shot();
    }

    pub fn stop(&mut self) {
        self.peripheral.stop();
    }

    pub fn counter(&self) -> u16 {
        self.peripheral.counter()
    }

    pub fn counter_ticks(&self) -> u16 {
        self.counter()
    }

    pub fn period(&self) -> u16 {
        self.peripheral.period()
    }

    pub fn set_period(&mut self, period: u16) {
        self.peripheral.set_period(period);
    }

    pub fn set_auto_reload_raw(&mut self, auto_reload: u16) {
        self.set_period(auto_reload);
    }

    pub fn set_period_ticks(&mut self, ticks: u32) -> Result<(), Error> {
        if ticks == 0 {
            return Err(Error::ZeroPeriod);
        }
        if ticks > MAX_COUNTER_TICKS as u32 {
            return Err(Error::PeriodTooLong);
        }
        self.set_period((ticks - 1) as u16);
        Ok(())
    }

    pub fn set_duration(&mut self, duration: MicrosDurationU32) -> Result<Period, Error> {
        if duration.as_ticks() == 0 {
            return Err(Error::ZeroPeriod);
        }

        let numerator = self.clock_frequency.0 as u128 * duration.as_ticks() as u128;
        let source_ticks = rounded_div(numerator, 1_000_000);
        if source_ticks == 0 {
            return Err(Error::ZeroPeriod);
        }
        if source_ticks > u64::MAX as u128 {
            return Err(Error::PeriodTooLong);
        }

        self.apply_source_ticks(source_ticks as u64, Error::PeriodTooLong)
    }

    pub fn set_frequency(&mut self, frequency: HertzU32) -> Result<Period, Error> {
        let frequency = frequency.to_raw();
        if frequency == 0 {
            return Err(Error::FrequencyTooLow);
        }
        if frequency > self.clock_frequency.0 {
            return Err(Error::FrequencyTooHigh);
        }

        let source_ticks = rounded_div(self.clock_frequency.0 as u128, frequency as u128) as u64;
        self.apply_source_ticks(source_ticks, Error::FrequencyTooLow)
    }

    pub fn start_periodic(&mut self, duration: MicrosDurationU32) -> Result<Period, Error> {
        let actual = self.set_duration(duration)?;
        self.start();
        Ok(actual)
    }

    pub fn start_one_shot_duration(
        &mut self,
        duration: MicrosDurationU32,
    ) -> Result<Period, Error> {
        let actual = self.set_duration(duration)?;
        self.start_one_shot();
        Ok(actual)
    }

    pub fn period_elapsed(&self) -> bool {
        self.event_pending(Event::AutoReloadMatch)
    }

    pub fn clear_period_elapsed(&mut self) {
        self.clear_event(Event::AutoReloadMatch);
    }

    pub fn clock_frequency(&self) -> Hertz {
        self.clock_frequency
    }

    pub fn compare(&self) -> u16 {
        self.peripheral.compare()
    }

    pub fn set_compare(&mut self, compare: u16) {
        self.peripheral.set_compare(compare);
    }

    pub fn set_prescaler(&mut self, prescaler: Prescaler) {
        self.peripheral.set_prescaler(prescaler);
    }

    pub fn enable_interrupt(&mut self, event: Event) {
        self.peripheral.enable_interrupt(event, true);
    }

    pub fn disable_interrupt(&mut self, event: Event) {
        self.peripheral.enable_interrupt(event, false);
    }

    pub fn event_pending(&self, event: Event) -> bool {
        self.peripheral.event_pending(event)
    }

    pub fn clear_event(&mut self, event: Event) {
        self.peripheral.clear_event(event);
    }

    pub fn release(self) -> T {
        self.peripheral
    }

    fn apply_source_ticks(&mut self, source_ticks: u64, too_long: Error) -> Result<Period, Error> {
        let (prescaler, divisor, counter_ticks) = select_prescaler(source_ticks).ok_or(too_long)?;
        self.set_prescaler(prescaler);
        self.set_period((counter_ticks - 1) as u16);

        let actual_source_ticks = counter_ticks * divisor;
        let nanos = rounded_div(
            actual_source_ticks as u128 * 1_000_000_000,
            self.clock_frequency.0 as u128,
        );

        Ok(Period {
            prescaler,
            auto_reload: (counter_ticks - 1) as u16,
            actual: NanosDurationU64::from_ticks(nanos as u64),
        })
    }
}

fn rounded_div(numerator: u128, denominator: u128) -> u128 {
    (numerator + denominator / 2) / denominator
}

fn select_prescaler(source_ticks: u64) -> Option<(Prescaler, u64, u64)> {
    const PRESCALERS: [(Prescaler, u64); 8] = [
        (Prescaler::Div1, 1),
        (Prescaler::Div2, 2),
        (Prescaler::Div4, 4),
        (Prescaler::Div8, 8),
        (Prescaler::Div16, 16),
        (Prescaler::Div32, 32),
        (Prescaler::Div64, 64),
        (Prescaler::Div128, 128),
    ];

    PRESCALERS.iter().find_map(|&(prescaler, divisor)| {
        let counter_ticks = ((source_ticks + divisor / 2) / divisor).max(1);
        (counter_ticks <= MAX_COUNTER_TICKS).then_some((prescaler, divisor, counter_ticks))
    })
}

fn clock_is_ready(clock: TimerClock) -> bool {
    let p = unsafe { mik32_pac::Peripherals::steal() };
    let status = p.pm.freq_status().read();

    match clock {
        TimerClock::SysClk | TimerClock::Hclk => true,
        TimerClock::Osc32m => status.mask_osc32m().bit_is_set(),
        TimerClock::Hsi32m => status.mask_hsi32m().bit_is_set(),
        TimerClock::Osc32k => status.mask_osc32k().bit_is_set(),
        TimerClock::Lsi32k => status.mask_lsi32k().bit_is_set(),
    }
}

mod sealed {
    pub trait Sealed {}
}

/// PAC peripherals accepted by [`Timer16`].
pub trait Instance: sealed::Sealed {
    #[doc(hidden)]
    fn enable_clock();
    #[doc(hidden)]
    fn select_clock(clock: TimerClock);
    #[doc(hidden)]
    fn current_clock() -> TimerClock;
    #[doc(hidden)]
    fn configure(&self, config: Config);
    #[doc(hidden)]
    fn start_continuous(&self);
    #[doc(hidden)]
    fn start_one_shot(&self);
    #[doc(hidden)]
    fn stop(&self);
    #[doc(hidden)]
    fn counter(&self) -> u16;
    #[doc(hidden)]
    fn period(&self) -> u16;
    #[doc(hidden)]
    fn set_period(&self, period: u16);
    #[doc(hidden)]
    fn compare(&self) -> u16;
    #[doc(hidden)]
    fn set_compare(&self, compare: u16);
    #[doc(hidden)]
    fn set_prescaler(&self, prescaler: Prescaler);
    #[doc(hidden)]
    fn enable_interrupt(&self, event: Event, enabled: bool);
    #[doc(hidden)]
    fn event_pending(&self, event: Event) -> bool;
    #[doc(hidden)]
    fn clear_event(&self, event: Event);
    #[doc(hidden)]
    fn clear_all_events(&self);
}

macro_rules! impl_instance {
    ($pac:ty, $clock_enable:ident, $clock_mux:ident) => {
        impl sealed::Sealed for $pac {}

        impl Instance for $pac {
            fn enable_clock() {
                let p = unsafe { mik32_pac::Peripherals::steal() };
                p.pm.clk_apb_p_set()
                    .modify(|_, w| w.$clock_enable().enable());
            }

            fn select_clock(clock: TimerClock) {
                let p = unsafe { mik32_pac::Peripherals::steal() };
                p.pm.timer_cfg().modify(|_, w| match clock {
                    TimerClock::SysClk => w.$clock_mux().sys_clk(),
                    TimerClock::Hclk => w.$clock_mux().hclk(),
                    TimerClock::Osc32m => w.$clock_mux().osc32m(),
                    TimerClock::Hsi32m => w.$clock_mux().hsi32m(),
                    TimerClock::Osc32k => w.$clock_mux().osc32k(),
                    TimerClock::Lsi32k => w.$clock_mux().lsi32k(),
                });
            }

            fn current_clock() -> TimerClock {
                let p = unsafe { mik32_pac::Peripherals::steal() };
                match p.pm.timer_cfg().read().$clock_mux().bits() {
                    0 => TimerClock::SysClk,
                    1 => TimerClock::Hclk,
                    2 => TimerClock::Osc32m,
                    3 => TimerClock::Hsi32m,
                    4 => TimerClock::Osc32k,
                    5 => TimerClock::Lsi32k,
                    _ => TimerClock::SysClk,
                }
            }

            fn configure(&self, config: Config) {
                self.cfgr().modify(|_, w| unsafe {
                    w.cksel()
                        .internal()
                        .presc()
                        .bits(config.prescaler as u8)
                        .wave()
                        .bit(matches!(config.output, OutputMode::SetOnce))
                        .wavwpol()
                        .bit(matches!(config.output, OutputMode::Inverted))
                        .preload()
                        .bit(config.preload)
                });
            }

            fn start_continuous(&self) {
                self.cr()
                    .write(|w| w.enable().set_bit().cntstrt().set_bit());
            }

            fn start_one_shot(&self) {
                self.cr()
                    .write(|w| w.enable().set_bit().sngstrt().set_bit());
            }

            fn stop(&self) {
                self.cr().write(|w| w.enable().clear_bit());
            }

            fn counter(&self) -> u16 {
                self.cnt().read().cnt().bits()
            }
            fn period(&self) -> u16 {
                self.arr().read().arr().bits()
            }
            fn compare(&self) -> u16 {
                self.cmp().read().cmp().bits()
            }

            fn set_period(&self, period: u16) {
                self.arr().write(|w| unsafe { w.arr().bits(period) });
            }

            fn set_compare(&self, compare: u16) {
                self.cmp().write(|w| unsafe { w.cmp().bits(compare) });
            }

            fn set_prescaler(&self, prescaler: Prescaler) {
                self.cfgr()
                    .modify(|_, w| unsafe { w.presc().bits(prescaler as u8) });
            }

            fn enable_interrupt(&self, event: Event, enabled: bool) {
                self.ier().modify(|_, w| match event {
                    Event::CompareMatch => w.cmpmie().bit(enabled),
                    Event::AutoReloadMatch => w.arrmie().bit(enabled),
                    Event::ExternalTrigger => w.exttrigie().bit(enabled),
                    Event::AutoReloadUpdateDone => w.arrokie().bit(enabled),
                    Event::CountUp => w.upie().bit(enabled),
                    Event::CountDown => w.downie().bit(enabled),
                });
            }

            fn event_pending(&self, event: Event) -> bool {
                let status = self.isr().read();
                match event {
                    Event::CompareMatch => status.cmpm().bit_is_set(),
                    Event::AutoReloadMatch => status.arrm().bit_is_set(),
                    Event::ExternalTrigger => status.exttrig().bit_is_set(),
                    Event::AutoReloadUpdateDone => status.arrok().bit_is_set(),
                    Event::CountUp => status.up().bit_is_set(),
                    Event::CountDown => status.down().bit_is_set(),
                }
            }

            fn clear_event(&self, event: Event) {
                self.icr().write(|w| match event {
                    Event::CompareMatch => w.cmpmcf().bit(true),
                    Event::AutoReloadMatch => w.arrmcf().bit(true),
                    Event::ExternalTrigger => w.exttrigcf().bit(true),
                    Event::AutoReloadUpdateDone => w.arrrocf().bit(true),
                    Event::CountUp => w.upcf().bit(true),
                    Event::CountDown => w.downcf().bit(true),
                });
            }

            fn clear_all_events(&self) {
                self.icr().write(|w| {
                    w.cmpmcf()
                        .bit(true)
                        .arrmcf()
                        .bit(true)
                        .exttrigcf()
                        .bit(true)
                        .arrrocf()
                        .bit(true)
                        .upcf()
                        .bit(true)
                        .downcf()
                        .bit(true)
                });
            }
        }
    };
}

impl_instance!(Timer16_0, timer16_0, mux_tim16_0);
impl_instance!(Timer16_1, timer16_1, mux_tim16_1);
impl_instance!(Timer16_2, timer16_2, mux_tim16_2);
