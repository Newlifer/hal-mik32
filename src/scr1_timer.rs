//! HAL for the SCR1 64-bit system timer.

use fugit::{HertzU32, NanosDurationU64};
use mik32_pac::Scr1Timer as Scr1TimerPeripheral;

/// Clock feeding the system timer's divider.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ClockSource {
    /// HCLK, the default source after reset.
    #[default]
    Hclk,
    /// Clock selected by `PM.CPU_RTC_CLK_MUX`.
    Rtc,
}

/// The divider register has only ten bits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    DividerTooLarge,
    ZeroDuration,
    ZeroClockFrequency,
    DurationTooLong,
}

/// Result of scheduling a duration-based comparison.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CompareSchedule {
    /// Number of system timer increments, rounded up to avoid an early match.
    pub ticks: u64,
    /// Value written to `MTIMECMP` and `MTIMECMPH`.
    pub threshold: u64,
}

/// Owns the SCR1 system timer peripheral.
pub struct Scr1Timer {
    peripheral: Scr1TimerPeripheral,
}

impl Scr1Timer {
    /// Takes ownership without changing the running timer or its compare value.
    pub fn new(peripheral: Scr1TimerPeripheral) -> Self {
        Self { peripheral }
    }

    pub fn start(&mut self) {
        self.peripheral
            .timer_ctrl()
            .modify(|_, w| w.enable().enable());
    }

    pub fn stop(&mut self) {
        self.peripheral
            .timer_ctrl()
            .modify(|_, w| w.enable().disable());
    }

    pub fn is_running(&self) -> bool {
        self.peripheral.timer_ctrl().read().enable().is_enable()
    }

    pub fn clock_source(&self) -> ClockSource {
        if self.peripheral.timer_ctrl().read().clksrc().is_hclk() {
            ClockSource::Hclk
        } else {
            ClockSource::Rtc
        }
    }

    /// Selects the input clock. Configure `PM.CPU_RTC_CLK_MUX` separately for RTC.
    pub fn set_clock_source(&mut self, source: ClockSource) {
        self.peripheral.timer_ctrl().modify(|_, w| match source {
            ClockSource::Hclk => w.clksrc().hclk(),
            ClockSource::Rtc => w.clksrc().external_clk(),
        });
    }

    /// Divider value; the counter increments once every `divider + 1` input clocks.
    pub fn divider(&self) -> u16 {
        self.peripheral.timer_div().read().div().bits()
    }

    pub fn set_divider(&mut self, divider: u16) -> Result<(), Error> {
        if divider > 0x03ff {
            return Err(Error::DividerTooLarge);
        }
        self.peripheral
            .timer_div()
            .write(|w| unsafe { w.div().bits(divider) });
        Ok(())
    }

    /// Reads a consistent 64-bit counter even if the low word wraps during the read.
    pub fn counter(&self) -> u64 {
        loop {
            let high = self.peripheral.mtimeh().read().bits();
            let low = self.peripheral.mtime().read().bits();
            if high == self.peripheral.mtimeh().read().bits() {
                return (u64::from(high) << 32) | u64::from(low);
            }
        }
    }

    /// Changes the counter while the timer is stopped.
    pub fn set_counter(&mut self, value: u64) {
        self.stop();
        self.peripheral
            .mtime()
            .write(|w| unsafe { w.bits(value as u32) });
        self.peripheral
            .mtimeh()
            .write(|w| unsafe { w.bits((value >> 32) as u32) });
    }

    /// Reads the programmed comparison value.
    pub fn compare(&self) -> u64 {
        (u64::from(self.peripheral.mtimecmph().read().bits()) << 32)
            | u64::from(self.peripheral.mtimecmp().read().bits())
    }

    /// Programs the machine timer interrupt threshold.
    ///
    /// The temporary maximum low word prevents an early match while the two
    /// 32-bit halves are being changed. Synchronize callers if an interrupt
    /// handler can also write the comparison registers.
    pub fn set_compare(&mut self, value: u64) {
        self.peripheral
            .mtimecmp()
            .write(|w| unsafe { w.bits(u32::MAX) });
        self.peripheral
            .mtimecmph()
            .write(|w| unsafe { w.bits((value >> 32) as u32) });
        self.peripheral
            .mtimecmp()
            .write(|w| unsafe { w.bits(value as u32) });
    }

    /// Schedules a compare `ticks` counter increments after the current value.
    /// Returns the programmed threshold.
    pub fn set_compare_after(&mut self, ticks: u64) -> u64 {
        let threshold = self.counter().wrapping_add(ticks);
        self.set_compare(threshold);
        threshold
    }

    /// Schedules a comparison after a `fugit` duration.
    ///
    /// `input_frequency` is the clock feeding the divider, before `divider + 1`.
    /// Pass the current HCLK or the clock selected by `PM.CPU_RTC_CLK_MUX`.
    /// The duration is rounded up to a whole timer tick. This conversion assumes
    /// the input clock and divider remain unchanged until the comparison.
    pub fn set_compare_after_duration(
        &mut self,
        duration: NanosDurationU64,
        input_frequency: HertzU32,
    ) -> Result<CompareSchedule, Error> {
        let ticks = duration_to_ticks(
            duration.as_ticks(),
            input_frequency.to_raw(),
            self.divider(),
        )?;
        let threshold = self.set_compare_after(ticks);
        Ok(CompareSchedule { ticks, threshold })
    }

    pub fn release(self) -> Scr1TimerPeripheral {
        self.peripheral
    }
}

fn duration_to_ticks(nanos: u64, input_hz: u32, divider: u16) -> Result<u64, Error> {
    if nanos == 0 {
        return Err(Error::ZeroDuration);
    }
    if input_hz == 0 {
        return Err(Error::ZeroClockFrequency);
    }

    let numerator = u128::from(nanos) * u128::from(input_hz);
    let denominator = 1_000_000_000_u128 * (u128::from(divider) + 1);
    let ticks = numerator.div_ceil(denominator);
    u64::try_from(ticks).map_err(|_| Error::DurationTooLong)
}

#[cfg(test)]
mod tests {
    use super::{Error, duration_to_ticks};

    #[test]
    fn converts_duration_using_input_clock_and_divider() {
        assert_eq!(duration_to_ticks(1_000_000, 32_000_000, 31), Ok(1_000));
        assert_eq!(duration_to_ticks(1_001, 1_000_000, 0), Ok(2));
        assert_eq!(duration_to_ticks(1, 32_000, 0), Ok(1));
    }

    #[test]
    fn rejects_invalid_or_unrepresentable_duration() {
        assert_eq!(duration_to_ticks(0, 1, 0), Err(Error::ZeroDuration));
        assert_eq!(duration_to_ticks(1, 0, 0), Err(Error::ZeroClockFrequency));
        assert_eq!(
            duration_to_ticks(u64::MAX, u32::MAX, 0),
            Err(Error::DurationTooLong)
        );
    }
}
