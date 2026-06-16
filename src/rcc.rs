/// Configuration and initialization module for the MIK32 microcontroller's clock and power management system.
///
/// This module provides structures and functions to configure various clock sources and their dividers:
/// - HSI32M: 32 MHz internal oscillator
/// - OSC32M: 32 MHz external oscillator
/// - LSI32K: 32 kHz internal oscillator
/// - OSC32K: 32 kHz external oscillator
///
/// # Structures
///
/// - [`FreqMonitor`]: Controls frequency monitoring settings including clock source selection,
///   automatic frequency switching, and reference clock selection.
/// - [`Config`]: Main configuration structure for clock initialization with support for:
///   - Individual clock source enable/disable
///   - Calibration values for internal oscillators
///   - AHB and APB bus clock dividers
///   - RTC clock source selection
///
/// # Usage
///
/// Create a `Config` instance, customize as needed, and call `Config::init()`:
///
/// ```ignore
/// let config = Config::default();
/// Config::init(config);
/// let sys_clock = system_clock();
/// ```
///
/// # Clock Frequency Calculation
///
/// - FAHB = FSYS / (DIV_AHB + 1)
/// - FAPM_M = FAPB / (DIV_APM_M + 1)
/// - FAPM_P = FAPB / (DIV_APM_P + 1)
use crate::clock::Hertz;

use mik32_pac::pm::ahb_mux::{AhbClkMux, ForceMux};
use mik32_pac::pm::cpu_rtc_clk_mux::CpuRtcClkMux;
use mik32_pac::wake_up::clocks_bu::RtcClkMux;
use mik32_pac::wake_up::clocks_sys::Force32kClk;
use mik32_pac::{Peripherals, Pm, WakeUp};

pub const HSI32M_FREQ: Hertz = Hertz(32_000_000); // Частота от внутреннего источника тактирования (32 МГц)
pub const OSC32M_FREQ: Hertz = Hertz(32_000_000); // Частота от внешнего источника тактирования (32 МГц)
pub const LSI32K_FREQ: Hertz = Hertz(32_768); // Частота от внутреннего источника тактирования (32 кГц)
pub const OSC32K_FREQ: Hertz = Hertz(32_768); // Частота от внешнего источника тактирования (32 кГц)

#[derive(Debug, Clone, Copy)]
pub struct Clocks {
    ahb: Hertz,
    ahb_div: u32,
}

impl Clocks {
    pub fn new(ahb: Hertz, ahb_div: u32) -> Self {
        Self { ahb, ahb_div }
    }

    pub fn ahbclk(&self) -> Hertz {
        self.ahb
    }

    pub fn ahb_div_clk(&self) -> u32 {
        self.ahb_div
    }
}

pub struct FreqMonitor {
    pub sys: AhbClkMux,            // Выбор источника тактирования
    pub force_osc_sys: ForceMux,   // Автоматическая смена частоты
    pub force32k_clk: Force32kClk, // Принудительное переключение на опорный источник для монитора частоты
}

impl Default for FreqMonitor {
    fn default() -> Self {
        Self {
            sys: AhbClkMux::Osc32m,
            force_osc_sys: ForceMux::Unfixed,
            force32k_clk: Force32kClk::Automatic,
        }
    }
}

pub struct RCC {
    pub hsi32m: bool,              // Внутренний генератор 32 МГц
    pub osc32m: bool,              // Внешний генератор 32 МГц
    pub lsi32k: bool,              // Внутренний генератор 32 кГц
    pub osc32k: bool,              // Внешний генератор 32 кГц
    pub freq_monitor: FreqMonitor, // Монитор частоты

    // Задает значение делителя шины AHB.
    // Частота шины AHB (FAHB) рассчитывается, как FSYS/( DIV_AHB+1)ы
    pub ahb_div: u8,

    // Задает значение делителя шины APB_M.
    // Частота шины APB_M (FAPM_M) рассчитывается, как FAPB/( Div_APM_M+1)
    pub apb_m_div: u8,

    // Задает значение делителя шины APB_P.
    // Частота шины APB_P (FAPM_P) рассчитывается, как FAPB/( Div_APM_P+1)
    pub apb_p_div: u8,

    pub hsi32m_calibration_value: u8,
    pub lsi32k_calibration_value: u8,

    // Выбор приоритетного источника тактирования часов реального времени:
    // 0x0 – автоматический выбор. При наличии обоих источников
    // 32кГц выбирается внутренний LSI32K
    // nValue on reset: 0
    pub rtcclk: RtcClkMux,
    pub rtccpuclk: CpuRtcClkMux,

    pub clocks: Clocks,
}

impl Default for RCC {
    fn default() -> Self {
        Self {
            hsi32m: true, // Включим внутренний генератор 32 МГц
            osc32m: true, // Включим внешний генератор 32 МГц
            lsi32k: true, // Включим внутренний генератор 32 кГц
            osc32k: true, // Включим внешний генератор 32 кГц
            freq_monitor: FreqMonitor::default(),
            ahb_div: 0,
            apb_m_div: 0,
            apb_p_div: 0,
            hsi32m_calibration_value: 128,
            lsi32k_calibration_value: 8,
            rtcclk: RtcClkMux::Automatic,
            rtccpuclk: CpuRtcClkMux::Osc32k,
            clocks: Clocks::new(OSC32M_FREQ, 0), // TODO: брать частоту из конфигурации тактирования
        }
    }
}

impl RCC {
    pub fn init(config: &RCC) {
        let wu = unsafe { WakeUp::steal() };
        let pm = unsafe { Pm::steal() };

        wu.clocks_sys().modify(|_, w| {
            w
                // Включим внутренний генератор 32 МГц
                .hsi32m_en()
                .enable()
                // Включим внешний генератор 32 МГц
                .osc32m_en()
                .enable()
        });

        wu.clocks_bu().modify(|_, w| {
            w
                // Включим внутренний генератор 32 кГц
                .lsi32k_en()
                .enable()
                // Включим внутренний генератор 32 кГц
                .osc32k_en()
                .enable()
        });

        // Устанавливаем поправочные коэффициенты внутреннего генератора 32 МГц (HSI32M)
        wu.clocks_sys()
            .modify(|_, w| unsafe { w.adj_hsi32m().bits(config.hsi32m_calibration_value) });

        // Устанавливаем поправочные коэффициенты внутреннего генератора 32 кГц
        wu.clocks_bu()
            .modify(|_, w| unsafe { w.adj_lsi32k().bits(config.lsi32k_calibration_value) });

        // Установим принудительное переключение на опорный источник для монитора частоты
        wu.clocks_sys()
            .modify(|_, w| match config.freq_monitor.force32k_clk {
                Force32kClk::Automatic => w.force_32k_clk().automatic(),
                Force32kClk::Lsi32k => w.force_32k_clk().lsi32k(),
                Force32kClk::Osc32k => w.force_32k_clk().osc32k(),
            });

        // Разрешаем или нет автоматическую смену тактирования
        pm.ahb_mux()
            .modify(|_, w| match config.freq_monitor.force_osc_sys {
                ForceMux::Unfixed => w.force_mux().unfixed(),
                ForceMux::Fixed => w.force_mux().fixed(),
            });

        // Выбор источника тактирования
        pm.ahb_mux().modify(|_, w| match config.freq_monitor.sys {
            AhbClkMux::Osc32m => w.ahb_clk_mux().osc32m(),
            AhbClkMux::Hsi32m => w.ahb_clk_mux().hsi32m(),
            AhbClkMux::Osc32k => w.ahb_clk_mux().osc32k(),
            AhbClkMux::Lsi32k => w.ahb_clk_mux().lsi32k(),
        });

        // Зададим значение делителя шины AHB
        pm.div_ahb()
            .modify(|_, w| unsafe { w.bits(config.ahb_div as u32) });

        // Зададим значение делителя шины APB
        pm.div_apb_m()
            .modify(|_, w| unsafe { w.bits(config.apb_m_div as u32) });

        // Установим регистр управления тактированием батарейного домена
        wu.clocks_bu().modify(|_, w| match config.rtcclk {
            RtcClkMux::Automatic => w.rtc_clk_mux().automatic(),
            RtcClkMux::Lsi32k => w.rtc_clk_mux().lsi32k(),
            RtcClkMux::Osc32k => w.rtc_clk_mux().osc32k(),
        });
        wu.rtc_control().reset();

        // Выбор источника тактирования RTC для системного таймера в составе ядра
        pm.cpu_rtc_clk_mux().modify(|_, w| match config.rtccpuclk {
            CpuRtcClkMux::Osc32k => w.cpu_rtc_clk_mux().osc32k(),
            CpuRtcClkMux::Lsi32k => w.cpu_rtc_clk_mux().osc32k(),
        });

        // Отключим внешний источник тактирования на 32 МГц
        if !config.osc32m {
            wu.clocks_sys().modify(|_, w| w.osc32m_en().disable());
        }

        // Отключим внутренний источник тактирования на 32 МГц
        if !config.hsi32m {
            wu.clocks_sys().modify(|_, w| w.hsi32m_en().disable());
        }

        // Отключим внешний источник тактирования на 32 кГц
        if !config.osc32k {
            wu.clocks_bu().modify(|_, w| w.osc32k_en().disable());
        }

        // Отключим внутренний источник тактирования на 32 кГц
        if !config.lsi32k {
            wu.clocks_bu().modify(|_, w| w.lsi32k_en().disable());
        }
    }
}

/// Returns the current system clock frequency in Hertz.
///
/// This function queries the active clock source selected in the Power Manager (PM)
/// AHB multiplexer and returns the corresponding frequency.
///
/// # Clock Sources
/// The MIK32 microcontroller supports four clock sources:
/// - **OSC32M** (32 MHz external oscillator): 32,000,000 Hz
/// - **OSC32K** (32.768 kHz external oscillator): 32,768 Hz
/// - **HSI32M** (32 MHz internal oscillator): 32,000,000 Hz
/// - **LSI32K** (32.768 kHz internal oscillator): 32,768 Hz (fallback default)
///
/// # Returns
/// The frequency of the currently active clock source as a [`Hertz`] value.
///
/// # Safety Note
/// This function uses `Peripherals::steal()` to access the PM peripheral, which is unsafe
/// because it bypasses Rust's borrow checker. Multiple concurrent calls are theoretically
/// possible, but the read operation is atomic so this is safe in practice for read-only access.
///
/// # See Also
/// - [`Config::init()`] for setting the clock source
pub fn system_clock() -> Hertz {
    let p = unsafe { Peripherals::steal() };

    if p.pm.ahb_mux().read().ahb_clk_mux().is_osc32m() {
        OSC32M_FREQ
    } else if p.pm.ahb_mux().read().ahb_clk_mux().is_osc32k() {
        OSC32K_FREQ
    } else if p.pm.ahb_mux().read().ahb_clk_mux().is_hsi32m() {
        HSI32M_FREQ
    } else {
        LSI32K_FREQ
    }
}
