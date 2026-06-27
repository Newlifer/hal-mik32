//! Управление тактированием и сбросами MIK32.
//!
//! Модуль настраивает источники системной частоты, делители шин AHB/APB,
//! источники RTC и калибровочные значения внутренних генераторов.
//! `RCC::init` применяет конфигурацию и возвращает рассчитанное дерево частот.
//!
//! Расчет частот:
//! - `sysclk` = выбранный источник `AHB_CLK_MUX`
//! - `ahbclk` = `sysclk / (DIV_AHB + 1)`
//! - `apb_m_clk` = `ahbclk / (DIV_APB_M + 1)`
//! - `apb_p_clk` = `ahbclk / (DIV_APB_P + 1)`

use crate::clock::Hertz;

use mik32_pac::pm::ahb_mux::{AhbClkMux, ForceMux};
use mik32_pac::pm::cpu_rtc_clk_mux::CpuRtcClkMux;
use mik32_pac::wake_up::clocks_bu::RtcClkMux;
use mik32_pac::wake_up::clocks_sys::Force32kClk;
use mik32_pac::{Peripherals, Pm, WakeUp};

pub const HSI32M_FREQ: Hertz = Hertz(32_000_000);
pub const OSC32M_FREQ: Hertz = Hertz(32_000_000);
pub const LSI32K_FREQ: Hertz = Hertz(32_768);
pub const OSC32K_FREQ: Hertz = Hertz(32_768);

/// Рассчитанные частоты после применения RCC-конфигурации.
///
/// Значения в этой структуре используются периферийными драйверами для
/// вычисления делителей, таймингов и baudrate. Хранятся как итоговые частоты
/// шин, так и сырые значения делителей, записанные в регистры.
#[derive(Debug, Clone, Copy)]
pub struct Clocks {
    /// Частота выбранного системного источника до делителя AHB.
    sys: Hertz,
    /// Частота шины AHB после `DIV_AHB`.
    ahb: Hertz,
    /// Частота домена APB_M после `DIV_APB_M`.
    apb_m: Hertz,
    /// Частота домена APB_P после `DIV_APB_P`.
    apb_p: Hertz,
    /// Сырое значение регистра `DIV_AHB`.
    ahb_div: u32,
    /// Сырое значение регистра `DIV_APB_M`.
    apb_m_div: u32,
    /// Сырое значение регистра `DIV_APB_P`.
    apb_p_div: u32,
}

impl Clocks {
    /// Создает дерево частот с заданным системным источником и делителем AHB.
    ///
    /// Делители APB считаются равными нулю, то есть APB_M/APB_P работают на
    /// частоте AHB.
    pub const fn new(sys: Hertz, ahb_div: u32) -> Self {
        Self::from_config(sys, ahb_div, 0, 0)
    }

    /// Создает дерево частот из системной частоты и сырых значений делителей.
    pub const fn from_config(sys: Hertz, ahb_div: u32, apb_m_div: u32, apb_p_div: u32) -> Self {
        let ahb = Hertz(sys.0 / (ahb_div + 1));
        Self {
            sys,
            ahb,
            apb_m: Hertz(ahb.0 / (apb_m_div + 1)),
            apb_p: Hertz(ahb.0 / (apb_p_div + 1)),
            ahb_div,
            apb_m_div,
            apb_p_div,
        }
    }

    /// Частота выбранного системного источника до делителя AHB.
    pub fn sysclk(&self) -> Hertz {
        self.sys
    }

    /// Частота шины AHB.
    pub fn ahbclk(&self) -> Hertz {
        self.ahb
    }

    /// Частота домена APB_M.
    pub fn apb_m_clk(&self) -> Hertz {
        self.apb_m
    }

    /// Частота домена APB_P.
    pub fn apb_p_clk(&self) -> Hertz {
        self.apb_p
    }

    /// Сырое значение делителя AHB.
    pub fn ahb_div_clk(&self) -> u32 {
        self.ahb_div
    }

    /// Сырое значение делителя APB_M.
    pub fn apb_m_div_clk(&self) -> u32 {
        self.apb_m_div
    }

    /// Сырое значение делителя APB_P.
    pub fn apb_p_div_clk(&self) -> u32 {
        self.apb_p_div
    }
}

/// Логическое имя источника тактирования для ошибок конфигурации.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockSource {
    /// Внутренний 32 МГц генератор.
    Hsi32m,
    /// Внешний 32 МГц генератор.
    Osc32m,
    /// Внутренний 32 кГц генератор.
    Lsi32k,
    /// Внешний 32 кГц генератор.
    Osc32k,
}

/// Ошибки RCC-конфигурации.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Конфигурация выбирает источник, который затем должен быть выключен.
    DisabledClockSelected(ClockSource),
    /// Для автоматического или явного выбора 32 кГц источника не включен ни
    /// один 32 кГц генератор.
    No32kClockEnabled,
}

/// Конфигурация монитора частоты и выбора системного источника.
pub struct FreqMonitor {
    /// Источник системной частоты, записываемый в `PM.AHB_MUX.AHB_CLK_MUX`.
    pub sys: AhbClkMux,
    /// Разрешение автоматического переключения при пропадании выбранного
    /// источника тактирования.
    pub force_osc_sys: ForceMux,
    /// Опорный 32 кГц источник для монитора частоты.
    pub force32k_clk: Force32kClk,
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

/// Конфигурация RCC.
///
/// Поля `hsi32m`, `osc32m`, `lsi32k`, `osc32k` задают, останется ли
/// соответствующий генератор включенным после инициализации. `RCC::init`
/// сначала включает все генераторы, нужные для безопасного переключения, а в
/// конце отключает те, которые пользователь пометил как `false`.
pub struct RCC {
    /// Оставить включенным внутренний 32 МГц генератор.
    pub hsi32m: bool,
    /// Оставить включенным внешний 32 МГц генератор.
    pub osc32m: bool,
    /// Оставить включенным внутренний 32 кГц генератор.
    pub lsi32k: bool,
    /// Оставить включенным внешний 32 кГц генератор.
    pub osc32k: bool,
    /// Выбор системного источника и поведение монитора частоты.
    pub freq_monitor: FreqMonitor,
    /// Сырое значение `DIV_AHB`: частота AHB = `sysclk / (ahb_div + 1)`.
    pub ahb_div: u8,
    /// Сырое значение `DIV_APB_M`: частота APB_M = `ahbclk / (apb_m_div + 1)`.
    pub apb_m_div: u8,
    /// Сырое значение `DIV_APB_P`: частота APB_P = `ahbclk / (apb_p_div + 1)`.
    pub apb_p_div: u8,
    /// Калибровочное значение внутреннего 32 МГц генератора.
    pub hsi32m_calibration_value: u8,
    /// Калибровочное значение внутреннего 32 кГц генератора.
    pub lsi32k_calibration_value: u8,
    /// Приоритетный источник RTC в backup-домене.
    pub rtcclk: RtcClkMux,
    /// Источник RTC для системного таймера ядра.
    pub rtccpuclk: CpuRtcClkMux,
    /// Расчетные частоты по умолчанию. Для актуальных частот используйте
    /// результат `RCC::init` или функцию [`clocks`].
    pub clocks: Clocks,
}

impl Default for RCC {
    fn default() -> Self {
        Self {
            hsi32m: true,
            osc32m: true,
            lsi32k: true,
            osc32k: true,
            freq_monitor: FreqMonitor::default(),
            ahb_div: 0,
            apb_m_div: 0,
            apb_p_div: 0,
            hsi32m_calibration_value: 128,
            lsi32k_calibration_value: 8,
            rtcclk: RtcClkMux::Automatic,
            rtccpuclk: CpuRtcClkMux::Osc32k,
            clocks: Clocks::from_config(OSC32M_FREQ, 0, 0, 0),
        }
    }
}

impl RCC {
    /// Применяет RCC-конфигурацию и возвращает рассчитанные частоты шин.
    ///
    /// Метод валидирует выбранные источники до записи регистров, чтобы не
    /// отключить генератор, который используется системной частотой, RTC или
    /// монитором частоты.
    pub fn init(config: &RCC) -> Result<Clocks, Error> {
        Self::validate(config)?;

        let wu = unsafe { WakeUp::steal() };
        let pm = unsafe { Pm::steal() };

        // Включаем генераторы перед переключением mux'ов: это уменьшает шанс
        // оставить систему без активного источника тактирования.
        wu.clocks_sys()
            .modify(|_, w| w.hsi32m_en().enable().osc32m_en().enable());

        wu.clocks_bu()
            .modify(|_, w| w.lsi32k_en().enable().osc32k_en().enable());

        // Применяем калибровку внутренних генераторов до выбора их в качестве
        // рабочего источника.
        wu.clocks_sys()
            .modify(|_, w| unsafe { w.adj_hsi32m().bits(config.hsi32m_calibration_value) });

        wu.clocks_bu()
            .modify(|_, w| unsafe { w.adj_lsi32k().bits(config.lsi32k_calibration_value) });

        // Настраиваем опорный 32 кГц источник монитора частоты.
        wu.clocks_sys()
            .modify(|_, w| match config.freq_monitor.force32k_clk {
                Force32kClk::Automatic => w.force_32k_clk().automatic(),
                Force32kClk::Lsi32k => w.force_32k_clk().lsi32k(),
                Force32kClk::Osc32k => w.force_32k_clk().osc32k(),
            });

        // `FORCE_MUX` управляет автоматическим переключением при потере
        // выбранного источника.
        pm.ahb_mux()
            .modify(|_, w| match config.freq_monitor.force_osc_sys {
                ForceMux::Unfixed => w.force_mux().unfixed(),
                ForceMux::Fixed => w.force_mux().fixed(),
            });

        // Выбор системного источника и делителей шин.
        pm.ahb_mux().modify(|_, w| match config.freq_monitor.sys {
            AhbClkMux::Osc32m => w.ahb_clk_mux().osc32m(),
            AhbClkMux::Hsi32m => w.ahb_clk_mux().hsi32m(),
            AhbClkMux::Osc32k => w.ahb_clk_mux().osc32k(),
            AhbClkMux::Lsi32k => w.ahb_clk_mux().lsi32k(),
        });

        pm.div_ahb()
            .modify(|_, w| unsafe { w.bits(config.ahb_div as u32) });

        pm.div_apb_m()
            .modify(|_, w| unsafe { w.bits(config.apb_m_div as u32) });

        pm.div_apb_p()
            .modify(|_, w| unsafe { w.bits(config.apb_p_div as u32) });

        // RTC в backup-домене и RTC-источник для системного таймера ядра
        // настраиваются отдельными mux'ами.
        wu.clocks_bu().modify(|_, w| match config.rtcclk {
            RtcClkMux::Automatic => w.rtc_clk_mux().automatic(),
            RtcClkMux::Lsi32k => w.rtc_clk_mux().lsi32k(),
            RtcClkMux::Osc32k => w.rtc_clk_mux().osc32k(),
        });
        wu.rtc_control().reset();

        pm.cpu_rtc_clk_mux().modify(|_, w| match config.rtccpuclk {
            CpuRtcClkMux::Osc32k => w.cpu_rtc_clk_mux().osc32k(),
            CpuRtcClkMux::Lsi32k => w.cpu_rtc_clk_mux().lsi32k(),
        });

        // После переключения можно выключить генераторы, которые не нужны
        // пользователю и не участвуют в выбранной конфигурации.
        if !config.osc32m {
            wu.clocks_sys().modify(|_, w| w.osc32m_en().disable());
        }

        if !config.hsi32m {
            wu.clocks_sys().modify(|_, w| w.hsi32m_en().disable());
        }

        if !config.osc32k {
            wu.clocks_bu().modify(|_, w| w.osc32k_en().disable());
        }

        if !config.lsi32k {
            wu.clocks_bu().modify(|_, w| w.lsi32k_en().disable());
        }

        Ok(Self::clocks(config))
    }

    /// Рассчитывает дерево частот из RCC-конфигурации без записи регистров.
    pub fn clocks(config: &RCC) -> Clocks {
        Clocks::from_config(
            Self::sysclk_for(config.freq_monitor.sys),
            config.ahb_div as u32,
            config.apb_m_div as u32,
            config.apb_p_div as u32,
        )
    }

    /// Проверяет, что выбранные источники не будут выключены конфигурацией.
    fn validate(config: &RCC) -> Result<(), Error> {
        Self::ensure_enabled(config, Self::source_for_ahb(config.freq_monitor.sys))?;

        match config.freq_monitor.force32k_clk {
            Force32kClk::Automatic => Self::ensure_any_32k_enabled(config)?,
            Force32kClk::Lsi32k => Self::ensure_enabled(config, ClockSource::Lsi32k)?,
            Force32kClk::Osc32k => Self::ensure_enabled(config, ClockSource::Osc32k)?,
        }

        match config.rtcclk {
            RtcClkMux::Automatic => Self::ensure_any_32k_enabled(config)?,
            RtcClkMux::Lsi32k => Self::ensure_enabled(config, ClockSource::Lsi32k)?,
            RtcClkMux::Osc32k => Self::ensure_enabled(config, ClockSource::Osc32k)?,
        }

        match config.rtccpuclk {
            CpuRtcClkMux::Osc32k => Self::ensure_enabled(config, ClockSource::Osc32k)?,
            CpuRtcClkMux::Lsi32k => Self::ensure_enabled(config, ClockSource::Lsi32k)?,
        }

        Ok(())
    }

    fn ensure_any_32k_enabled(config: &RCC) -> Result<(), Error> {
        if config.lsi32k || config.osc32k {
            Ok(())
        } else {
            Err(Error::No32kClockEnabled)
        }
    }

    fn ensure_enabled(config: &RCC, source: ClockSource) -> Result<(), Error> {
        let enabled = match source {
            ClockSource::Hsi32m => config.hsi32m,
            ClockSource::Osc32m => config.osc32m,
            ClockSource::Lsi32k => config.lsi32k,
            ClockSource::Osc32k => config.osc32k,
        };

        if enabled {
            Ok(())
        } else {
            Err(Error::DisabledClockSelected(source))
        }
    }

    fn source_for_ahb(source: AhbClkMux) -> ClockSource {
        match source {
            AhbClkMux::Osc32m => ClockSource::Osc32m,
            AhbClkMux::Hsi32m => ClockSource::Hsi32m,
            AhbClkMux::Osc32k => ClockSource::Osc32k,
            AhbClkMux::Lsi32k => ClockSource::Lsi32k,
        }
    }

    fn sysclk_for(source: AhbClkMux) -> Hertz {
        match source {
            AhbClkMux::Osc32m => OSC32M_FREQ,
            AhbClkMux::Hsi32m => HSI32M_FREQ,
            AhbClkMux::Osc32k => OSC32K_FREQ,
            AhbClkMux::Lsi32k => LSI32K_FREQ,
        }
    }
}

/// Возвращает частоту текущего системного источника по состоянию регистров.
pub fn system_clock() -> Hertz {
    let p = unsafe { Peripherals::steal() };

    match p.pm.ahb_mux().read().ahb_clk_mux().variant() {
        AhbClkMux::Osc32m => OSC32M_FREQ,
        AhbClkMux::Hsi32m => HSI32M_FREQ,
        AhbClkMux::Osc32k => OSC32K_FREQ,
        AhbClkMux::Lsi32k => LSI32K_FREQ,
    }
}

/// Возвращает текущее дерево частот по состоянию регистров.
pub fn clocks() -> Clocks {
    let p = unsafe { Peripherals::steal() };

    Clocks::from_config(
        system_clock(),
        p.pm.div_ahb().read().bits(),
        p.pm.div_apb_m().read().bits(),
        p.pm.div_apb_p().read().bits(),
    )
}
