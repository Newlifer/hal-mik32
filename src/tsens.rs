//! Temperature sensor (TSENS) HAL stub

/// Конфигурация встроенного термодатчика
///
use mik32_pac::Tsens;

use crate::rcc::{Clocks, HSI32M_FREQ, LSI32K_FREQ, OSC32K_FREQ, OSC32M_FREQ};

const TSENS_OPTIMAL_FREQUENCY: u32 = 40000;
///! Рекомендуемая частота работы термосенсора.

/// Источник частоты встроенного датчика температуры
///
/// # Variants
///
/// - `SycClock = 0x0` - TODO:
/// - `HLCL = 0x1` - TODO:
/// - `OSC32M = 0x2` - внешний осциллятор 32МГц
/// - `HSI32M = 0x3` - внутренний осциллятор 32МГц
/// - `OSC32K = 0x4` - внешний осциллятор 32кГц
/// - `LSI32K = 0x5` - внутренний осциллятор 32кГц
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ClockSource {
    SycClock = 0x0,
    HLCL = 0x1,
    OSC32M = 0x2,
    HSI32M = 0x3,
    OSC32K = 0x4,
    LSI32K = 0x5,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    pub source: ClockSource,
    pub frequency: u32,
}

impl Config {
    pub const fn default() -> Self {
        Self {
            source: ClockSource::SycClock,
            frequency: TSENS_OPTIMAL_FREQUENCY,
        }
    }

    ///  Выбор источника тактирования термодатчика
    ///
    /// # Arguments
    ///
    /// - `source` (`ClockSource`) - источник
    ///
    /// # Returns
    ///
    /// - `Self` - объект конфигурации
    ///
    /// # Examples
    ///
    /// ```
    /// let _ = clock_from_source(ClockSource::OSC32K);
    /// ```
    pub fn clock_from_source(mut self, source: ClockSource) -> Self {
        self.source = source;
        self
    }

    /// Установка частоты встроенного термодатчика
    ///
    /// # Arguments
    ///
    /// - `frequency` (`u32`) - частота
    ///
    /// # Returns
    ///
    /// - `Self` - объект конфигурации
    ///
    /// # Examples
    ///
    /// ```
    /// let _ = with_frequency(40_000u32);
    /// ```
    pub fn with_frequency(mut self, frequency: u32) -> Self {
        self.frequency = frequency;
        self
    }
}

/// Ошибки TSENS
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Таймаут ожидания окончания преобразования
    Timeout,
    /// Порог (threshold) выходит за допустимый диапазон
    ThresholdOutOfRange,
    /// Частота, установленная для TSENS, выходит за допустимый диапазон
    FrequencyOutOfRange,
    /// Вычисленный делитель ниже нужного или 0
    DividerUnderflow,
    /// Вычисленный делитель выше максимального значения (1024)
    DividerOverflow,
    /// Другие ошибки аппаратного уровня
    Hardware,
}

/// Встроенный термодатчи
///
/// # Fields
///
/// - `dp` (`Tsens`) - объект переферии из PAC
/// - `config` (`Config`) - конфигурация встроенного термодатчика
///
/// # Examples
///
/// ```
/// use crate::tsens::(TSENS, config::{Config, ClockSource});
/// use mik32_pac::Peripherals;
///
/// let p = Peripherals::take().unwrap();
/// let t_sensor = TSENS {
///     dp: p.tsens,
///     config: Config {
///         source: ClockSource::OSC32K,
///         frequency: 3000,
///     },
/// };
///
/// t_sensor.start_continuous();
/// let temperature: u32 = t_sensor.get_temperature();
/// ```
pub struct TSENS {
    dp: Tsens,
    config: Config,
}

impl TSENS {
    /// Конструктор объекта встренного термодатчика
    ///
    /// # Arguments
    ///
    /// - `dp` (`Tsens`) - объект переферии из PAC
    /// - `clocks` (`&Clocks`) - объект тактирования RCC
    /// - `config` (`Config`) - конфигурация термодатчика
    ///
    /// # Returns
    ///
    /// - `Result<Self, Error>` - построенный объект термодатчика или ошибка
    ///
    /// # Errors
    ///
    /// - делитель не находится не в допустимом промежутке: [1, 1023]
    pub fn new(dp: Tsens, clocks: &Clocks, config: Config) -> Result<Self, Error> {
        let result = Self {
            dp: dp,
            config: config,
        };

        // Подбор делителя для заданной частоты термосенсора
        let real_clock = Self::get_real_clocks(&result.config, clocks);
        let divider = Self::calc_divider(real_clock, &result.config, clocks)?;

        // Установка источника тактирования термосенсора
        result
            .dp
            .tsens_cfg()
            .modify(|_, w| match result.config.source {
                ClockSource::SycClock => w.clk_mux().sys_clk(),
                ClockSource::HLCL => w.clk_mux().hclk(),
                ClockSource::OSC32M => w.clk_mux().osc32m(),
                ClockSource::HSI32M => w.clk_mux().hsi32m(),
                ClockSource::OSC32K => w.clk_mux().osc32k(),
                ClockSource::LSI32K => w.clk_mux().lsi32k(),
            });

        result.dp.tsens_cfg().modify(|_, w| unsafe {
            w.nrst()
                .set_bit() // уберём сброс
                .npd_clk()
                .set_bit() // включим тактирование
                .npd()
                .set_bit() // включим датчик
                .div()
                .bits(divider as u16)
        });
        Ok(result)
    }

    /// Выполняет одиночное измерение температуры и возвращает результат в градусах Цельсия
    ///
    /// Функция выводит TSENS из сброса, запускает одиночное преобразование и
    /// ожидает установку флага `EOC` (End Of Conversion) в регистре `TSENS_VALUE`.
    ///
    /// # Параметры
    /// - `timeout`: Максимальное число итераций ожидания `EOC`.
    ///   Если `None`, используется значение по умолчанию `100_000`.
    ///
    /// # Возвращает
    /// - `Ok(temp_c)`: Измеренная температура в градусах Цельсия.
    /// - `Err(Error::Timeout)`: Если `EOC` не установился до исчерпания `timeout`.
    ///
    /// # Примечания
    /// - Таймаут задаётся в “тиках” цикла ожидания (не в микросекундах).
    /// - Значение температуры вычисляется через `value_to_celsius()` из сырого
    ///   значения регистра `TSENS_VALUE`.
    pub fn single_measurement(&mut self, timeout: Option<u32>) -> Result<u32, Error> {
        let mut timeout_counter: u32 = timeout.unwrap_or(100_000);

        self.dp.tsens_cfg().modify(|_, w| w.nrst().set_bit());
        self.dp.tsens_single().write(|w| w.single().set_bit());

        while !self.dp.tsens_value().read().eoc().bit_is_set() {
            timeout_counter = timeout_counter.checked_sub(1).ok_or(Error::Timeout)?;
        }

        Ok(value_to_celsius(
            self.dp.tsens_value().read().bits() & (0x3FF << 0),
        ))
    }

    /// Запускает непрерываное измерение температуры с прерываниями
    ///
    /// # Returns
    ///
    /// - `Self` - экземпляр TSENS с запущенным непрерывным режимом измерения
    pub fn start_continuous_interrupt(&mut self) {
        self.dp.tsens_clear_irq().write(|w| unsafe {
            w.bits(0b111) // Очистим все флаги прерываний
        });

        self.dp.tsens_irq().write(|w| unsafe {
            w.bits(0b111) // Включим все маски прерываний
        });

        self.dp.tsens_cfg().modify(|_, w| {
            w.nrst().clear_bit() // уберём сброс
        });

        self.dp.tsens_continuous().write(|w| {
            w.continuous().set_bit() // Запустим непрерывный режим
        });
    }

    /// Запускает однократное измерение с прерываниями (один замер, затем остановка)
    ///
    /// # Returns
    ///
    /// - `Self` - экземпляр TSENS с запущенным однократным режимом измерения
    pub fn start_single_interrupt(&mut self) {
        self.dp.tsens_clear_irq().write(|w| unsafe {
            w.bits(0b111) // Очистим все флаги прерываний
        });

        self.dp.tsens_irq().write(|w| unsafe {
            w.bits(0b111) // Включим все маски прерываний
        });

        self.dp.tsens_cfg().modify(|_, w| w.nrst().set_bit());

        self.dp.tsens_single().write(|w| {
            w.single().set_bit() // Запустим однократный режим
        });
    }

    /// Останавливает прерывания
    ///
    /// # Arguments
    ///
    /// # Returns
    ///
    /// - `Self` - экземпляр TSENS с остановленными измерениями и отключенными прерываниями
    pub fn stop_interrupt(self) -> Self {
        self.dp.tsens_cfg().modify(|_, w| w.nrst().clear_bit());

        self.dp.tsens_irq().write(|w| unsafe {
            w.bits(0b000) // Отключим все маски прерываний
        });

        self.dp.tsens_clear_irq().write(|w| unsafe {
            w.bits(0b111) // Очистим все флаги прерываний
        });

        self
    }

    /// Установка верхнего порога температуры
    ///
    /// # Arguments
    ///
    /// - `value` (`u32`) - температура в цельсиях
    ///
    /// # Returns
    ///
    /// - `Result<Self, Error>` - результат операции`
    pub fn on_upper_threshold(&mut self, value: u32) -> Result<(), Error> {
        let raw_value = celsius_to_value(value) as u32;

        if (raw_value > 603u32) || (raw_value < 255u32) {
            return Err(Error::ThresholdOutOfRange);
        }

        self.dp.tsens_treshold().modify(|_, w| unsafe {
            w.treshold_hi().bits(raw_value as u16) // Transform value to
        });
        Ok(())
    }

    /// Установка нижнего порога температуры
    ///
    /// # Arguments
    ///
    /// - `value` (`u32`) - температура в цельсиях
    ///
    /// # Returns
    ///
    /// - `Result<Self, Error>` - результат операции`
    pub fn on_lower_threshold(self, value: u32) -> Result<(), Error> {
        let raw_value = celsius_to_value(value) as u32;

        if (raw_value > 603u32) || (raw_value < 255u32) {
            return Err(Error::ThresholdOutOfRange);
        }

        self.dp
            .tsens_treshold()
            .modify(|_, w| unsafe { w.treshold_low().bits(raw_value as u16) });
        Ok(())
    }

    /// Запускает режим непрерывного измерения температуры
    pub fn start_continuous(&mut self) {
        self.dp.tsens_cfg().modify(|_, w| w.nrst().set_bit());
        self.dp.tsens_continuous().write(|w| unsafe { w.bits(1) });
    }

    /// Текущая температура в цельсиях
    ///
    /// Функция используется для получения значения температуры
    /// в непрерывном режиме измерения (Continuous)
    ///
    /// # Returns
    ///
    /// - `u32` - температура в цельсиях
    pub fn get_temperature(&self) -> u32 {
        value_to_celsius(self.dp.tsens_value().read().value().bits() as u32)
    }

    /// Вычисляет делитель для достижения оптимальной частоты работы TSENS (40 кГц)
    ///
    /// # Arguments
    ///
    /// - `value` (`u32`) - желаемая частота работы TSENS в герцах
    /// - `clocks` (`&Clocks`) - структура с текущими частотами тактирования
    ///
    /// # Returns
    ///
    /// - `Result<u32, Error>` - вычисленный делитель или ошибка, если частота вне допустимого диапазона
    fn calc_divider(real_clock: u32, config: &Config, clocks: &Clocks) -> Result<u32, Error> {
        if config.frequency == 0 || config.frequency > 100_000 {
            return Err(Error::FrequencyOutOfRange);
        }

        let mut divider = (real_clock / config.frequency) >> 1;
        if divider == 0 {
            return Err(Error::DividerUnderflow);
        }

        let mut pre_result = real_clock / (divider << 1);
        while (pre_result > 100_000) && (divider <= 0x400) {
            divider += 1;
            if divider > 0x400 {
                return Err(Error::DividerOverflow);
            }
            pre_result = real_clock / (divider << 1);
        }

        divider = divider - 1;

        if divider >= 1024 {
            return Err(Error::DividerOverflow);
        }

        Ok(divider)
    }

    /// Получает реальную частоту тактирования для TSENS на основе выбранного источника и текущих настроек тактирования
    ///
    /// # Arguments
    ///
    /// - `config` (`&Config`) - конфигурация TSENS, содержащая выбранный источник тактирования
    /// - `clocks` (`&Clocks`) - структура с текущими частотами тактирования
    ///
    /// # Returns
    ///
    /// - `u32` - реальная частота тактирования для TSENS в герцах
    fn get_real_clocks(config: &Config, clocks: &Clocks) -> u32 {
        match config.source {
            ClockSource::SycClock => clocks.ahbclk().0,
            ClockSource::HLCL => clocks.ahbclk().0 / (clocks.ahb_div_clk() + 1),
            ClockSource::OSC32M => OSC32M_FREQ.0,
            ClockSource::HSI32M => HSI32M_FREQ.0,
            ClockSource::OSC32K => OSC32K_FREQ.0,
            ClockSource::LSI32K => LSI32K_FREQ.0,
        }
    }
}

/// Конвертирует внутренее значение датчика температуры в цельсиях
///
/// # Arguments
///
/// - `value` (`u32`) - значение датчика температуры
///
/// # Returns
///
/// - `u32` - температура в цельсиях
#[inline(always)]
fn value_to_celsius(value: u32) -> u32 {
    // return (640660 * value) / (40960 + 93 * value) * 10 - 27315;
    return ((640_660u32 * value) / (40_960u32 + 93u32 * value) * 10 - 27_315) as u32;
}

///  Конвертирует температуру в цельсиях во внутренее значение датчика TSENS
///
/// # Arguments
///
/// - `value` (`u32`) - температура в цельсиях
///
/// # Returns
///
/// - `u32` - внутренее значение датчика TSENS
#[inline(always)]
fn celsius_to_value(value: u32) -> u32 {
    return 40960 * 100 / (((6406600 - 93 * (value * 100 + 27315)) * 100) / (value * 100 + 27315));
}
