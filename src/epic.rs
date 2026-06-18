//! Управление внешним контроллером прерываний EPIC.
//!
//! "Штатный" контроллер прерываний в ядре отключен. Управление
//! статусом (вкл, выкл) осуществляется через регистр mie (см. модуль interrupts).
//!
//! Все прерывания обрабатываются единым обработчиком trap_handler.
//! // TODO: вынести listen для level в отдельный, чтобы можно было писать сразу несколько линий,
//! а не по одной, так как при записи в регистр mask_level_set перезаписывается вся маска целиком
use core::option::Option;
use core::u32;
use mik32_pac::Epic;

/// Тип срабатывания прерывания
///
/// # Variants
///
/// - `Edge` - по фронту
/// - `Level` - по уровню
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Trigger {
    Edge,  // Срабатывание по фронту
    Level, // Срабатывание по уровню
}

/// Линии прерывания
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum InterruptLine {
    Timer32_0 = 1 << 0,         // Таймер 32_0 - линия 0
    USART0 = 1 << 1,            // USART0 - линия 1
    USART1 = 1 << 2,            // USART1 - линия 2
    SPI0 = 1 << 3,              // SPI0 - линия 3
    SPI1 = 1 << 4,              // SPI1 - линия 4
    GPIO = 1 << 5,              // GPIO - линия 5
    I2C0 = 1 << 6,              // I2C0 - линия 6
    I2C1 = 1 << 7,              // I2C1 - линия 7
    WDT = 1 << 8,               // WDT - линия 8
    Timer16_0 = 1 << 9,         // Таймер 16_0 - линия 9
    Timer16_1 = 1 << 10,        // Таймер 16_1 - линия 10
    Timer16_2 = 1 << 11,        // Таймер 16_2 - линия 11
    Timer32_1 = 1 << 12,        // Таймер 32_1 - линия 12
    Timer32_2 = 1 << 13,        // Таймер 32_2 - линия 13
    SPIFI = 1 << 14,            // SPIFI - линия 14
    RTC = 1 << 15,              // RTC - линия 15
    EEPROM = 1 << 16,           // EEPROM - линия 16
    WdtDom3 = 1 << 17,          // WDT домен 3 - линия 17
    WdtSpifi = 1 << 18,         // WDT SPIFI - линия 18
    WdtEeprom = 1 << 19,        // WDT EEPROM - линия 19
    DMA = 1 << 20,              // DMA - линия 20
    FrequencyMonitor = 1 << 21, // Монитор частоты - линия 21
    AVCCOver = 1 << 22,         // AVCC выше порога - линия 22
    AVCCUnder = 1 << 23,        // AVCC ниже порога - линия 23
    VCCOver = 1 << 24,          // VCC выше порога - линия 24
    VCCUnder = 1 << 25,         // VCC ниже порога - линия 25
    LowBattery = 1 << 26,       // Низкий заряд батареи - линия 26
    BrownOut = 1 << 27,         // Brown Out - линия 27
    TSENS = 1 << 28,            // Датчик температуры - линия 28
    ADC = 1 << 29,              // ADC - линия 29
    DAC0 = 1 << 30,             // DAC0 - линия 30
    DAC1 = 1 << 31,             // DAC1 - линия 31
}

#[derive(Debug)]
pub enum Error {
    LineLevelSet,
    LineEdgeSet,
    LineNeverSet,
}

/// Контроллер прерываний
pub struct EPIC {
    dp: Epic,
    line_mask: u32,
}

impl EPIC {
    /// Конструктор
    ///
    /// # Arguments
    ///
    /// - `dp` (`Epic`) - контроллер прерываний из PAC
    ///
    /// # Returns
    ///
    /// - `Self` - экземпляр контроллера
    ///
    /// # Examples
    ///
    /// ```
    /// use crate::epic::EPIC;
    ///
    /// let dp = Peripherals::take().unwrap();
    /// let mut epic = EPIC::new(dp.epic);
    /// ```
    pub fn new(dp: Epic) -> Self {
        Self {
            dp: dp,
            line_mask: 0u32,
        }
    }

    /// Включает линию прерывания
    ///
    /// Если на линию уже было включено прерывание, но другого типа
    /// (включаете "по фронту", а уже было включено "по уровеню"), то вернётся ошибка
    ///
    /// # Arguments
    ///
    /// - `line` (`InterruptLine`) - линия прерывания
    /// - `trigger` (`Trigger`) - тип срабатывания прерывания (по фронту или по уровню)
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - успех включения прерывания
    ///
    /// # Errors
    ///
    /// На линию уже было включено прерывание другого типа
    ///
    /// # Examples
    ///
    /// ```
    /// use crate::epic::EPIC;
    ///
    /// let dp = Peripherals::take().unwrap();
    /// let mut epic = EPIC::new(dp.epic);
    /// epic.listen(InterruptLine::Timer32_1); // Включим прерывание по линии таймера
    /// ```
    pub fn listen(&mut self, line: InterruptLine, trigger: Trigger) -> Result<(), Error> {
        match trigger {
            Trigger::Edge => {
                if self.line_mask & (line as u32) != 0 {
                    return Err(Error::LineLevelSet);
                }

                self.dp
                    .mask_edge_set()
                    .modify(|_, w| unsafe { w.bits(line as u32) });
            }
            Trigger::Level => {
                self.line_mask = self.line_mask | (line as u32);

                if self.dp.mask_edge_set().read().bits() & self.line_mask != 0 {
                    return Err(Error::LineEdgeSet);
                }

                self.dp
                    .mask_level_set()
                    .write(|w| unsafe { w.bits(self.line_mask) });
            }
        }
        Ok(())
    }

    /// Выключает прерывания по конкретной линии
    /// Отключаются все прерввания, по фронту и по уровню.
    ///
    /// # Arguments
    ///
    /// - `line` (`InterruptLine`) - какую линию прервываний отключить, если `None`, то отключить все линии
    ///
    /// # Examples
    ///
    /// ```
    /// use crate::epic::EPIC;
    /// use mik32_pac::Peripherals;
    ///
    /// let dp = Peripherals::take().unwrap();
    /// let mut epic = EPIC::new(dp.epic);
    /// epic.unlisten(InterruptLine::Timer32_1); // Выключим прерывание по линии таймера
    /// ```
    pub fn unlisten(&mut self, line: Option<InterruptLine>) {
        if let Some(line) = line {
            // Выключаем "по фронту"
            self.dp
                .mask_edge_clear()
                .modify(|_, w| unsafe { w.bits(line as u32) });

            // Выключаем "по уровню"
            self.line_mask = self.line_mask & !(line as u32);
            self.dp
                .mask_level_clear()
                .write(|w| unsafe { w.bits(line as u32) });
        } else {
            // Выключаем все "по фронту"
            self.dp
                .mask_edge_clear()
                .modify(|_, w| unsafe { w.bits(u32::MAX) });

            // Выключаем все "по уровню"
            self.line_mask = 0u32;
            self.dp
                .mask_level_clear()
                .write(|w| unsafe { w.bits(u32::MAX) });
        }
    }

    /// Произошло ли событие?
    ///
    /// # Arguments
    /// - `event` (`InterruptEvent`) - проверяемое событие
    ///
    /// # Returns
    ///
    /// - `bool` - случилось ли событие
    ///
    /// # Examples
    ///
    /// ```
    /// use crate::epic::EPIC;
    /// use mik32_pac::Peripherals;
    ///
    /// let dp = Peripherals::take().unwrap();
    /// let mut epic = EPIC::new(dp.epic);
    /// epic.listen(InterruptLine::Timer32_1); // Включим прерывание по линии таймера
    /// let is_timer_event_happend: bool = epic.event(InterruptLine::Timer32_1);
    /// ```
    pub fn event(&self, line: InterruptLine) -> bool {
        match line {
            InterruptLine::ADC => {
                return self.dp.raw_status().read().adc().bit_is_set();
            }
            InterruptLine::DAC0 => {
                return self.dp.raw_status().read().dac0().bit_is_set();
            }
            InterruptLine::DAC1 => {
                return self.dp.raw_status().read().dac1().bit_is_set();
            }
            InterruptLine::DMA => {
                return self.dp.raw_status().read().dma().bit_is_set();
            }
            InterruptLine::EEPROM => {
                return self.dp.raw_status().read().eeprom().bit_is_set();
            }
            InterruptLine::FrequencyMonitor => {
                return self.dp.raw_status().read().frequency_monitor().bit_is_set();
            }
            InterruptLine::GPIO => {
                return self.dp.raw_status().read().gpio().bit_is_set();
            }
            InterruptLine::I2C0 => {
                return self.dp.raw_status().read().i2c_0().bit_is_set();
            }
            InterruptLine::I2C1 => {
                return self.dp.raw_status().read().i2c_1().bit_is_set();
            }
            InterruptLine::AVCCOver => {
                return self.dp.raw_status().read().pvd_avcc_over().bit_is_set();
            }
            InterruptLine::AVCCUnder => {
                return self.dp.raw_status().read().pvd_avcc_under().bit_is_set();
            }
            InterruptLine::VCCOver => {
                return self.dp.raw_status().read().pvd_vcc_over().bit_is_set();
            }
            InterruptLine::VCCUnder => {
                return self.dp.raw_status().read().pvd_vcc_under().bit_is_set();
            }
            InterruptLine::RTC => {
                return self.dp.raw_status().read().rtc().bit_is_set();
            }
            InterruptLine::SPI0 => {
                return self.dp.raw_status().read().spi_0().bit_is_set();
            }
            InterruptLine::SPI1 => {
                return self.dp.raw_status().read().spi_1().bit_is_set();
            }
            InterruptLine::SPIFI => {
                return self.dp.raw_status().read().spifi().bit_is_set();
            }
            InterruptLine::LowBattery => {
                return self.dp.raw_status().read().battery_non_good().bit_is_set();
            }
            InterruptLine::Timer16_0 => {
                return self.dp.raw_status().read().timer16_0().bit_is_set();
            }
            InterruptLine::Timer16_1 => {
                return self.dp.raw_status().read().timer16_1().bit_is_set();
            }
            InterruptLine::Timer16_2 => {
                return self.dp.raw_status().read().timer16_2().bit_is_set();
            }
            InterruptLine::Timer32_0 => {
                return self.dp.raw_status().read().timer32_0().bit_is_set();
            }
            InterruptLine::Timer32_1 => {
                return self.dp.raw_status().read().timer32_1().bit_is_set();
            }
            InterruptLine::Timer32_2 => {
                return self.dp.raw_status().read().timer32_2().bit_is_set();
            }
            InterruptLine::TSENS => {
                return self.dp.raw_status().read().tsens().bit_is_set();
            }
            InterruptLine::USART0 => {
                return self.dp.raw_status().read().usart_0().bit_is_set();
            }
            InterruptLine::USART1 => {
                return self.dp.raw_status().read().usart_1().bit_is_set();
            }
            InterruptLine::WDT => {
                return self.dp.raw_status().read().wdt().bit_is_set();
            }
            InterruptLine::WdtDom3 => {
                return self.dp.raw_status().read().wdt_bus_dom3().bit_is_set();
            }
            InterruptLine::WdtEeprom => {
                return self.dp.raw_status().read().wdt_bus_eeprom().bit_is_set();
            }
            InterruptLine::WdtSpifi => {
                return self.dp.raw_status().read().wdt_bus_spifi().bit_is_set();
            }
            InterruptLine::BrownOut => {
                return self.dp.raw_status().read().bor().bit_is_set();
            }
        }
    }

    ///  Очищает флаги всех прерываний
    ///
    /// # Examples
    ///
    /// ```
    /// use crate::epic::EPIC;
    /// use mik32_pac::Peripherals;
    /// let dp = Peripherals::take().unwrap();
    /// let mut epic = EPIC::new(dp.epic);
    /// epic.unlisten(InterruptLine::Timer32_1); // Выключим прерывание по линии таймера
    /// ```
    pub fn clear(&mut self) {
        self.dp.clear().write(|w| unsafe { w.bits(0xFFFFFFFF) });
    }
}
