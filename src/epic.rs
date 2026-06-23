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
use mik32_pac::{Epic, Peripherals};

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

impl InterruptLine {
    #[inline(always)]
    pub const fn mask(self) -> u32 {
        self as u32
    }
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
        // EPIC sits on the APB_M clock domain. The C HAL enables this clock
        // before accessing EPIC registers; do the same here so the driver is
        // ready to use right after construction.
        unsafe {
            Peripherals::steal()
                .pm
                .clk_apb_m_set()
                .write(|w| w.epic().enable());
        }

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
        let line_mask = line.mask();

        match trigger {
            Trigger::Edge => {
                if self.line_mask & line_mask != 0 {
                    return Err(Error::LineLevelSet);
                }

                self.dp
                    .mask_edge_set()
                    .write(|w| unsafe { w.bits(line_mask) });
            }
            Trigger::Level => {
                if self.dp.mask_edge_set().read().bits() & line_mask != 0 {
                    return Err(Error::LineEdgeSet);
                }

                self.line_mask |= line_mask;
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
            let line_mask = line.mask();

            // Выключаем "по фронту"
            self.dp
                .mask_edge_clear()
                .write(|w| unsafe { w.bits(line_mask) });

            // Выключаем "по уровню"
            self.line_mask &= !line_mask;
            self.dp
                .mask_level_clear()
                .write(|w| unsafe { w.bits(line_mask) });
        } else {
            // Выключаем все "по фронту"
            self.dp
                .mask_edge_clear()
                .write(|w| unsafe { w.bits(u32::MAX) });

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
        self.raw_pending(line)
    }

    /// Returns true when a configured, unmasked interrupt is pending.
    ///
    /// Reads EPIC `STATUS`, which is the right register for dispatching from a
    /// trap handler because it takes EPIC masks into account.
    pub fn pending(&self, line: InterruptLine) -> bool {
        self.pending_mask() & line.mask() != 0
    }

    /// Returns the full EPIC `STATUS` register.
    pub fn pending_mask(&self) -> u32 {
        self.dp.status().read().bits()
    }

    /// Returns true when the interrupt line is asserted regardless of masks.
    ///
    /// Reads EPIC `RAW_STATUS`.
    pub fn raw_pending(&self, line: InterruptLine) -> bool {
        self.raw_pending_mask() & line.mask() != 0
    }

    /// Returns the full EPIC `RAW_STATUS` register.
    pub fn raw_pending_mask(&self) -> u32 {
        self.dp.raw_status().read().bits()
    }

    /// Clears the pending flag for one interrupt line.
    pub fn clear_line(&mut self, line: InterruptLine) {
        self.clear_mask(line.mask());
    }

    /// Clears pending flags selected by `mask`.
    pub fn clear_mask(&mut self, mask: u32) {
        self.dp.clear().write(|w| unsafe { w.bits(mask) });
    }

    /// Clears pending flags for all interrupt lines.
    pub fn clear_all(&mut self) {
        self.clear_mask(u32::MAX);
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
        self.clear_all();
    }
}
