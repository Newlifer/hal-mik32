//! External Peripheral Interrupt Controller (EPIC).
//!
//! EPIC combines 32 peripheral interrupt lines into one machine external
//! interrupt. It has no vector table or hardware priorities; dispatching is
//! done in software by reading [`Epic::pending`].

use core::ops::{BitAnd, BitAndAssign, BitOr, BitOrAssign, Not};
use mik32_pac::Epic as EpicPeripheral;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Trigger {
    Edge,
    Level,
}

/// One of the 32 EPIC input lines.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Interrupt {
    Timer32_0 = 0,
    Usart0 = 1,
    Usart1 = 2,
    Spi0 = 3,
    Spi1 = 4,
    Gpio = 5,
    I2c0 = 6,
    I2c1 = 7,
    Wdt = 8,
    Timer16_0 = 9,
    Timer16_1 = 10,
    Timer16_2 = 11,
    Timer32_1 = 12,
    Timer32_2 = 13,
    Spifi = 14,
    Rtc = 15,
    Eeprom = 16,
    WdtBusDom3 = 17,
    WdtBusSpifi = 18,
    WdtBusEeprom = 19,
    Dma = 20,
    FrequencyMonitor = 21,
    PvdAvccUnder = 22,
    PvdAvccOver = 23,
    PvdVccUnder = 24,
    PvdVccOver = 25,
    BatteryLow = 26,
    BrownOut = 27,
    Tsens = 28,
    Adc = 29,
    Dac0 = 30,
    Dac1 = 31,
}

impl Interrupt {
    pub const fn mask(self) -> InterruptMask {
        InterruptMask(1 << self as u8)
    }

    const fn from_index(index: u32) -> Self {
        ALL_INTERRUPTS[index as usize]
    }
}

const ALL_INTERRUPTS: [Interrupt; 32] = [
    Interrupt::Timer32_0,
    Interrupt::Usart0,
    Interrupt::Usart1,
    Interrupt::Spi0,
    Interrupt::Spi1,
    Interrupt::Gpio,
    Interrupt::I2c0,
    Interrupt::I2c1,
    Interrupt::Wdt,
    Interrupt::Timer16_0,
    Interrupt::Timer16_1,
    Interrupt::Timer16_2,
    Interrupt::Timer32_1,
    Interrupt::Timer32_2,
    Interrupt::Spifi,
    Interrupt::Rtc,
    Interrupt::Eeprom,
    Interrupt::WdtBusDom3,
    Interrupt::WdtBusSpifi,
    Interrupt::WdtBusEeprom,
    Interrupt::Dma,
    Interrupt::FrequencyMonitor,
    Interrupt::PvdAvccUnder,
    Interrupt::PvdAvccOver,
    Interrupt::PvdVccUnder,
    Interrupt::PvdVccOver,
    Interrupt::BatteryLow,
    Interrupt::BrownOut,
    Interrupt::Tsens,
    Interrupt::Adc,
    Interrupt::Dac0,
    Interrupt::Dac1,
];

/// A set of EPIC interrupt lines.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct InterruptMask(u32);

impl InterruptMask {
    pub const NONE: Self = Self(0);
    pub const ALL: Self = Self(u32::MAX);

    pub const fn from_bits(bits: u32) -> Self {
        Self(bits)
    }

    pub const fn bits(self) -> u32 {
        self.0
    }

    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    pub const fn contains(self, interrupt: Interrupt) -> bool {
        self.0 & interrupt.mask().0 != 0
    }

    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    /// Removes and returns the lowest-numbered pending interrupt.
    pub fn next(&mut self) -> Option<Interrupt> {
        if self.is_empty() {
            return None;
        }
        let index = self.0.trailing_zeros();
        self.0 &= !(1 << index);
        Some(Interrupt::from_index(index))
    }
}

impl From<Interrupt> for InterruptMask {
    fn from(interrupt: Interrupt) -> Self {
        interrupt.mask()
    }
}

impl BitOr for InterruptMask {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl BitOrAssign for InterruptMask {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

impl BitAnd for InterruptMask {
    type Output = Self;
    fn bitand(self, rhs: Self) -> Self::Output {
        Self(self.0 & rhs.0)
    }
}

impl BitAndAssign for InterruptMask {
    fn bitand_assign(&mut self, rhs: Self) {
        self.0 &= rhs.0;
    }
}

impl Not for InterruptMask {
    type Output = Self;
    fn not(self) -> Self::Output {
        Self(!self.0)
    }
}

/// Owner of the EPIC PAC peripheral.
pub struct Epic {
    peripheral: EpicPeripheral,
}

impl Epic {
    /// Creates the driver without changing existing masks.
    ///
    /// The caller must enable the EPIC clock in the APB_M clock domain first.
    pub const fn new(peripheral: EpicPeripheral) -> Self {
        Self { peripheral }
    }

    /// Disables all lines and clears all latched pending events.
    pub fn reset(&mut self) {
        self.disable_mask(InterruptMask::ALL);
        self.clear_pending_mask(InterruptMask::ALL);
    }

    pub fn enable(&mut self, interrupt: Interrupt, trigger: Trigger) {
        self.enable_mask(interrupt.mask(), trigger);
    }

    /// Enables a group of lines and selects edge or level mode for all of them.
    /// Any opposite-mode configuration for these lines is removed first.
    pub fn enable_mask(&mut self, mask: InterruptMask, trigger: Trigger) {
        let bits = mask.bits();
        match trigger {
            Trigger::Edge => {
                self.peripheral
                    .mask_level_clear()
                    .write(|w| unsafe { w.bits(bits) });
                self.peripheral
                    .mask_edge_set()
                    .write(|w| unsafe { w.bits(bits) });
            }
            Trigger::Level => {
                self.peripheral
                    .mask_edge_clear()
                    .write(|w| unsafe { w.bits(bits) });
                self.peripheral
                    .mask_level_set()
                    .write(|w| unsafe { w.bits(bits) });
            }
        }
    }

    pub fn disable(&mut self, interrupt: Interrupt) {
        self.disable_mask(interrupt.mask());
    }

    /// Disables both edge and level handling for selected lines.
    pub fn disable_mask(&mut self, mask: InterruptMask) {
        let bits = mask.bits();
        self.peripheral
            .mask_edge_clear()
            .write(|w| unsafe { w.bits(bits) });
        self.peripheral
            .mask_level_clear()
            .write(|w| unsafe { w.bits(bits) });
    }

    /// Returns enabled interrupts that currently require handling (`STATUS`).
    pub fn pending(&self) -> InterruptMask {
        InterruptMask::from_bits(self.peripheral.status().read().bits())
    }

    pub fn is_pending(&self, interrupt: Interrupt) -> bool {
        self.pending().contains(interrupt)
    }

    /// Returns current input-line states without applying masks (`RAW_STATUS`).
    pub fn asserted(&self) -> InterruptMask {
        InterruptMask::from_bits(self.peripheral.raw_status().read().bits())
    }

    pub fn is_asserted(&self, interrupt: Interrupt) -> bool {
        self.asserted().contains(interrupt)
    }

    pub fn clear_pending(&mut self, interrupt: Interrupt) {
        self.clear_pending_mask(interrupt.mask());
    }

    /// Clears selected latched EPIC events.
    ///
    /// For a level interrupt, clear the cause in the source peripheral first;
    /// otherwise the line becomes pending again immediately.
    pub fn clear_pending_mask(&mut self, mask: InterruptMask) {
        self.peripheral
            .clear()
            .write(|w| unsafe { w.bits(mask.bits()) });
    }

    pub fn release(self) -> EpicPeripheral {
        self.peripheral
    }
}
