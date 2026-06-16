//! Common system constants for MIK32
//!
//! This module contains system-wide constants including clock frequencies,
//! dividers, and communication parameters.

// ============================================================================
// System Clock Configuration
// ============================================================================

/// MIK32 system clock frequency (OSC32M - External 32 MHz oscillator)
pub const SYS_CLOCK_FREQ: u32 = 32_000_000;

/// External 32 MHz oscillator value (OSC32M)
pub const OSC_SYSTEM_VALUE: u32 = 32_000_000;

/// Internal 32 MHz oscillator value (HSI32M)
pub const HSI_VALUE: u32 = 32_000_000;

/// External 32.768 kHz oscillator value (OSC32K)
pub const OSC_CLOCK_VALUE: u32 = 32_768;

/// Internal 32.768 kHz oscillator value (LSI32K)
pub const LSI_VALUE: u32 = 32_768;

/// AHB bus divider (0 = divide by 1)
pub const DIV_AHB: u32 = 0;

/// APB_P bus divider (0 = divide by 1)
pub const DIV_APB_P: u32 = 0;

/// APB_M bus divider (0 = divide by 1)
pub const DIV_APB_M: u32 = 0;

/// Effective clock frequency for peripherals on APB_P bus
/// Formula: SYS_CLOCK_FREQ / (DIV_AHB + 1) / (DIV_APB_P + 1)
pub const APB_P_CLOCK_FREQ: u32 = SYS_CLOCK_FREQ / (DIV_AHB + 1) / (DIV_APB_P + 1);

// ============================================================================
// USART Configuration
// ============================================================================

/// Standard USART baudrate (bits per second)
pub const USART_BAUDRATE: u32 = 115_200;

/// USART baudrate divisor (BRR) for 115200 baud
/// Formula: APB_P_CLOCK_FREQ / USART_BAUDRATE
pub const USART_BRR: u32 = APB_P_CLOCK_FREQ / USART_BAUDRATE;

// ============================================================================
// DMA Configuration
// ============================================================================

/// DMA channel 1 for USART1 TX transfers
pub const DMA_CHANNEL_USART1_TX: u32 = 1;

// ============================================================================
// GPIO Configuration
// ============================================================================

/// LED pin on GPIO0
pub const PIN_LED: u32 = 9; // PORT 0.9
