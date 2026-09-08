//! Hardware CRC-32 calculator.
//!
//! The peripheral accepts 8-, 16-, and 32-bit writes at the same `DATA`
//! address. [`Crc::update`] uses 32-bit writes where possible and finishes a
//! partial word with a 16- and/or 8-bit write.

use core::ptr;

use embedded_hal_nb::nb::{Error as NbError, Result as NbResult};
use mik32_pac::Crc as CrcPeripheral;

use crate::rcc::RCC;

/// Number of bounded spin iterations used before reading `BUSY`.
///
/// Hardware asserts `BUSY` one AHB clock after a `DATA` write. The reference
/// implementation leaves a larger margin before its first status read.
const BUSY_VISIBILITY_DELAY: u8 = 100;

/// Input or output bit/byte permutation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum Transpose {
    /// Keep bits and bytes unchanged.
    #[default]
    None = 0,
    /// Reverse the bits in every byte.
    Bits = 1,
    /// Reverse both the bits in every byte and the byte order.
    BitsAndBytes = 2,
    /// Reverse the byte order.
    Bytes = 3,
}

/// CRC calculation parameters.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    pub polynomial: u32,
    pub initial_value: u32,
    pub input_transpose: Transpose,
    pub output_transpose: Transpose,
    pub final_xor: bool,
}

impl Config {
    /// Creates a configuration for the supplied 32-bit polynomial.
    pub const fn new(polynomial: u32) -> Self {
        Self {
            polynomial,
            initial_value: 0,
            input_transpose: Transpose::None,
            output_transpose: Transpose::None,
            final_xor: false,
        }
    }

    pub const fn initial_value(mut self, value: u32) -> Self {
        self.initial_value = value;
        self
    }

    pub const fn input_transpose(mut self, value: Transpose) -> Self {
        self.input_transpose = value;
        self
    }

    pub const fn output_transpose(mut self, value: Transpose) -> Self {
        self.output_transpose = value;
        self
    }

    pub const fn final_xor(mut self, enabled: bool) -> Self {
        self.final_xor = enabled;
        self
    }

    /// CRC-32/ISO-HDLC (`CRC-32`, check value `0xcbf43926`).
    pub const fn crc32_iso_hdlc() -> Self {
        Self::new(0x04c1_1db7)
            .initial_value(0xffff_ffff)
            .input_transpose(Transpose::BitsAndBytes)
            .output_transpose(Transpose::BitsAndBytes)
            .final_xor(true)
    }

    /// CRC-32/MPEG-2 (check value `0x0376e6e7`).
    pub const fn crc32_mpeg2() -> Self {
        Self::new(0x04c1_1db7)
            .initial_value(0xffff_ffff)
            .input_transpose(Transpose::Bytes)
            .output_transpose(Transpose::None)
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::crc32_iso_hdlc()
    }
}

/// Error returned when configuration is changed during a calculation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The hardware is still processing previously supplied data.
    Busy,
}

/// Error returned by an operation with a finite poll budget.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Timeout;

/// Owned CRC-32 peripheral.
pub struct Crc {
    peripheral: CrcPeripheral,
    config: Config,
    pending: bool,
}

impl Crc {
    /// Enables the peripheral clock, applies `config`, and loads its seed.
    pub fn new(peripheral: CrcPeripheral, config: Config) -> Self {
        RCC::enable_crc32();

        let mut crc = Self {
            peripheral,
            config,
            pending: false,
        };
        crc.apply_config();
        crc.load_initial_value(config.initial_value);
        crc
    }

    /// Returns the active configuration.
    pub const fn config(&self) -> Config {
        self.config
    }

    /// Applies new parameters and starts a fresh calculation.
    pub fn configure(&mut self, config: Config) -> Result<(), Error> {
        if !self.poll_ready() {
            return Err(Error::Busy);
        }
        self.config = config;
        self.apply_config();
        self.load_initial_value(config.initial_value);
        Ok(())
    }

    /// Discards the current result and reloads the configured initial value.
    pub fn reset(&mut self) {
        self.wait_ready();
        self.load_initial_value(self.config.initial_value);
    }

    /// Discards the current result and loads a one-off initial value.
    pub fn reset_with(&mut self, initial_value: u32) {
        self.wait_ready();
        self.load_initial_value(initial_value);
    }

    /// Tries to submit one 32-bit data word.
    pub fn try_update_word(&mut self, word: u32) -> NbResult<(), Error> {
        if !self.poll_ready() {
            return Err(NbError::WouldBlock);
        }
        self.write_u32(word);
        Ok(())
    }

    /// Submits one 32-bit data word, waiting for space if necessary.
    pub fn update_word(&mut self, word: u32) {
        self.wait_ready();
        self.write_u32(word);
    }

    /// Submits one word after waiting for space for at most `timeout` polls.
    pub fn update_word_timeout(&mut self, word: u32, timeout: u32) -> Result<(), Timeout> {
        self.wait_ready_timeout(timeout)?;
        self.write_u32(word);
        Ok(())
    }

    /// Submits a sequence of native 32-bit data words.
    pub fn update_words(&mut self, words: &[u32]) {
        for &word in words {
            self.update_word(word);
        }
    }

    /// Submits an arbitrary byte slice.
    ///
    /// Four-byte groups are packed most-significant byte first, matching the
    /// vendor reference implementation. The final 1-3 bytes use narrower
    /// writes and therefore do not add padding to the CRC stream.
    pub fn update(&mut self, bytes: &[u8]) {
        let mut chunks = bytes.chunks_exact(4);
        for chunk in &mut chunks {
            self.wait_ready();
            self.write_u32(pack_u32(chunk));
        }

        match chunks.remainder() {
            [a, b, c] => {
                self.wait_ready();
                self.write_u16(u16::from_be_bytes([*a, *b]));
                self.wait_ready();
                self.write_u8(*c);
            }
            [a, b] => {
                self.wait_ready();
                self.write_u16(u16::from_be_bytes([*a, *b]));
            }
            [a] => {
                self.wait_ready();
                self.write_u8(*a);
            }
            [] => {}
            _ => unreachable!(),
        }
    }

    /// Returns the result if all submitted data has been processed.
    pub fn try_finalize(&mut self) -> NbResult<u32, Error> {
        if !self.poll_ready() {
            return Err(NbError::WouldBlock);
        }
        Ok(self.peripheral.data().read().bits())
    }

    /// Waits for and returns the current result.
    pub fn finalize(&mut self) -> u32 {
        self.wait_ready();
        self.peripheral.data().read().bits()
    }

    /// Waits for the result for at most `timeout` status polls.
    pub fn finalize_timeout(&mut self, timeout: u32) -> Result<u32, Timeout> {
        self.wait_ready_timeout(timeout)?;
        Ok(self.peripheral.data().read().bits())
    }

    /// Starts a fresh calculation, processes `bytes`, and returns its CRC.
    pub fn checksum(&mut self, bytes: &[u8]) -> u32 {
        self.reset();
        self.update(bytes);
        self.finalize()
    }

    /// Returns whether a submitted operation has not yet been observed complete.
    pub fn is_busy(&self) -> bool {
        self.pending || self.peripheral.ctrl().read().busy().is_busy()
    }

    /// Releases the PAC peripheral after completing pending work.
    pub fn release(mut self) -> CrcPeripheral {
        self.wait_ready();
        self.peripheral
    }

    fn apply_config(&mut self) {
        self.peripheral
            .poly()
            .write(|w| unsafe { w.bits(self.config.polynomial) });
        self.write_ctrl(false);
    }

    fn load_initial_value(&mut self, initial_value: u32) {
        self.write_ctrl(true);
        self.peripheral
            .data()
            .write(|w| unsafe { w.bits(initial_value) });
        self.write_ctrl(false);
        self.pending = false;
    }

    fn write_ctrl(&mut self, initial_value: bool) {
        let config = self.config;
        self.peripheral.ctrl().write(|w| {
            let w = w
                .tot()
                .variant(input_transpose(config.input_transpose))
                .totr()
                .variant(output_transpose(config.output_transpose));
            let w = if config.final_xor {
                w.fxor().inversion_enable()
            } else {
                w.fxor().inversion_disable()
            };
            if initial_value {
                w.was().init_data()
            } else {
                w.was().data()
            }
        });
    }

    fn poll_ready(&mut self) -> bool {
        if !self.pending {
            return !self.peripheral.ctrl().read().busy().is_busy();
        }

        delay_busy_visibility();
        if self.peripheral.ctrl().read().busy().is_busy() {
            false
        } else {
            self.pending = false;
            true
        }
    }

    fn wait_ready(&mut self) {
        while !self.poll_ready() {
            core::hint::spin_loop();
        }
    }

    fn wait_ready_timeout(&mut self, timeout: u32) -> Result<(), Timeout> {
        for _ in 0..timeout {
            if self.poll_ready() {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(Timeout)
    }

    fn write_u8(&mut self, value: u8) {
        unsafe { ptr::write_volatile(CrcPeripheral::PTR.cast_mut().cast::<u8>(), value) };
        self.pending = true;
    }

    fn write_u16(&mut self, value: u16) {
        unsafe { ptr::write_volatile(CrcPeripheral::PTR.cast_mut().cast::<u16>(), value) };
        self.pending = true;
    }

    fn write_u32(&mut self, value: u32) {
        self.peripheral.data().write(|w| unsafe { w.bits(value) });
        self.pending = true;
    }
}

fn input_transpose(value: Transpose) -> mik32_pac::crc::ctrl::Tot {
    use mik32_pac::crc::ctrl::Tot;
    match value {
        Transpose::None => Tot::None,
        Transpose::Bits => Tot::Bits,
        Transpose::BitsAndBytes => Tot::BitsBytes,
        Transpose::Bytes => Tot::Bytes,
    }
}

fn output_transpose(value: Transpose) -> mik32_pac::crc::ctrl::Totr {
    use mik32_pac::crc::ctrl::Totr;
    match value {
        Transpose::None => Totr::None,
        Transpose::Bits => Totr::Bits,
        Transpose::BitsAndBytes => Totr::BitsBytes,
        Transpose::Bytes => Totr::Bytes,
    }
}

fn pack_u32(bytes: &[u8]) -> u32 {
    u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
}

#[inline(never)]
fn delay_busy_visibility() {
    for _ in 0..BUSY_VISIBILITY_DELAY {
        core::hint::spin_loop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_match_documented_parameters() {
        assert_eq!(
            Config::crc32_iso_hdlc(),
            Config {
                polynomial: 0x04c1_1db7,
                initial_value: 0xffff_ffff,
                input_transpose: Transpose::BitsAndBytes,
                output_transpose: Transpose::BitsAndBytes,
                final_xor: true,
            }
        );
        assert_eq!(Config::crc32_mpeg2().output_transpose, Transpose::None);
        assert!(!Config::crc32_mpeg2().final_xor);
    }

    #[test]
    fn full_words_are_packed_like_the_reference_driver() {
        assert_eq!(pack_u32(&[0x12, 0x34, 0x56, 0x78]), 0x1234_5678);
    }
}
