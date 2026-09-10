//! Hardware-accelerated block ciphers.
//!
//! The peripheral implements AES-128, Kuznechik and Magma in ECB, CBC and
//! CTR modes. Keys, initialization vectors and data are transferred through
//! 32-bit streaming registers. This module presents them as byte arrays and
//! keeps algorithm-specific sizes in the type system.
//!
//! This peripheral provides encryption, not authentication. Applications
//! using CBC or CTR must provide integrity protection separately.

use core::{marker::PhantomData, mem::size_of};

use mik32_pac::{Crypto as CryptoPeripheral, crypto::config};

use crate::{
    dma::{Channel as DmaChannel, ChannelId as DmaChannelId, Error as DmaError},
    rcc::RCC,
};

/// Error returned when a buffer cannot be processed in the selected mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// ECB and CBC input must contain a whole number of cipher blocks.
    InvalidLength,
}

/// Error returned by a blocking crypto DMA operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DmaTransferError {
    /// Input and output lengths differ or do not contain whole cipher blocks.
    InvalidLength,
    /// A DMA channel rejected the transfer configuration.
    Dma(DmaError),
}

impl From<DmaError> for DmaTransferError {
    fn from(error: DmaError) -> Self {
        Self::Dma(error)
    }
}

mod sealed {
    pub trait Sealed {}
}

/// Description of an algorithm implemented by the crypto peripheral.
pub trait Algorithm: sealed::Sealed {
    /// Master-key representation accepted by the algorithm.
    type Key: AsRef<[u8]>;
    /// One cipher block.
    type Block: AsRef<[u8]> + AsMut<[u8]>;
    /// Initialization vector used by CBC.
    type CbcIv: AsRef<[u8]>;
    /// Nonce portion loaded for CTR; the remaining half starts at zero.
    type CtrIv: AsRef<[u8]>;

    #[doc(hidden)]
    const CORE: config::CoreSel;
    #[doc(hidden)]
    const BLOCK_BYTES: usize;
}

/// AES with a 128-bit key.
pub enum Aes {}
/// GOST R 34.12-2015 Kuznechik.
pub enum KuznechikAlgorithm {}
/// GOST R 34.12-2015 Magma.
pub enum MagmaAlgorithm {}

impl sealed::Sealed for Aes {}
impl sealed::Sealed for KuznechikAlgorithm {}
impl sealed::Sealed for MagmaAlgorithm {}

impl Algorithm for Aes {
    type Key = [u8; 16];
    type Block = [u8; 16];
    type CbcIv = [u8; 16];
    type CtrIv = [u8; 8];

    const CORE: config::CoreSel = config::CoreSel::Aes;
    const BLOCK_BYTES: usize = 16;
}

impl Algorithm for KuznechikAlgorithm {
    type Key = [u8; 32];
    type Block = [u8; 16];
    type CbcIv = [u8; 16];
    type CtrIv = [u8; 8];

    const CORE: config::CoreSel = config::CoreSel::Kuznechik;
    const BLOCK_BYTES: usize = 16;
}

impl Algorithm for MagmaAlgorithm {
    type Key = [u8; 32];
    type Block = [u8; 8];
    type CbcIv = [u8; 8];
    type CtrIv = [u8; 4];

    const CORE: config::CoreSel = config::CoreSel::Magma;
    const BLOCK_BYTES: usize = 8;
}

/// Owned crypto peripheral.
pub struct Crypto {
    peripheral: CryptoPeripheral,
}

impl Crypto {
    /// Enables the peripheral clock and takes ownership of the crypto block.
    pub fn new(peripheral: CryptoPeripheral) -> Self {
        RCC::enable_crypto();
        Self { peripheral }
    }

    /// Loads an AES-128 key and returns a configured cipher builder.
    pub fn aes128<'a>(&'a mut self, key: &[u8; 16]) -> Aes128<'a> {
        self.load::<Aes>(key);
        Cipher::new(self)
    }

    /// Loads a 256-bit Kuznechik key and returns a configured cipher builder.
    pub fn kuznechik<'a>(&'a mut self, key: &[u8; 32]) -> Kuznechik<'a> {
        self.load::<KuznechikAlgorithm>(key);
        Cipher::new(self)
    }

    /// Loads a 256-bit Magma key and returns a configured cipher builder.
    pub fn magma<'a>(&'a mut self, key: &[u8; 32]) -> Magma<'a> {
        self.load::<MagmaAlgorithm>(key);
        Cipher::new(self)
    }

    /// Returns the underlying PAC peripheral after pending work completes.
    pub fn release(self) -> CryptoPeripheral {
        self.wait_ready();
        self.peripheral
    }

    fn load<A: Algorithm>(&mut self, key: &A::Key) {
        self.peripheral.config().write(|w| {
            w.decode()
                .encode()
                .core_sel()
                .variant(A::CORE)
                .mode_sel()
                .ecb()
                .swap_mode()
                .none()
                .order_mode()
                .msw()
                .c_reset()
                .set_bit()
        });
        self.write_words_to_key(key.as_ref());
        self.wait_ready();
    }

    fn configure(&mut self, mode: config::ModeSel, decrypt: bool) {
        self.wait_ready();
        self.peripheral.config().modify(|_, w| {
            let w = w.mode_sel().variant(mode);
            let w = if decrypt {
                w.decode().decode()
            } else {
                w.decode().encode()
            };
            w.c_reset().set_bit()
        });
    }

    fn load_iv(&mut self, iv: &[u8], zero_words: usize) {
        for word in words(iv) {
            self.peripheral.init().write(|w| unsafe { w.bits(word) });
        }
        for _ in 0..zero_words {
            self.peripheral.init().write(|w| unsafe { w.bits(0) });
        }
    }

    fn process_block(&mut self, block: &mut [u8]) {
        for word in words(block) {
            self.peripheral.block().write(|w| unsafe { w.bits(word) });
        }
        self.wait_ready();
        for chunk in block.chunks_exact_mut(4) {
            chunk.copy_from_slice(&self.peripheral.block().read().bits().to_be_bytes());
        }
    }

    fn write_words_to_key(&mut self, bytes: &[u8]) {
        for word in words(bytes) {
            self.peripheral.key().write(|w| unsafe { w.bits(word) });
        }
    }

    fn wait_ready(&self) {
        while self.peripheral.config().read().ready().is_busy() {}
    }
}

/// Algorithm-specific mode builder holding a loaded key.
pub struct Cipher<'a, A: Algorithm> {
    crypto: &'a mut Crypto,
    algorithm: PhantomData<A>,
}

/// AES-128 mode builder.
pub type Aes128<'a> = Cipher<'a, Aes>;
/// Kuznechik mode builder.
pub type Kuznechik<'a> = Cipher<'a, KuznechikAlgorithm>;
/// Magma mode builder.
pub type Magma<'a> = Cipher<'a, MagmaAlgorithm>;

impl<'a, A: Algorithm> Cipher<'a, A> {
    fn new(crypto: &'a mut Crypto) -> Self {
        Self {
            crypto,
            algorithm: PhantomData,
        }
    }

    /// Selects electronic codebook mode.
    pub fn ecb(self) -> Ecb<'a, A> {
        self.crypto.configure(config::ModeSel::Ecb, false);
        Ecb { cipher: self }
    }

    /// Selects CBC encryption and loads its initialization vector.
    pub fn cbc_encrypt(self, iv: &A::CbcIv) -> CbcEncrypt<'a, A> {
        self.crypto.configure(config::ModeSel::Cbc, false);
        self.crypto.load_iv(iv.as_ref(), 0);
        CbcEncrypt { cipher: self }
    }

    /// Selects CBC decryption and loads its initialization vector.
    pub fn cbc_decrypt(self, iv: &A::CbcIv) -> CbcDecrypt<'a, A> {
        self.crypto.configure(config::ModeSel::Cbc, true);
        self.crypto.load_iv(iv.as_ref(), 0);
        CbcDecrypt { cipher: self }
    }

    /// Selects counter mode and loads the nonce followed by a zero counter.
    pub fn ctr(self, nonce: &A::CtrIv) -> Ctr<'a, A> {
        self.crypto.configure(config::ModeSel::Ctr, false);
        self.crypto
            .load_iv(nonce.as_ref(), nonce.as_ref().len() / 4);
        Ctr { cipher: self }
    }
}

/// Electronic codebook mode.
pub struct Ecb<'a, A: Algorithm> {
    cipher: Cipher<'a, A>,
}

impl<A: Algorithm> Ecb<'_, A> {
    /// Encrypts exactly one block in place.
    pub fn encrypt_block(&mut self, block: &mut A::Block) {
        self.cipher.crypto.configure(config::ModeSel::Ecb, false);
        self.cipher.crypto.process_block(block.as_mut());
    }

    /// Decrypts exactly one block in place.
    pub fn decrypt_block(&mut self, block: &mut A::Block) {
        self.cipher.crypto.configure(config::ModeSel::Ecb, true);
        self.cipher.crypto.process_block(block.as_mut());
    }

    /// Encrypts a whole number of blocks in place.
    pub fn encrypt_blocks_in_place(&mut self, data: &mut [u8]) -> Result<(), Error> {
        self.cipher.crypto.configure(config::ModeSel::Ecb, false);
        process_full_blocks::<A>(self.cipher.crypto, data)
    }

    /// Decrypts a whole number of blocks in place.
    pub fn decrypt_blocks_in_place(&mut self, data: &mut [u8]) -> Result<(), Error> {
        self.cipher.crypto.configure(config::ModeSel::Ecb, true);
        process_full_blocks::<A>(self.cipher.crypto, data)
    }

    /// Encrypts native 32-bit words through two DMA channels and waits for
    /// both transfers to finish.
    ///
    /// The input and output must have equal lengths and contain a whole number
    /// of cipher blocks. Words use the numeric order presented to the crypto
    /// streaming register (for example, `0x0011_2233`).
    pub fn encrypt_words_dma<TX: DmaChannelId, RX: DmaChannelId>(
        &mut self,
        tx: &mut DmaChannel<TX>,
        rx: &mut DmaChannel<RX>,
        input: &[u32],
        output: &mut [u32],
        timeout: u32,
    ) -> Result<(), DmaTransferError> {
        let words_per_block = A::BLOCK_BYTES / size_of::<u32>();
        if input.is_empty() || input.len() != output.len() || input.len() % words_per_block != 0 {
            return Err(DmaTransferError::InvalidLength);
        }

        self.cipher.crypto.configure(config::ModeSel::Ecb, false);
        let length = input
            .len()
            .checked_mul(size_of::<u32>())
            .ok_or(DmaTransferError::Dma(DmaError::TransferTooLong))?;
        let block = CryptoPeripheral::PTR.cast_mut().cast::<u8>();

        if let Err(error) = tx.start(input.as_ptr().cast(), block, length, crypto_dma_tx_config()) {
            return Err(error.into());
        }
        if let Err(error) = rx.start(
            block.cast_const(),
            output.as_mut_ptr().cast(),
            length,
            crypto_dma_rx_config(),
        ) {
            tx.stop();
            return Err(error.into());
        }

        for _ in 0..timeout {
            let tx_done = match tx.poll() {
                Ok(done) => done,
                Err(error) => {
                    rx.stop();
                    return Err(error.into());
                }
            };
            let rx_done = match rx.poll() {
                Ok(done) => done,
                Err(error) => {
                    tx.stop();
                    return Err(error.into());
                }
            };
            if tx_done && rx_done {
                self.cipher.crypto.wait_ready();
                return Ok(());
            }
            core::hint::spin_loop();
        }

        tx.stop();
        rx.stop();
        Err(DmaError::Timeout.into())
    }

    /// Encrypts words through DMA, sleeping until DMA interrupts signal that
    /// both channels have completed. The interrupt handler must clear the DMA
    /// and EPIC interrupt sources before returning.
    pub fn encrypt_words_dma_interrupt<TX, RX, WAIT>(
        &mut self,
        tx: &mut DmaChannel<TX>,
        rx: &mut DmaChannel<RX>,
        input: &[u32],
        output: &mut [u32],
        mut wait: WAIT,
    ) -> Result<(), DmaTransferError>
    where
        TX: DmaChannelId,
        RX: DmaChannelId,
        WAIT: FnMut(),
    {
        let words_per_block = A::BLOCK_BYTES / size_of::<u32>();
        if input.is_empty() || input.len() != output.len() || input.len() % words_per_block != 0 {
            return Err(DmaTransferError::InvalidLength);
        }

        self.cipher.crypto.configure(config::ModeSel::Ecb, false);
        let length = input
            .len()
            .checked_mul(size_of::<u32>())
            .ok_or(DmaTransferError::Dma(DmaError::TransferTooLong))?;
        let block = CryptoPeripheral::PTR.cast_mut().cast::<u8>();
        const LOCAL_IRQ: u32 = 1 << 27;

        tx.start(
            input.as_ptr().cast(),
            block,
            length,
            crypto_dma_tx_config() | LOCAL_IRQ,
        )?;
        if let Err(error) = rx.start(
            block.cast_const(),
            output.as_mut_ptr().cast(),
            length,
            crypto_dma_rx_config() | LOCAL_IRQ,
        ) {
            tx.stop();
            return Err(error.into());
        }

        loop {
            wait();
            let tx_done = tx.poll()?;
            let rx_done = rx.poll()?;
            if tx_done && rx_done {
                self.cipher.crypto.wait_ready();
                return Ok(());
            }
        }
    }
}

/// Cipher block chaining encryption mode.
pub struct CbcEncrypt<'a, A: Algorithm> {
    cipher: Cipher<'a, A>,
}

impl<A: Algorithm> CbcEncrypt<'_, A> {
    /// Encrypts a whole number of blocks in place.
    pub fn encrypt_blocks_in_place(&mut self, data: &mut [u8]) -> Result<(), Error> {
        process_full_blocks::<A>(self.cipher.crypto, data)
    }
}

/// Cipher block chaining decryption mode.
pub struct CbcDecrypt<'a, A: Algorithm> {
    cipher: Cipher<'a, A>,
}

impl<A: Algorithm> CbcDecrypt<'_, A> {
    /// Decrypts a whole number of blocks in place.
    pub fn decrypt_blocks_in_place(&mut self, data: &mut [u8]) -> Result<(), Error> {
        process_full_blocks::<A>(self.cipher.crypto, data)
    }
}

/// Counter mode. Encryption and decryption are the same operation.
pub struct Ctr<'a, A: Algorithm> {
    cipher: Cipher<'a, A>,
}

impl<A: Algorithm> Ctr<'_, A> {
    /// Applies the counter-mode keystream in place.
    ///
    /// A final partial block is zero-padded only for the hardware transaction;
    /// padding bytes are not added to `data`.
    pub fn apply_keystream(&mut self, data: &mut [u8]) {
        let mut chunks = data.chunks_exact_mut(A::BLOCK_BYTES);
        for block in &mut chunks {
            self.cipher.crypto.process_block(block);
        }

        let remainder = chunks.into_remainder();
        if !remainder.is_empty() {
            let mut block = [0u8; 16];
            block[..remainder.len()].copy_from_slice(remainder);
            self.cipher
                .crypto
                .process_block(&mut block[..A::BLOCK_BYTES]);
            remainder.copy_from_slice(&block[..remainder.len()]);
        }
    }
}

fn process_full_blocks<A: Algorithm>(crypto: &mut Crypto, data: &mut [u8]) -> Result<(), Error> {
    if data.len() % A::BLOCK_BYTES != 0 {
        return Err(Error::InvalidLength);
    }
    for block in data.chunks_exact_mut(A::BLOCK_BYTES) {
        crypto.process_block(block);
    }
    Ok(())
}

fn words(bytes: &[u8]) -> impl Iterator<Item = u32> + '_ {
    bytes
        .chunks_exact(4)
        .map(|chunk| u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]))
}

const fn crypto_dma_tx_config() -> u32 {
    const REQUEST: u32 = 2;
    (3 << 1) | (1 << 3) | (1 << 5) | (2 << 7) | (2 << 9) | (2 << 11) | (2 << 14) | (REQUEST << 21)
}

const fn crypto_dma_rx_config() -> u32 {
    const REQUEST: u32 = 2;
    (3 << 1) | (1 << 4) | (1 << 6) | (2 << 7) | (2 << 9) | (2 << 11) | (2 << 14) | (REQUEST << 17)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn byte_stream_is_packed_as_big_endian_words() {
        let input = [0x00, 0x11, 0x22, 0x33, 0xaa, 0xbb, 0xcc, 0xdd];
        let mut packed = words(&input);
        assert_eq!(packed.next(), Some(0x0011_2233));
        assert_eq!(packed.next(), Some(0xaabb_ccdd));
        assert_eq!(packed.next(), None);
    }

    #[test]
    fn algorithm_sizes_match_the_hardware() {
        assert_eq!(Aes::BLOCK_BYTES, 16);
        assert_eq!(KuznechikAlgorithm::BLOCK_BYTES, 16);
        assert_eq!(MagmaAlgorithm::BLOCK_BYTES, 8);
        assert_eq!(core::mem::size_of::<<Aes as Algorithm>::Key>(), 16);
        assert_eq!(
            core::mem::size_of::<<KuznechikAlgorithm as Algorithm>::Key>(),
            32
        );
        assert_eq!(
            core::mem::size_of::<<MagmaAlgorithm as Algorithm>::CtrIv>(),
            4
        );
    }
}
