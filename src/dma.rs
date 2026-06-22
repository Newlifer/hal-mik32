//! Direct memory access controller.

use core::marker::PhantomData;
use core::sync::atomic::{fence, Ordering};

use mik32_pac::{Dma as DmaPeripheral, Peripherals};

pub struct Dma {
    peripheral: DmaPeripheral,
}

pub struct Channels {
    pub channel1: Channel<0>,
    pub channel2: Channel<1>,
    pub channel3: Channel<2>,
    pub channel4: Channel<3>,
}

pub struct Channel<const N: u8> {
    _private: PhantomData<*mut ()>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    EmptyBuffer,
    TransferTooLong,
    Timeout,
    Bus,
    InvalidChannel,
}

impl Dma {
    pub fn new(peripheral: DmaPeripheral) -> Self {
        let p = unsafe { Peripherals::steal() };
        p.pm.clk_ahb_set().modify(|_, w| w.dma().enable());
        Self { peripheral }
    }

    pub fn split(self) -> Channels {
        let _ = self.peripheral;
        Channels {
            channel1: Channel::new(),
            channel2: Channel::new(),
            channel3: Channel::new(),
            channel4: Channel::new(),
        }
    }
}

impl<const N: u8> Channel<N> {
    const fn new() -> Self {
        Self {
            _private: PhantomData,
        }
    }

    pub fn is_ready(&self) -> bool {
        if N >= 4 {
            return false;
        }
        let dma = unsafe { &*DmaPeripheral::ptr() };
        dma.status().read().channel_ready().bits() & (1 << N) != 0
    }

    pub fn stop(&mut self) {
        if N < 4 {
            unsafe { channel_register(N, 3).write_volatile(0) };
        }
    }

    pub(crate) fn transfer(
        &mut self,
        source: *const u8,
        destination: *mut u8,
        length: usize,
        config: u32,
        timeout: u32,
    ) -> Result<(), Error> {
        if N >= 4 {
            return Err(Error::InvalidChannel);
        }
        if length == 0 {
            return Err(Error::EmptyBuffer);
        }
        if length > u32::MAX as usize {
            return Err(Error::TransferTooLong);
        }

        self.stop();
        fence(Ordering::Release);
        unsafe {
            channel_register(N, 0).write_volatile(destination as usize as u32);
            channel_register(N, 1).write_volatile(source as usize as u32);
            channel_register(N, 2).write_volatile(length as u32 - 1);
            channel_register(N, 3).write_volatile(config | 1);
        }

        for _ in 0..timeout {
            let dma = unsafe { &*DmaPeripheral::ptr() };
            let status = dma.status().read();
            if status.bits() & (1 << (8 + N)) != 0 {
                self.stop();
                fence(Ordering::Acquire);
                return Err(Error::Bus);
            }
            if status.channel_ready().bits() & (1 << N) != 0 {
                fence(Ordering::Acquire);
                return Ok(());
            }
            core::hint::spin_loop();
        }

        self.stop();
        fence(Ordering::Acquire);
        Err(Error::Timeout)
    }
}

#[inline(always)]
unsafe fn channel_register(channel: u8, register: u8) -> *mut u32 {
    unsafe { (DmaPeripheral::ptr() as *mut u32).add(channel as usize * 4 + register as usize) }
}
