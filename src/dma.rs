//! Direct memory access controller.

use core::marker::PhantomData;
use core::sync::atomic::{Ordering, fence};

use mik32_pac::{Dma as DmaPeripheral, Peripherals};

pub use channel::Id as ChannelId;

pub struct Dma {
    peripheral: DmaPeripheral,
}

pub struct Channels {
    pub channel1: Channel1,
    pub channel2: Channel2,
    pub channel3: Channel3,
    pub channel4: Channel4,
}

pub type Channel1 = Channel<channel::Ch1>;
pub type Channel2 = Channel<channel::Ch2>;
pub type Channel3 = Channel<channel::Ch3>;
pub type Channel4 = Channel<channel::Ch4>;

pub mod channel {
    mod sealed {
        pub trait Sealed {}
    }

    pub trait Id: sealed::Sealed {
        const INDEX: u8;
    }

    pub enum Ch1 {}
    pub enum Ch2 {}
    pub enum Ch3 {}
    pub enum Ch4 {}

    impl sealed::Sealed for Ch1 {}
    impl sealed::Sealed for Ch2 {}
    impl sealed::Sealed for Ch3 {}
    impl sealed::Sealed for Ch4 {}

    impl Id for Ch1 {
        const INDEX: u8 = 0;
    }

    impl Id for Ch2 {
        const INDEX: u8 = 1;
    }

    impl Id for Ch3 {
        const INDEX: u8 = 2;
    }

    impl Id for Ch4 {
        const INDEX: u8 = 3;
    }
}

pub struct Channel<CHANNEL: ChannelId> {
    _private: PhantomData<*mut CHANNEL>,
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

impl<CHANNEL: ChannelId> Channel<CHANNEL> {
    const fn new() -> Self {
        Self {
            _private: PhantomData,
        }
    }

    pub fn is_ready(&self) -> bool {
        let dma = unsafe { &*DmaPeripheral::ptr() };
        dma.status().read().channel_ready().bits() & (1 << CHANNEL::INDEX) != 0
    }

    pub fn stop(&mut self) {
        unsafe { channel_register(CHANNEL::INDEX, 3).write_volatile(0) };
        fence(Ordering::SeqCst);
    }

    pub(crate) fn transfer(
        &mut self,
        source: *const u8,
        destination: *mut u8,
        length: usize,
        config: u32,
        timeout: u32,
    ) -> Result<(), Error> {
        self.start(source, destination, length, config)?;

        for _ in 0..timeout {
            if self.poll()? {
                return Ok(());
            }
            core::hint::spin_loop();
        }

        self.stop();
        fence(Ordering::Acquire);
        Err(Error::Timeout)
    }

    pub(crate) fn start(
        &mut self,
        source: *const u8,
        destination: *mut u8,
        length: usize,
        config: u32,
    ) -> Result<(), Error> {
        if length == 0 {
            return Err(Error::EmptyBuffer);
        }
        if length > u32::MAX as usize {
            return Err(Error::TransferTooLong);
        }

        self.stop();
        fence(Ordering::Release);
        unsafe {
            channel_register(CHANNEL::INDEX, 0).write_volatile(destination as usize as u32);
            channel_register(CHANNEL::INDEX, 1).write_volatile(source as usize as u32);
            channel_register(CHANNEL::INDEX, 2).write_volatile(length as u32 - 1);
            channel_register(CHANNEL::INDEX, 3).write_volatile(config | 1);
        }

        Ok(())
    }

    pub(crate) fn poll(&mut self) -> Result<bool, Error> {
        let dma = unsafe { &*DmaPeripheral::ptr() };
        let status = dma.status().read();
        if status.bits() & (1 << (8 + CHANNEL::INDEX)) != 0 {
            self.stop();
            fence(Ordering::Acquire);
            return Err(Error::Bus);
        }

        let ready = status.channel_ready().bits() & (1 << CHANNEL::INDEX) != 0;
        if ready {
            fence(Ordering::Acquire);
        }
        Ok(ready)
    }
}

#[inline(always)]
unsafe fn channel_register(channel: u8, register: u8) -> *mut u32 {
    unsafe { (DmaPeripheral::ptr() as *mut u32).add(channel as usize * 4 + register as usize) }
}
