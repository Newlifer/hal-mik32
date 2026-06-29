//! GPIO
use core::convert::Infallible;
use core::marker::PhantomData;
use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};

use mik32_pac::Peripherals;

/// Floating input (type state)
pub struct Floating;

/// Pulled down input (type state)
pub struct PullDown;

/// Pulled up input (type state)
pub struct PullUp;

/// Output mode (type state)
pub struct Output;

/// Func2Mode mode (type state)
pub struct Func2Mode;
pub struct Func3Mode;

pub struct Pin<const P: u8, const N: u8, MODE = Floating> {
    _mode: PhantomData<MODE>,
}

impl<const P: u8, const N: u8, MODE> Pin<P, N, MODE> {
    pub const fn new() -> Self {
        Self { _mode: PhantomData }
    }
}

pub trait OutputPermitted {}
pub trait SerialPermitted {}
pub trait TimerSerialPermitted {}
pub trait InputMode {}

impl InputMode for Floating {}
impl InputMode for PullDown {}
impl InputMode for PullUp {}

#[inline(always)]
fn set_gpio_function<const P: u8, const N: u8>(p: &Peripherals) {
    let shift = 2 * N;
    let mask = 0b11u32 << shift;

    match P {
        0 => p
            .pad_config
            .pad0_cfg()
            .modify(|r, w| unsafe { w.bits(r.bits() & !mask) }),
        1 => p
            .pad_config
            .pad1_cfg()
            .modify(|r, w| unsafe { w.bits(r.bits() & !mask) }),
        2 => p
            .pad_config
            .pad2_cfg()
            .modify(|r, w| unsafe { w.bits(r.bits() & !mask) }),
        _ => panic!("Invalid port number {}", P),
    };
}

#[inline(always)]
fn set_alternate_function<const P: u8, const N: u8>(p: &Peripherals, function: u32) {
    let shift = 2 * N;
    let mask = 0b11u32 << shift;
    let value = function << shift;

    match P {
        0 => p
            .pad_config
            .pad0_cfg()
            .modify(|r, w| unsafe { w.bits((r.bits() & !mask) | value) }),
        1 => p
            .pad_config
            .pad1_cfg()
            .modify(|r, w| unsafe { w.bits((r.bits() & !mask) | value) }),
        2 => p
            .pad_config
            .pad2_cfg()
            .modify(|r, w| unsafe { w.bits((r.bits() & !mask) | value) }),
        _ => panic!("Invalid port number {}", P),
    };
}

#[inline(always)]
fn set_pull<const P: u8, const N: u8>(p: &Peripherals, pull: u32) {
    let shift = 2 * N;
    let mask = 0b11u32 << shift;

    match P {
        0 => p
            .pad_config
            .pad0_pupd()
            .modify(|r, w| unsafe { w.bits((r.bits() & !mask) | (pull << shift)) }),
        1 => p
            .pad_config
            .pad1_pupd()
            .modify(|r, w| unsafe { w.bits((r.bits() & !mask) | (pull << shift)) }),
        2 => p
            .pad_config
            .pad2_pupd()
            .modify(|r, w| unsafe { w.bits((r.bits() & !mask) | (pull << shift)) }),
        _ => panic!("Invalid port number {}", P),
    };
}

#[inline(always)]
fn set_direction_in<const P: u8, const N: u8>(p: &Peripherals) {
    match P {
        0 => p
            .gpio16_0
            .direction_in()
            .modify(|r, w| unsafe { w.bits(r.bits() | (1u32 << N)) }),
        1 => p
            .gpio16_1
            .direction_in()
            .modify(|r, w| unsafe { w.bits(r.bits() | (1u32 << N)) }),
        2 => p
            .gpio8_2
            .direction_in()
            .modify(|r, w| unsafe { w.bits(r.bits() | (1u32 << N)) }),
        _ => panic!("Invalid port number {}", P),
    };
}

impl<const P: u8, const N: u8, MODE> Pin<P, N, MODE> {
    pub fn into_output(self) -> Pin<P, N, Output>
    where
        Pin<P, N>: OutputPermitted,
    {
        let p = unsafe { Peripherals::steal() };
        set_gpio_function::<P, N>(&p);
        set_pull::<P, N>(&p, 0);

        match P {
            0 => {
                p.gpio16_0
                    .direction_out()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1u32 << N)) });
            }
            1 => {
                p.gpio16_1
                    .direction_out()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1u32 << N)) });
            }
            2 => {
                p.gpio8_2
                    .direction_out()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1u32 << N)) });
            }
            _ => panic!("Invalid port number {}", P),
        }

        Pin::new()
    }

    pub fn into_floating_input(self) -> Pin<P, N, Floating> {
        let p = unsafe { Peripherals::steal() };
        set_gpio_function::<P, N>(&p);
        set_pull::<P, N>(&p, 0);
        set_direction_in::<P, N>(&p);

        Pin::new()
    }

    pub fn into_pull_up_input(self) -> Pin<P, N, PullUp> {
        let p = unsafe { Peripherals::steal() };
        set_gpio_function::<P, N>(&p);
        set_pull::<P, N>(&p, 1);
        set_direction_in::<P, N>(&p);

        Pin::new()
    }

    pub fn into_pull_down_input(self) -> Pin<P, N, PullDown> {
        let p = unsafe { Peripherals::steal() };
        set_gpio_function::<P, N>(&p);
        set_pull::<P, N>(&p, 2);
        set_direction_in::<P, N>(&p);

        Pin::new()
    }

    pub fn into_serial_port(self) -> Pin<P, N, Func2Mode>
    where
        Pin<P, N>: SerialPermitted,
    {
        let p = unsafe { Peripherals::steal() };

        set_alternate_function::<P, N>(&p, 0b01);

        // // FIXME gpio16_0 – нужно выбирать в зависимости от P
        // p.gpio16_0.direction_in().write(|w| unsafe { w.bits(1 << N) });

        // let mask = 0b11 << 2 * N;
        // let value = 0b01 << 2 * N;

        // p.pad_config.pad0_cfg().modify(
        //     |r, w|
        //     unsafe { w.bits((r.bits() & !mask) | value) }
        // );

        // p.gpio16_0.func_sel().modify(
        //     |r, w|
        //     unsafe { w.bits(r.bits() | (1 << N)) }
        // );

        Pin::new()
    }

    pub fn into_timer_serial_port(self) -> Pin<P, N, Func3Mode>
    where
        Pin<P, N>: TimerSerialPermitted,
    {
        let p = unsafe { Peripherals::steal() };
        set_alternate_function::<P, N>(&p, 0b10);
        Pin::new()
    }
}

impl<const P: u8, const N: u8, MODE> ErrorType for Pin<P, N, MODE> {
    type Error = Infallible;
}

/// Single digital push-pull output pin.
impl<const P: u8, const N: u8> OutputPin for Pin<P, N, Output> {
    /// Drives the pin high.
    #[inline(always)]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        let p = unsafe { Peripherals::steal() };

        match P {
            0 => {
                p.gpio16_0.set().write(|w| unsafe { w.bits(1u32 << N) });
            }
            1 => {
                p.gpio16_1.set().write(|w| unsafe { w.bits(1u32 << N) });
            }
            2 => {
                p.gpio8_2.set().write(|w| unsafe { w.bits(1u32 << N) });
            }
            _ => panic!("Invalid port number {}", P),
        }

        Ok(())
    }

    /// Drives the pin low.
    #[inline(always)]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        let p = unsafe { Peripherals::steal() };

        match P {
            0 => {
                p.gpio16_0.clear().write(|w| unsafe { w.bits(1u32 << N) });
            }
            1 => {
                p.gpio16_1.clear().write(|w| unsafe { w.bits(1u32 << N) });
            }
            2 => {
                p.gpio8_2.clear().write(|w| unsafe { w.bits(1u32 << N) });
            }
            _ => panic!("Invalid port number {}", P),
        }

        Ok(())
    }
}

impl<const P: u8, const N: u8> StatefulOutputPin for Pin<P, N, Output> {
    #[inline(always)]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        let p = unsafe { Peripherals::steal() };
        let mask = 1u32 << N;

        let is_set = match P {
            0 => p.gpio16_0.output().read().bits() & mask != 0,
            1 => p.gpio16_1.output().read().bits() & mask != 0,
            2 => p.gpio8_2.output().read().bits() & mask != 0,
            _ => panic!("Invalid port number {}", P),
        };

        Ok(is_set)
    }

    #[inline(always)]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        self.is_set_high().map(|is_high| !is_high)
    }
}

impl<const P: u8, const N: u8, MODE> InputPin for Pin<P, N, MODE>
where
    MODE: InputMode,
{
    #[inline(always)]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        let p = unsafe { Peripherals::steal() };
        let mask = 1u32 << N;

        let is_set = match P {
            0 => p.gpio16_0.state().read().bits() & mask != 0,
            1 => p.gpio16_1.state().read().bits() & mask != 0,
            2 => p.gpio8_2.state().read().bits() & mask != 0,
            _ => panic!("Invalid port number {}", P),
        };

        Ok(is_set)
    }

    #[inline(always)]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        self.is_high().map(|is_high| !is_high)
    }
}

pub mod port_0 {
    use super::{OutputPermitted, Pin, SerialPermitted};

    pub type Pin00 = Pin<0, 0>;
    impl OutputPermitted for Pin<0, 0> {}

    pub type Pin01 = Pin<0, 1>;
    impl OutputPermitted for Pin<0, 1> {}

    pub type Pin02 = Pin<0, 2>;
    impl OutputPermitted for Pin<0, 2> {}

    pub type Pin03 = Pin<0, 3>;
    impl OutputPermitted for Pin<0, 3> {}

    pub type Pin04 = Pin<0, 4>;
    impl OutputPermitted for Pin<0, 4> {}

    // GPIO
    // UART 0 RX
    pub type Pin05 = Pin<0, 5>;
    impl OutputPermitted for Pin<0, 5> {}
    impl SerialPermitted for Pin<0, 5> {}

    // GPIO
    // UART 0 TX
    pub type Pin06 = Pin<0, 6>;
    impl OutputPermitted for Pin<0, 6> {}
    impl SerialPermitted for Pin<0, 6> {}

    // GPIO
    // UART 0 NCTS
    pub type Pin07 = Pin<0, 7>;
    impl OutputPermitted for Pin<0, 7> {}
    impl SerialPermitted for Pin<0, 7> {}

    // GPIO
    // UART 0 NRTS
    pub type Pin08 = Pin<0, 8>;
    impl OutputPermitted for Pin<0, 8> {}
    impl SerialPermitted for Pin<0, 8> {}

    pub type Pin09 = Pin<0, 9>;
    impl OutputPermitted for Pin<0, 9> {}

    pub type Pin10 = Pin<0, 10>;
    impl OutputPermitted for Pin<0, 10> {}

    pub type Pin11 = Pin<0, 11>;
    impl OutputPermitted for Pin<0, 11> {}

    pub type Pin12 = Pin<0, 12>;
    impl OutputPermitted for Pin<0, 12> {}

    pub type Pin13 = Pin<0, 13>;
    impl OutputPermitted for Pin<0, 13> {}

    pub type Pin14 = Pin<0, 14>;
    impl OutputPermitted for Pin<0, 14> {}

    pub type Pin15 = Pin<0, 15>;
    impl OutputPermitted for Pin<0, 15> {}
}

pub mod port_1 {
    use super::{OutputPermitted, Pin, SerialPermitted, TimerSerialPermitted};

    pub type Pin00 = Pin<1, 0>;
    impl OutputPermitted for Pin<1, 0> {}

    pub type Pin01 = Pin<1, 1>;
    impl OutputPermitted for Pin<1, 1> {}

    pub type Pin02 = Pin<1, 2>;
    impl OutputPermitted for Pin<1, 2> {}

    pub type Pin03 = Pin<1, 3>;
    impl OutputPermitted for Pin<1, 3> {}

    pub type Pin04 = Pin<1, 4>;
    impl OutputPermitted for Pin<1, 4> {}

    pub type Pin05 = Pin<1, 5>;
    impl OutputPermitted for Pin<1, 5> {}
    impl TimerSerialPermitted for Pin<1, 5> {}

    pub type Pin06 = Pin<1, 6>;
    impl OutputPermitted for Pin<1, 6> {}
    impl TimerSerialPermitted for Pin<1, 6> {}

    pub type Pin07 = Pin<1, 7>;
    impl OutputPermitted for Pin<1, 7> {}

    // GPIO
    // UART 1 RX
    pub type Pin08 = Pin<1, 8>;
    impl OutputPermitted for Pin<1, 8> {}
    impl SerialPermitted for Pin<1, 8> {}

    // GPIO
    // UART 1 TX
    pub type Pin09 = Pin<1, 9>;
    impl OutputPermitted for Pin<1, 9> {}
    impl SerialPermitted for Pin<1, 9> {}

    pub type Pin10 = Pin<1, 10>;
    impl OutputPermitted for Pin<1, 10> {}
    impl SerialPermitted for Pin<1, 10> {}

    pub type Pin11 = Pin<1, 11>;
    impl OutputPermitted for Pin<1, 11> {}
    impl SerialPermitted for Pin<1, 11> {}

    pub type Pin12 = Pin<1, 12>;
    impl OutputPermitted for Pin<1, 12> {}
    impl TimerSerialPermitted for Pin<1, 12> {}

    pub type Pin13 = Pin<1, 13>;
    impl OutputPermitted for Pin<1, 13> {}
    impl TimerSerialPermitted for Pin<1, 13> {}

    pub type Pin14 = Pin<1, 14>;
    impl OutputPermitted for Pin<1, 14> {}
    impl TimerSerialPermitted for Pin<1, 14> {}

    pub type Pin15 = Pin<1, 15>;
    impl OutputPermitted for Pin<1, 15> {}
    impl TimerSerialPermitted for Pin<1, 15> {}
}

pub mod port_2 {
    use super::{OutputPermitted, Pin, TimerSerialPermitted};

    pub type Pin00 = Pin<2, 0>;
    impl OutputPermitted for Pin<2, 0> {}
    impl TimerSerialPermitted for Pin<2, 0> {}

    pub type Pin01 = Pin<2, 1>;
    impl OutputPermitted for Pin<2, 1> {}
    impl TimerSerialPermitted for Pin<2, 1> {}

    pub type Pin02 = Pin<2, 2>;
    impl OutputPermitted for Pin<2, 2> {}
    impl TimerSerialPermitted for Pin<2, 2> {}

    pub type Pin03 = Pin<2, 3>;
    impl OutputPermitted for Pin<2, 3> {}
    impl TimerSerialPermitted for Pin<2, 3> {}

    pub type Pin04 = Pin<2, 4>;
    impl OutputPermitted for Pin<2, 4> {}

    pub type Pin05 = Pin<2, 5>;
    impl OutputPermitted for Pin<2, 5> {}

    pub type Pin06 = Pin<2, 6>;
    impl OutputPermitted for Pin<2, 6> {}
    impl TimerSerialPermitted for Pin<2, 6> {}

    pub type Pin07 = Pin<2, 7>;
    impl OutputPermitted for Pin<2, 7> {}
    impl TimerSerialPermitted for Pin<2, 7> {}
}
