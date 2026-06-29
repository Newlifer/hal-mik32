//! GPIO
use core::cell::Cell;
use core::convert::Infallible;
use core::marker::PhantomData;
use critical_section::Mutex;
use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};

use mik32_pac::Peripherals;

const GPIO_IRQ_LINE_SHIFT: u32 = 4;
const GPIO_IRQ_LINE_MUX_MASK: u32 = 0b1111;
const GPIO_MODE_BIT_LEVEL: u32 = 1 << 0;
const GPIO_MODE_BIT_EDGE: u32 = 1 << 1;
const GPIO_MODE_BIT_ANY_EDGE: u32 = 1 << 2;

static CURRENT_IRQ_LINE_MUX: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

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

/// Analog mode (type state)
pub struct Analog;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum DriveStrength {
    Ma2 = 0b00,
    Ma4 = 0b01,
    Ma8 = 0b10,
}

impl DriveStrength {
    #[inline(always)]
    const fn bits(self) -> u32 {
        self as u32
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum InterruptLine {
    Line0 = 0,
    Line1 = 1,
    Line2 = 2,
    Line3 = 3,
    Line4 = 4,
    Line5 = 5,
    Line6 = 6,
    Line7 = 7,
}

impl InterruptLine {
    #[inline(always)]
    const fn index(self) -> u32 {
        self as u32
    }

    #[inline(always)]
    const fn mask(self) -> u32 {
        1u32 << self.index()
    }

    #[inline(always)]
    const fn mux_shift(self) -> u32 {
        GPIO_IRQ_LINE_SHIFT * self.index()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum InterruptMode {
    Low = 0,
    High = GPIO_MODE_BIT_LEVEL as u8,
    Falling = GPIO_MODE_BIT_EDGE as u8,
    Rising = (GPIO_MODE_BIT_LEVEL | GPIO_MODE_BIT_EDGE) as u8,
    Change = (GPIO_MODE_BIT_EDGE | GPIO_MODE_BIT_ANY_EDGE) as u8,
}

impl InterruptMode {
    #[inline(always)]
    const fn bits(self) -> u32 {
        self as u32
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LineConfig {
    Line0Port0_0 = 0 | (0 << GPIO_IRQ_LINE_SHIFT),
    Line0Port0_8 = 1 | (0 << GPIO_IRQ_LINE_SHIFT),
    Line0Port1_0 = 2 | (0 << GPIO_IRQ_LINE_SHIFT),
    Line0Port1_8 = 3 | (0 << GPIO_IRQ_LINE_SHIFT),
    Line0Port2_0 = 4 | (0 << GPIO_IRQ_LINE_SHIFT),
    Line0Port0_4 = 5 | (0 << GPIO_IRQ_LINE_SHIFT),
    Line0Port0_12 = 6 | (0 << GPIO_IRQ_LINE_SHIFT),
    Line0Port1_4 = 7 | (0 << GPIO_IRQ_LINE_SHIFT),
    Line0Port1_12 = 8 | (0 << GPIO_IRQ_LINE_SHIFT),
    Line0Port2_4 = 9 | (0 << GPIO_IRQ_LINE_SHIFT),

    Line1Port0_1 = 0 | (1 << GPIO_IRQ_LINE_SHIFT),
    Line1Port0_9 = 1 | (1 << GPIO_IRQ_LINE_SHIFT),
    Line1Port1_1 = 2 | (1 << GPIO_IRQ_LINE_SHIFT),
    Line1Port1_9 = 3 | (1 << GPIO_IRQ_LINE_SHIFT),
    Line1Port2_1 = 4 | (1 << GPIO_IRQ_LINE_SHIFT),
    Line1Port0_5 = 5 | (1 << GPIO_IRQ_LINE_SHIFT),
    Line1Port0_13 = 6 | (1 << GPIO_IRQ_LINE_SHIFT),
    Line1Port1_5 = 7 | (1 << GPIO_IRQ_LINE_SHIFT),
    Line1Port1_13 = 8 | (1 << GPIO_IRQ_LINE_SHIFT),
    Line1Port2_5 = 9 | (1 << GPIO_IRQ_LINE_SHIFT),

    Line2Port0_2 = 0 | (2 << GPIO_IRQ_LINE_SHIFT),
    Line2Port0_10 = 1 | (2 << GPIO_IRQ_LINE_SHIFT),
    Line2Port1_2 = 2 | (2 << GPIO_IRQ_LINE_SHIFT),
    Line2Port1_10 = 3 | (2 << GPIO_IRQ_LINE_SHIFT),
    Line2Port2_2 = 4 | (2 << GPIO_IRQ_LINE_SHIFT),
    Line2Port0_6 = 5 | (2 << GPIO_IRQ_LINE_SHIFT),
    Line2Port0_14 = 6 | (2 << GPIO_IRQ_LINE_SHIFT),
    Line2Port1_6 = 7 | (2 << GPIO_IRQ_LINE_SHIFT),
    Line2Port1_14 = 8 | (2 << GPIO_IRQ_LINE_SHIFT),
    Line2Port2_6 = 9 | (2 << GPIO_IRQ_LINE_SHIFT),

    Line3Port0_3 = 0 | (3 << GPIO_IRQ_LINE_SHIFT),
    Line3Port0_11 = 1 | (3 << GPIO_IRQ_LINE_SHIFT),
    Line3Port1_3 = 2 | (3 << GPIO_IRQ_LINE_SHIFT),
    Line3Port1_11 = 3 | (3 << GPIO_IRQ_LINE_SHIFT),
    Line3Port2_3 = 4 | (3 << GPIO_IRQ_LINE_SHIFT),
    Line3Port0_7 = 5 | (3 << GPIO_IRQ_LINE_SHIFT),
    Line3Port0_15 = 6 | (3 << GPIO_IRQ_LINE_SHIFT),
    Line3Port1_7 = 7 | (3 << GPIO_IRQ_LINE_SHIFT),
    Line3Port1_15 = 8 | (3 << GPIO_IRQ_LINE_SHIFT),
    Line3Port2_7 = 9 | (3 << GPIO_IRQ_LINE_SHIFT),

    Line4Port0_4 = 0 | (4 << GPIO_IRQ_LINE_SHIFT),
    Line4Port0_12 = 1 | (4 << GPIO_IRQ_LINE_SHIFT),
    Line4Port1_4 = 2 | (4 << GPIO_IRQ_LINE_SHIFT),
    Line4Port1_12 = 3 | (4 << GPIO_IRQ_LINE_SHIFT),
    Line4Port2_4 = 4 | (4 << GPIO_IRQ_LINE_SHIFT),
    Line4Port0_0 = 5 | (4 << GPIO_IRQ_LINE_SHIFT),
    Line4Port0_8 = 6 | (4 << GPIO_IRQ_LINE_SHIFT),
    Line4Port1_0 = 7 | (4 << GPIO_IRQ_LINE_SHIFT),
    Line4Port1_8 = 8 | (4 << GPIO_IRQ_LINE_SHIFT),
    Line4Port2_0 = 9 | (4 << GPIO_IRQ_LINE_SHIFT),

    Line5Port0_5 = 0 | (5 << GPIO_IRQ_LINE_SHIFT),
    Line5Port0_13 = 1 | (5 << GPIO_IRQ_LINE_SHIFT),
    Line5Port1_5 = 2 | (5 << GPIO_IRQ_LINE_SHIFT),
    Line5Port1_13 = 3 | (5 << GPIO_IRQ_LINE_SHIFT),
    Line5Port2_5 = 4 | (5 << GPIO_IRQ_LINE_SHIFT),
    Line5Port0_1 = 5 | (5 << GPIO_IRQ_LINE_SHIFT),
    Line5Port0_9 = 6 | (5 << GPIO_IRQ_LINE_SHIFT),
    Line5Port1_1 = 7 | (5 << GPIO_IRQ_LINE_SHIFT),
    Line5Port1_9 = 8 | (5 << GPIO_IRQ_LINE_SHIFT),
    Line5Port2_1 = 9 | (5 << GPIO_IRQ_LINE_SHIFT),

    Line6Port0_6 = 0 | (6 << GPIO_IRQ_LINE_SHIFT),
    Line6Port0_14 = 1 | (6 << GPIO_IRQ_LINE_SHIFT),
    Line6Port1_6 = 2 | (6 << GPIO_IRQ_LINE_SHIFT),
    Line6Port1_14 = 3 | (6 << GPIO_IRQ_LINE_SHIFT),
    Line6Port2_6 = 4 | (6 << GPIO_IRQ_LINE_SHIFT),
    Line6Port0_2 = 5 | (6 << GPIO_IRQ_LINE_SHIFT),
    Line6Port0_10 = 6 | (6 << GPIO_IRQ_LINE_SHIFT),
    Line6Port1_2 = 7 | (6 << GPIO_IRQ_LINE_SHIFT),
    Line6Port1_10 = 8 | (6 << GPIO_IRQ_LINE_SHIFT),
    Line6Port2_2 = 9 | (6 << GPIO_IRQ_LINE_SHIFT),

    Line7Port0_7 = 0 | (7 << GPIO_IRQ_LINE_SHIFT),
    Line7Port0_15 = 1 | (7 << GPIO_IRQ_LINE_SHIFT),
    Line7Port1_7 = 2 | (7 << GPIO_IRQ_LINE_SHIFT),
    Line7Port1_15 = 3 | (7 << GPIO_IRQ_LINE_SHIFT),
    Line7Port2_7 = 4 | (7 << GPIO_IRQ_LINE_SHIFT),
    Line7Port0_3 = 5 | (7 << GPIO_IRQ_LINE_SHIFT),
    Line7Port0_11 = 6 | (7 << GPIO_IRQ_LINE_SHIFT),
    Line7Port1_3 = 7 | (7 << GPIO_IRQ_LINE_SHIFT),
    Line7Port1_11 = 8 | (7 << GPIO_IRQ_LINE_SHIFT),
    Line7Port2_3 = 9 | (7 << GPIO_IRQ_LINE_SHIFT),
}

impl LineConfig {
    #[inline(always)]
    const fn bits(self) -> u32 {
        self as u32
    }

    #[inline(always)]
    const fn line(self) -> InterruptLine {
        match self.bits() >> GPIO_IRQ_LINE_SHIFT {
            0 => InterruptLine::Line0,
            1 => InterruptLine::Line1,
            2 => InterruptLine::Line2,
            3 => InterruptLine::Line3,
            4 => InterruptLine::Line4,
            5 => InterruptLine::Line5,
            6 => InterruptLine::Line6,
            _ => InterruptLine::Line7,
        }
    }

    #[inline(always)]
    const fn mux(self) -> u32 {
        self.bits() & GPIO_IRQ_LINE_MUX_MASK
    }
}

pub fn init_interrupt_line(config: LineConfig, mode: InterruptMode) {
    let p = unsafe { Peripherals::steal() };
    let line = config.line();
    let line_mask = line.mask();
    let mode_bits = mode.bits();

    critical_section::with(|cs| {
        let mux = CURRENT_IRQ_LINE_MUX.borrow(cs);
        let shift = line.mux_shift();
        let mut current = mux.get();

        current &= !(GPIO_IRQ_LINE_MUX_MASK << shift);
        current |= config.mux() << shift;

        mux.set(current);
        p.gpio_irq.line_mux().write(|w| unsafe { w.bits(current) });
    });

    if mode_bits & GPIO_MODE_BIT_LEVEL != 0 {
        p.gpio_irq
            .level_set()
            .write(|w| unsafe { w.bits(line_mask) });
    } else {
        p.gpio_irq
            .level_clear()
            .write(|w| unsafe { w.bits(line_mask) });
    }

    if mode_bits & GPIO_MODE_BIT_EDGE != 0 {
        p.gpio_irq.edge().write(|w| unsafe { w.bits(line_mask) });
    } else {
        p.gpio_irq.level().write(|w| unsafe { w.bits(line_mask) });
    }

    if mode_bits & GPIO_MODE_BIT_ANY_EDGE != 0 {
        p.gpio_irq
            .any_edge_set()
            .write(|w| unsafe { w.bits(line_mask) });
    } else {
        p.gpio_irq
            .any_edge_clear()
            .write(|w| unsafe { w.bits(line_mask) });
    }

    enable_interrupt_line(line);
}

pub fn deinit_interrupt_line(line: InterruptLine) {
    let p = unsafe { Peripherals::steal() };
    let line_mask = line.mask();

    disable_interrupt_line(line);

    critical_section::with(|cs| {
        let mux = CURRENT_IRQ_LINE_MUX.borrow(cs);
        let shift = line.mux_shift();
        let current = mux.get() & !(GPIO_IRQ_LINE_MUX_MASK << shift);

        mux.set(current);
        p.gpio_irq.line_mux().write(|w| unsafe { w.bits(current) });
    });

    p.gpio_irq.level().write(|w| unsafe { w.bits(line_mask) });
    p.gpio_irq
        .level_clear()
        .write(|w| unsafe { w.bits(line_mask) });
    p.gpio_irq
        .any_edge_clear()
        .write(|w| unsafe { w.bits(line_mask) });
}

#[inline(always)]
pub fn enable_interrupt_line(line: InterruptLine) {
    let p = unsafe { Peripherals::steal() };
    p.gpio_irq
        .enable_set()
        .write(|w| unsafe { w.bits(line.mask()) });
}

#[inline(always)]
pub fn disable_interrupt_line(line: InterruptLine) {
    let p = unsafe { Peripherals::steal() };
    p.gpio_irq
        .enable_clear()
        .write(|w| unsafe { w.bits(line.mask()) });
}

#[inline(always)]
pub fn line_interrupt_state(line: InterruptLine) -> bool {
    let p = unsafe { Peripherals::steal() };
    p.gpio_irq.interrupt().read().bits() & line.mask() != 0
}

#[inline(always)]
pub fn line_pin_state(line: InterruptLine) -> bool {
    let p = unsafe { Peripherals::steal() };
    p.gpio_irq.state().read().bits() & line.mask() != 0
}

#[inline(always)]
pub fn clear_interrupt(line: InterruptLine) {
    let p = unsafe { Peripherals::steal() };
    p.gpio_irq.clear().write(|w| unsafe { w.bits(line.mask()) });
}

#[inline(always)]
pub fn clear_interrupts() {
    let p = unsafe { Peripherals::steal() };
    p.gpio_irq.clear().write(|w| unsafe { w.bits(0xff) });
}

pub trait InterruptSource<const LINE: u8> {
    const LINE_CONFIG: LineConfig;
}

pub struct InterruptPin<PIN, const LINE: u8>
where
    PIN: InterruptSource<LINE>,
{
    pin: PIN,
}

impl<PIN, const LINE: u8> InterruptPin<PIN, LINE>
where
    PIN: InterruptSource<LINE>,
{
    pub fn new(pin: PIN, mode: InterruptMode) -> Self {
        init_interrupt_line(PIN::LINE_CONFIG, mode);
        Self { pin }
    }

    pub fn release(self) -> PIN {
        self.pin
    }

    pub const fn line_config(&self) -> LineConfig {
        PIN::LINE_CONFIG
    }

    pub const fn line(&self) -> InterruptLine {
        PIN::LINE_CONFIG.line()
    }

    pub fn enable(&self) {
        enable_interrupt_line(self.line());
    }

    pub fn disable(&self) {
        disable_interrupt_line(self.line());
    }

    pub fn is_pending(&self) -> bool {
        line_interrupt_state(self.line())
    }

    pub fn is_high(&self) -> bool {
        line_pin_state(self.line())
    }

    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    pub fn clear_interrupt(&self) {
        clear_interrupt(self.line());
    }
}

macro_rules! impl_interrupt_source {
    ($port:literal, $pin:literal, $line:literal, $config:ident) => {
        impl<MODE> InterruptSource<$line> for Pin<$port, $pin, MODE> {
            const LINE_CONFIG: LineConfig = LineConfig::$config;
        }
    };
}

impl_interrupt_source!(0, 0, 0, Line0Port0_0);
impl_interrupt_source!(0, 8, 0, Line0Port0_8);
impl_interrupt_source!(1, 0, 0, Line0Port1_0);
impl_interrupt_source!(1, 8, 0, Line0Port1_8);
impl_interrupt_source!(2, 0, 0, Line0Port2_0);
impl_interrupt_source!(0, 4, 0, Line0Port0_4);
impl_interrupt_source!(0, 12, 0, Line0Port0_12);
impl_interrupt_source!(1, 4, 0, Line0Port1_4);
impl_interrupt_source!(1, 12, 0, Line0Port1_12);
impl_interrupt_source!(2, 4, 0, Line0Port2_4);

impl_interrupt_source!(0, 1, 1, Line1Port0_1);
impl_interrupt_source!(0, 9, 1, Line1Port0_9);
impl_interrupt_source!(1, 1, 1, Line1Port1_1);
impl_interrupt_source!(1, 9, 1, Line1Port1_9);
impl_interrupt_source!(2, 1, 1, Line1Port2_1);
impl_interrupt_source!(0, 5, 1, Line1Port0_5);
impl_interrupt_source!(0, 13, 1, Line1Port0_13);
impl_interrupt_source!(1, 5, 1, Line1Port1_5);
impl_interrupt_source!(1, 13, 1, Line1Port1_13);
impl_interrupt_source!(2, 5, 1, Line1Port2_5);

impl_interrupt_source!(0, 2, 2, Line2Port0_2);
impl_interrupt_source!(0, 10, 2, Line2Port0_10);
impl_interrupt_source!(1, 2, 2, Line2Port1_2);
impl_interrupt_source!(1, 10, 2, Line2Port1_10);
impl_interrupt_source!(2, 2, 2, Line2Port2_2);
impl_interrupt_source!(0, 6, 2, Line2Port0_6);
impl_interrupt_source!(0, 14, 2, Line2Port0_14);
impl_interrupt_source!(1, 6, 2, Line2Port1_6);
impl_interrupt_source!(1, 14, 2, Line2Port1_14);
impl_interrupt_source!(2, 6, 2, Line2Port2_6);

impl_interrupt_source!(0, 3, 3, Line3Port0_3);
impl_interrupt_source!(0, 11, 3, Line3Port0_11);
impl_interrupt_source!(1, 3, 3, Line3Port1_3);
impl_interrupt_source!(1, 11, 3, Line3Port1_11);
impl_interrupt_source!(2, 3, 3, Line3Port2_3);
impl_interrupt_source!(0, 7, 3, Line3Port0_7);
impl_interrupt_source!(0, 15, 3, Line3Port0_15);
impl_interrupt_source!(1, 7, 3, Line3Port1_7);
impl_interrupt_source!(1, 15, 3, Line3Port1_15);
impl_interrupt_source!(2, 7, 3, Line3Port2_7);

impl_interrupt_source!(0, 4, 4, Line4Port0_4);
impl_interrupt_source!(0, 12, 4, Line4Port0_12);
impl_interrupt_source!(1, 4, 4, Line4Port1_4);
impl_interrupt_source!(1, 12, 4, Line4Port1_12);
impl_interrupt_source!(2, 4, 4, Line4Port2_4);
impl_interrupt_source!(0, 0, 4, Line4Port0_0);
impl_interrupt_source!(0, 8, 4, Line4Port0_8);
impl_interrupt_source!(1, 0, 4, Line4Port1_0);
impl_interrupt_source!(1, 8, 4, Line4Port1_8);
impl_interrupt_source!(2, 0, 4, Line4Port2_0);

impl_interrupt_source!(0, 5, 5, Line5Port0_5);
impl_interrupt_source!(0, 13, 5, Line5Port0_13);
impl_interrupt_source!(1, 5, 5, Line5Port1_5);
impl_interrupt_source!(1, 13, 5, Line5Port1_13);
impl_interrupt_source!(2, 5, 5, Line5Port2_5);
impl_interrupt_source!(0, 1, 5, Line5Port0_1);
impl_interrupt_source!(0, 9, 5, Line5Port0_9);
impl_interrupt_source!(1, 1, 5, Line5Port1_1);
impl_interrupt_source!(1, 9, 5, Line5Port1_9);
impl_interrupt_source!(2, 1, 5, Line5Port2_1);

impl_interrupt_source!(0, 6, 6, Line6Port0_6);
impl_interrupt_source!(0, 14, 6, Line6Port0_14);
impl_interrupt_source!(1, 6, 6, Line6Port1_6);
impl_interrupt_source!(1, 14, 6, Line6Port1_14);
impl_interrupt_source!(2, 6, 6, Line6Port2_6);
impl_interrupt_source!(0, 2, 6, Line6Port0_2);
impl_interrupt_source!(0, 10, 6, Line6Port0_10);
impl_interrupt_source!(1, 2, 6, Line6Port1_2);
impl_interrupt_source!(1, 10, 6, Line6Port1_10);
impl_interrupt_source!(2, 2, 6, Line6Port2_2);

impl_interrupt_source!(0, 7, 7, Line7Port0_7);
impl_interrupt_source!(0, 15, 7, Line7Port0_15);
impl_interrupt_source!(1, 7, 7, Line7Port1_7);
impl_interrupt_source!(1, 15, 7, Line7Port1_15);
impl_interrupt_source!(2, 7, 7, Line7Port2_7);
impl_interrupt_source!(0, 3, 7, Line7Port0_3);
impl_interrupt_source!(0, 11, 7, Line7Port0_11);
impl_interrupt_source!(1, 3, 7, Line7Port1_3);
impl_interrupt_source!(1, 11, 7, Line7Port1_11);
impl_interrupt_source!(2, 3, 7, Line7Port2_3);

pub struct Pin<const P: u8, const N: u8, MODE = Floating> {
    _mode: PhantomData<MODE>,
}

impl<const P: u8, const N: u8, MODE> Pin<P, N, MODE> {
    pub const fn new() -> Self {
        Self { _mode: PhantomData }
    }

    pub fn set_drive_strength(&mut self, drive_strength: DriveStrength) {
        let p = unsafe { Peripherals::steal() };
        set_drive_strength::<P, N>(&p, drive_strength);
    }

    pub fn with_drive_strength(mut self, drive_strength: DriveStrength) -> Self {
        self.set_drive_strength(drive_strength);
        self
    }
}

pub trait OutputPermitted {}
pub trait SerialPermitted {}
pub trait TimerSerialPermitted {}
pub trait AnalogPermitted {}
pub trait InputMode {}

impl InputMode for Floating {}
impl InputMode for PullDown {}
impl InputMode for PullUp {}

impl<const P: u8, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: InputMode,
{
    pub fn into_interrupt_pin<const LINE: u8>(self, mode: InterruptMode) -> InterruptPin<Self, LINE>
    where
        Self: InterruptSource<LINE>,
    {
        InterruptPin::new(self, mode)
    }
}

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
fn set_drive_strength<const P: u8, const N: u8>(p: &Peripherals, drive_strength: DriveStrength) {
    let shift = 2 * N;
    let mask = 0b11u32 << shift;
    let value = drive_strength.bits() << shift;

    match P {
        0 => p
            .pad_config
            .pad0_ds()
            .modify(|r, w| unsafe { w.bits((r.bits() & !mask) | value) }),
        1 => p
            .pad_config
            .pad1_ds()
            .modify(|r, w| unsafe { w.bits((r.bits() & !mask) | value) }),
        2 => p
            .pad_config
            .pad2_ds()
            .modify(|r, w| unsafe { w.bits((r.bits() & !mask) | value) }),
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
        set_pull::<P, N>(&p, 0);

        Pin::new()
    }

    pub fn into_serial_port_pull_up(self) -> Pin<P, N, Func2Mode>
    where
        Pin<P, N>: SerialPermitted,
    {
        let p = unsafe { Peripherals::steal() };

        set_alternate_function::<P, N>(&p, 0b01);
        set_pull::<P, N>(&p, 1);

        Pin::new()
    }

    pub fn into_timer_serial_port(self) -> Pin<P, N, Func3Mode>
    where
        Pin<P, N>: TimerSerialPermitted,
    {
        let p = unsafe { Peripherals::steal() };
        set_alternate_function::<P, N>(&p, 0b10);
        set_pull::<P, N>(&p, 0);
        Pin::new()
    }

    pub fn into_analog(self) -> Pin<P, N, Analog>
    where
        Pin<P, N>: AnalogPermitted,
    {
        let p = unsafe { Peripherals::steal() };
        set_alternate_function::<P, N>(&p, 0b11);
        set_pull::<P, N>(&p, 0);
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
    use super::{AnalogPermitted, OutputPermitted, Pin, SerialPermitted, TimerSerialPermitted};

    pub type Pin00 = Pin<0, 0>;
    impl OutputPermitted for Pin<0, 0> {}
    impl SerialPermitted for Pin<0, 0> {}
    impl TimerSerialPermitted for Pin<0, 0> {}

    pub type Pin01 = Pin<0, 1>;
    impl OutputPermitted for Pin<0, 1> {}
    impl SerialPermitted for Pin<0, 1> {}
    impl TimerSerialPermitted for Pin<0, 1> {}

    pub type Pin02 = Pin<0, 2>;
    impl OutputPermitted for Pin<0, 2> {}
    impl SerialPermitted for Pin<0, 2> {}
    impl TimerSerialPermitted for Pin<0, 2> {}
    impl AnalogPermitted for Pin<0, 2> {}

    pub type Pin03 = Pin<0, 3>;
    impl OutputPermitted for Pin<0, 3> {}
    impl SerialPermitted for Pin<0, 3> {}
    impl TimerSerialPermitted for Pin<0, 3> {}

    pub type Pin04 = Pin<0, 4>;
    impl OutputPermitted for Pin<0, 4> {}
    impl SerialPermitted for Pin<0, 4> {}
    impl TimerSerialPermitted for Pin<0, 4> {}
    impl AnalogPermitted for Pin<0, 4> {}

    // GPIO
    // UART 0 RX
    pub type Pin05 = Pin<0, 5>;
    impl OutputPermitted for Pin<0, 5> {}
    impl SerialPermitted for Pin<0, 5> {}
    impl TimerSerialPermitted for Pin<0, 5> {}

    // GPIO
    // UART 0 TX
    pub type Pin06 = Pin<0, 6>;
    impl OutputPermitted for Pin<0, 6> {}
    impl SerialPermitted for Pin<0, 6> {}
    impl TimerSerialPermitted for Pin<0, 6> {}

    // GPIO
    // UART 0 NCTS
    pub type Pin07 = Pin<0, 7>;
    impl OutputPermitted for Pin<0, 7> {}
    impl SerialPermitted for Pin<0, 7> {}
    impl TimerSerialPermitted for Pin<0, 7> {}
    impl AnalogPermitted for Pin<0, 7> {}

    // GPIO
    // UART 0 NRTS
    pub type Pin08 = Pin<0, 8>;
    impl OutputPermitted for Pin<0, 8> {}
    impl SerialPermitted for Pin<0, 8> {}
    impl TimerSerialPermitted for Pin<0, 8> {}

    pub type Pin09 = Pin<0, 9>;
    impl OutputPermitted for Pin<0, 9> {}
    impl SerialPermitted for Pin<0, 9> {}
    impl TimerSerialPermitted for Pin<0, 9> {}
    impl AnalogPermitted for Pin<0, 9> {}

    pub type Pin10 = Pin<0, 10>;
    impl OutputPermitted for Pin<0, 10> {}
    impl SerialPermitted for Pin<0, 10> {}
    impl TimerSerialPermitted for Pin<0, 10> {}

    pub type Pin11 = Pin<0, 11>;
    impl OutputPermitted for Pin<0, 11> {}
    impl TimerSerialPermitted for Pin<0, 11> {}
    impl AnalogPermitted for Pin<0, 11> {}

    pub type Pin12 = Pin<0, 12>;
    impl OutputPermitted for Pin<0, 12> {}
    impl TimerSerialPermitted for Pin<0, 12> {}

    pub type Pin13 = Pin<0, 13>;
    impl OutputPermitted for Pin<0, 13> {}
    impl TimerSerialPermitted for Pin<0, 13> {}
    impl AnalogPermitted for Pin<0, 13> {}

    pub type Pin14 = Pin<0, 14>;
    impl OutputPermitted for Pin<0, 14> {}

    pub type Pin15 = Pin<0, 15>;
    impl OutputPermitted for Pin<0, 15> {}
}

pub mod port_1 {
    use super::{AnalogPermitted, OutputPermitted, Pin, SerialPermitted, TimerSerialPermitted};

    pub type Pin00 = Pin<1, 0>;
    impl OutputPermitted for Pin<1, 0> {}
    impl SerialPermitted for Pin<1, 0> {}
    impl TimerSerialPermitted for Pin<1, 0> {}

    pub type Pin01 = Pin<1, 1>;
    impl OutputPermitted for Pin<1, 1> {}
    impl SerialPermitted for Pin<1, 1> {}
    impl TimerSerialPermitted for Pin<1, 1> {}

    pub type Pin02 = Pin<1, 2>;
    impl OutputPermitted for Pin<1, 2> {}
    impl SerialPermitted for Pin<1, 2> {}
    impl TimerSerialPermitted for Pin<1, 2> {}

    pub type Pin03 = Pin<1, 3>;
    impl OutputPermitted for Pin<1, 3> {}
    impl SerialPermitted for Pin<1, 3> {}
    impl TimerSerialPermitted for Pin<1, 3> {}

    pub type Pin04 = Pin<1, 4>;
    impl OutputPermitted for Pin<1, 4> {}
    impl SerialPermitted for Pin<1, 4> {}
    impl TimerSerialPermitted for Pin<1, 4> {}

    pub type Pin05 = Pin<1, 5>;
    impl OutputPermitted for Pin<1, 5> {}
    impl SerialPermitted for Pin<1, 5> {}
    impl TimerSerialPermitted for Pin<1, 5> {}
    impl AnalogPermitted for Pin<1, 5> {}

    pub type Pin06 = Pin<1, 6>;
    impl OutputPermitted for Pin<1, 6> {}
    impl SerialPermitted for Pin<1, 6> {}
    impl TimerSerialPermitted for Pin<1, 6> {}

    pub type Pin07 = Pin<1, 7>;
    impl OutputPermitted for Pin<1, 7> {}
    impl SerialPermitted for Pin<1, 7> {}
    impl AnalogPermitted for Pin<1, 7> {}

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
    impl AnalogPermitted for Pin<1, 10> {}

    pub type Pin11 = Pin<1, 11>;
    impl OutputPermitted for Pin<1, 11> {}
    impl SerialPermitted for Pin<1, 11> {}
    impl AnalogPermitted for Pin<1, 11> {}

    pub type Pin12 = Pin<1, 12>;
    impl OutputPermitted for Pin<1, 12> {}
    impl SerialPermitted for Pin<1, 12> {}
    impl TimerSerialPermitted for Pin<1, 12> {}
    impl AnalogPermitted for Pin<1, 12> {}

    pub type Pin13 = Pin<1, 13>;
    impl OutputPermitted for Pin<1, 13> {}
    impl SerialPermitted for Pin<1, 13> {}
    impl TimerSerialPermitted for Pin<1, 13> {}
    impl AnalogPermitted for Pin<1, 13> {}

    pub type Pin14 = Pin<1, 14>;
    impl OutputPermitted for Pin<1, 14> {}
    impl SerialPermitted for Pin<1, 14> {}
    impl TimerSerialPermitted for Pin<1, 14> {}

    pub type Pin15 = Pin<1, 15>;
    impl OutputPermitted for Pin<1, 15> {}
    impl SerialPermitted for Pin<1, 15> {}
    impl TimerSerialPermitted for Pin<1, 15> {}
}

pub mod port_2 {
    use super::{OutputPermitted, Pin, SerialPermitted, TimerSerialPermitted};

    pub type Pin00 = Pin<2, 0>;
    impl OutputPermitted for Pin<2, 0> {}
    impl SerialPermitted for Pin<2, 0> {}
    impl TimerSerialPermitted for Pin<2, 0> {}

    pub type Pin01 = Pin<2, 1>;
    impl OutputPermitted for Pin<2, 1> {}
    impl SerialPermitted for Pin<2, 1> {}
    impl TimerSerialPermitted for Pin<2, 1> {}

    pub type Pin02 = Pin<2, 2>;
    impl OutputPermitted for Pin<2, 2> {}
    impl SerialPermitted for Pin<2, 2> {}
    impl TimerSerialPermitted for Pin<2, 2> {}

    pub type Pin03 = Pin<2, 3>;
    impl OutputPermitted for Pin<2, 3> {}
    impl SerialPermitted for Pin<2, 3> {}
    impl TimerSerialPermitted for Pin<2, 3> {}

    pub type Pin04 = Pin<2, 4>;
    impl OutputPermitted for Pin<2, 4> {}
    impl SerialPermitted for Pin<2, 4> {}

    pub type Pin05 = Pin<2, 5>;
    impl OutputPermitted for Pin<2, 5> {}
    impl SerialPermitted for Pin<2, 5> {}

    pub type Pin06 = Pin<2, 6>;
    impl OutputPermitted for Pin<2, 6> {}
    impl SerialPermitted for Pin<2, 6> {}
    impl TimerSerialPermitted for Pin<2, 6> {}

    pub type Pin07 = Pin<2, 7>;
    impl OutputPermitted for Pin<2, 7> {}
    impl TimerSerialPermitted for Pin<2, 7> {}
}
