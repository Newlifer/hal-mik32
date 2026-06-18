#![no_std]
#![no_main]

use core::fmt::Write;
use core::panic::PanicInfo;

use hal_mik32::gpio::port_1::{Pin08, Pin09};
use hal_mik32::rcc::RCC;
use hal_mik32::usart::{Config, Serial};
use mik32_pac::Peripherals;

const MESSAGE_DELAY_SPINS: u32 = 500_000;

#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();

    let rcc_config = RCC::default();
    RCC::init(&rcc_config);

    peripherals
        .pm
        .clk_apb_m_set()
        .modify(|_, w| w.pad_config().enable().pm().enable());

    let rx = Pin08::new().into_serial_port();
    let tx = Pin09::new().into_serial_port();

    let serial = Serial::new(peripherals.usart_1, (tx, rx), Config::default());
    let (mut tx, _rx) = serial.split();

    loop {
        let _ = writeln!(tx, "Hello from MIK32 USART1");
        delay(MESSAGE_DELAY_SPINS);
    }
}

#[inline(always)]
fn delay(spins: u32) {
    for _ in 0..spins {
        core::hint::spin_loop();
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn trap_handler() {
    loop {
        core::hint::spin_loop();
    }
}
