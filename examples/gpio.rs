#![no_std]
#![no_main]

use core::panic::PanicInfo;

use embedded_hal::digital::{InputPin, StatefulOutputPin};
use hal_mik32::gpio::port_0::{Pin09, Pin10};
use hal_mik32::gpio::{self, DriveStrength};
use hal_mik32::rcc::RCC;

// GPIO 0.10 - button input.
// GPIO 0.9  - LED output.
#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    let rcc_config = RCC::default();
    RCC::init(&rcc_config).unwrap();
    let _gpio0 = gpio::init_port::<0>();

    let mut led_pin = Pin09::new()
        .into_output()
        .with_drive_strength(DriveStrength::Ma8);
    let mut button_pin = Pin10::new().into_pull_down_input();
    let mut button_was_pressed = false;

    loop {
        let button_is_pressed = button_pin.is_high().unwrap();

        if button_is_pressed && !button_was_pressed {
            let _ = led_pin.toggle();
        }

        button_was_pressed = button_is_pressed;
        delay(10_000);
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
