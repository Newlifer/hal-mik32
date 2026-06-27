#![no_std]
#![no_main]

use core::panic::PanicInfo;

use embedded_hal::digital::{InputPin, OutputPin};
use hal_mik32::gpio::port_0::{Pin09, Pin10};
use hal_mik32::rcc::RCC;
use mik32_pac::Peripherals;

// GPIO 0.10 - button input.
// GPIO 0.9  - LED output.
#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    let p = Peripherals::take().unwrap();

    let rcc_config = RCC::default();
    RCC::init(&rcc_config).unwrap();

    p.pm.clk_apb_p_set().modify(|_, w| w.gpio_0().enable());
    p.pm.clk_apb_m_set()
        .modify(|_, w| w.pad_config().enable().pm().enable());

    let mut led_pin = Pin09::new().into_output();
    let mut button_pin = Pin10::new().into_pull_down_input();

    loop {
        if button_pin.is_high().unwrap() {
            let _ = led_pin.set_high();
        } else {
            let _ = led_pin.set_low();
        }

        for _ in 0..1000 {
            core::hint::spin_loop();
        }
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn trap_handler() {
    loop {}
}
