#![no_std]
#![no_main]

use core::panic::PanicInfo;

use hal_mik32::rcc::RCC;
use hal_mik32::tsens::{ClockSource, Config, TSENS};
use mik32_pac::Peripherals;

#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    let peripherals = unsafe { Peripherals::steal() };

    let rcc_config = RCC::default();
    RCC::init(&rcc_config);

    let mut sensor = TSENS::new(
        peripherals.tsens,
        &rcc_config.clocks,
        Config::default().clock_from_source(ClockSource::HSI32M),
    )
    .unwrap();

    if let Ok(temp_celsius) = sensor.single_measurement(None) {
        let _ = temp_celsius;
    }

    sensor.start_continuous();

    loop {
        let _temperature = sensor.get_temperature();
        core::sync::atomic::spin_loop_hint();
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
