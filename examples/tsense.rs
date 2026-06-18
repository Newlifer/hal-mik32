#![no_std]
#![no_main]

use core::panic::PanicInfo;

use hal_mik32::rcc::RCC;
use hal_mik32::tsens::{ClockSource, Config, TSENS};
use mik32_pac::Peripherals;

const SENSOR_CLOCK_HZ: u32 = 40_000;
const STARTUP_TIMEOUT: u32 = 100_000;
const SAMPLE_DELAY_SPINS: u32 = 25_000;

#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();

    let rcc_config = RCC::default();
    RCC::init(&rcc_config);

    let sensor_config = Config::default()
        .clock_from_source(ClockSource::HSI32M)
        .with_frequency(SENSOR_CLOCK_HZ);

    let mut sensor = TSENS::new(peripherals.tsens, &rcc_config.clocks, sensor_config).unwrap();

    let mut current_temperature_c = sensor.single_measurement(Some(STARTUP_TIMEOUT)).unwrap();
    let mut min_temperature_c = current_temperature_c;
    let mut max_temperature_c = current_temperature_c;

    sensor.start_continuous();

    loop {
        current_temperature_c = sensor.get_temperature();

        if current_temperature_c < min_temperature_c {
            min_temperature_c = current_temperature_c;
        }

        if current_temperature_c > max_temperature_c {
            max_temperature_c = current_temperature_c;
        }

        let _temperature_snapshot = (current_temperature_c, min_temperature_c, max_temperature_c);

        delay(SAMPLE_DELAY_SPINS);
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
