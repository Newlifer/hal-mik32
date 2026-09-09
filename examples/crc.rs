#![no_std]
#![no_main]

use core::panic::PanicInfo;

use hal_mik32::crc::{Config, Crc};
use mik32_pac::Peripherals;

#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut crc = Crc::new(peripherals.crc, Config::crc32_iso_hdlc());

    // CRC-32/ISO-HDLC check value for the conventional test string.
    let checksum = crc.checksum(b"123456789");
    assert_eq!(checksum, 0xcbf4_3926);

    loop {
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
