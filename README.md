# hal-mik32

[![stability-experimental](https://img.shields.io/badge/stability-experimental-orange.svg)](https://github.com/emersion/stability-badges#experimental)

`hal-mik32` is a Rust hardware abstraction library for the MIK32 microcontroller family.
It provides safe high-level access to MCU peripherals built on top of the `mik32-pac` peripheral access crate.

## License

This project is released under the **CC0 1.0 Universal** license.
See [`LICENSE`](LICENSE) for the full text.

## PAC

This HAL is based on the `mik32-pac` crate, which provides the low-level register definitions and peripheral access for the MIK32 device.

Repository: https://github.com/Newlifer/mik32-pac

## Versioning and roadmap to `1.0.0`

The library will reach `1.0.0` once the core peripheral set required for stable embedded development is implemented and tested.
The release path is based on incremental HAL support for MIK32 peripherals, with the initial stable API covering the most important device functions.

### Required first steps

These steps must be implemented in this order because they are needed for the minimal bring-up.

- [ ] RCC config
- [ ] GPIO
- [ ] USART

### Remaining roadmap

After RCC, GPIO, and USART are in place, the order is not important. Each completed item increments the crate version by `0.1`.

- [ ] EPIC
- [ ] Timer32 0
- [ ] Timer32 1+2
- [ ] Timer16 (both)
- [ ] DMA
- [ ] SPI
- [ ] SPIFI
- [ ] I2C
- [ ] WDT
- [ ] EEPROM
- [ ] AVCC
- [ ] VCC
- [ ] Battery
- [ ] BrownOut
- [x] TSENS
- [ ] ADC
- [ ] DAC
- [ ] RTC

## Usage

Add `hal-mik32` as a dependency in your `Cargo.toml` and use the HAL modules that correspond to your target peripheral.

Example configuration and initialization will be provided in the crate examples.

## Notes

- This is a `#![no_std]` library intended for bare-metal embedded use.
- The project tracks support for MIK32-specific hardware features and aims to provide a stable HAL API before the `1.0.0` release.
