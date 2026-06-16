set windows-shell := ["powershell.exe", "-NoLogo", "-Command"]

# Toolchain setup
install-toolchain:
	rustup toolchain install 1.87.0

install-components:
	rustup component add rustfmt clippy --toolchain 1.87.0

add-riscv-target:
	rustup target add riscv32imac-unknown-none-elf --toolchain 1.87.0

setup: install-toolchain install-components add-riscv-target

# Code checks
fmt-check:
	cargo fmt --all -- --check

check-host:
	cargo check --lib --tests --benches --target x86_64-pc-windows-msvc

check-riscv:
	cargo check --target riscv32imac-unknown-none-elf --examples

check-all: fmt-check check-host check-riscv
