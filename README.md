# BounceAAL-rs
Rust port of firmware for BounceAAL. Assumes an EOS Linux system or similar Arch derivative with `rustup` for Rust toolchain management installed.

Hardware is an ESP32 generic make, VL53L5CX TOF sensor, 128x64 OLED display, and IRLZ44N mosfets paired with 36V PSU for driving solenoids at various duty cycles. 

# Dependencies
Needs `xtensa-esp32-elf` toolchain and `esp-idf` for building. `espup` is used to install and update these. [esp-rs/espup docs](https://github.com/esp-rs/rust-build#espup-installation).
```sh
cargo install espup
espup install # To install Espressif Rust ecosystem
# [Unix]: Source the following file in every terminal before building a project
. $HOME/export-esp.sh
```

If using an IDE with run-configurations for building, the following environment variables (generated and contained in `$HOME/user/export-esp.sh`) should be set:
```sh
export PATH="/home/user/.rustup/toolchains/esp/xtensa-esp-elf/esp-13.2.0_20230928/xtensa-esp-elf/bin:$PATH"
export LIBCLANG_PATH="/home/user/.rustup/toolchains/esp/xtensa-esp32-elf-clang/esp-16.0.4-20231113/esp-clang/lib"
```
## Hardware driver (VL53L5CX)
The VL53L5CX driver is a C library that is used to communicate with the VL53L5CX sensor. 
The driver is provided as a source code from STMicroelectronics under MIT license. 
Manual finagling is required to get it to work with Rust, e.g. merging `platform.h/c` into `vl53l5cx_api.h/c`.