# BounceAAL-rs
Rust port of firmware for _hopefully_ lesser overhead in development. Assumes an EOS Linux system or similar Arch derivative.

Consider doing below setup in a terminal and run your IDE, e.g. rustrover by forking off to retain env `rustrover &`.

# Setup
Using esp-rs and local ESP-IDF
```sh
$ pacman -S esp-idf
$ source /opt/esp-idf/export.fish
```

## Setup of bindgen
Must have Clang and set path for bindings to be compiled through `build.rs`
```sh
$ pacman -S clang
$ pacman -Ql clang | grep libclang
clang /usr/lib/libclang-cpp.so
clang /usr/lib/libclang-cpp.so.16
clang /usr/lib/libclang.so
clang /usr/lib/libclang.so.16
clang /usr/lib/libclang.so.16.0.6
$ export LIBCLANG_PATH=/home/user/.rustup/toolchains/esp/xtensa-esp32-elf-clang/esp-16.0.4-20231113/esp-clang/lib
```

## VL53L5CX
Only mandatory API sources:
- include:
  vl53l5cx_api.h
  vl53l5cx_buffers.h
- src:
    - vl53l5cx_api.c

- platform
    - platform.c
    - platform.h