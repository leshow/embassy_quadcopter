# embassy drone esp32-c3 (WIP)

currently still a work in progress, building a (very) mini quadcopter for esp32. right now using the esp32-c3 supermini, I may upgrade this to the s3 if the extra core and speed is necessary.

## Build

Requires [espflash](https://github.com/esp-rs/espflash) for flashing (`cargo install espflash`).

### ESP32-C3 (default)

```sh
cargo build
cargo run          # build + flash + open serial monitor
```

Or using the aliases:

```sh
cargo build-c3
cargo flash-c3
```

### ESP32-C6

The C6 uses a different RISC-V target (`riscv32imac` vs `riscv32imc`). The
`chip-c3` and `chip-c6` features are mutually exclusive — always use
`--no-default-features` when building for C6 to avoid activating both:

```sh
cargo build-c6
cargo flash-c6
```

These aliases expand to:

```sh
cargo build --no-default-features --features chip-c6 --target riscv32imac-unknown-none-elf
cargo run   --no-default-features --features chip-c6 --target riscv32imac-unknown-none-elf
```

### Log level

`ESP_LOG` is read at compile time by `esp-println`:

```sh
ESP_LOG=debug cargo flash-c3
```
