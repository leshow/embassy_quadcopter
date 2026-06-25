# mini quadcopter in Rust w/ embassy for esp32-c3 (WIP)

currently still a work in progress, building a (very) mini quadcopter for esp32 on bare metal with [embassy.dev](https://embassy.dev). right now using the esp32-c3 supermini, I may upgrade this to the s3 if the extra core is necessary.

## Quad assets

![drone frame](/images/assembly.png)

The frame stl & 3mf files are in the `stl/` dir and can be used to 3d print the frame. I created them in OnShape. For the battery compartment, I opted for a friction fit with pegs that you can also glue in. This got around some issues with printing supports that were a pain to remove.

There's a top stand that serves as a mount for the microcontroller and MPU6050 or ICM20948. I'll include a full parts list as the project progresses.

## Parts list

### Microcontrollers (choose one)

Currently only building on the c3 and c6. I'll try the s3 if I have time to attempt a build on the xtensa cores but RISC-V is better supported by embassy & Rust. The s3 supermini might be interesting because it's dual-core and has the same footprint as the c3.

| Part               | Chip     | Cores   | Flash  | RAM        | WiFi      | Notes                                                          |
| ------------------ | -------- | ------- | ------ | ---------- | --------- | -------------------------------------------------------------- |
| ESP32-C3 SuperMini | ESP32-C3 | 1× RV32 | 4 MB   | 400 KB     | 2.4 GHz   | **Current build** — `c3` feature                               |
| ESP32-C6 SuperMini | ESP32-C6 | 1× RV32 | 4 MB   | 512 KB     | 2.4/5 GHz | Drop-in upgrade — `c6` feature                                 |
| ESP32-S3 (16R8)    | ESP32-S3 | 2× LX7  | 16 MB  | 8 MB PSRAM | 2.4 GHz   | Dual-core; needs Xtensa toolchain — see `docs/s3-migration.md` |
| ESP32-S3 Mini      | ESP32-S3 | 2× LX7  | 4/8 MB | — / 2 MB   | 2.4 GHz   | Compact form factor                                            |

### IMU Sensors (choose)

| Part      | DOF  | Interface | Accel | Gyro | Mag       | Notes                                    |
| --------- | ---- | --------- | ----- | ---- | --------- | ---------------------------------------- |
| ICM-20948 | 9DOF | I²C / SPI | ✓     | ✓    | ✓ AK09916 | **Current build** — yaw via magnetometer |
| MPU-6050  | 6DOF | I²C       | ✓     | ✓    | ✗         | No yaw reference; cheaper and common     |

### Other Components

| Part                   | Qty | Notes                                         |
| ---------------------- | --- | --------------------------------------------- |
| 8520 brushed motor     | 4   | find it on aliexpress like everything else    |
| mosfet 100N03A         | 4   | One per motor                                 |
| 1S LiPo battery (3.7v) | 1   | 3.7v 1s battery 25C or more discharge rate\*  |
| Propeller              | 4   | 55 or 65mm                                    |
| 3D printed frame       | 1   | STL/3MF files in `stl/` — designed in OnShape |

Requires [espflash](https://github.com/esp-rs/espflash) for flashing (`cargo install espflash`).

\*I tried with a 503040 3.7v lipo recycled from a keyboard build but the BMS (battery management system) on it will automatically shut off after a few seconds. It's not really build to power these motors.

### ESP32-C3 (default)

```sh
cargo build -p embassy_quad --target riscv32imc-unknown-none-elf
cargo run -p embassy_quad --target riscv32imc-unknown-none-elf
```

Or using the aliases:

```sh
cargo build-c3
cargo flash-c3
```

### ESP32-C6

The C6 uses a different RISC-V target (`riscv32imac` vs `riscv32imc`). The
`c3` and `c6` features are mutually exclusive — always use
`--no-default-features` when building for C6 to avoid activating both:

```sh
cargo build-c6
cargo flash-c6
```

These aliases expand to:

```sh
cargo build --no-default-features --features c6 --target riscv32imac-unknown-none-elf
cargo run   --no-default-features --features c6 --target riscv32imac-unknown-none-elf
```

### Log level

`DEFMT_LOG` is read at compile time by `esp-println`:

```sh
DEFMT_LOG=debug cargo flash-c3
```

## Visualizer

to see a 3d rendering of the orientation run:

```bash
DEFMT_LOG="info" LOG_RATE_MS=1 cargo flash-c3 --features visualize | (cd visualizer && cargo run)
```

It feeds the esp32 log output to a binary reading stdin and rendering a cube on screen

By default, sensor readings from the ICM-20948 are sent to the ESP32-C3 where either a Madgwick filter fuses accel and gyro data in software to correct orientation or the IMU's DMP is used to get the already fused output (`dmp` feature is on by default). The ICM-20948 also has an onboard DMP that fuses data directly on the sensor board. Output looks pretty good and doesn't have the yaw drift that the software fusion does when the magnetometer is enabled.

```sh
DEFMT_LOG="info" LOG_RATE_MS=1 cargo flash-c3 --features visualize | (cd visualizer && cargo run)
```

## LLM usage

Docs and tests are sometimes generated with the use of LLMs, along with explanation/discovery, but the purpose of this project is to actually learn, so the code is still written by a human (hi!)
