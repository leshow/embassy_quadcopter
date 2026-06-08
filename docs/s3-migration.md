# ESP32-S3 Migration Notes

If more compute or dual-core WiFi is needed, here is how to add S3 support
alongside the existing C3/C6 RISC-V builds.

## Why a separate subdirectory

The S3 uses an Xtensa LX7 core. Xtensa support is not in upstream Rust — it
requires Espressif's compiler fork installed via `espup`. That fork uses
`channel = "esp"` in `rust-toolchain.toml`, which does not support
`rustup target add` and cannot share a toolchain file with the stable RISC-V
builds. A subdirectory with its own `rust-toolchain.toml` is the cleanest
solution.

## Prerequisites

```sh
cargo install espup
espup install
# espup prints two exports — add them to ~/.zshrc or source before S3 builds:
export LIBCLANG_PATH="..."
export PATH="...:$PATH"
# found in ~/export-esp.sh
```

## Generating the S3 crate

From the repo root:

```sh
# For the ESP32-S3 16R8 (16MB flash, 8MB octal PSRAM)
esp-generate -c esp32s3-wroom-1-octal-psram \
  -o alloc -o wifi -o unstable-hal -o embassy -o log -o wokwi -o vscode \
  s3

# For the ESP32-S3 Mini with 2MB PSRAM
esp-generate -c esp32s3-mini-1-psram \
  -o alloc -o wifi -o unstable-hal -o embassy -o log -o wokwi -o vscode \
  s3-mini
```

Then exclude the new crate from the root workspace so the parent toolchain
does not try to build it:

```toml
# Cargo.toml (root)
[workspace]
exclude = ["s3", "s3-mini"]
```

## Porting the fusion code

The fusion module (`src/fusion.rs`) is `no_std` and has no chip-specific
dependencies — it can be shared directly.

```sh
cargo new --lib fusion-lib
```

Move `fusion.rs` content into `fusion-lib/src/lib.rs`, then reference it
from both crates:

```toml
# root Cargo.toml and s3/Cargo.toml
fusion-lib = { path = "../fusion-lib" }
```

## Key differences in main.rs for S3

- The I2C peripheral is the same `esp-hal` API — no sensor code changes.
- Two cores available via Embassy: pin the control loop task to core 0 and
  WiFi/comms to core 1 using `#[embassy_executor::task]` with core affinity.
- `esp_println` remains the same; `init_logger_from_env()` is unchanged.
- The `LOOP_PERIOD_MS` constant and fusion builder setup are identical.

## Build commands once set up

```sh
# From the s3/ subdirectory:
cd s3
cargo build
cargo run   # flash + monitor via espflash
```

The generated `.cargo/config.toml` inside `s3/` will have the correct
`xtensa-esp32s3-none-elf` target and runner already configured.

---

## Alternative: ESP-IDF (FreeRTOS) instead of bare-metal Embassy

Rather than bare-metal Embassy, the S3 can run on top of Espressif's
**ESP-IDF** SDK, which uses **FreeRTOS** as its underlying RTOS. Rust code
runs as a standard application on top of it via the `esp-idf-hal` and
`esp-idf-svc` crates.

### Generating an ESP-IDF project

```sh
cargo install cargo-generate
cargo generate --git https://github.com/esp-rs/esp-idf-template
```

The template asks for the target chip and configures the build system
(CMake + idf.py underneath, driven by `embuild` from Cargo).

### Building and flashing

```sh
cargo run --target xtensa-esp32s3-espidf
```

Note the target is `xtensa-esp32s3-espidf` (not `none-elf`) — the `espidf`
suffix means the standard library is available via ESP-IDF's newlib libc.

### Advantages over bare-metal Embassy

- **`std` is available** — `String`, `Vec`, `HashMap`, threads, sockets all
  work. Much easier to write application-level code.
- **WiFi/BLE are well supported** — `esp-idf-svc` wraps the mature ESP-IDF
  WiFi stack with a clean Rust API. Getting a TCP socket or HTTP client
  working is straightforward.
- **FreeRTOS tasks** — familiar task/thread model with priorities. Pinning
  the control loop to core 0 and WiFi to core 1 is simple.
- **Larger ecosystem** — ESP-IDF has years of production use; drivers for
  most peripherals already exist as C libraries bindable from Rust.

### Disadvantages

- **Latency and determinism** — FreeRTOS introduces scheduling jitter. The
  control loop is not guaranteed to run at exactly 200 Hz; an Embassy
  bare-metal loop is more predictable.
- **Much larger binary and RAM footprint** — ESP-IDF pulls in a full network
  stack, NVS, partition table management etc. Bare-metal Embassy fits in
  ~100 KB; an ESP-IDF build starts at ~1 MB.
- **Slower build times** — the first build compiles ESP-IDF from source via
  CMake, which takes several minutes. Incremental builds are faster but still
  slower than a pure Rust build.
- **`no_std` code needs porting** — `fusion.rs` uses `no_std` with `libm`
  for float ops. Under ESP-IDF you can keep it `no_std` or switch to `std`
  float (which is fine since `std` is available). Either works; just remove
  the `libm` calls and use `f32::atan2` etc. directly.

### Which to choose

For a **flight controller** where the PID loop timing matters: stick with
bare-metal Embassy. For a **companion computer** role (WiFi telemetry,
configuration server, OTA updates) running alongside a dedicated flight
controller: ESP-IDF is the better fit.
