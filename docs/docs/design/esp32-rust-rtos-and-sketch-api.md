---
sidebar_position: 1
---

# ESP32-S3 Rust RTOS & Sketch API — Design Summary

**Status:** Draft
**Branch:** `claude/esp32-rust-rtos-design-QdyoB`

## Project Overview

A Rust-native operating system for the ESP32-S3 microcontroller, providing:

- Preemptive multithreading RTOS kernel
- Rust `std` library support via a custom Platform Abstraction Layer (PAL)
- A beginner-friendly "Sketch API" for Arduino-style peripheral access
- Full esp-hal interoperability for advanced users

The target hardware is the ESP32-S3: dual-core Xtensa LX7 at 240MHz, 512KB SRAM, 8MB PSRAM (module-dependent), WiFi, BLE 5.0.

---

## Architecture

```
┌──────────────────────────────────────────────────┐
│  User Sketch (Rust, setup/step pattern)          │
├──────────────────────────────────────────────────┤
│  Sketch API Crate                                │
│  (Board, OutputPin, SerialPort, I2cBus, etc.)    │
├──────────────────────────────────────────────────┤
│  esp-hal (type-safe HAL, also directly usable)   │
├──────────────────────────────────────────────────┤
│  Rust std (via custom PAL)                       │
├──────────────────────────────────────────────────┤
│  RTOS Kernel                                     │
│  ┌────────────┬─────────────┬──────────────┐     │
│  │ Scheduler  │ Thread Mgmt │ Sync Prims   │     │
│  │ (preempt + │ (TCBs,      │ (Mutex w/    │     │
│  │  round-    │  stacks,    │  priority    │     │
│  │  robin)    │  core       │  inheritance,│     │
│  │            │  affinity)  │  Condvar,    │     │
│  │            │             │  Semaphore,  │     │
│  │            │             │  RwLock)     │     │
│  ├────────────┼─────────────┼──────────────┤     │
│  │ Timer/Tick │ Sleep Queue │ VFS (fd I/O) │     │
│  └────────────┴─────────────┴──────────────┘     │
├──────────────────────────────────────────────────┤
│  Hardware (ESP32-S3 dual-core Xtensa LX7)        │
└──────────────────────────────────────────────────┘
```

### Key Architectural Principles

- **No C interop debt.** The entire stack from kernel to sketch API is Rust. No newlib, no libc FFI, no POSIX shim. The Rust `std` PAL is implemented directly against the kernel.
- **esp-hal is the HAL layer.** The OS does not reimplement hardware abstraction. esp-hal 1.0.0 provides GPIO, SPI, I2C, UART, MCPWM, ADC, timers, WiFi/BLE (via binary blobs).
- **Two API tiers.** The Sketch API is for simple use; esp-hal and the kernel API are available for advanced/RT use. They coexist and share the same scheduler, allocator, and hardware.

---

## RTOS Kernel

### Scheduling

The kernel provides preemptive priority-based scheduling with optional round-robin time-slicing within the same priority level.

- **Preemption mechanism:** A hardware timer fires periodic tick interrupts. The tick handler inspects the ready queue and performs context switches by saving/restoring register state via TCBs (Thread Control Blocks). Higher-priority threads becoming ready triggers immediate preemption via software-triggered CPU interrupts (no waiting for the next tick).
- **Dual-core:** Each core runs its own scheduling decisions independently. Cross-core preemption uses software-triggered CPU interrupts (one per core, since ESP32-S3 has no hardware IPC channel).
- **Priority inheritance:** Mutexes implement priority inheritance to prevent priority inversion (lessons from Mars Pathfinder).
- **Two scheduling modes:**
  - High-priority preemptive mode for RT tasks (FOC motor control, sensor polling)
  - Round-robin time-sliced mode for normal tasks (sketch code, networking, logging)

### Context Switching

On ESP32-S3 Xtensa LX7, context switching involves saving/restoring register windows (not a flat register file). This is inherently `unsafe` and architecture-specific. Ariel OS's implementation is ~66 LOC for the chip-level abstraction — a good reference for minimality.

### Threading

- `std::thread::spawn` creates normal-priority round-robin threads via the Rust `std` PAL.
- `kernel::spawn` creates threads with explicit priority, core affinity, and scheduling class for RT tasks.
- Both use the same scheduler and share the same address space (no process isolation — not meaningful on a flat-memory MCU).

---

## Rust `std` Support

The OS provides Rust `std` by implementing the Platform Abstraction Layer (PAL) at `library/std/src/sys/pal/`. This requires forking the Rust std source (out-of-tree PAL is not yet supported upstream).

### PAL Modules to Implement

| Module | Maps To |
|---|---|
| **alloc** | Heap allocator over ESP32-S3 SRAM (~512KB). `GlobalAlloc` trait. |
| **thread** | Kernel thread create/join/detach/sleep. |
| **mutex** | Kernel mutex with priority inheritance. |
| **condvar** | Kernel condition variable. |
| **rwlock** | Kernel reader-writer lock. |
| **once** | One-time initialization primitive. |
| **thread_local** | Key-based TLS (pthread_key-style). |
| **time** | `Instant` via 64-bit system timer (16MHz). `SystemTime` via RTC/NTP. |
| **fs** | VFS layer over LittleFS on SPI flash (fd-based). |
| **net** | Sockets over lwIP/WiFi. `TcpStream`, `TcpListener`, `UdpSocket`. |
| **io** | stdin/stdout/stderr mapped to UART0 by default. |
| **process** | Stubbed (return errors). No `fork()` on MCU. |
| **env** | Minimal or stubbed. |
| **random** | ESP32-S3 hardware RNG. |
| **panic** | Abort-on-panic (simpler, smaller) initially. |

### What `std` Enables

Full Rust ecosystem access: `String`, `Vec`, `HashMap`, `std::sync::Mutex`, `std::thread`, `std::net::TcpStream`, `std::fs::File`, `std::io::Write`, plus any crate that depends on `std`.

---

## Sketch API

### Entry Point

```rust
#[sketch(board = "esp32s3-devkitc")]
mod blink {
    fn setup(board: &Board) -> State {
        State { led: board.output(13) }
    }

    fn step(board: &Board, state: &mut State) {
        state.led.toggle();
        delay(500);
    }
}
```

- `#[sketch]` is a proc macro that generates the runtime bootstrap: kernel init, Board construction from the selected board config, spawning the sketch thread.
- `setup` runs once, receives a `&Board` reference, returns a user-defined `State` struct.
- `step` runs repeatedly, receives both `&Board` and `&mut State`.
- `delay()` is blocking from the caller's perspective but yields to the scheduler under the hood (not a busy-wait). Resets the watchdog as a side effect.

### Board

`Board` is the peripheral ownership root. It is always internally `Arc<Mutex<_>>`, making it `Clone` and safely shareable across threads. Users never see the synchronization — it's an implementation detail.

```rust
impl Board {
    pub fn output(&self, pin: u8) -> OutputPin { ... }
    pub fn input(&self, pin: u8, pull: Pull) -> InputPin { ... }
    pub fn analog_input(&self, pin: u8, atten: Attenuation) -> AnalogPin { ... }
    pub fn pwm(&self, pin: u8, freq: Frequency) -> PwmPin { ... }
    pub fn serial(&self, baud: u32) -> SerialPort { ... }
    pub fn serial_on(&self, port: u8, baud: u32, tx: u8, rx: u8) -> SerialPort { ... }
    pub fn i2c(&self, freq: Frequency) -> I2cBus { ... }
    pub fn i2c_on(&self, sda: u8, scl: u8, freq: Frequency) -> I2cBus { ... }
    pub fn spi(&self, clk: u8, mosi: u8, miso: u8, freq: Frequency, mode: SpiMode) -> SpiBus { ... }
    pub fn led_builtin(&self) -> Option<OutputPin> { ... }
    pub fn spawn(&self, name: &str, f: impl FnOnce() + Send + 'static) { ... }
    pub fn spawn_async(&self, fut: impl Future<Output = ()> + Send + 'static) { ... }
    pub fn release(&self, pin: impl Into<PinHandle>) { ... }
}
```

- **Pin claiming uses runtime numeric lookup.** `board.output(13)` locks the internal mutex, checks the pin registry, marks the pin as claimed, configures the GPIO via esp-hal, and returns an owned `OutputPin`. Panics if the pin is already claimed or doesn't exist on the board.
- **Bus constructors use board-default pins** when called without explicit pin arguments (e.g., `board.serial(115200)` uses the board's configured default TX/RX). Explicit pin variants are available.
- **Peripherals returned from Board are owned outright.** `OutputPin`, `SerialPort`, etc. do not hold a reference back to Board. The hot path (`led.toggle()`, `spi.transfer()`) never touches the Board mutex.

### Board Configuration (Per-Board)

Each board variant provides a compile-time configuration:

```rust
pub struct BoardConfig {
    pub led_builtin: Option<u8>,
    pub default_serial_tx: u8,
    pub default_serial_rx: u8,
    pub default_i2c_sda: u8,
    pub default_i2c_scl: u8,
    pub default_spi_mosi: u8,
    pub default_spi_miso: u8,
    pub default_spi_clk: u8,
    pub default_spi_cs: u8,
    pub cpu_freq_mhz: u32,
    pub xtal_freq_mhz: u32,
    pub valid_pins: &'static [u8],
    pub pin_caps: &'static [PinCaps],
}
```

Board crates (e.g., `boards/esp32s3-devkitc`) define a `const CONFIG: BoardConfig` with the correct pin mappings, valid GPIO list, and defaults for that specific board.

### Pin API (Numeric, Typed Underneath)

```rust
// User writes numeric pins
let led = board.output(13);
let btn = board.input(4, Pull::Up);

// esp-hal is also directly available for advanced users
let mcpwm = esp_hal::mcpwm::McPwm::new(...);
```

Numeric at the API boundary for simplicity. esp-hal's type-state pins are used internally but not exposed through the sketch API. Users who need compile-time pin safety use esp-hal directly — the two are interoperable.

### Peripheral API

#### Digital I/O

```rust
led.high();
led.low();
led.toggle();
let pressed: bool = button.read();
```

#### Analog

```rust
let sensor = board.analog_input(34, Attenuation::Db11);
let raw: u16 = sensor.read();
let volts: f32 = sensor.read_voltage();
```

#### PWM

```rust
let motor = board.pwm(25, 1000.hz());
motor.set_duty(0.75);        // 0.0 to 1.0
motor.set_duty_raw(192);     // 0 to 255 (Arduino-compatible)
```

#### Serial (UART)

```rust
let serial = board.serial(115200);
serial.println("hello");
serial.write_bytes(&[0x01, 0x02]);
if serial.available() > 0 {
    let byte = serial.read_byte();
}
// std::fmt integration
writeln!(serial, "temp = {:.1}°C", temp).unwrap();
```

#### I2C

```rust
let i2c = board.i2c(400.khz());
i2c.write(0x68, &[0x75])?;
i2c.read(0x68, &mut buf)?;
i2c.write_read(0x68, &[0x3B], &mut buf)?;
```

#### SPI

```rust
let spi = board.spi(clk: 18, mosi: 23, miso: 19, 1.mhz(), SpiMode::Mode0);
let cs = board.output(5);
cs.low();
spi.transfer(&mut data)?;
cs.high();
```

#### Interrupts

```rust
let button = board.input(4, Pull::Up);
button.on_change(Edge::Falling, || {
    FLAG.store(true, Ordering::Relaxed);
});
```

#### Timing

```rust
let now: u64 = millis();
let now_us: u64 = micros();
delay(500);       // blocking, yields to scheduler
delay_us(100);    // blocking, microseconds
```

### Error Handling

| Category | Behavior | Rationale |
|---|---|---|
| Pin claiming (`board.output(13)`) | **Panic** if pin already claimed or invalid | Programmer bug, not recoverable |
| Digital I/O (`led.high()`, `button.read()`) | **Infallible** | Register write/read cannot fail |
| Analog read (`sensor.read()`) | **Infallible** | Matches Arduino, no meaningful recovery |
| PWM (`pwm.set_duty(0.5)`) | **Infallible** | Register write cannot fail |
| Bus transactions (`i2c.write()`, `spi.transfer()`) | **`Result<_, IoError>`** | Real bus failures (NACK, timeout, fault) |
| Serial write (`serial.println()`) | **Infallible** | Fire-and-forget to TX buffer |
| Serial read with timeout | **`Result<usize, IoError>`** | Timeout is meaningful |

Unified error type for bus operations:

```rust
pub enum IoError {
    Nack,
    Timeout,
    BusFault,
    InvalidAddress,
    BufferOverflow,
}
```

### Async Integration

The default sketch model is synchronous (`setup`/`step`). `delay()` yields to the RTOS scheduler — it suspends the calling thread, does not spin.

Async is available as an opt-in for users who need concurrent I/O:

```rust
fn setup(board: &Board) -> State {
    let sensor_i2c = board.i2c(400.khz());

    board.spawn_async(async move {
        loop {
            let temp = read_temp(&sensor_i2c).await;
            log!("temp: {}", temp);
            sleep(Duration::from_secs(10)).await;
        }
    });

    State { led: board.output(13) }
}
```

Async tasks run on the same scheduler as synchronous threads.

### Multi-Task Sketches

Board is `Clone` (internally `Arc<Mutex<_>>`). Multi-task sketches clone the board and move it into spawned closures:

```rust
fn setup(board: &Board) -> State {
    let led1 = board.output(13);
    let led2 = board.output(14);
    let serial = Shared::new(board.serial(115200));
    let serial2 = serial.clone();

    board.spawn("aux_blinker", move || {
        loop {
            led2.toggle();
            serial2.lock().println("aux");
            delay(250);
        }
    });

    State { led: led1, serial }
}
```

- All peripheral claiming happens during `setup`.
- Peripherals that need cross-thread access are wrapped in `Shared<T>` (the sketch API's friendly name for `Arc<Mutex<T>>`).
- Peripherals that are exclusive to one thread move directly — no wrapping needed.

### esp-hal Interop

The sketch API and esp-hal coexist. For use cases that need direct hardware access (FOC motor control, custom peripheral drivers):

```rust
fn setup(board: &Board) -> State {
    let led = board.output(13);

    // RT motor control task uses esp-hal and kernel API directly
    kernel::spawn(Priority::RealTime, Core::Core1, || {
        // Direct esp-hal MCPWM usage
        loop {
            foc_tick();
            kernel::yield_until_next_period();
        }
    });

    State { led }
}
```

Pins claimed through the sketch API are tracked in Board's registry and unavailable to esp-hal (prevents conflicts). Unclaimed pins/peripherals remain available for direct esp-hal use.

---

## Runtime Bootstrap (Framework-Generated)

The `#[sketch]` proc macro generates:

```rust
// Generated by #[sketch(board = "esp32s3-devkitc")]
fn _sketch_main() {
    // 1. Take esp-hal peripheral singleton
    let peripherals = esp_hal::init(hal_config());

    // 2. Construct Board from board-specific config
    let board = Board::from_config(
        &boards::esp32s3_devkitc::CONFIG,
        peripherals,
    );

    // 3. Run user's setup
    let mut state = setup(&board);

    // 4. Loop: call step, yield, watchdog
    loop {
        step(&board, &mut state);
        kernel::yield_point();
    }
}
```

The kernel boots, starts the scheduler, and spawns `_sketch_main` as a normal-priority thread. RT tasks and async tasks spawned during `setup` are already running by the time `step` begins.

---

## Existing Landscape (Prior Art)

### RTOS Kernel Design

- **FreeRTOS** (2003): Default on ESP32-S3 via ESP-IDF. Preemptive priority + round-robin. The scheduling model to match or exceed.
- **VxWorks** (1987): POSIX PSE52 profile over RT kernel. Proves *nix semantics + RTOS coexist.
- **QNX Neutrino** (1990): Full POSIX-certified microkernel RTOS. 256 priority levels. The gold standard for *nix-on-RTOS.
- **Zephyr** (2016): Modern C-based, partial POSIX subsystem, device tree model. Closest contemporary MCU-scale reference.
- **NuttX** (Apache): Most POSIX-complete MCU RTOS. Everything is file descriptors. Proves full *nix semantics work in 512KB RAM.
- **Ariel OS** (2025): First Rust RTOS with multicore preemptive scheduling on ESP32-S3. 66 LOC for chip-level scheduler. Key architecture reference for the Rust-on-Xtensa scheduling layer.

### Rust Embedded OS Landscape

- **Embassy**: Async cooperative runtime. No preemption, no threads. Foundation for Ariel OS.
- **Tock**: Security-focused (MPU isolation, capsules). Non-preemptive kernel. ARM Cortex-M / RISC-V only, no Xtensa.
- **Hubris** (Oxide Computer): Scheduler + IPC framework. ARM Cortex-M only, narrow board support.
- **RTIC**: Interrupt-driven concurrency. Not an OS. Cortex-M only.

None provide Rust `std` + preemptive multithreading + ESP32-S3 + *nix-like API. This project is novel in that combination.

### ESP32-S3 Rust Toolchain

- Xtensa LX7 requires a custom LLVM fork, installed via `espup`.
- esp-hal 1.0.0 provides the unified HAL across ESP32 variants.
- Target triple: `xtensa-esp32s3-none-elf` (or custom for this OS).
- WiFi/BLE requires Espressif binary blobs (sore point, but unavoidable currently).

---

## Crate Structure (Preliminary)

```
project/
├── kernel/              # RTOS kernel
│   ├── scheduler/       # Priority queue, context switch, tick handler
│   ├── thread/          # TCB, stack allocation, create/join/sleep
│   ├── sync/            # Mutex, Condvar, Semaphore, RwLock
│   ├── timer/           # Hardware timer, sleep queue, watchdog
│   ├── vfs/             # File descriptor table, device registration
│   └── arch/            # Xtensa-specific: context switch, interrupt matrix
│       └── esp32s3/
├── std-pal/             # Rust std PAL implementation
│   ├── alloc.rs
│   ├── thread.rs
│   ├── mutex.rs
│   ├── condvar.rs
│   ├── time.rs
│   ├── fs.rs
│   ├── net.rs
│   └── io.rs
├── sketch-api/          # The beginner-friendly peripheral API
│   ├── board.rs         # Board struct, pin registry
│   ├── digital.rs       # OutputPin, InputPin
│   ├── analog.rs        # AnalogPin
│   ├── pwm.rs           # PwmPin
│   ├── serial.rs        # SerialPort
│   ├── i2c.rs           # I2cBus
│   ├── spi.rs           # SpiBus
│   ├── timing.rs        # millis, micros, delay
│   ├── interrupt.rs     # Pin interrupt registration
│   ├── shared.rs        # Shared<T> wrapper (Arc<Mutex<T>>)
│   └── prelude.rs       # Common re-exports
├── sketch-macros/       # #[sketch] proc macro
├── boards/              # Board definition crates
│   ├── esp32s3-devkitc/
│   ├── esp32s3-eye/
│   └── ...
└── examples/
    ├── blink.rs
    ├── serial_echo.rs
    ├── i2c_sensor.rs
    └── multi_task.rs
```
