# Ox

**A Rust microkernel for real-time robotics on ESP32**

---

## The Name

**Ox** — a triple reference:

1. **The Animal**: An ox is a beast of burden — tough, dependable, built for work. For thousands of years, oxen have pulled plows, hauled loads, and kept civilizations moving. A kernel should be the same: unglamorous, reliable, always there when you need it.

2. **Chemistry**: Ox is shorthand for oxidation — the "ox" in redox reactions. Oxidation-reduction reactions are fundamental to chemistry: batteries, combustion, corrosion. And yes, rust itself: Fe → Fe²⁺ + 2e⁻ (oxidation) while O₂ gains those electrons (reduction). Rust is *by definition* the product of iron oxidation. No oxidation, no rust — that's not poetry, it's chemistry.

3. **Computing**: Ox is visually similar to `0x`, the hexadecimal prefix used throughout systems programming:
   - `0xDEADBEEF` — Stack canary and memory debugging marker
   - `0xCAFEBABE` — Java class file magic number
   - `0x7F454C46` — ELF binary header (`.ELF`)
   - `0xFEEDFACE` — Mach-O binary header (macOS)
   - Memory addresses: `0x20000000` (SRAM), `0x40000000` (peripherals)
   - Capability tokens, IPC message IDs, hardware registers — all hex

When you're reading kernel code, you're reading hex. When you're debugging memory, you're reading hex. `Ox` sits at the intersection of the chemical and computational.

---

## Why Not Python?

Every robotics tutorial starts the same way: *"pip install robot-framework"*. And for learning, that's fine. Python and Arduino have earned their place — they're approachable, well-documented, and have taught millions of people to program robots. ROS runs on Python. MicroPython gets people blinking LEDs in minutes. That accessibility matters.

But when you're building something that matters — a quadcopter that needs sub-millisecond motor response, a combat robot where latency means losing a match, a rocket where a missed deadline means a crater — the author wanted something different.

- No garbage collection pauses at the worst possible moment
- Native-speed execution, not 100x interpreter overhead
- Real-time guarantees, not "usually fast enough"
- Memory efficiency on constrained microcontrollers
- Compile-time errors, not "it worked on my laptop"

This isn't about Python being bad. It's about wanting different trade-offs. **In the author's opinion, Rust is better for this domain** — but that's a preference, not a proclamation. The author is a professional software developer and architect, but that doesn't make these opinions authoritative. It's just what one person prefers. Ox takes a different path.

## Why Rust?

Rust provides memory safety without garbage collection, and performance matching C/C++. This combination enables *true* real-time operations — deterministic timing measured in microseconds, not "usually fast enough." When your control loop must complete in under 1ms, every time, without exception, the language runtime can't be a variable.

Rust gives you everything Python doesn't:

| Property | Python | Rust |
|----------|--------|------|
| Memory safety | Runtime crashes | Compile-time guarantees |
| Speed | Interpreted, ~100x slower | Native, zero-cost abstractions |
| Real-time | GC pauses, unpredictable | No GC, deterministic timing |
| Concurrency | GIL, threading pain | Fearless concurrency |
| Binary size | Needs interpreter | 50KB kernels |
| Embedded | MicroPython compromises | First-class `no_std` |

The catch? Rust has a learning curve. But that curve teaches you things Python hides: ownership, lifetimes, how memory actually works. Knowledge that makes you a better systems programmer.

## Why a Microkernel?

Because **fault isolation matters** when your robot is moving.

Traditional monolithic designs mean one bug brings down everything. A microkernel isolates failures:

```
┌─────────────────────────────────────────────────────────┐
│                      Your App                            │
├──────────┬──────────┬──────────┬──────────┬─────────────┤
│  Motor   │  Sensor  │   GPS    │  Comms   │  Vehicle    │
│  Server  │  Server  │  Server  │  Server  │  Server     │
├──────────┴──────────┴──────────┴──────────┴─────────────┤
│              Ox Microkernel (~500 LOC)                  │
│         IPC  •  Capabilities  •  Supervisor             │
├─────────────────────────────────────────────────────────┤
│                    ESP32 Hardware                        │
└─────────────────────────────────────────────────────────┘
```

- **Motor server crashes?** Supervisor restarts it. Other servers keep running.
- **GPS loses signal?** GPS server reports the error. Navigation degrades gracefully.
- **Rogue code tries to access a pin it shouldn't?** Capability system blocks it at compile time.

The kernel does three things well:
1. **IPC** — Message passing between servers (~1μs latency)
2. **Capabilities** — Unforgeable tokens for hardware access
3. **Supervision** — Heartbeat monitoring and automatic restart

Everything else runs in userspace servers.

## What "Real-Time" Actually Means

The term "real-time" is widely misunderstood. It doesn't mean "fast." It means **deterministic** — guaranteed execution within a time constraint.

| Type | Definition | Example |
|------|------------|---------|
| **Hard real-time** | Missing a deadline is system failure | Airbags, flight control, motor loops |
| **Soft real-time** | Occasional misses degrade quality | Video streaming, audio playback |
| **Not real-time** | "Usually fast enough" | Web servers, desktop apps |

The critical distinction:

- **Fast code**: Average case is quick (e.g., 100μs typical response)
- **Real-time code**: Worst case is bounded (e.g., *never* exceeds 1ms)

A control loop averaging 100μs but occasionally spiking to 50ms is fast, but **not real-time**. A loop taking 900μs that *never* exceeds 1ms **is real-time**.

This is why Ox tracks `max_us` in timing statistics — proving the worst case stays within budget is what matters. It's also why garbage-collected languages like Python are unsuitable: a GC pause at the wrong moment violates the timing guarantee, and your quadcopter falls out of the sky.

```
Real-time budget: 1000μs (1ms)
┌────────────────────────────────────────────────────────┐
│ Encoder read      ████ 5μs                             │
│ PID computation   ████████ 10μs                        │
│ PWM update        ███ 3μs                              │
│ IPC overhead      ██ 2μs                               │
│ Task switch       ████ 5μs                             │
│ ─────────────────────────────────────────────────────  │
│ Worst case total  █████████████████████████ 30μs       │
│ Margin remaining  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░ 970μs   │
└────────────────────────────────────────────────────────┘
Verdict: 97% margin. Real-time guarantee met.
```

**The practical benefit**: When your control loop guarantees sub-millisecond response, the robot reacts as fast as physics and your control algorithm allow. The limiting factor becomes the quality of your code and the human operator's reflexes — not random interpreter pauses or GC stutters. In theory and practice, this means tighter control, faster recovery from disturbances, and the confidence that your timing budget is *yours* to spend.

## Target Applications

Ox is designed for robotics where failure has consequences:

### Flight Control
- Sub-millisecond control loops
- Attitude estimation with complementary filters
- PID-controlled motor outputs
- Failsafe modes when things go wrong

### Combat Robotics
- Weapon motor control with current limiting
- RC receiver input (SBUS, CRSF)
- Damage detection and response
- No Python interpreter to crash mid-match

### Rocketry
- Pyrotechnic channel control with capability-based safety
- Barometric altitude and IMU fusion
- Parachute deployment timing
- Flight data logging

## Architecture

```
ox/
├── crates/
│   ├── ox-kernel      # Microkernel core: IPC, capabilities, supervisor
│   ├── ox-hal         # Hardware abstraction: mockable traits
│   ├── ox-chip        # ESP32 variants: C3, C6, H2, S3
│   ├── ox-control     # Control theory: PID, filters
│   ├── ox-services    # Userspace servers: motor, sensor, GPS, nav
│   └── ox-app         # Your application
└── docs/              # Docusaurus documentation
```

### Supported Hardware

| Chip | Cores | Architecture | WiFi/BLE | Notes |
|------|-------|--------------|----------|-------|
| ESP32-C3 | 1 | RISC-V | Yes | Default target |
| ESP32-C6 | 2 | RISC-V | Yes + Thread | HP/LP heterogeneous cores |
| ESP32-H2 | 1 | RISC-V | BLE + Thread | Low power, no WiFi |
| ESP32-S3 | 2 | Xtensa | Yes | 240MHz, 8MB PSRAM option |

## Quick Start

```bash
# Install Rust and ESP toolchain
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
cargo install espup espflash
espup install  # For ESP32-S3 Xtensa support

# Clone and build
git clone https://github.com/example/ox
cd ox

# Build for ESP32-C3 (default)
just build

# Flash and monitor
just run
```

### Building for Different Chips

```bash
just build        # ESP32-C3 (default)
just build-c6     # ESP32-C6 with dual-core
just build-h2     # ESP32-H2
just build-s3     # ESP32-S3 (requires: source ~/export-esp.sh)
```

## Example: Motor Control

```rust
use ox_kernel::ipc::{Call, ResponseSlot};
use ox_services::motor::{MotorCommand, MotorStatus};

#[embassy_executor::task]
async fn control_loop(motor_call: &'static Call<MotorCommand, MotorStatus>) {
    loop {
        // Read sensors, compute control output
        let (left, right) = compute_motor_speeds();

        // Send command to motor server via IPC
        let slot = ResponseSlot::new();
        let status = motor_call.call(
            MotorCommand::SetVelocity { left, right },
            &slot
        ).await;

        // Check for faults
        if let Some(fault) = status.fault {
            handle_fault(fault);
        }

        Timer::after(Duration::from_millis(1)).await; // 1kHz loop
    }
}
```

## Philosophy

This project exists because the author wanted to:

1. **Learn to write a kernel** — Not a toy, but something that could actually run a robot
2. **Understand real-time systems** — What "deterministic" actually means in practice
3. **Stop fighting Python** — No more "why is my control loop taking 50ms?"
4. **Build something that matters** — Robots that fly, fight, and launch

Ox is opinionated:
- **Rust only.** No C, no Python, no "just wrap this C library."
- **Microkernel architecture.** Isolation by design, not afterthought.
- **Embassy async.** Cooperative multitasking, not RTOS threads.
- **ESP32 first.** Cheap, capable, well-supported by the Rust ecosystem.

## Status

🚧 **Active Development** — Not production ready, but flyable.

- [x] Microkernel core (IPC, capabilities, supervisor)
- [x] Motor control with PID
- [x] IMU fusion and attitude estimation
- [x] GPS and compass integration
- [x] Autonomous waypoint navigation
- [x] RC receiver input (SBUS, CRSF)
- [x] ESP32-S3 dual-core support
- [ ] Flight controller firmware
- [ ] Combat robot firmware
- [ ] Rocketry firmware with pyro channels

## Documentation

Full documentation at [ox.example.com](https://ox.example.com) (coming soon)

```bash
cd docs
npm install
npm run start  # Local dev server at localhost:3000
```

## Contributing

Contributions welcome. This is a learning project — if you're also learning Rust/embedded/kernels, you're in good company.

## License

MIT OR Apache-2.0 — your choice.

---

*"The best time to stop using Python for robotics was years ago. The second best time is now."*
