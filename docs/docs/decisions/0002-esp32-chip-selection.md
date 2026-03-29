---
sidebar_position: 3
---

# ADR 0002: Use ESP32 Over Arduino/STM32

**Status:** Accepted
**Date:** 2024-01

## Context

Choosing a microcontroller for robotics involves trade-offs between cost, capability, ecosystem, and availability. The main contenders are:

### Arduino (AVR/SAMD)

The default "learn embedded" platform — and for good reason.

| Model | Price | Clock | RAM | Flash | WiFi |
|-------|-------|-------|-----|-------|------|
| Uno R3 | $27 | 16 MHz | 2 KB | 32 KB | No |
| Nano | $24 | 16 MHz | 2 KB | 32 KB | No |
| Nano 33 IoT | $25 | 48 MHz | 32 KB | 256 KB | Yes |
| Mega | $46 | 16 MHz | 8 KB | 256 KB | No |

Arduino has taught millions of people to program microcontrollers. The simplified tooling, abundant tutorials, and forgiving development experience have real value. For education, prototyping, and many hobbyist projects, Arduino remains an excellent choice.

For Ox's specific needs, however, there are trade-offs:
- **Cost vs. capability** — $25+ for specs that limit complex control
- **RAM constraints** — 2KB RAM is tight for microkernel + services
- **Clock speed** — 16MHz limits control loop frequency
- **Simplified tooling** — Helpful for learning, but hides concepts we want to understand

### STM32

Professional embedded choice.

| Model | Price | Clock | RAM | Flash | WiFi |
|-------|-------|-------|-----|-------|------|
| STM32F103C8 | $2 | 72 MHz | 20 KB | 64 KB | No |
| STM32F411CE | $4 | 100 MHz | 128 KB | 512 KB | No |
| STM32H743 | $15 | 480 MHz | 1 MB | 2 MB | No |

Problems:
- **No integrated WiFi** — Need separate modules
- **Complex ecosystem** — HAL, LL, Cube, registers... pick one
- **Rust support varies** — Good for some chips, nonexistent for others
- **Supply chain issues** — Chip shortages hit STM32 hard

### ESP32 (Espressif)

WiFi/BLE SoC designed for IoT.

| Model | Price | Clock | RAM | Flash | WiFi | Architecture |
|-------|-------|-------|-----|-------|------|--------------|
| ESP32-C3 | $2 | 160 MHz | 400 KB | 4 MB | Yes | RISC-V |
| ESP32-C6 | $3 | 160 MHz | 512 KB | 4 MB | Yes + Thread | RISC-V |
| ESP32-S3 | $3 | 240 MHz | 512 KB | 8 MB | Yes | Xtensa (2 cores) |
| ESP32-H2 | $2 | 96 MHz | 320 KB | 4 MB | Thread/Zigbee | RISC-V |

## Decision

**Use ESP32 as the primary target platform.**

This is the author's preference based on the specific constraints of this project: real-time control, wireless connectivity, and cost sensitivity when building multiple robots. The author is a professional software developer and architect, but this is just one person's opinion — Arduino and STM32 are both valid choices, and many successful robots run on them.

Specifically:
- **ESP32-C3** as the default (single-core RISC-V, cheapest)
- **ESP32-S3** for high-performance needs (dual-core, 240MHz, PSRAM)
- **ESP32-C6** for Thread/Matter IoT integration

### Why ESP32?

#### 1. Cost

| What You Get | Arduino Nano | ESP32-C3 |
|--------------|--------------|----------|
| Price | $24 | $2 |
| RAM | 2 KB | 400 KB |
| Flash | 32 KB | 4 MB |
| Clock | 16 MHz | 160 MHz |
| WiFi | No | Yes |
| BLE | No | Yes |

**The ESP32-C3 costs 1/10th the price with 200x the RAM and 10x the clock speed.**

For a combat robot with 4 motor drivers, the chip cost difference is:
- Arduino: 4 × $24 = $96
- ESP32-C3: 4 × $2 = $8

That's $88 saved per robot, before you add the WiFi modules Arduino would need.

#### 2. Rust Ecosystem

The `esp-rs` project provides excellent Rust support:

- **esp-hal** — Hardware abstraction layer
- **esp-wifi** — WiFi/BLE drivers (pure Rust)
- **embassy-esp** — Async executor integration
- **probe-rs** — Debugging and flashing
- **espflash** — Simple deployment

This is a maintained, active ecosystem with Espressif's official support.

#### 3. Integrated Connectivity

WiFi and BLE are built-in, not afterthoughts:
- No extra modules to wire up
- No separate power supplies
- No protocol translation
- Firmware updates over WiFi

For robotics, this enables:
- Wireless parameter tuning
- Remote telemetry
- OTA updates
- RC control over WiFi

#### 4. RISC-V

The C3/C6/H2 chips use RISC-V, an open instruction set:
- Standard Rust toolchain (no custom forks)
- Better debugging support
- Future-proof architecture
- Academic and industry momentum

#### 5. Availability

ESP32 chips remained available during the 2021-2023 chip shortage when STM32 lead times hit 52+ weeks. Espressif has multiple fabs and maintains stock.

## Consequences

### Positive

- **10x cost reduction** — More robots, more experiments
- **Excellent Rust support** — First-class, maintained tooling
- **Integrated WiFi/BLE** — No external modules needed
- **Sufficient performance** — 160-240MHz handles control loops easily
- **Good availability** — Actually purchasable

### Negative

- **Not "industry standard"** — Less common in professional robotics (STM32 dominates there)
- **Less beginner-friendly** — Arduino's ecosystem is more approachable for newcomers
- **Limited analog inputs** — ADC is 12-bit, only 2 channels on C3
- **3.3V logic** — Need level shifters for 5V sensors
- **Xtensa toolchain** — S3 requires separate toolchain (espup)

### Mitigations

- Document ESP32-specific patterns
- Provide level shifter recommendations
- Support both RISC-V (C3/C6/H2) and Xtensa (S3)

## Alternatives Considered

### RP2040 (Raspberry Pi Pico)

Pros: $1, dual-core ARM, good Rust support
Cons: No WiFi (Pico W adds it at $6), smaller ecosystem

Good chip, but ESP32 has integrated WiFi at similar price.

### nRF52840

Pros: Excellent BLE, very low power
Cons: No WiFi, $7+, primarily for wearables

Better for battery sensors than control systems.

### STM32 + ESP32 combo

Pros: Best of both worlds
Cons: Two chips, two toolchains, two failure points

Complexity not justified when ESP32 handles both.

## References

- [ESP32-C3 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-c3_datasheet_en.pdf)
- [esp-rs Project](https://github.com/esp-rs)
- [Why the ESP32 is the Swiss Army Knife of IoT](https://randomnerdtutorials.com/esp32-vs-esp8266/)
