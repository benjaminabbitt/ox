---
sidebar_position: 2
---

# ADR 0001: Use Rust Instead of Python

**Status:** Accepted
**Date:** 2024-01

## Context

The robotics community has standardized on Python for high-level control. ROS, MicroPython, and countless tutorials assume Python as the default language. This creates pressure to "just use Python" for new robotics projects.

**Python has earned that position.** It's approachable, readable, and has an enormous ecosystem. For learning robotics concepts, prototyping algorithms, or building systems where real-time performance isn't critical, Python is genuinely excellent. ROS wouldn't be what it is without Python. MicroPython has introduced countless people to embedded programming.

However, for real-time robotics applications — flight control, combat robots, rocketry — Python has fundamental limitations that matter to this author:

### Python's Problems

1. **Garbage Collection Pauses**
   - CPython's GC can pause execution for 10-100ms
   - MicroPython is better but still unpredictable
   - A 50ms pause during a flip maneuver = crash

2. **Interpreted Execution**
   - 10-100x slower than compiled code
   - Control loops that should run at 1kHz struggle at 100Hz
   - CPU time wasted on interpretation, not computation

3. **Memory Overhead**
   - Python objects have significant overhead
   - MicroPython needs 256KB+ RAM for basic functionality
   - ESP32-C3 has 400KB total — can't spare it

4. **No Compile-Time Safety**
   - Type errors discovered at runtime
   - `motor.sped(100)` doesn't fail until that line executes
   - Null/None errors in production

5. **Global Interpreter Lock**
   - Only one thread executes Python at a time
   - Dual-core ESP32 can't parallelize Python
   - "Threading" is mostly illusion

### What We Need

- Deterministic timing (no GC pauses)
- Native performance (1kHz+ control loops)
- Memory efficiency (run on <100KB)
- Compile-time error detection
- True parallelism on multi-core chips

## Decision

**Use Rust as the sole language for Ox.**

No Python bindings. No C wrappers. Pure Rust from kernel to application.

This is a personal preference. The author is a professional software developer and architect, but that doesn't make this choice authoritative — it's simply what one person prefers for their own projects. Others may reasonably disagree. The goal isn't to convince anyone that Python is bad, but to explore what's possible when you commit fully to a systems language designed for safety and performance.

### Why Rust Specifically?

| Requirement | Rust Solution |
|-------------|---------------|
| No GC pauses | No garbage collector — ownership system |
| Native speed | Compiles to machine code, zero-cost abstractions |
| Memory efficiency | No runtime, no interpreter, 50KB binaries |
| Compile-time safety | Borrow checker, type system, `Option<T>` not null |
| Parallelism | `Send`/`Sync` traits, fearless concurrency |
| Embedded support | First-class `no_std`, excellent ESP32 ecosystem |

### The Learning Curve

Rust's ownership system takes time to learn. This is a feature, not a bug:

- **Ownership teaches memory** — You learn where allocations happen
- **Lifetimes teach scope** — You understand reference validity
- **The compiler teaches correctness** — Errors at compile time, not runtime

This knowledge transfers to any systems programming work.

## Consequences

### Positive

- **Predictable timing** — No surprise GC pauses
- **Maximum performance** — Control loops run at full speed
- **Minimal memory** — More RAM for your application
- **Fewer production bugs** — Compiler catches entire categories of errors
- **Single language** — One toolchain, one set of skills

### Negative

- **Steeper learning curve** — New developers need to learn Rust
- **Smaller community** — Fewer robotics examples in Rust
- **Library gaps** — Some robotics algorithms only exist in Python/C++
- **Compile times** — Rust compiles slower than Python starts

### Mitigations

- Document Rust patterns specific to robotics
- Implement needed algorithms in pure Rust
- Use incremental compilation and `cargo check`

## Alternatives Considered

### C/C++

Pros: Mature, fast, huge ecosystem
Cons: Memory unsafe, no ownership system, harder to write correct code

We want the performance of C without the footguns.

### Zig

Pros: Simple, fast compilation, good for embedded
Cons: Smaller ecosystem, no borrow checker, less mature

We need the safety guarantees Zig doesn't provide.

### MicroPython

Pros: Python syntax, existing knowledge, great for learning and prototyping
Cons: The runtime characteristics don't fit this project's real-time requirements

MicroPython is genuinely impressive for what it does — bringing Python to microcontrollers is no small feat. For many projects, it's the right choice. For Ox's specific goals (sub-millisecond control loops, deterministic timing), we need a different runtime model.

## References

- [Rust for Embedded Systems](https://docs.rust-embedded.org/)
- [esp-rs (ESP32 Rust ecosystem)](https://github.com/esp-rs)
- [Why Rust for robots?](https://www.tangramvision.com/blog/why-rust-for-robots)
