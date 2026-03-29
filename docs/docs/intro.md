---
sidebar_position: 1
slug: /
---

# Introduction

**Ox** is a Rust microkernel for real-time robotics on ESP32 microcontrollers.

## What is Ox?

Ox provides a minimal, fault-isolated foundation for building robots that need:

- **Sub-millisecond response times** — Control loops that run at 1kHz+
- **Fault isolation** — A crashed motor driver doesn't take down your whole robot
- **Memory safety** — Rust's compile-time guarantees, not runtime crashes
- **Capability-based security** — Hardware access controlled by unforgeable tokens

## Who is Ox For?

Ox is designed for roboticists who are tired of:

- Python's garbage collection pauses ruining control loops
- Arduino's blocking APIs making multitasking painful
- C's memory bugs causing mysterious crashes
- "It works on my laptop" not translating to embedded hardware

If you're building flight controllers, combat robots, rocket avionics, or anything where milliseconds matter — Ox is for you.

## Core Principles

### 1. Rust Only

No C wrappers, no Python bindings. Pure Rust from kernel to application. This means:
- One language to learn
- One toolchain to maintain
- Compile-time safety everywhere

### 2. Microkernel Architecture

The kernel does three things well:
- **IPC** — Message passing between isolated servers
- **Capabilities** — Hardware access control
- **Supervision** — Automatic restart of failed servers

Everything else runs in userspace.

### 3. Embassy Async

Cooperative multitasking via Rust's async/await:
- No RTOS scheduler overhead
- Predictable task switching
- Zero-cost abstractions

### 4. ESP32 First

Cheap, capable, well-supported:
- $3-5 per chip
- WiFi/BLE built-in
- Excellent Rust ecosystem via `esp-hal`

## Quick Links

- [Installation](/docs/getting-started/installation) — Get up and running
- [Architecture Overview](/docs/architecture/overview) — How Ox works
- [Design Decisions](/docs/decisions/intro) — Why we made the choices we did
