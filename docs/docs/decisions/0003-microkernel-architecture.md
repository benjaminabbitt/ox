---
sidebar_position: 4
---

# ADR 0003: Microkernel Over Monolithic Architecture

**Status:** Accepted
**Date:** 2024-01

## Context

Operating system kernels fall on a spectrum from monolithic to microkernel:

### Monolithic Kernel

Everything runs in kernel space with full hardware access:

```
┌─────────────────────────────────────────────┐
│              Application                     │
├─────────────────────────────────────────────┤
│              Kernel Space                    │
│  ┌─────────────────────────────────────┐    │
│  │ Scheduler │ IPC │ Memory │ Drivers  │    │
│  │ Filesystem │ Network │ USB │ ...    │    │
│  └─────────────────────────────────────┘    │
├─────────────────────────────────────────────┤
│              Hardware                        │
└─────────────────────────────────────────────┘
```

Examples: Linux, FreeBSD, most RTOSes (FreeRTOS, Zephyr)

**Problem:** One bug in any driver can crash the entire system. A motor driver bug takes down your GPS, telemetry, and control loop.

### Microkernel

Minimal kernel, everything else in userspace:

```
┌─────────────────────────────────────────────┐
│              Application                     │
├───────┬───────┬───────┬───────┬─────────────┤
│ Motor │ Sensor│  GPS  │ Comms │  Vehicle    │
│ Server│ Server│ Server│ Server│  Server     │
├───────┴───────┴───────┴───────┴─────────────┤
│          Microkernel (~500 LOC)              │
│     IPC  │  Capabilities  │  Supervisor      │
├─────────────────────────────────────────────┤
│              Hardware                        │
└─────────────────────────────────────────────┘
```

Examples: seL4, L4, QNX, Minix 3

**Benefit:** Fault isolation. Motor server crashes? Supervisor restarts it. Other servers keep running.

## Decision

**Use a microkernel architecture for Ox.**

The kernel provides exactly three primitives:

### 1. IPC (Inter-Process Communication)

Message passing between isolated servers:

```rust
// Request-response pattern
pub struct Call<Req, Resp, const N: usize>;

// Streaming data
pub type Stream<T, const N: usize>;

// Wake-up signal
pub struct Notify;
```

Latency: ~1μs per message on ESP32.

### 2. Capabilities

Unforgeable tokens for hardware access:

```rust
// Only holders of this capability can access GPIO pin 5
pub struct Cap<GpioPin<5>>;

// Kernel grants at startup, servers can't forge
let motor_pin = Cap::grant();
```

Compile-time type safety + runtime unforgeable IDs.

### 3. Supervisor

Health monitoring and automatic restart:

```rust
// Servers send heartbeats
heartbeat.beat();

// Supervisor detects timeouts
if supervisor.check() == ServerStatus::Unresponsive {
    supervisor.restart(server_id);
}
```

Erlang-style "let it crash" philosophy.

## Consequences

### Positive

- **Fault isolation** — One server crash doesn't bring down the system
- **Testability** — Servers can be tested independently with mock IPC
- **Security** — Capability system prevents unauthorized hardware access
- **Reliability** — Supervisor can restart failed servers automatically
- **Simplicity** — Kernel is ~500 LOC, easy to audit and understand

### Negative

- **IPC overhead** — Every server interaction crosses a boundary (~1μs)
- **Complexity for simple tasks** — "Just blink an LED" requires understanding IPC
- **More code overall** — Server interfaces add boilerplate

### Trade-offs Accepted

For a safety-critical robotics system, we accept:
- 1μs IPC overhead for fault isolation
- Initial learning curve for long-term reliability
- More lines of code for better organization

## Why Not Just Use FreeRTOS?

FreeRTOS is excellent, but it's monolithic:

| Aspect | FreeRTOS | Ox Microkernel |
|--------|----------|----------------|
| Fault isolation | None — shared address space | Full — servers are isolated |
| Restart on crash | Manual, complex | Automatic via supervisor |
| Hardware access | Any task can access anything | Capability-gated |
| Testing | Requires hardware or complex mocks | Mock IPC, test servers independently |

FreeRTOS assumes you write correct code. Ox assumes you don't.

## Inspiration

### seL4

The world's most formally verified microkernel. Proven mathematically correct. Ox borrows:
- Capability-based security model
- Minimal kernel surface area
- IPC as the fundamental primitive

### L4

Fast microkernel family showing IPC overhead can be minimal. Ox targets similar per-message latency (~1μs).

### Erlang/OTP

"Let it crash" philosophy. Instead of defensive programming everywhere, let failures happen and restart cleanly. Ox's supervisor system is directly inspired by OTP supervisors.

### QNX

Real-time microkernel used in cars, medical devices, nuclear plants. Proof that microkernels work in safety-critical systems.

## Implementation Notes

### Why "Servers" Not "Processes"?

On a microcontroller, we don't have virtual memory or true process isolation. "Server" better describes what we have: isolated async tasks communicating via IPC.

### Why Embassy Channels for IPC?

Embassy channels provide:
- Zero-allocation message passing
- Async/await integration
- Bounded queues with backpressure
- ~0.5μs send latency

Perfect fit for microkernel IPC.

### Capability Enforcement

Capabilities are enforced two ways:
1. **Compile-time**: Generic types prevent passing wrong capability
2. **Runtime**: Unique IDs prevent forgery across server boundaries

## References

- [seL4 Whitepaper](https://sel4.systems/About/seL4-whitepaper.pdf)
- [L4 Microkernel Family](https://en.wikipedia.org/wiki/L4_microkernel_family)
- [QNX Neutrino RTOS](https://blackberry.qnx.com/en/software-solutions/embedded-software/qnx-neutrino-rtos)
- [Erlang Supervisor Behaviour](https://www.erlang.org/doc/design_principles/sup_princ.html)
