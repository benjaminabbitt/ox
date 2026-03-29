---
sidebar_position: 5
---

# ADR 0004: Real-Time Design Requirements

**Status:** Accepted
**Date:** 2024-01

## Context

"Real-time" means different things to different people:

### Hard Real-Time

Missing a deadline is a system failure. Examples:
- Airbag deployment (10ms or useless)
- Pacemaker pulse timing
- Anti-lock braking

### Soft Real-Time

Missing deadlines degrades quality but isn't catastrophic. Examples:
- Video streaming (dropped frames are annoying)
- Music playback (glitches are noticeable)
- Game rendering (low FPS but still playable)

### Firm Real-Time

Missing occasional deadlines is tolerable, but the result has no value. Examples:
- Financial trading (late trade is worthless, not dangerous)
- Sensor sampling (late sample is discarded)

### Robotics Reality

Most robotics falls between hard and firm:

| Application | Loop Rate | Deadline Miss Consequence |
|-------------|-----------|---------------------------|
| Quadcopter attitude | 1-4 kHz | Crash within seconds |
| Combat robot drive | 100-500 Hz | Sluggish response, lose match |
| Rocket guidance | 100-400 Hz | Off-course, possible failure |
| Ground robot nav | 10-50 Hz | Bumps into things |
| Arm kinematics | 50-200 Hz | Jerky motion, imprecision |

## Decision

**Design Ox for firm real-time with soft real-time guarantees.**

### Target Performance

| Metric | Target | Rationale |
|--------|--------|-----------|
| Control loop rate | 1 kHz | Standard for flight control |
| Loop jitter | <100μs | Consistent timing for PID |
| IPC latency | <5μs | Server communication overhead |
| Interrupt latency | <10μs | Respond to hardware events |
| Worst-case latency | <1ms | Bound on maximum delay |

### Design Principles

#### 1. No Garbage Collection

Rust has no GC. Memory is freed deterministically when ownership ends.

```rust
{
    let buffer = Vec::new();  // Allocated
    // Use buffer...
}  // Freed here, deterministically
```

No surprise pauses. No "stop the world" collection.

#### 2. No Unbounded Allocation

All allocations happen at startup or use bounded pools:

```rust
// Static allocation at compile time
static CHANNEL: Channel<Mutex, Message, 8> = Channel::new();

// Bounded pool, never grows
static POOL: Pool<[u8; 256], 16> = Pool::new();
```

No heap fragmentation. No allocation failures mid-operation.

#### 3. No Blocking in Control Path

Control loops use async/await, never block:

```rust
// GOOD: Async, yields to other tasks
let command = motor_channel.receive().await;

// BAD: Blocks entire executor
let command = motor_channel.receive_blocking();
```

Blocking would prevent other control loops from running.

#### 4. Priority-Based Scheduling

Critical tasks run at higher priority:

```rust
#[embassy_executor::task]
#[priority(HIGH)]
async fn attitude_control() {
    // Runs before lower-priority tasks
}

#[embassy_executor::task]
#[priority(LOW)]
async fn telemetry() {
    // Runs when high-priority tasks yield
}
```

Attitude control beats telemetry when CPU is contested.

#### 5. Bounded Queues

All IPC uses bounded queues with backpressure:

```rust
// Queue of 8 messages maximum
type MotorStream = Stream<MotorCommand, 8>;

// If full, sender waits (doesn't drop)
stream.send(command).await;
```

No unbounded growth. No memory exhaustion.

## Consequences

### Positive

- **Predictable timing** — Know your worst-case latency
- **No surprise pauses** — GC-free, allocation-bounded
- **Testable deadlines** — Measure and verify loop timing
- **Graceful degradation** — Bounded queues prevent cascade failures

### Negative

- **More upfront design** — Must think about timing early
- **Static allocation** — Can't dynamically resize buffers
- **Complexity** — Priority inversion, queue sizing, etc.

### What We Don't Guarantee

Ox is **not** a certified hard real-time system:
- No formal verification (unlike seL4)
- No safety certification (DO-178C, ISO 26262)
- ESP32 has interrupt latency variance

For hobby flight controllers: excellent.
For commercial aircraft: use certified solutions.

## Timing Budget

For a 1kHz control loop (1ms period):

| Operation | Budget | Notes |
|-----------|--------|-------|
| Sensor read | 100μs | I2C/SPI transaction |
| Attitude estimation | 50μs | Complementary filter |
| PID computation | 10μs | Three PID controllers |
| Motor output | 50μs | PWM update |
| IPC overhead | 10μs | ~2-3 messages |
| **Margin** | 780μs | For jitter and extensions |

With 78% margin, we can handle:
- Occasional cache misses
- Interrupt servicing
- Additional sensors

## Measurement

Validate timing with instrumentation:

```rust
#[embassy_executor::task]
async fn control_loop() {
    let mut max_duration = Duration::ZERO;

    loop {
        let start = Instant::now();

        // Control loop body

        let duration = start.elapsed();
        max_duration = max_duration.max(duration);

        if duration > Duration::from_micros(900) {
            defmt::warn!("Loop overrun: {}μs", duration.as_micros());
        }

        Timer::after(Duration::from_millis(1)).await;
    }
}
```

Log overruns. Fix them.

## References

- [Real-Time Systems Design](https://www.cs.cmu.edu/~raj/real-time.html)
- [Rate Monotonic Scheduling](https://en.wikipedia.org/wiki/Rate-monotonic_scheduling)
- [Embassy Executor Priorities](https://embassy.dev/book/dev/runtime.html)
