---
sidebar_position: 6
---

# ADR 0005: Embassy Async Over Traditional RTOS

**Status:** Accepted
**Date:** 2024-01

## Context

Embedded systems traditionally use Real-Time Operating Systems (RTOS) for multitasking:

### Traditional RTOS Model

Preemptive scheduling with threads:

```c
// FreeRTOS style
void motor_task(void* params) {
    while (1) {
        read_encoder();
        compute_pid();
        set_pwm();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void sensor_task(void* params) {
    while (1) {
        read_imu();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Each task has its own stack
xTaskCreate(motor_task, "motor", 2048, NULL, 5, NULL);
xTaskCreate(sensor_task, "sensor", 2048, NULL, 3, NULL);
```

Problems:
- **Stack per task** — 2KB × N tasks = significant RAM
- **Context switch overhead** — Save/restore registers
- **Synchronization complexity** — Mutexes, semaphores, priority inversion
- **Preemption bugs** — Hard to debug race conditions

### Async/Await Model

Cooperative multitasking with state machines:

```rust
// Embassy style
#[embassy_executor::task]
async fn motor_task() {
    loop {
        read_encoder().await;
        compute_pid();
        set_pwm();
        Timer::after_millis(1).await;
    }
}

#[embassy_executor::task]
async fn sensor_task() {
    loop {
        read_imu().await;
        Timer::after_millis(10).await;
    }
}
```

Benefits:
- **Shared stack** — Tasks yield, don't need separate stacks
- **No context switch** — State machine transitions, not register saves
- **No preemption races** — Yield points are explicit
- **Compiler-verified** — Borrow checker catches data races

## Decision

**Use Embassy async executor instead of a traditional RTOS.**

### Why Embassy?

Embassy is an async runtime designed for embedded Rust:

| Feature | Embassy | FreeRTOS |
|---------|---------|----------|
| Multitasking | Cooperative async | Preemptive threads |
| Stack usage | Shared | Per-task (2KB+ each) |
| Context switch | State machine | Register save/restore |
| Synchronization | Channels, signals | Mutexes, semaphores |
| Data race safety | Compile-time (Rust) | Runtime (hope) |
| ESP32 support | First-class | Via esp-idf |

### Memory Efficiency

Compare stack usage for 10 tasks:

```
FreeRTOS: 10 tasks × 2KB stack = 20KB minimum
Embassy:  10 tasks × ~200B state = 2KB + shared stack
```

On an ESP32-C3 with 400KB RAM, this matters.

### No Preemption Races

In an RTOS, any task can be preempted at any instruction:

```c
// FreeRTOS: Race condition
shared_value++;  // Not atomic! Task switch here = bug
```

In Embassy, yield points are explicit:

```rust
// Embassy: No race
shared_value += 1;  // Can't be interrupted here
channel.send(x).await;  // Yield point is explicit
shared_value += 1;  // Also safe
```

The borrow checker ensures you can't hold references across yield points incorrectly.

### Integrated Hardware Abstraction

Embassy provides async HAL traits:

```rust
// Async I2C read
let data = i2c.read(address, &mut buffer).await?;

// Async UART write
uart.write_all(&message).await?;

// Async timer
Timer::after_secs(1).await;
```

Hardware operations don't block — they yield to other tasks.

## Consequences

### Positive

- **Lower RAM usage** — Shared stack, smaller task state
- **No preemption bugs** — Explicit yield points
- **Compile-time safety** — Borrow checker catches races
- **Integrated with Rust** — Native async/await syntax
- **Good ESP32 support** — Embassy + esp-hal work well together

### Negative

- **Cooperative scheduling** — Long computations block others
- **Learning curve** — Async Rust has its own complexity
- **Debugging** — Async stack traces can be confusing
- **No hard preemption** — Can't interrupt a task mid-computation

### Mitigations

For long computations:

```rust
async fn long_computation() {
    for chunk in data.chunks(100) {
        process(chunk);
        yield_now().await;  // Let other tasks run
    }
}
```

For hard real-time interrupts:

```rust
// Use interrupt handlers for truly critical timing
#[interrupt]
fn TIM0() {
    // Runs with interrupt priority, not task priority
    critical_timing_code();
}
```

## Comparison: FreeRTOS vs Embassy

### Task Creation

```c
// FreeRTOS
xTaskCreate(my_task, "task", 2048, NULL, 5, &handle);
```

```rust
// Embassy
#[embassy_executor::task]
async fn my_task() { }
spawner.spawn(my_task()).unwrap();
```

### Delays

```c
// FreeRTOS
vTaskDelay(pdMS_TO_TICKS(100));
```

```rust
// Embassy
Timer::after_millis(100).await;
```

### Synchronization

```c
// FreeRTOS
xSemaphoreTake(mutex, portMAX_DELAY);
shared_resource++;
xSemaphoreGive(mutex);
```

```rust
// Embassy
{
    let mut guard = mutex.lock().await;
    *guard += 1;
}  // Released automatically
```

### Message Passing

```c
// FreeRTOS
xQueueSend(queue, &message, portMAX_DELAY);
xQueueReceive(queue, &received, portMAX_DELAY);
```

```rust
// Embassy
channel.send(message).await;
let received = channel.receive().await;
```

## When to Use Interrupts

Embassy tasks are for application logic. Use interrupts for:

- **Microsecond-precision timing** — PWM edges, encoder pulses
- **Hardware events** — Button presses, limit switches
- **DMA completion** — Signal task to process buffer

```rust
#[embassy_executor::task]
async fn motor_control() {
    loop {
        // Wait for encoder interrupt to signal new position
        ENCODER_SIGNAL.wait().await;
        let position = ENCODER_COUNT.load(Ordering::Relaxed);
        // Process...
    }
}

#[interrupt]
fn PCNT() {
    // Encoder pulse detected
    ENCODER_COUNT.fetch_add(1, Ordering::Relaxed);
    ENCODER_SIGNAL.signal(());
}
```

## References

- [Embassy Book](https://embassy.dev/book/dev/index.html)
- [Async Rust Book](https://rust-lang.github.io/async-book/)
- [Embassy vs RTOS Discussion](https://github.com/embassy-rs/embassy/discussions/1292)
- [esp-hal Embassy Integration](https://github.com/esp-rs/esp-hal)
