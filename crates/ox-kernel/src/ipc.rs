//! IPC (Inter-Process Communication) primitives
//!
//! Built on Embassy channels for ~0.5-2μs latency.
//! All communication between servers goes through these primitives.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;

/// Type alias for the mutex type used in IPC
pub type IpcMutex = CriticalSectionRawMutex;

/// A streaming data channel with backpressure
///
/// Used for continuous data flow (sensor readings, telemetry, etc.)
pub type Stream<T, const N: usize> = Channel<IpcMutex, T, N>;

/// A notification signal (fire-and-forget)
///
/// Used to wake up a task without sending data
pub struct Notify {
    signal: Signal<IpcMutex, ()>,
}

impl Notify {
    /// Create a new notification signal
    pub const fn new() -> Self {
        Self {
            signal: Signal::new(),
        }
    }

    /// Send a notification (non-blocking)
    pub fn notify(&self) {
        self.signal.signal(());
    }

    /// Wait for a notification
    pub async fn wait(&self) {
        self.signal.wait().await;
    }

    /// Reset the notification state
    pub fn reset(&self) {
        self.signal.reset();
    }
}

impl Default for Notify {
    fn default() -> Self {
        Self::new()
    }
}

/// Response slot for synchronous calls
pub struct ResponseSlot<T> {
    signal: Signal<IpcMutex, Option<T>>,
}

impl<T> ResponseSlot<T> {
    /// Create a new response slot
    pub const fn new() -> Self {
        Self {
            signal: Signal::new(),
        }
    }

    /// Send a response
    pub fn respond(&self, value: T) {
        self.signal.signal(Some(value));
    }

    /// Wait for the response
    pub async fn recv(&self) -> T {
        loop {
            if let Some(v) = self.signal.wait().await {
                return v;
            }
        }
    }
}

impl<T> Default for ResponseSlot<T> {
    fn default() -> Self {
        Self::new()
    }
}

/// A synchronous call primitive (request-response pattern)
///
/// Models the microkernel "call" syscall pattern.
/// Caller blocks until server responds.
pub struct Call<Req, Resp, const N: usize = 4> {
    requests: Channel<IpcMutex, (Req, *const ResponseSlot<Resp>), N>,
}

impl<Req, Resp, const N: usize> Call<Req, Resp, N> {
    /// Create a new call channel
    pub const fn new() -> Self {
        Self {
            requests: Channel::new(),
        }
    }

    /// Client side: make a synchronous call and await response
    ///
    /// # Safety
    /// The ResponseSlot must remain valid until the response is received.
    pub async fn call(&self, request: Req, slot: &ResponseSlot<Resp>) -> Resp {
        // Send request with pointer to response slot
        self.requests
            .send((request, slot as *const ResponseSlot<Resp>))
            .await;
        // Wait for response
        slot.recv().await
    }

    /// Server side: receive a request
    pub async fn recv(&self) -> (Req, *const ResponseSlot<Resp>) {
        self.requests.receive().await
    }

    /// Server side: try to receive a request (non-blocking)
    pub fn try_recv(&self) -> Option<(Req, *const ResponseSlot<Resp>)> {
        self.requests.try_receive().ok()
    }
}

impl<Req, Resp, const N: usize> Default for Call<Req, Resp, N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Inter-core channel for ESP32-C6 dual-core communication
///
/// Uses shared RAM for communication between HP and LP cores.
#[cfg(feature = "dual-core")]
pub struct InterCoreChannel<T, const N: usize> {
    // TODO: Implement using shared memory region
    inner: Channel<IpcMutex, T, N>,
}

#[cfg(feature = "dual-core")]
impl<T, const N: usize> InterCoreChannel<T, N> {
    pub const fn new() -> Self {
        Self {
            inner: Channel::new(),
        }
    }

    pub async fn send(&self, value: T) {
        self.inner.send(value).await;
    }

    pub async fn receive(&self) -> T {
        self.inner.receive().await
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Note: These tests run on the host, not embedded target
    // Embassy async tests require special setup

    #[test]
    fn notify_can_be_created() {
        let _notify = Notify::new();
    }

    #[test]
    fn stream_can_be_created() {
        let _stream: Stream<u32, 4> = Channel::new();
    }

    #[test]
    fn call_can_be_created() {
        let _call: Call<u32, u32, 4> = Call::new();
    }
}
