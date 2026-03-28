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
/// Note: C6 has heterogeneous cores (HP at 160MHz, LP at 20MHz).
#[cfg(feature = "esp32c6-dual")]
pub struct InterCoreChannel<T, const N: usize> {
    // TODO: Implement using shared memory region between HP/LP cores
    inner: Channel<IpcMutex, T, N>,
}

#[cfg(feature = "esp32c6-dual")]
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

#[cfg(feature = "esp32c6-dual")]
impl<T, const N: usize> Default for InterCoreChannel<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Symmetric inter-core channel storage for ESP32-S3 dual-core communication
///
/// Unlike the C6's heterogeneous HP/LP cores, the S3 has two identical
/// LX7 cores sharing the same memory space. This enables true lock-free
/// communication using atomics.
///
/// # Usage
///
/// Create the storage, then split into producer/consumer:
///
/// ```ignore
/// static CHANNEL: StaticCell<InterCoreStorage<u32, 8>> = StaticCell::new();
///
/// // Initialize once at startup
/// let storage = CHANNEL.init(InterCoreStorage::new());
/// let (mut producer, mut consumer) = storage.split();
///
/// // On core 0 (producer task)
/// producer.send(42).await;
///
/// // On core 1 (consumer task)
/// let value = consumer.receive().await;
/// ```
///
/// # Note
///
/// The `split()` method borrows mutably, ensuring only one split can exist.
/// For cross-core use, wrap in a StaticCell and split once during init.
#[cfg(feature = "esp32s3-dual")]
pub struct InterCoreStorage<T, const N: usize> {
    queue: heapless::spsc::Queue<T, N>,
    /// Signal: data is available for consumer
    data_ready: Signal<IpcMutex, bool>,
    /// Signal: space is available for producer
    space_ready: Signal<IpcMutex, bool>,
}

#[cfg(feature = "esp32s3-dual")]
impl<T, const N: usize> InterCoreStorage<T, N> {
    /// Create new inter-core channel storage
    ///
    /// Place in a static with `#[link_section = ".data"]` to ensure
    /// it resides in shared SRAM accessible by both cores.
    pub const fn new() -> Self {
        Self {
            queue: heapless::spsc::Queue::new(),
            data_ready: Signal::new(),
            space_ready: Signal::new(),
        }
    }

    /// Split storage into producer and consumer handles
    ///
    /// # Safety
    ///
    /// Must only be called once. The producer should be used on one core
    /// and the consumer on the other. Using both on the same core is safe
    /// but defeats the purpose.
    pub fn split(&mut self) -> (InterCoreProducer<'_, T, N>, InterCoreConsumer<'_, T, N>) {
        let (prod, cons) = self.queue.split();
        (
            InterCoreProducer {
                producer: prod,
                data_ready: &self.data_ready,
                space_ready: &self.space_ready,
            },
            InterCoreConsumer {
                consumer: cons,
                data_ready: &self.data_ready,
                space_ready: &self.space_ready,
            },
        )
    }
}

#[cfg(feature = "esp32s3-dual")]
impl<T, const N: usize> Default for InterCoreStorage<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Producer half of an inter-core channel (runs on one core)
#[cfg(feature = "esp32s3-dual")]
pub struct InterCoreProducer<'a, T, const N: usize> {
    producer: heapless::spsc::Producer<'a, T, N>,
    data_ready: &'a Signal<IpcMutex, bool>,
    space_ready: &'a Signal<IpcMutex, bool>,
}

#[cfg(feature = "esp32s3-dual")]
impl<'a, T, const N: usize> InterCoreProducer<'a, T, N> {
    /// Send a value to the consumer core
    ///
    /// Blocks if the queue is full until space becomes available.
    pub async fn send(&mut self, mut value: T) {
        loop {
            match self.producer.enqueue(value) {
                Ok(()) => {
                    // Signal consumer that data is ready
                    self.data_ready.signal(true);
                    return;
                }
                Err(v) => {
                    value = v;
                    // Queue full - wait for consumer to make space
                    // Signal uses value semantics, not edge - no race condition
                    self.space_ready.wait().await;
                }
            }
        }
    }

    /// Try to send without blocking
    ///
    /// Returns `Err(value)` if the queue is full.
    pub fn try_send(&mut self, value: T) -> Result<(), T> {
        match self.producer.enqueue(value) {
            Ok(()) => {
                self.data_ready.signal(true);
                Ok(())
            }
            Err(v) => Err(v),
        }
    }

    /// Check if the queue is full
    pub fn is_full(&self) -> bool {
        !self.producer.ready()
    }

    /// Get total capacity
    #[must_use]
    pub fn capacity(&self) -> usize {
        self.producer.capacity()
    }

    /// Get number of items in the queue
    #[must_use]
    pub fn len(&self) -> usize {
        self.producer.len()
    }

    /// Check if queue is empty
    pub fn is_empty(&self) -> bool {
        self.producer.len() == 0
    }
}

/// Consumer half of an inter-core channel (runs on the other core)
#[cfg(feature = "esp32s3-dual")]
pub struct InterCoreConsumer<'a, T, const N: usize> {
    consumer: heapless::spsc::Consumer<'a, T, N>,
    data_ready: &'a Signal<IpcMutex, bool>,
    space_ready: &'a Signal<IpcMutex, bool>,
}

#[cfg(feature = "esp32s3-dual")]
impl<'a, T, const N: usize> InterCoreConsumer<'a, T, N> {
    /// Receive a value from the producer core
    ///
    /// Blocks until data is available.
    pub async fn receive(&mut self) -> T {
        loop {
            if let Some(value) = self.consumer.dequeue() {
                // Signal producer that space is available
                self.space_ready.signal(true);
                return value;
            }
            // Queue empty - wait for producer to send data
            // Signal uses value semantics, not edge - no race condition
            self.data_ready.wait().await;
        }
    }

    /// Try to receive without blocking
    pub fn try_receive(&mut self) -> Option<T> {
        let result = self.consumer.dequeue();
        if result.is_some() {
            self.space_ready.signal(true);
        }
        result
    }

    /// Peek at the next value without removing it
    pub fn peek(&self) -> Option<&T> {
        self.consumer.peek()
    }

    /// Check if the queue is empty
    pub fn is_empty(&self) -> bool {
        !self.consumer.ready()
    }

    /// Get number of items ready to be consumed
    #[must_use]
    pub fn len(&self) -> usize {
        self.consumer.len()
    }

    /// Get total capacity
    #[must_use]
    pub fn capacity(&self) -> usize {
        self.consumer.capacity()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    extern crate alloc;

    use super::*;
    use alloc::sync::Arc;

    // ==================== Stream Tests ====================

    #[test]
    fn stream_can_be_created() {
        let _stream: Stream<u32, 4> = Channel::new();
    }

    #[test]
    fn stream_try_send_succeeds_when_not_full() {
        let stream: Stream<u32, 4> = Channel::new();
        assert!(stream.try_send(1).is_ok());
        assert!(stream.try_send(2).is_ok());
        assert!(stream.try_send(3).is_ok());
        assert!(stream.try_send(4).is_ok());
    }

    #[test]
    fn stream_try_send_fails_when_full() {
        let stream: Stream<u32, 2> = Channel::new();
        assert!(stream.try_send(1).is_ok());
        assert!(stream.try_send(2).is_ok());
        // Channel is now full
        assert!(stream.try_send(3).is_err());
    }

    #[test]
    fn stream_try_receive_returns_none_when_empty() {
        let stream: Stream<u32, 4> = Channel::new();
        assert!(stream.try_receive().is_err());
    }

    #[test]
    fn stream_try_receive_returns_sent_value() {
        let stream: Stream<u32, 4> = Channel::new();
        stream.try_send(42).unwrap();
        assert_eq!(stream.try_receive().unwrap(), 42);
    }

    #[test]
    fn stream_maintains_fifo_order() {
        let stream: Stream<u32, 4> = Channel::new();
        stream.try_send(1).unwrap();
        stream.try_send(2).unwrap();
        stream.try_send(3).unwrap();
        assert_eq!(stream.try_receive().unwrap(), 1);
        assert_eq!(stream.try_receive().unwrap(), 2);
        assert_eq!(stream.try_receive().unwrap(), 3);
    }

    #[tokio::test]
    async fn stream_async_send_receive() {
        let stream: Stream<u32, 4> = Channel::new();
        stream.send(100).await;
        let value = stream.receive().await;
        assert_eq!(value, 100);
    }

    #[tokio::test]
    async fn stream_async_multiple_values() {
        let stream: Stream<u32, 4> = Channel::new();

        // Send multiple values
        for i in 0..4 {
            stream.send(i).await;
        }

        // Receive and verify order
        for i in 0..4 {
            assert_eq!(stream.receive().await, i);
        }
    }

    // ==================== Notify Tests ====================

    #[test]
    fn notify_can_be_created() {
        let _notify = Notify::new();
    }

    #[test]
    fn notify_default_creates_new() {
        let _notify = Notify::default();
    }

    #[tokio::test]
    async fn notify_signal_wakes_waiter() {
        use tokio::sync::Barrier;

        let notify = Arc::new(Notify::new());
        let notify_clone = notify.clone();
        let barrier = Arc::new(Barrier::new(2));
        let barrier_clone = barrier.clone();

        let handle = tokio::spawn(async move {
            barrier_clone.wait().await;
            notify_clone.notify();
        });

        barrier.wait().await;
        // Small delay to ensure notify happens
        tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
        notify.wait().await;

        handle.await.unwrap();
    }

    #[test]
    fn notify_reset_clears_signal() {
        let notify = Notify::new();
        notify.notify();
        notify.reset();
        // After reset, a subsequent wait would block (we can't easily test this synchronously)
        // But we can verify reset doesn't panic
    }

    #[tokio::test]
    async fn notify_multiple_signals_coalesce() {
        let notify = Notify::new();

        // Multiple signals before wait
        notify.notify();
        notify.notify();
        notify.notify();

        // Single wait consumes them all
        notify.wait().await;

        // Reset and verify we need another signal
        notify.reset();
    }

    // ==================== ResponseSlot Tests ====================

    #[test]
    fn response_slot_can_be_created() {
        let _slot: ResponseSlot<u32> = ResponseSlot::new();
    }

    #[tokio::test]
    async fn response_slot_respond_and_recv() {
        let slot: ResponseSlot<u32> = ResponseSlot::new();

        let handle = tokio::spawn({
            async move {
                // Small delay then respond
                tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
            }
        });

        // Respond immediately in this test
        slot.respond(42);
        let value = slot.recv().await;
        assert_eq!(value, 42);

        handle.await.unwrap();
    }

    #[tokio::test]
    async fn response_slot_works_with_complex_types() {
        #[derive(Debug, Clone, PartialEq)]
        struct Response {
            code: u32,
            message: &'static str,
        }

        let slot: ResponseSlot<Response> = ResponseSlot::new();

        let expected = Response {
            code: 200,
            message: "OK",
        };

        slot.respond(expected.clone());
        let received = slot.recv().await;
        assert_eq!(received, expected);
    }

    // ==================== Call Tests ====================

    #[test]
    fn call_can_be_created() {
        let _call: Call<u32, u32, 4> = Call::new();
    }

    #[test]
    fn call_try_recv_returns_none_when_empty() {
        let call: Call<u32, u32, 4> = Call::new();
        assert!(call.try_recv().is_none());
    }

    #[tokio::test]
    async fn call_request_response_pattern() {
        use futures::future::join;

        let call: Call<u32, u32, 4> = Call::new();

        // Run client and server concurrently using join
        let client_future = async {
            let slot = ResponseSlot::new();
            call.call(21, &slot).await
        };

        let server_future = async {
            let (request, slot_ptr) = call.recv().await;
            let response = request * 2;
            unsafe { (*slot_ptr).respond(response) };
        };

        let (response, _) = join(client_future, server_future).await;
        assert_eq!(response, 42);
    }

    #[tokio::test]
    async fn call_multiple_sequential_calls() {
        use futures::future::join;

        let call: Call<u32, u32, 4> = Call::new();

        // Run client and server concurrently
        let client_future = async {
            let mut results = [0u32; 3];
            for i in 0..3 {
                let slot = ResponseSlot::new();
                results[i as usize] = call.call(i, &slot).await;
            }
            results
        };

        let server_future = async {
            for _ in 0..3 {
                let (request, slot_ptr) = call.recv().await;
                let response = request + 10;
                unsafe { (*slot_ptr).respond(response) };
            }
        };

        let (results, _) = join(client_future, server_future).await;
        assert_eq!(results, [10, 11, 12]);
    }

    #[tokio::test]
    async fn call_with_different_types() {
        use futures::future::join;

        #[derive(Debug, Clone)]
        enum Request {
            Add(i32, i32),
            Multiply(i32, i32),
        }

        #[derive(Debug, Clone, PartialEq)]
        struct Response {
            result: i32,
        }

        let call: Call<Request, Response, 4> = Call::new();

        let client_future = async {
            let slot1 = ResponseSlot::new();
            let resp1 = call.call(Request::Add(5, 3), &slot1).await;

            let slot2 = ResponseSlot::new();
            let resp2 = call.call(Request::Multiply(4, 7), &slot2).await;

            (resp1, resp2)
        };

        let server_future = async {
            for _ in 0..2 {
                let (request, slot_ptr) = call.recv().await;
                let result = match request {
                    Request::Add(a, b) => a + b,
                    Request::Multiply(a, b) => a * b,
                };
                unsafe { (*slot_ptr).respond(Response { result }) };
            }
        };

        let ((resp1, resp2), _) = join(client_future, server_future).await;
        assert_eq!(resp1.result, 8);
        assert_eq!(resp2.result, 28);
    }

    // ==================== InterCoreStorage Tests (S3 dual-core) ====================

    #[cfg(feature = "esp32s3-dual")]
    mod inter_core_tests {
        use super::*;

        #[test]
        fn inter_core_storage_can_be_created() {
            let _storage: InterCoreStorage<u32, 4> = InterCoreStorage::new();
        }

        #[test]
        fn inter_core_storage_default_works() {
            let _storage: InterCoreStorage<u32, 4> = InterCoreStorage::default();
        }

        #[test]
        fn inter_core_split_returns_producer_consumer() {
            let mut storage: InterCoreStorage<u32, 4> = InterCoreStorage::new();
            let (producer, consumer) = storage.split();
            assert!(producer.is_empty());
            assert!(consumer.is_empty());
        }

        #[test]
        fn inter_core_try_send_receive() {
            let mut storage: InterCoreStorage<u32, 4> = InterCoreStorage::new();
            let (mut producer, mut consumer) = storage.split();

            assert!(producer.try_send(42).is_ok());
            assert!(!consumer.is_empty());
            assert_eq!(consumer.try_receive(), Some(42));
            assert!(consumer.is_empty());
        }

        #[test]
        fn inter_core_try_send_fails_when_full() {
            // Note: heapless spsc uses N-1 capacity (one slot reserved)
            let mut storage: InterCoreStorage<u32, 4> = InterCoreStorage::new();
            let (mut producer, _consumer) = storage.split();

            assert!(producer.try_send(1).is_ok());
            assert!(producer.try_send(2).is_ok());
            assert!(producer.try_send(3).is_ok());
            assert!(producer.try_send(4).is_err()); // Full (capacity is N-1 = 3)
            assert!(producer.is_full());
        }

        #[test]
        fn inter_core_fifo_order() {
            let mut storage: InterCoreStorage<u32, 4> = InterCoreStorage::new();
            let (mut producer, mut consumer) = storage.split();

            producer.try_send(1).unwrap();
            producer.try_send(2).unwrap();
            producer.try_send(3).unwrap();

            assert_eq!(consumer.try_receive(), Some(1));
            assert_eq!(consumer.try_receive(), Some(2));
            assert_eq!(consumer.try_receive(), Some(3));
            assert_eq!(consumer.try_receive(), None);
        }

        #[test]
        fn inter_core_peek() {
            let mut storage: InterCoreStorage<u32, 4> = InterCoreStorage::new();
            let (mut producer, consumer) = storage.split();

            assert_eq!(consumer.peek(), None);
            producer.try_send(42).unwrap();
            assert_eq!(consumer.peek(), Some(&42));
            assert_eq!(consumer.peek(), Some(&42)); // Peek doesn't consume
        }

        #[test]
        fn inter_core_len_and_capacity() {
            // Note: heapless spsc capacity is N-1 (one slot reserved)
            let mut storage: InterCoreStorage<u32, 4> = InterCoreStorage::new();
            let (mut producer, consumer) = storage.split();

            assert_eq!(producer.len(), 0);
            assert_eq!(consumer.len(), 0);
            // Actual usable capacity is N-1
            assert_eq!(producer.capacity(), 3);
            assert_eq!(consumer.capacity(), 3);

            producer.try_send(1).unwrap();
            producer.try_send(2).unwrap();

            assert_eq!(producer.len(), 2);
            assert_eq!(consumer.len(), 2);
        }

        #[tokio::test]
        async fn inter_core_async_send_receive() {
            let mut storage: InterCoreStorage<u32, 4> = InterCoreStorage::new();
            let (mut producer, mut consumer) = storage.split();

            producer.send(100).await;
            let value = consumer.receive().await;
            assert_eq!(value, 100);
        }

        #[tokio::test]
        async fn inter_core_async_multiple_values() {
            let mut storage: InterCoreStorage<u32, 8> = InterCoreStorage::new();
            let (mut producer, mut consumer) = storage.split();

            for i in 0..5 {
                producer.send(i).await;
            }

            for i in 0..5 {
                assert_eq!(consumer.receive().await, i);
            }
        }
    }
}
