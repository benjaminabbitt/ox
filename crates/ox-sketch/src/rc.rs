//! RC receiver handle for reading radio control input
//!
//! Provides a simple async interface for reading RC channels from
//! CRSF/ELRS, SBUS, or other protocols.

use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Read;

// Re-export useful types from ox-services
pub use ox_services::rc::{ChannelMap, CrsfDecoder, FailsafeConfig, RcChannels, RcServer};

/// RC receiver handle for reading control input
///
/// # Example
/// ```ignore
/// let rc = RcHandle::new_crsf(uart_rx);
///
/// loop {
///     let input = rc.read().await;
///     if !input.failsafe {
///         motor.drive(input.throttle, input.steering);
///     }
/// }
/// ```
pub struct RcHandle<RX> {
    rx: RX,
    decoder: CrsfDecoder,
    server: RcServer,
    last_update: Instant,
}

/// Simplified RC input data
#[derive(Debug, Clone, Copy, Default)]
pub struct RcInput {
    /// Throttle value [-1.0, 1.0]
    pub throttle: f32,
    /// Steering value [-1.0, 1.0]
    pub steering: f32,
    /// Arm switch active
    pub armed: bool,
    /// In failsafe mode (no signal)
    pub failsafe: bool,
    /// Link quality (0-100, if available)
    pub link_quality: u8,
    /// RSSI (0-100, if available)
    pub rssi: u8,
}

impl<RX: Read> RcHandle<RX> {
    /// Create a new RC handle with CRSF decoder
    ///
    /// CRSF is used by ExpressLRS and TBS Crossfire receivers.
    /// Requires UART configured at 420000 baud.
    pub fn new_crsf(rx: RX) -> Self {
        Self {
            rx,
            decoder: CrsfDecoder::new(),
            server: RcServer::default(),
            last_update: Instant::now(),
        }
    }

    /// Create with custom channel mapping
    pub fn new_crsf_with_map(rx: RX, map: ChannelMap) -> Self {
        Self {
            rx,
            decoder: CrsfDecoder::new(),
            server: RcServer::new(map, FailsafeConfig::default()),
            last_update: Instant::now(),
        }
    }

    /// Read the next RC input (blocks until data available)
    ///
    /// This reads bytes from UART and decodes them. Returns when
    /// a complete frame is received.
    pub async fn read(&mut self) -> RcInput {
        let mut buf = [0u8; 1];

        loop {
            // Update failsafe timer
            let now = Instant::now();
            let elapsed_ms = now.duration_since(self.last_update).as_millis() as u32;
            self.last_update = now;
            self.server.update(elapsed_ms);

            // Try to read a byte
            match self.rx.read(&mut buf).await {
                Ok(_) => {
                    // Feed to decoder
                    if let Some(channels) = self.decoder.decode(buf[0]) {
                        self.server.process(&channels);

                        return RcInput {
                            throttle: self.server.throttle(),
                            steering: self.server.steering(),
                            armed: self.server.is_armed(),
                            failsafe: self.server.is_failsafe(),
                            link_quality: self.decoder.link_quality(),
                            rssi: self.decoder.rssi(),
                        };
                    }
                }
                Err(_) => {
                    // UART error, wait a bit and retry
                    Timer::after(Duration::from_millis(1)).await;
                }
            }
        }
    }

    /// Try to read without blocking
    ///
    /// Returns None if no complete frame is available.
    pub fn try_read(&mut self) -> Option<RcInput> {
        // Update failsafe timer
        let now = Instant::now();
        let elapsed_ms = now.duration_since(self.last_update).as_millis() as u32;
        self.last_update = now;
        self.server.update(elapsed_ms);

        // Return current state if we have valid data
        if !self.server.is_failsafe() {
            Some(RcInput {
                throttle: self.server.throttle(),
                steering: self.server.steering(),
                armed: self.server.is_armed(),
                failsafe: false,
                link_quality: self.decoder.link_quality(),
                rssi: self.decoder.rssi(),
            })
        } else {
            None
        }
    }

    /// Get current throttle value [-1.0, 1.0]
    pub fn throttle(&self) -> f32 {
        self.server.throttle()
    }

    /// Get current steering value [-1.0, 1.0]
    pub fn steering(&self) -> f32 {
        self.server.steering()
    }

    /// Check if armed
    pub fn is_armed(&self) -> bool {
        self.server.is_armed()
    }

    /// Check if in failsafe (no signal)
    pub fn is_failsafe(&self) -> bool {
        self.server.is_failsafe()
    }

    /// Get raw channel value by index
    pub fn channel(&self, index: usize) -> f32 {
        self.server.channels().get(index)
    }

    /// Get aux channel (1-4)
    pub fn aux(&self, index: usize) -> f32 {
        self.server.aux(index)
    }
}
