//! Supervisor and watchdog functionality
//!
//! The supervisor monitors server health via heartbeats and can
//! trigger server restarts on failure.

#[cfg(not(test))]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(not(test))]
use embassy_sync::signal::Signal;

// For tests, we use a minimal Duration type that doesn't require the time driver
#[cfg(test)]
pub use test_types::Duration;

#[cfg(not(test))]
pub use embassy_time::Duration;

#[cfg(not(test))]
use embassy_time::Instant;

/// Error indicating a task failed to send heartbeat in time
#[derive(Debug, Clone, Copy)]
pub struct HeartbeatTimeout;

/// Heartbeat signal that tasks must pulse periodically
///
/// If a task fails to call `beat()` within the timeout period,
/// the supervisor can detect the failure and take action.
#[cfg(not(test))]
pub struct Heartbeat {
    last_beat: Signal<CriticalSectionRawMutex, Instant>,
    timeout: Duration,
}

/// Test-only Heartbeat that doesn't require embassy runtime
#[cfg(test)]
pub struct Heartbeat {
    timeout: Duration,
    healthy: core::sync::atomic::AtomicBool,
}

#[cfg(not(test))]
impl Heartbeat {
    /// Create a new heartbeat monitor with the given timeout
    pub const fn new(timeout: Duration) -> Self {
        Self {
            last_beat: Signal::new(),
            timeout,
        }
    }

    /// Called by supervised task to indicate health
    pub fn beat(&self) {
        self.last_beat.signal(Instant::now());
    }

    /// Get the configured timeout duration
    pub fn timeout(&self) -> Duration {
        self.timeout
    }

    /// Check if heartbeat is healthy (non-blocking)
    ///
    /// Returns true if a beat was received recently, false otherwise
    pub fn is_healthy(&self) -> bool {
        // This is a simplified check - in practice we'd need
        // to track the last beat time
        true // TODO: Implement proper timing check
    }

    /// Wait for the next heartbeat with timeout
    ///
    /// Returns Ok(()) if heartbeat received, Err if timeout
    pub async fn wait_timeout(&self) -> Result<(), HeartbeatTimeout> {
        match embassy_time::with_timeout(self.timeout, self.last_beat.wait()).await {
            Ok(_) => Ok(()),
            Err(_) => Err(HeartbeatTimeout),
        }
    }
}

#[cfg(test)]
impl Heartbeat {
    /// Create a new heartbeat monitor with the given timeout
    pub const fn new(timeout: Duration) -> Self {
        Self {
            timeout,
            healthy: core::sync::atomic::AtomicBool::new(true),
        }
    }

    /// Called by supervised task to indicate health
    pub fn beat(&self) {
        self.healthy
            .store(true, core::sync::atomic::Ordering::SeqCst);
    }

    /// Get the configured timeout duration
    pub fn timeout(&self) -> Duration {
        self.timeout
    }

    /// Check if heartbeat is healthy (non-blocking)
    pub fn is_healthy(&self) -> bool {
        self.healthy.load(core::sync::atomic::Ordering::SeqCst)
    }
}

/// Supervisor configuration
pub struct SupervisorConfig {
    /// How often to check heartbeats
    pub check_interval: Duration,
    /// Maximum restart attempts before giving up
    pub max_restarts: u8,
}

impl Default for SupervisorConfig {
    fn default() -> Self {
        Self {
            check_interval: Duration::from_millis(100),
            max_restarts: 3,
        }
    }
}

/// Test-only types that don't require embassy runtime
#[cfg(test)]
mod test_types {
    /// Minimal Duration type for host testing
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
    pub struct Duration {
        ticks: u64,
    }

    impl Duration {
        /// Ticks per second (1MHz for 1us resolution)
        const TICKS_PER_SEC: u64 = 1_000_000;

        /// Create duration from milliseconds
        pub const fn from_millis(ms: u64) -> Self {
            Self {
                ticks: ms * (Self::TICKS_PER_SEC / 1000),
            }
        }

        /// Create duration from seconds
        pub const fn from_secs(secs: u64) -> Self {
            Self {
                ticks: secs * Self::TICKS_PER_SEC,
            }
        }

        /// Get duration as milliseconds
        pub const fn as_millis(&self) -> u64 {
            self.ticks / (Self::TICKS_PER_SEC / 1000)
        }

        /// Get duration as seconds
        pub const fn as_secs(&self) -> u64 {
            self.ticks / Self::TICKS_PER_SEC
        }
    }
}

/// Server status as tracked by supervisor
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ServerStatus {
    /// Server is running normally
    Running,
    /// Server missed heartbeat, may be hung
    Unresponsive,
    /// Server is being restarted
    Restarting,
    /// Server failed to restart, given up
    Failed,
}

/// Tests for supervisor module
///
/// These tests use a mock Duration type for host testing without embassy runtime.
#[cfg(test)]
mod tests {
    use super::*;

    // ==================== Server Status Tests ====================

    #[test]
    fn server_status_running() {
        let status = ServerStatus::Running;
        assert_eq!(status, ServerStatus::Running);
    }

    #[test]
    fn server_status_unresponsive() {
        let status = ServerStatus::Unresponsive;
        assert_eq!(status, ServerStatus::Unresponsive);
    }

    #[test]
    fn server_status_restarting() {
        let status = ServerStatus::Restarting;
        assert_eq!(status, ServerStatus::Restarting);
    }

    #[test]
    fn server_status_failed() {
        let status = ServerStatus::Failed;
        assert_eq!(status, ServerStatus::Failed);
    }

    #[test]
    fn server_status_can_be_cloned() {
        let status1 = ServerStatus::Running;
        let status2 = status1;
        assert_eq!(status1, status2);
    }

    #[test]
    fn server_status_can_be_copied() {
        let status1 = ServerStatus::Running;
        let status2 = status1;
        // Both are still valid (Copy semantics)
        assert_eq!(status1, ServerStatus::Running);
        assert_eq!(status2, ServerStatus::Running);
    }

    #[test]
    fn server_status_inequality() {
        assert_ne!(ServerStatus::Running, ServerStatus::Unresponsive);
        assert_ne!(ServerStatus::Running, ServerStatus::Restarting);
        assert_ne!(ServerStatus::Running, ServerStatus::Failed);
        assert_ne!(ServerStatus::Unresponsive, ServerStatus::Restarting);
        assert_ne!(ServerStatus::Unresponsive, ServerStatus::Failed);
        assert_ne!(ServerStatus::Restarting, ServerStatus::Failed);
    }

    #[test]
    fn server_status_implements_debug() {
        extern crate std;
        use std::format;

        assert!(format!("{:?}", ServerStatus::Running).contains("Running"));
        assert!(format!("{:?}", ServerStatus::Unresponsive).contains("Unresponsive"));
        assert!(format!("{:?}", ServerStatus::Restarting).contains("Restarting"));
        assert!(format!("{:?}", ServerStatus::Failed).contains("Failed"));
    }

    #[test]
    fn server_status_all_variants_distinct() {
        let variants = [
            ServerStatus::Running,
            ServerStatus::Unresponsive,
            ServerStatus::Restarting,
            ServerStatus::Failed,
        ];

        // Each variant should only equal itself
        for i in 0..variants.len() {
            for j in 0..variants.len() {
                if i == j {
                    assert_eq!(variants[i], variants[j]);
                } else {
                    assert_ne!(variants[i], variants[j]);
                }
            }
        }
    }

    // ==================== HeartbeatTimeout Tests ====================

    #[test]
    fn heartbeat_timeout_implements_debug() {
        extern crate std;
        use std::format;

        let timeout = HeartbeatTimeout;
        let debug_str = format!("{:?}", timeout);
        assert!(debug_str.contains("HeartbeatTimeout"));
    }

    #[test]
    fn heartbeat_timeout_can_be_cloned() {
        let t1 = HeartbeatTimeout;
        let t2 = t1;
        let _ = (t1, t2); // Both usable
    }

    #[test]
    fn heartbeat_timeout_can_be_copied() {
        let t1 = HeartbeatTimeout;
        let t2 = t1;
        let t3 = t2;
        let _ = (t1, t2, t3);
    }

    #[test]
    fn heartbeat_timeout_is_zero_sized() {
        // HeartbeatTimeout is a unit struct, should be zero-sized
        assert_eq!(core::mem::size_of::<HeartbeatTimeout>(), 0);
    }

    // ==================== Duration Tests (using test mock) ====================

    #[test]
    fn duration_from_millis() {
        let d = Duration::from_millis(500);
        assert_eq!(d.as_millis(), 500);
    }

    #[test]
    fn duration_from_secs() {
        let d = Duration::from_secs(2);
        assert_eq!(d.as_secs(), 2);
    }

    #[test]
    fn duration_comparison() {
        let short = Duration::from_millis(100);
        let long = Duration::from_millis(1000);
        assert!(short < long);
    }

    #[test]
    fn duration_equality() {
        let d1 = Duration::from_millis(100);
        let d2 = Duration::from_millis(100);
        assert_eq!(d1, d2);
    }

    #[test]
    fn duration_millis_to_secs() {
        let d = Duration::from_millis(2000);
        assert_eq!(d.as_secs(), 2);
    }

    // ==================== SupervisorConfig Tests ====================

    #[test]
    fn supervisor_config_max_restarts_default() {
        let config = SupervisorConfig::default();
        assert_eq!(config.max_restarts, 3);
    }

    #[test]
    fn supervisor_config_check_interval_default() {
        let config = SupervisorConfig::default();
        assert_eq!(config.check_interval, Duration::from_millis(100));
    }

    #[test]
    fn supervisor_config_max_restarts_custom() {
        let config = SupervisorConfig {
            check_interval: Duration::from_millis(100),
            max_restarts: 5,
        };
        assert_eq!(config.max_restarts, 5);
    }

    #[test]
    fn supervisor_config_check_interval_custom() {
        let config = SupervisorConfig {
            check_interval: Duration::from_millis(50),
            max_restarts: 5,
        };
        assert_eq!(config.check_interval, Duration::from_millis(50));
    }

    #[test]
    fn supervisor_config_zero_restarts() {
        let config = SupervisorConfig {
            check_interval: Duration::from_millis(100),
            max_restarts: 0,
        };
        assert_eq!(config.max_restarts, 0);
    }

    #[test]
    fn supervisor_config_max_restarts_u8_max() {
        let config = SupervisorConfig {
            check_interval: Duration::from_millis(100),
            max_restarts: u8::MAX,
        };
        assert_eq!(config.max_restarts, 255);
    }

    #[test]
    fn supervisor_config_long_interval() {
        let config = SupervisorConfig {
            check_interval: Duration::from_secs(60),
            max_restarts: 10,
        };
        assert_eq!(config.check_interval, Duration::from_secs(60));
    }

    // ==================== Heartbeat Tests (using test mock) ====================

    #[test]
    fn heartbeat_can_be_created() {
        let _hb = Heartbeat::new(Duration::from_millis(100));
    }

    #[test]
    fn heartbeat_stores_timeout() {
        let hb = Heartbeat::new(Duration::from_millis(250));
        assert_eq!(hb.timeout(), Duration::from_millis(250));
    }

    #[test]
    fn heartbeat_with_different_timeouts() {
        let hb1 = Heartbeat::new(Duration::from_millis(50));
        let hb2 = Heartbeat::new(Duration::from_secs(1));

        assert_eq!(hb1.timeout(), Duration::from_millis(50));
        assert_eq!(hb2.timeout(), Duration::from_secs(1));
    }

    #[test]
    fn heartbeat_beat_does_not_panic() {
        let hb = Heartbeat::new(Duration::from_millis(100));
        hb.beat();
        hb.beat();
        hb.beat();
    }

    #[test]
    fn heartbeat_is_healthy_after_creation() {
        let hb = Heartbeat::new(Duration::from_millis(100));
        assert!(hb.is_healthy());
    }

    #[test]
    fn heartbeat_remains_healthy_after_beat() {
        let hb = Heartbeat::new(Duration::from_millis(100));
        hb.beat();
        assert!(hb.is_healthy());
    }

    #[test]
    fn heartbeat_timeout_long_duration() {
        let hb = Heartbeat::new(Duration::from_secs(3600)); // 1 hour
        assert_eq!(hb.timeout(), Duration::from_secs(3600));
    }

    #[test]
    fn heartbeat_timeout_short_duration() {
        let hb = Heartbeat::new(Duration::from_millis(1));
        assert_eq!(hb.timeout(), Duration::from_millis(1));
    }
}
