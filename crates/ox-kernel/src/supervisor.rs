//! Supervisor and watchdog functionality
//!
//! The supervisor monitors server health via heartbeats and can
//! trigger server restarts on failure.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant};

/// Error indicating a task failed to send heartbeat in time
#[derive(Debug, Clone, Copy)]
pub struct HeartbeatTimeout;

/// Heartbeat signal that tasks must pulse periodically
///
/// If a task fails to call `beat()` within the timeout period,
/// the supervisor can detect the failure and take action.
pub struct Heartbeat {
    last_beat: Signal<CriticalSectionRawMutex, Instant>,
    timeout: Duration,
}

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heartbeat_can_be_created() {
        let _hb = Heartbeat::new(Duration::from_millis(100));
    }

    #[test]
    fn supervisor_config_has_defaults() {
        let config = SupervisorConfig::default();
        assert_eq!(config.max_restarts, 3);
    }
}
