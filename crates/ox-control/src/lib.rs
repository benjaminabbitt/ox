//! Ox Control Algorithms
//!
//! Control theory implementations for robotics:
//! - PID controllers
//! - Filters (low-pass, complementary, Kalman)
//! - State estimation

#![no_std]

pub mod pid;
pub mod filter;

pub use pid::PidController;
pub use filter::{LowPassFilter, ComplementaryFilter};
