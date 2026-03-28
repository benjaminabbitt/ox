//! Ox Microkernel Core
//!
//! This crate provides the minimal kernel primitives:
//! - IPC (Inter-Process Communication) channels
//! - Capability-based access control
//! - Supervisor/watchdog functionality
//!
//! The kernel is designed to be ~500 lines of code, with all drivers
//! running as "servers" outside the kernel.

#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]

pub mod ipc;
pub mod cap;
pub mod supervisor;

// Re-export commonly used types
pub use ipc::{Call, Notify, Stream};
pub use cap::{Cap, Resource};
pub use supervisor::Heartbeat;
