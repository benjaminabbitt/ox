//! Ox Microkernel Servers
//!
//! All drivers run as "servers" outside the kernel, communicating
//! via IPC channels. Each server receives capabilities from the
//! kernel to access hardware.

#![no_std]

pub mod motor;
pub mod gpio;
pub mod sensor;

#[cfg(feature = "wifi")]
pub mod comms;

#[cfg(feature = "rc")]
pub mod rc;

#[cfg(feature = "can")]
pub mod can;
