//! GPIO server
//!
//! Provides GPIO access to other servers/apps via IPC.

// These imports will be used when the server task is implemented
#[allow(unused_imports)]
use ox_kernel::ipc::{Call, Stream};
#[allow(unused_imports)]
use ox_kernel::cap::{Cap, GpioPin};
use ox_hal::gpio::PinMode;

/// GPIO command
#[derive(Debug, Clone, Copy)]
pub enum GpioCommand {
    SetMode { pin: u8, mode: PinMode },
    Write { pin: u8, value: bool },
    Read { pin: u8 },
    Toggle { pin: u8 },
}

/// GPIO response
#[derive(Debug, Clone, Copy)]
pub enum GpioResponse {
    Ok,
    Value(bool),
    Error(GpioError),
}

/// GPIO error codes
#[derive(Debug, Clone, Copy)]
pub enum GpioError {
    InvalidPin,
    NotConfigured,
    PermissionDenied,
}

// GPIO server task will be implemented when integrating with Embassy executor
