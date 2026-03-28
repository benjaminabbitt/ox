//! Test harness utilities

/// Test result type
pub type TestResult = Result<(), TestError>;

/// Test error
#[derive(Debug)]
pub enum TestError {
    Assertion(String),
    Timeout,
    HardwareError,
}

/// Assert with custom message
#[macro_export]
macro_rules! assert_test {
    ($cond:expr, $msg:expr) => {
        if !$cond {
            return Err(TestError::Assertion($msg.to_string()));
        }
    };
}
