//! Errors of this crate

use std::fmt;
use std::io;

/// What can go wrong talking to the sensor core
#[derive(Debug)]
pub enum Error {
    /// Socket error
    Io(io::Error),
    /// Bytes that don't parse as the message they should be
    Malformed(&'static str),
    /// QMI response with a failure result
    Qmi {
        /// QMI result code
        result: u16,
        /// QMI error code
        error: u16,
    },
    /// No answer in time
    Timeout,
    /// The sensor core service went away
    ServiceGone,
    /// Nothing of the requested kind
    NotFound,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(e) => write!(f, "socket: {e}"),
            Self::Malformed(what) => write!(f, "malformed {what}"),
            Self::Qmi { result, error } => write!(f, "QMI result {result} error {error}"),
            Self::Timeout => f.write_str("timed out"),
            Self::ServiceGone => f.write_str("sensor core service went away"),
            Self::NotFound => f.write_str("not found"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<io::Error> for Error {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}

/// Result of this crate
pub type Result<T> = std::result::Result<T, Error>;
