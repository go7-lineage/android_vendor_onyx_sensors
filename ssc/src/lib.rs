//! Client for the Qualcomm Snapdragon Sensor Core (SSC): QMI service 400 over QRTR, protobuf inside

pub mod accel;
pub mod backoff;
pub mod client;
pub mod clock;
pub mod error;
pub mod msg;
pub mod pb;
pub mod qmi;
pub mod qrtr;
#[cfg(test)]
mod testutil;
pub mod worker;

pub use error::{Error, Result};
