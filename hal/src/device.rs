//! What the worker uses on the device: QRTR, the arch counter, logcat

use std::time::Duration;

use log::{info, warn};
use ssc::client::Qrtr;
use ssc::clock::Now;
use ssc::worker::{Env, Note};

use crate::every::Every;

const LOOKUP: Duration = Duration::from_secs(2);
const WARN_EVERY: Duration = Duration::from_secs(10);

/// The device's [`Env`]
pub struct Device {
    bad_events: u64,
    warn: Every,
}

impl Device {
    /// Nothing logged yet
    pub const fn new() -> Self {
        Self {
            bad_events: 0,
            warn: Every::new(WARN_EVERY),
        }
    }
}

impl Env for Device {
    type Link = Qrtr;

    fn connect(&mut self) -> ssc::Result<Qrtr> {
        Qrtr::connect(LOOKUP)
    }

    fn now(&mut self) -> ssc::Result<Now> {
        Ok(Now::read()?)
    }

    fn note(&mut self, note: &Note<'_>) {
        match note {
            Note::Connected(suid) => info!("sensor core linked, accelerometer {suid}"),
            Note::ConnectFailed(e, retry) => info!("sensor core not ready ({e}), next try in {retry:?}"),
            Note::Lost(e, retry) => warn!("sensor core link lost ({e}), next try in {retry:?}"),
            Note::Streaming(hz) => info!("accelerometer streaming at {hz} Hz"),
            Note::Stopped => info!("accelerometer stopped"),
            Note::BadEvent(e) => {
                self.bad_events = self.bad_events.saturating_add(1);
                if self.warn.due() {
                    warn!("bad sensor core event ({e}), {} so far", self.bad_events);
                }
            }
            Note::SensorError(code) => warn!("sensor core error {code}"),
        }
    }
}
