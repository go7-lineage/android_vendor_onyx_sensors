//! Rate limit for repeated warnings

use std::time::{Duration, Instant};

/// True at most once per period
pub struct Every {
    period: Duration,
    last: Option<Instant>,
}

impl Every {
    /// Due at once, then once per `period`
    pub const fn new(period: Duration) -> Self {
        Self { period, last: None }
    }

    /// Whether the period has passed since it was last due; if so, starts the next one
    pub fn due(&mut self) -> bool {
        if self.last.is_some_and(|at| at.elapsed() < self.period) {
            return false;
        }
        self.last = Some(Instant::now());
        true
    }
}
