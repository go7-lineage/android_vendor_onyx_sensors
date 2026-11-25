//! Delays between connection attempts: doubling from the first, up to a cap

use std::time::Duration;

/// Retry delays
#[derive(Debug, Clone)]
pub struct Backoff {
    first: Duration,
    cap: Duration,
    next: Duration,
}

impl Backoff {
    /// Delays `first`, twice that, and so on, never more than `cap`
    #[must_use]
    pub const fn new(first: Duration, cap: Duration) -> Self {
        Self {
            first,
            cap,
            next: first,
        }
    }

    /// The delay before the next attempt
    pub fn next_delay(&mut self) -> Duration {
        let delay = self.next.min(self.cap);
        self.next = delay.saturating_mul(2);
        delay
    }

    /// Starts over at the first delay
    pub const fn reset(&mut self) {
        self.next = self.first;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn delays_double_up_to_the_cap() {
        let mut b = Backoff::new(Duration::from_secs(1), Duration::from_secs(30));
        let delays: Vec<u64> = (0..7).map(|_| b.next_delay().as_secs()).collect();
        assert_eq!(delays, [1, 2, 4, 8, 16, 30, 30]);
    }

    #[test]
    fn reset_starts_over() {
        let mut b = Backoff::new(Duration::from_secs(1), Duration::from_secs(30));
        let _ = (b.next_delay(), b.next_delay());
        b.reset();
        assert_eq!(b.next_delay(), Duration::from_secs(1));
    }
}
