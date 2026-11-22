//! Sensor core timestamps are `QTimer` ticks, the counter arm64 reads as `CNTVCT_EL0`. This converts them to
//! `CLOCK_BOOTTIME` nanoseconds.

/// Boot time in ns of `tick`, given that the counter read `now_tick` at boot time `now_ns` and runs at `freq` Hz;
/// `None` if `freq` is 0 or the result doesn't fit
#[must_use]
pub fn boottime_ns(tick: u64, now_tick: u64, now_ns: i64, freq: u64) -> Option<i64> {
    let ago = i128::from(now_tick).checked_sub(i128::from(tick))?;
    let ago_ns = ago.checked_mul(1_000_000_000)?.checked_div(i128::from(freq))?;
    i64::try_from(i128::from(now_ns).checked_sub(ago_ns)?).ok()
}

#[cfg(all(target_arch = "aarch64", any(target_os = "linux", target_os = "android")))]
pub use device::Now;

#[cfg(all(target_arch = "aarch64", any(target_os = "linux", target_os = "android")))]
mod device {
    use std::io;

    /// The counter and `CLOCK_BOOTTIME`, read together
    #[derive(Debug, Clone, Copy)]
    pub struct Now {
        /// `CNTVCT_EL0`
        pub tick: u64,
        /// `CLOCK_BOOTTIME` in ns
        pub boottime_ns: i64,
        /// `CNTFRQ_EL0`
        pub freq: u64,
    }

    impl Now {
        /// Reads both clocks
        ///
        /// # Errors
        /// `clock_gettime` failure
        pub fn read() -> io::Result<Self> {
            let tick = counter();
            let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
            // SAFETY: `ts` is a valid, writable timespec for the duration of the call
            if unsafe { libc::clock_gettime(libc::CLOCK_BOOTTIME, &raw mut ts) } != 0 {
                return Err(io::Error::last_os_error());
            }
            let sec: i64 = ts.tv_sec;
            let nsec: i64 = ts.tv_nsec;
            let boottime_ns = sec
                .checked_mul(1_000_000_000)
                .and_then(|ns| ns.checked_add(nsec))
                .ok_or_else(|| io::Error::other("CLOCK_BOOTTIME out of range"))?;
            Ok(Self {
                tick,
                boottime_ns,
                freq: frequency(),
            })
        }

        /// Boot time in ns of `tick`
        #[must_use]
        pub fn boottime_of(&self, tick: u64) -> Option<i64> {
            super::boottime_ns(tick, self.tick, self.boottime_ns, self.freq)
        }
    }

    fn counter() -> u64 {
        let value: u64;
        // SAFETY: reads a register the kernel lets EL0 read; touches no memory
        unsafe { core::arch::asm!("mrs {}, cntvct_el0", out(reg) value, options(nomem, nostack, preserves_flags)) };
        value
    }

    fn frequency() -> u64 {
        let value: u64;
        // SAFETY: reads a register EL0 may always read; touches no memory
        unsafe { core::arch::asm!("mrs {}, cntfrq_el0", out(reg) value, options(nomem, nostack, preserves_flags)) };
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_tick_in_the_past_is_earlier_by_its_age() {
        assert_eq!(
            boottime_ns(19_199_808, 19_200_000, 5_000_000_000, 19_200_000),
            Some(4_999_990_000)
        );
    }

    #[test]
    fn a_tick_after_now_is_later() {
        assert_eq!(boottime_ns(1_920, 0, 1_000, 19_200_000), Some(101_000));
    }

    #[test]
    fn zero_frequency_or_overflow_gives_none() {
        assert_eq!(boottime_ns(0, 1, 0, 0), None);
        assert_eq!(boottime_ns(0, u64::MAX, i64::MIN, 1), None);
    }

    #[cfg(all(target_arch = "aarch64", any(target_os = "linux", target_os = "android")))]
    #[test]
    fn reading_now_converts_its_own_tick_to_its_own_time() {
        let now = Now::read().unwrap();
        assert!(now.freq > 0);
        assert_eq!(now.boottime_of(now.tick), Some(now.boottime_ns));
    }
}
