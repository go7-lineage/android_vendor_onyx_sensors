//! The one sensor the HAL lists, the LSM6DST accelerometer, with the values the sensor core reports for it
//! (`diag/ssc1.txt`), and how the HAL's requests map onto the sensor core

/// Sensor core data type
pub const DATA_TYPE: &str = "accel";
/// Handle in the HAL's sensor list
pub const HANDLE: i32 = 1;
/// Name: the `NAME` attribute and the sensor type
pub const NAME: &str = "lsm6dst Accelerometer";
/// The `VENDOR` attribute
pub const VENDOR: &str = "STMicro";
/// The `VERSION` attribute
pub const VERSION: i32 = 142_712;
/// m/s² per LSB at the selected ±16 g: 0.488 mg
pub const RESOLUTION: f32 = 0.488e-3 * 9.806_65;
/// Full scale in m/s²: 32768 LSB, so a multiple of the resolution as sensorservice wants it
pub const MAX_RANGE: f32 = 32_768.0 * RESOLUTION;
/// mA while streaming: 240 µA in high-performance mode
pub const POWER_MA: f32 = 0.24;
/// Slowest rate the sensor core offers
pub const MIN_RATE_HZ: u16 = 13;
/// Fastest rate the sensor core offers outside low-latency mode
pub const MAX_RATE_HZ: u16 = 416;
/// Shortest period in µs: 1 / 416 Hz, rounded up
pub const MIN_DELAY_US: i32 = 2_404;
/// Longest period in µs: 1 / 13 Hz, rounded down
pub const MAX_DELAY_US: i32 = 76_923;

const NS_PER_S: i64 = 1_000_000_000;

/// Sensor core rate for a sampling period: the nearest whole Hz, within the offered rates. The sensor core then
/// picks the next hardware rate at or above it.
#[must_use]
pub fn rate_hz(period_ns: i64) -> u16 {
    let period = period_ns.max(1);
    let half = period.checked_div(2).unwrap_or_default();
    let hz = NS_PER_S
        .saturating_add(half)
        .checked_div(period)
        .unwrap_or(NS_PER_S)
        .clamp(MIN_RATE_HZ.into(), MAX_RATE_HZ.into());
    u16::try_from(hz).unwrap_or(MAX_RATE_HZ)
}

/// Android `SensorStatus` for a `sns_std_sensor_sample_status`: the same values 0 to 3, unreliable otherwise
#[must_use]
pub fn status(sample_status: u64) -> i8 {
    i8::try_from(sample_status)
        .ok()
        .filter(|s| (0..=3).contains(s))
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn periods_round_to_the_nearest_hz() {
        assert_eq!(rate_hz(66_667_000), 15);
        assert_eq!(rate_hz(20_000_000), 50);
        assert_eq!(rate_hz(10_000_000), 100);
        assert_eq!(rate_hz(20_100_000), 50);
    }

    #[test]
    fn rates_stay_within_what_the_sensor_core_offers() {
        assert_eq!(rate_hz(1_000_000), MAX_RATE_HZ);
        assert_eq!(rate_hz(1_000_000_000), MIN_RATE_HZ);
        assert_eq!(rate_hz(0), MAX_RATE_HZ);
        assert_eq!(rate_hz(-5), MAX_RATE_HZ);
        assert_eq!(rate_hz(i64::MAX), MIN_RATE_HZ);
    }

    #[test]
    fn the_listed_delays_give_the_rate_limits() {
        assert_eq!(rate_hz(i64::from(MIN_DELAY_US) * 1000), MAX_RATE_HZ);
        assert_eq!(rate_hz(i64::from(MAX_DELAY_US) * 1000), MIN_RATE_HZ);
    }

    #[test]
    fn max_range_is_a_whole_number_of_steps() {
        assert!((MAX_RANGE / RESOLUTION - 32_768.0).abs() < 1e-3);
        assert!((MAX_RANGE - 156.816).abs() < 1e-3);
    }

    #[test]
    fn statuses_outside_0_to_3_are_unreliable() {
        assert_eq!([0, 1, 2, 3, 4, 255].map(status), [0, 1, 2, 3, 0, 0]);
    }
}
