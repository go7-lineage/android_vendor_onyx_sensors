//! Checks the sensor core on the device: finds a sensor, prints its attributes and streams it.
//! usage: `ssc-cli [DATA_TYPE [SECONDS [RATE_HZ]]]`, by default `ssc-cli accel 10 10`

use std::process::ExitCode;

#[cfg(all(target_arch = "aarch64", any(target_os = "linux", target_os = "android")))]
mod run {
    use std::time::{Duration, Instant};

    use ssc::client::{Client, Qrtr};
    use ssc::clock::Now;
    use ssc::msg::{self, Report, id};
    use ssc::{Error, Result};

    const T: Duration = Duration::from_secs(5);

    pub struct Args {
        pub data_type: String,
        pub seconds: u64,
        pub rate_hz: f32,
    }

    pub fn parse(mut args: impl Iterator<Item = String>) -> core::result::Result<Args, String> {
        let data_type = args.next().unwrap_or_else(|| "accel".to_owned());
        let seconds = args
            .next()
            .map_or(Ok(10), |s| s.parse().map_err(|_| format!("bad SECONDS {s:?}")))?;
        let rate_hz = args
            .next()
            .map_or(Ok(10.0), |s| s.parse().map_err(|_| format!("bad RATE_HZ {s:?}")))?;
        Ok(Args {
            data_type,
            seconds,
            rate_hz,
        })
    }

    pub fn main(args: &Args) -> Result<()> {
        let link = Qrtr::connect(T)?;
        let server = link.server();
        println!(
            "service {} instance {:#x} at node {} port {:#x}",
            server.service, server.instance, server.addr.node, server.addr.port
        );
        let mut client = Client::new(link);
        let suids = client.find(&args.data_type, T)?;
        for suid in &suids {
            println!("{} sensor {suid}", args.data_type);
        }
        let suid = *suids.first().ok_or(Error::NotFound)?;
        for attr in client.attributes(suid, T)? {
            let values: Vec<String> = attr.values.iter().map(ToString::to_string).collect();
            println!("attr {:>2} {:<30} {}", attr.id, attr.name(), values.join(", "));
        }
        client.enable(suid, args.rate_hz, T)?;
        println!("enabled at {} Hz", args.rate_hz);
        let streamed = stream(&mut client, suid, Duration::from_secs(args.seconds));
        let disabled = client.disable(suid, T);
        streamed?;
        disabled
    }

    #[derive(Default)]
    struct Totals {
        samples: u32,
        sum: [f64; 3],
    }

    fn stream(client: &mut Client<Qrtr>, suid: msg::Suid, length: Duration) -> Result<()> {
        let start = Instant::now();
        let mut totals = Totals::default();
        while start.elapsed() < length {
            let Some(report) = client.next_report(length.saturating_sub(start.elapsed()))? else {
                break;
            };
            let now = Now::read()?;
            if report.suid == suid {
                print_report(&report, &now, start, &mut totals)?;
            }
        }
        let seconds = start.elapsed().as_secs_f64();
        let n = f64::from(totals.samples.max(1));
        println!(
            "{seconds:.1} s: {} samples ({:.1} Hz), mean {:.3} {:.3} {:.3}",
            totals.samples,
            f64::from(totals.samples) / seconds,
            totals.sum[0] / n,
            totals.sum[1] / n,
            totals.sum[2] / n
        );
        Ok(())
    }

    fn print_report(report: &Report, now: &Now, start: Instant, totals: &mut Totals) -> Result<()> {
        for event in &report.events {
            match event.msg_id {
                id::SENSOR_EVENT => {
                    let sample = msg::decode_sample(&event.payload)?;
                    let at = now.boottime_of(event.timestamp).unwrap_or_default();
                    let late_ms = now.boottime_ns.saturating_sub(at) / 1_000_000;
                    let values: Vec<String> = sample.data.iter().map(|v| format!("{v:8.3}")).collect();
                    println!(
                        "{:7.3} s  tick {} boottime {at} ({late_ms} ms ago)  status {}  {}",
                        start.elapsed().as_secs_f64(),
                        event.timestamp,
                        sample.status,
                        values.join(" ")
                    );
                    totals.samples = totals.samples.saturating_add(1);
                    for (sum, v) in totals.sum.iter_mut().zip(&sample.data) {
                        *sum += f64::from(*v);
                    }
                }
                id::PHYSICAL_CONFIG_EVENT => {
                    let c = msg::decode_physical_config(&event.payload)?;
                    println!(
                        "config: rate {:?} Hz, range {:?}, resolution {:?}, mode {:?}, current {:?} uA",
                        c.sample_rate, c.range, c.resolution, c.operation_mode, c.active_current
                    );
                }
                id::ERROR_EVENT => println!("error event: {}", msg::decode_error(&event.payload)?),
                other => println!("event {other}, {} bytes", event.payload.len()),
            }
        }
        Ok(())
    }
}

#[cfg(all(target_arch = "aarch64", any(target_os = "linux", target_os = "android")))]
fn main() -> ExitCode {
    let args = match run::parse(std::env::args().skip(1)) {
        Ok(args) => args,
        Err(e) => {
            eprintln!("ssc-cli: {e}\nusage: ssc-cli [DATA_TYPE [SECONDS [RATE_HZ]]]");
            return ExitCode::from(2);
        }
    };
    match run::main(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("ssc-cli: {e}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(not(all(target_arch = "aarch64", any(target_os = "linux", target_os = "android"))))]
fn main() -> ExitCode {
    eprintln!("ssc-cli: runs on arm64 Linux or Android only");
    ExitCode::FAILURE
}
