//! The HAL's side of the sensor core: links to it with backoff, keeps the accelerometer streaming as the HAL asks,
//! hands samples and flushes to a sink, and links again after the sensor core goes away (a DSP restart)

use std::sync::mpsc::{Receiver, RecvTimeoutError, TryRecvError};
use std::time::{Duration, Instant};

use crate::accel;
use crate::backoff::Backoff;
use crate::client::{Client, Transport};
use crate::clock::Now;
use crate::error::{Error, Result};
use crate::msg::{self, Report, Suid, id};

/// What the HAL asks for
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Request {
    /// Streaming wanted
    pub active: bool,
    /// Sampling period in ns, as `ISensors.batch` gives it
    pub period_ns: i64,
}

/// An accelerometer sample, in the Android frame
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// `CLOCK_BOOTTIME` in ns
    pub timestamp_ns: i64,
    /// m/s²
    pub x: f32,
    /// m/s²
    pub y: f32,
    /// m/s²
    pub z: f32,
    /// Android `SensorStatus`
    pub status: i8,
}

/// Where samples and flushes go: the HAL's event queue
pub trait Sink {
    /// New samples, oldest first
    fn samples(&mut self, samples: &[Sample]);
    /// A flush is done: every sample before it has been handed over
    fn flush_complete(&mut self);
}

/// What the worker needs from the platform
pub trait Env {
    /// Link to the sensor core
    type Link: Transport;

    /// Opens a link to the sensor core
    ///
    /// # Errors
    /// [`Error::NotFound`] while the service isn't up, socket errors
    fn connect(&mut self) -> Result<Self::Link>;

    /// Reads the counter and `CLOCK_BOOTTIME`
    ///
    /// # Errors
    /// Clock errors
    fn now(&mut self) -> Result<Now>;

    /// Something worth a log line happened
    fn note(&mut self, note: &Note<'_>);
}

/// What the worker did, for the log
#[derive(Debug)]
pub enum Note<'a> {
    /// Linked to the sensor core; the accelerometer is this sensor
    Connected(Suid),
    /// No link yet; the next attempt is after the delay
    ConnectFailed(&'a Error, Duration),
    /// The link broke; the next attempt is after the delay
    Lost(&'a Error, Duration),
    /// Streaming at this many Hz
    Streaming(u16),
    /// Streaming stopped
    Stopped,
    /// An event that doesn't decode, or no clock reading for it
    BadEvent(&'a Error),
    /// The sensor core sent this `sns_std_error`
    SensorError(u64),
}

/// From the binder threads to the worker
#[derive(Debug)]
pub enum Command<S> {
    /// What the HAL wants now
    Request(Request),
    /// Hand a flush-complete to the sink, after the samples so far
    Flush,
    /// A new sink, from `ISensors.initialize`
    Sink(S),
}

/// Timeouts and retry delays
#[derive(Debug, Clone)]
pub struct Timing {
    /// Each request to the sensor core
    pub request: Duration,
    /// Longest wait for samples before commands are looked at again
    pub poll: Duration,
    /// Delays between attempts to link
    pub backoff: Backoff,
}

impl Timing {
    /// The HAL's: 5 s per request, 200 ms polls, attempts after 1, 2, 4, 8 and 16 s, then every 30 s
    pub const HAL: Self = Self {
        request: Duration::from_secs(5),
        poll: Duration::from_millis(200),
        backoff: Backoff::new(Duration::from_secs(1), Duration::from_secs(30)),
    };
}

struct Link<T> {
    client: Client<T>,
    suid: Suid,
    rate_hz: Option<u16>,
}

/// Runs the link to the sensor core; see the module docs
pub struct Worker<E: Env, S> {
    env: E,
    timing: Timing,
    sink: Option<S>,
    request: Request,
    link: Option<Link<E::Link>>,
    retry_at: Instant,
}

impl<E: Env, S: Sink> Worker<E, S> {
    /// A worker that tries to link right away
    pub fn new(env: E, timing: Timing) -> Self {
        Self {
            env,
            timing,
            sink: None,
            request: Request::default(),
            link: None,
            retry_at: Instant::now(),
        }
    }

    /// Steps until every sender of `commands` is gone
    pub fn run(mut self, commands: &Receiver<Command<S>>) {
        while self.step(commands) {}
    }

    /// Waits for samples or commands, then links if due and applies the request; `false` once every sender of
    /// `commands` is gone
    pub fn step(&mut self, commands: &Receiver<Command<S>>) -> bool {
        if !self.wait(commands) {
            return false;
        }
        self.connect_if_due();
        self.apply();
        true
    }

    fn wait(&mut self, commands: &Receiver<Command<S>>) -> bool {
        let first = if self.link.as_ref().is_some_and(|l| l.rate_hz.is_some()) {
            self.read();
            match commands.try_recv() {
                Ok(c) => Some(c),
                Err(TryRecvError::Empty) => None,
                Err(TryRecvError::Disconnected) => return false,
            }
        } else if self.link.is_some() {
            let Ok(c) = commands.recv() else {
                return false;
            };
            Some(c)
        } else {
            match commands.recv_timeout(self.retry_at.saturating_duration_since(Instant::now())) {
                Ok(c) => Some(c),
                Err(RecvTimeoutError::Timeout) => None,
                Err(RecvTimeoutError::Disconnected) => return false,
            }
        };
        let Some(first) = first else {
            return true;
        };
        self.handle(first);
        loop {
            match commands.try_recv() {
                Ok(c) => self.handle(c),
                Err(TryRecvError::Empty) => return true,
                Err(TryRecvError::Disconnected) => return false,
            }
        }
    }

    fn handle(&mut self, command: Command<S>) {
        match command {
            Command::Request(request) => self.request = request,
            Command::Flush => {
                if let Some(sink) = &mut self.sink {
                    sink.flush_complete();
                }
            }
            Command::Sink(sink) => self.sink = Some(sink),
        }
    }

    fn connect_if_due(&mut self) {
        if self.link.is_some() || Instant::now() < self.retry_at {
            return;
        }
        match self.open() {
            Ok(link) => {
                self.env.note(&Note::Connected(link.suid));
                self.link = Some(link);
            }
            Err(e) => {
                let delay = self.retry_later();
                self.env.note(&Note::ConnectFailed(&e, delay));
            }
        }
    }

    fn open(&mut self) -> Result<Link<E::Link>> {
        let mut client = Client::new(self.env.connect()?);
        let suid = *client
            .find(accel::DATA_TYPE, self.timing.request)?
            .first()
            .ok_or(Error::NotFound)?;
        Ok(Link {
            client,
            suid,
            rate_hz: None,
        })
    }

    fn retry_later(&mut self) -> Duration {
        let delay = self.timing.backoff.next_delay();
        let now = Instant::now();
        self.retry_at = now.checked_add(delay).unwrap_or(now);
        delay
    }

    fn apply(&mut self) {
        let Some(link) = &mut self.link else {
            return;
        };
        let want = self.request.active.then(|| accel::rate_hz(self.request.period_ns));
        if want == link.rate_hz {
            self.timing.backoff.reset();
            return;
        }
        let done = match want {
            Some(hz) if link.rate_hz.is_none() => {
                drain(&mut link.client).and_then(|()| link.client.enable(link.suid, f32::from(hz), self.timing.request))
            }
            Some(hz) => link.client.enable(link.suid, f32::from(hz), self.timing.request),
            None => link.client.disable(link.suid, self.timing.request),
        };
        match done {
            Ok(()) => {
                link.rate_hz = want;
                self.timing.backoff.reset();
                self.env.note(&want.map_or(Note::Stopped, Note::Streaming));
            }
            Err(e) => self.lose(&e),
        }
    }

    fn lose(&mut self, error: &Error) {
        self.link = None;
        let delay = self.retry_later();
        self.env.note(&Note::Lost(error, delay));
    }

    fn read(&mut self) {
        let Some(link) = &mut self.link else {
            return;
        };
        match link.client.next_report(self.timing.poll) {
            Ok(Some(report)) if report.suid == link.suid => self.deliver(&report),
            Ok(_) => {}
            Err(e) => self.lose(&e),
        }
    }

    fn deliver(&mut self, report: &Report) {
        let now = match self.env.now() {
            Ok(now) => now,
            Err(e) => return self.env.note(&Note::BadEvent(&e)),
        };
        let mut samples = Vec::with_capacity(report.events.len());
        for event in &report.events {
            match event.msg_id {
                id::SENSOR_EVENT => match sample(event, &now) {
                    Ok(s) => samples.push(s),
                    Err(e) => self.env.note(&Note::BadEvent(&e)),
                },
                id::ERROR_EVENT => match msg::decode_error(&event.payload) {
                    Ok(code) => self.env.note(&Note::SensorError(code)),
                    Err(e) => self.env.note(&Note::BadEvent(&e)),
                },
                _ => {}
            }
        }
        if let Some(sink) = &mut self.sink
            && !samples.is_empty()
        {
            sink.samples(&samples);
        }
    }
}

fn drain<T: Transport>(client: &mut Client<T>) -> Result<()> {
    while client.next_report(Duration::ZERO)?.is_some() {}
    Ok(())
}

fn sample(event: &msg::Event, now: &Now) -> Result<Sample> {
    let decoded = msg::decode_sample(&event.payload)?;
    let &[x, y, z] = decoded.data.as_slice() else {
        return Err(Error::Malformed("accel sample"));
    };
    let timestamp_ns = now
        .boottime_of(event.timestamp)
        .ok_or(Error::Malformed("sample timestamp"))?;
    Ok(Sample {
        timestamp_ns,
        x,
        y,
        z,
        status: accel::status(decoded.status),
    })
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::rc::Rc;
    use std::sync::mpsc::{Sender, channel};

    use super::*;
    use crate::testutil::{self, ACCEL, Service, report_at};

    const FAST: Timing = Timing {
        request: Duration::from_millis(10),
        poll: Duration::from_millis(1),
        backoff: Backoff::new(Duration::from_millis(1), Duration::from_millis(4)),
    };
    const NOW: Now = Now {
        tick: 19_200_000,
        boottime_ns: 5_000_000_000,
        freq: 19_200_000,
    };

    struct FakeEnv {
        service: Rc<RefCell<Service>>,
        notes: Rc<RefCell<Vec<String>>>,
    }

    impl Env for FakeEnv {
        type Link = testutil::Link;

        fn connect(&mut self) -> Result<testutil::Link> {
            testutil::Link::connect(&self.service)
        }

        fn now(&mut self) -> Result<Now> {
            Ok(NOW)
        }

        fn note(&mut self, note: &Note<'_>) {
            self.notes.borrow_mut().push(format!("{note:?}"));
        }
    }

    #[derive(Debug, PartialEq)]
    enum Out {
        Samples(Vec<Sample>),
        Flush,
    }

    struct FakeSink(Rc<RefCell<Vec<Out>>>);

    impl Sink for FakeSink {
        fn samples(&mut self, samples: &[Sample]) {
            self.0.borrow_mut().push(Out::Samples(samples.to_vec()));
        }

        fn flush_complete(&mut self) {
            self.0.borrow_mut().push(Out::Flush);
        }
    }

    struct Rig {
        worker: Worker<FakeEnv, FakeSink>,
        tx: Sender<Command<FakeSink>>,
        rx: Receiver<Command<FakeSink>>,
        service: Rc<RefCell<Service>>,
        notes: Rc<RefCell<Vec<String>>>,
        out: Rc<RefCell<Vec<Out>>>,
    }

    impl Rig {
        fn new(ready: bool) -> Self {
            let service = Service::shared(ready);
            let notes = Rc::new(RefCell::new(Vec::new()));
            let env = FakeEnv {
                service: Rc::clone(&service),
                notes: Rc::clone(&notes),
            };
            let (tx, rx) = channel();
            let out = Rc::new(RefCell::new(Vec::new()));
            tx.send(Command::Sink(FakeSink(Rc::clone(&out)))).unwrap();
            Self {
                worker: Worker::new(env, FAST),
                tx,
                rx,
                service,
                notes,
                out,
            }
        }

        fn send(&self, command: Command<FakeSink>) {
            self.tx.send(command).unwrap();
        }

        fn request(&self, active: bool, period_ns: i64) {
            self.send(Command::Request(Request { active, period_ns }));
        }

        fn step(&mut self) {
            assert!(self.worker.step(&self.rx));
        }

        fn configs(&self) -> Vec<Vec<u8>> {
            self.service.borrow().requests_of(id::SENSOR_CONFIG)
        }

        fn push(&self, tick: u64, payload: &[u8]) {
            self.service
                .borrow_mut()
                .incoming
                .push_back(report_at(ACCEL, &[(id::SENSOR_EVENT, tick, payload)]));
        }

        fn streaming(ready: bool, period_ns: i64) -> Self {
            let mut rig = Self::new(ready);
            rig.request(true, period_ns);
            rig.step();
            rig
        }
    }

    fn config(hz: f32) -> Vec<u8> {
        msg::sensor_config(hz)
    }

    #[test]
    fn links_once_the_service_appears_with_growing_delays() {
        let mut rig = Rig::new(true);
        rig.service.borrow_mut().refuse = 3;
        for _ in 0..4 {
            rig.step();
        }
        assert_eq!(
            *rig.notes.borrow(),
            [
                "ConnectFailed(NotFound, 1ms)",
                "ConnectFailed(NotFound, 2ms)",
                "ConnectFailed(NotFound, 4ms)",
                "Connected(Suid { low: 1, high: 2 })",
            ]
        );
    }

    #[test]
    fn no_accelerometer_yet_is_a_failed_attempt() {
        let mut rig = Rig::new(false);
        rig.step();
        assert_eq!(*rig.notes.borrow(), ["ConnectFailed(NotFound, 1ms)"]);
        rig.service.borrow_mut().ready = true;
        rig.step();
        assert_eq!(
            rig.notes.borrow().last().unwrap(),
            "Connected(Suid { low: 1, high: 2 })"
        );
    }

    #[test]
    fn a_request_made_before_the_link_is_applied_on_link() {
        let rig = Rig::streaming(true, 20_000_000);
        assert_eq!(rig.configs(), [config(50.0)]);
        assert_eq!(rig.notes.borrow().last().unwrap(), "Streaming(50)");
    }

    #[test]
    fn samples_reach_the_sink_with_boot_times() {
        let mut rig = Rig::streaming(true, 20_000_000);
        rig.push(19_199_808, &testutil::sample(&[0.0, 9.75, -0.2], 3));
        rig.step();
        assert_eq!(
            *rig.out.borrow(),
            [Out::Samples(vec![Sample {
                timestamp_ns: 4_999_990_000,
                x: 0.0,
                y: 9.75,
                z: -0.2,
                status: 3,
            }])]
        );
    }

    #[test]
    fn other_sensors_and_other_events_are_not_samples() {
        let mut rig = Rig::streaming(true, 20_000_000);
        let other = Suid { low: 7, high: 7 };
        let s = testutil::sample(&[1.0, 2.0, 3.0], 3);
        rig.service
            .borrow_mut()
            .incoming
            .push_back(report_at(other, &[(id::SENSOR_EVENT, 1, &s)]));
        rig.service
            .borrow_mut()
            .incoming
            .push_back(report_at(ACCEL, &[(id::PHYSICAL_CONFIG_EVENT, 1, &[])]));
        rig.step();
        rig.step();
        assert!(rig.out.borrow().is_empty());
    }

    #[test]
    fn a_short_sample_is_noted_and_dropped() {
        let mut rig = Rig::streaming(true, 20_000_000);
        rig.push(1, &testutil::sample(&[1.0, 2.0], 3));
        rig.step();
        assert!(rig.out.borrow().is_empty());
        assert_eq!(
            rig.notes.borrow().last().unwrap(),
            "BadEvent(Malformed(\"accel sample\"))"
        );
    }

    #[test]
    fn sensor_core_errors_are_noted() {
        let mut rig = Rig::streaming(true, 20_000_000);
        let error = crate::pb::Encoder::new().varint(1, 5).finish();
        rig.service
            .borrow_mut()
            .incoming
            .push_back(report_at(ACCEL, &[(id::ERROR_EVENT, 1, &error)]));
        rig.step();
        assert_eq!(rig.notes.borrow().last().unwrap(), "SensorError(5)");
    }

    #[test]
    fn a_new_rate_reconfigures_and_the_same_rate_does_not() {
        let mut rig = Rig::streaming(true, 20_000_000);
        rig.request(true, 20_100_000);
        rig.step();
        rig.request(true, 10_000_000);
        rig.step();
        assert_eq!(rig.configs(), [config(50.0), config(100.0)]);
    }

    #[test]
    fn deactivating_disables_once() {
        let mut rig = Rig::streaming(true, 20_000_000);
        rig.request(false, 20_000_000);
        rig.step();
        rig.request(false, 10_000_000);
        rig.step();
        assert_eq!(rig.service.borrow().requests_of(id::DISABLE_REQ).len(), 1);
        assert_eq!(rig.notes.borrow().last().unwrap(), "Stopped");
    }

    #[test]
    fn a_flush_follows_the_samples_before_it() {
        let mut rig = Rig::streaming(true, 20_000_000);
        rig.push(19_200_000, &testutil::sample(&[0.0, 0.0, 9.8], 3));
        rig.send(Command::Flush);
        rig.step();
        let out = rig.out.borrow();
        assert!(matches!(out.as_slice(), [Out::Samples(_), Out::Flush]));
    }

    #[test]
    fn a_flush_completes_without_a_link() {
        let mut rig = Rig::new(false);
        rig.send(Command::Flush);
        rig.step();
        assert_eq!(*rig.out.borrow(), [Out::Flush]);
    }

    #[test]
    fn a_new_sink_gets_the_samples_after_it() {
        let mut rig = Rig::streaming(true, 20_000_000);
        let second = Rc::new(RefCell::new(Vec::new()));
        rig.send(Command::Sink(FakeSink(Rc::clone(&second))));
        rig.step();
        rig.push(19_200_000, &testutil::sample(&[0.0, 0.0, 9.8], 3));
        rig.step();
        assert!(rig.out.borrow().is_empty());
        assert_eq!(second.borrow().len(), 1);
    }

    #[test]
    fn losing_the_service_links_again_and_restreams() {
        let mut rig = Rig::streaming(true, 20_000_000);
        rig.service.borrow_mut().incoming.push_back(crate::client::Packet::Gone);
        rig.step();
        assert_eq!(rig.notes.borrow().last().unwrap(), "Lost(ServiceGone, 1ms)");
        rig.step();
        assert_eq!(rig.service.borrow().requests_of(id::SUID_REQ).len(), 2);
        assert_eq!(rig.configs(), [config(50.0), config(50.0)]);
    }

    #[test]
    fn a_rejected_request_drops_the_link_and_links_again() {
        let mut rig = Rig::new(true);
        rig.service.borrow_mut().reject = Some(id::SENSOR_CONFIG);
        rig.request(true, 20_000_000);
        rig.step();
        assert_eq!(
            rig.notes.borrow().last().unwrap(),
            "Lost(Qmi { result: 1, error: 0 }, 1ms)"
        );
        rig.service.borrow_mut().reject = None;
        rig.step();
        assert_eq!(rig.service.borrow().requests_of(id::SUID_REQ).len(), 2);
        assert_eq!(rig.configs(), [config(50.0), config(50.0)]);
        assert_eq!(rig.notes.borrow().last().unwrap(), "Streaming(50)");
    }

    #[test]
    fn a_request_that_keeps_failing_is_retried_with_growing_delays() {
        let mut rig = Rig::new(true);
        rig.service.borrow_mut().reject = Some(id::SENSOR_CONFIG);
        rig.request(true, 20_000_000);
        for _ in 0..4 {
            rig.step();
        }
        let lost: Vec<String> = rig
            .notes
            .borrow()
            .iter()
            .filter(|n| n.starts_with("Lost"))
            .cloned()
            .collect();
        assert_eq!(
            lost,
            [
                "Lost(Qmi { result: 1, error: 0 }, 1ms)",
                "Lost(Qmi { result: 1, error: 0 }, 2ms)",
                "Lost(Qmi { result: 1, error: 0 }, 4ms)",
                "Lost(Qmi { result: 1, error: 0 }, 4ms)",
            ]
        );
        rig.service.borrow_mut().reject = None;
        rig.step();
        assert_eq!(rig.notes.borrow().last().unwrap(), "Streaming(50)");
        rig.service.borrow_mut().incoming.push_back(crate::client::Packet::Gone);
        rig.step();
        assert_eq!(rig.notes.borrow().last().unwrap(), "Lost(ServiceGone, 1ms)");
    }

    #[test]
    fn stepping_ends_when_the_senders_are_gone() {
        let Rig { mut worker, tx, rx, .. } = Rig::new(true);
        drop(tx);
        assert!(!worker.step(&rx));
    }

    #[test]
    fn reports_from_before_a_disable_are_not_delivered_after_the_next_enable() {
        let mut rig = Rig::streaming(true, 20_000_000);
        rig.push(19_000_000, &testutil::sample(&[1.0, 1.0, 1.0], 3));
        rig.push(19_100_000, &testutil::sample(&[2.0, 2.0, 2.0], 3));
        rig.request(false, 20_000_000);
        rig.step();
        rig.out.borrow_mut().clear();
        rig.request(true, 20_000_000);
        rig.step();
        rig.step();
        assert!(rig.out.borrow().is_empty());
        assert_eq!(rig.notes.borrow().last().unwrap(), "Streaming(50)");
    }
}
