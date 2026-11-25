//! Sensor core client: control requests and their responses, with the reports that arrive in between

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use crate::error::{Error, Result};
use crate::msg::{self, Attr, Event, Report, SUID_LOOKUP, Suid, id};
use crate::qmi::{self, Kind};

/// What a transport receives
#[derive(Debug)]
pub enum Packet {
    /// A QMI message from the service
    Qmi(Vec<u8>),
    /// The service went away
    Gone,
}

/// Datagram link to the sensor core service
pub trait Transport {
    /// Sends one QMI message to the service
    ///
    /// # Errors
    /// Socket errors
    fn send(&mut self, msg: &[u8]) -> Result<()>;

    /// Next packet from the service, waiting at most `timeout`; `None` on timeout
    ///
    /// # Errors
    /// Socket errors
    fn recv(&mut self, timeout: Duration) -> Result<Option<Packet>>;
}

enum Incoming {
    Response { txn: u16, client_id: Result<u64> },
    Report,
    Other,
}

/// Sensor core client over `T`
#[derive(Debug)]
pub struct Client<T> {
    transport: T,
    txn: u16,
    reports: VecDeque<Report>,
}

impl<T: Transport> Client<T> {
    /// Client over `transport`
    pub const fn new(transport: T) -> Self {
        Self {
            transport,
            txn: 0,
            reports: VecDeque::new(),
        }
    }

    /// The transport
    pub const fn transport(&self) -> &T {
        &self.transport
    }

    /// Sends `msg_id` with `payload` to `suid`; returns the client id from the response
    ///
    /// # Errors
    /// [`Error::Timeout`] without a response, [`Error::Qmi`] for a failure result, [`Error::ServiceGone`],
    /// socket and decoding errors
    pub fn request(&mut self, suid: Suid, msg_id: u32, payload: &[u8], timeout: Duration) -> Result<u64> {
        self.txn = match self.txn.wrapping_add(1) {
            0 => 1,
            txn => txn,
        };
        let txn = self.txn;
        self.transport
            .send(&msg::control(txn, &msg::client_request(suid, msg_id, payload))?)?;
        let start = Instant::now();
        loop {
            match self.receive(timeout.saturating_sub(start.elapsed()))? {
                None => return Err(Error::Timeout),
                Some(Incoming::Response { txn: t, client_id }) if t == txn => return client_id,
                Some(_) => {}
            }
        }
    }

    /// Next report, waiting at most `timeout`; `None` on timeout
    ///
    /// # Errors
    /// [`Error::ServiceGone`], socket and decoding errors
    pub fn next_report(&mut self, timeout: Duration) -> Result<Option<Report>> {
        let start = Instant::now();
        loop {
            if let Some(report) = self.reports.pop_front() {
                return Ok(Some(report));
            }
            if self.receive(timeout.saturating_sub(start.elapsed()))?.is_none() {
                return Ok(None);
            }
        }
    }

    /// First event `msg_id` from `suid`; reports in between are dropped
    ///
    /// # Errors
    /// [`Error::Timeout`], [`Error::ServiceGone`], socket and decoding errors
    pub fn wait_event(&mut self, suid: Suid, msg_id: u32, timeout: Duration) -> Result<Event> {
        let start = Instant::now();
        loop {
            let Some(report) = self.next_report(timeout.saturating_sub(start.elapsed()))? else {
                return Err(Error::Timeout);
            };
            if report.suid != suid {
                continue;
            }
            if let Some(event) = report.events.into_iter().find(|e| e.msg_id == msg_id) {
                return Ok(event);
            }
        }
    }

    /// Sensors of `data_type` ("accel"), the default one first
    ///
    /// # Errors
    /// [`Error::NotFound`] if there are none (yet), [`Error::Timeout`], and as [`Self::request`]
    pub fn find(&mut self, data_type: &str, timeout: Duration) -> Result<Vec<Suid>> {
        let start = Instant::now();
        let _ = self.request(SUID_LOOKUP, id::SUID_REQ, &msg::suid_request(data_type), timeout)?;
        loop {
            let event = self.wait_event(SUID_LOOKUP, id::SUID_EVENT, timeout.saturating_sub(start.elapsed()))?;
            let event = msg::decode_suid_event(&event.payload)?;
            if event.data_type == data_type {
                return if event.suids.is_empty() {
                    Err(Error::NotFound)
                } else {
                    Ok(event.suids)
                };
            }
        }
    }

    /// Attributes of `suid`
    ///
    /// # Errors
    /// [`Error::Timeout`], and as [`Self::request`]
    pub fn attributes(&mut self, suid: Suid, timeout: Duration) -> Result<Vec<Attr>> {
        let start = Instant::now();
        let _ = self.request(suid, id::ATTR_REQ, &[], timeout)?;
        let event = self.wait_event(suid, id::ATTR_EVENT, timeout.saturating_sub(start.elapsed()))?;
        msg::decode_attributes(&event.payload)
    }

    /// Starts streaming `suid` at `rate_hz`
    ///
    /// # Errors
    /// As [`Self::request`]
    pub fn enable(&mut self, suid: Suid, rate_hz: f32, timeout: Duration) -> Result<()> {
        self.request(suid, id::SENSOR_CONFIG, &msg::sensor_config(rate_hz), timeout)
            .map(drop)
    }

    /// Stops streaming `suid`
    ///
    /// # Errors
    /// As [`Self::request`]
    pub fn disable(&mut self, suid: Suid, timeout: Duration) -> Result<()> {
        self.request(suid, id::DISABLE_REQ, &[], timeout).map(drop)
    }

    fn receive(&mut self, timeout: Duration) -> Result<Option<Incoming>> {
        let Some(packet) = self.transport.recv(timeout)? else {
            return Ok(None);
        };
        let Packet::Qmi(bytes) = packet else {
            return Err(Error::ServiceGone);
        };
        let message = qmi::decode(&bytes)?;
        Ok(Some(match (message.kind, message.id) {
            (Kind::Response, msg::CONTROL) => Incoming::Response {
                txn: message.txn,
                client_id: msg::control_response(&message),
            },
            (Kind::Indication, msg::REPORT_SMALL | msg::REPORT_LARGE) => {
                self.reports.push_back(msg::decode_report(msg::report_data(&message)?)?);
                Incoming::Report
            }
            _ => Incoming::Other,
        }))
    }
}

#[cfg(any(target_os = "linux", target_os = "android"))]
pub use qrtr_link::Qrtr;

#[cfg(any(target_os = "linux", target_os = "android"))]
mod qrtr_link {
    use std::time::{Duration, Instant};

    use super::{Packet, Transport};
    use crate::error::{Error, Result};
    use crate::msg;
    use crate::qrtr::{self, Notice, PORT_CTRL, Server, Socket};

    /// The sensor core over QRTR
    #[derive(Debug)]
    pub struct Qrtr {
        sock: Socket,
        server: Server,
        buf: Vec<u8>,
    }

    impl Qrtr {
        /// Looks the sensor core service up and talks to its first instance
        ///
        /// # Errors
        /// [`Error::NotFound`] if the name service doesn't list it, [`Error::Timeout`], socket errors
        pub fn connect(timeout: Duration) -> Result<Self> {
            let sock = Socket::open()?;
            let server = qrtr::lookup(&sock, msg::SERVICE, timeout)?
                .into_iter()
                .next()
                .ok_or(Error::NotFound)?;
            Ok(Self {
                sock,
                server,
                buf: vec![0; 65536],
            })
        }

        /// The service instance in use
        #[must_use]
        pub const fn server(&self) -> Server {
            self.server
        }
    }

    impl Transport for Qrtr {
        fn send(&mut self, msg: &[u8]) -> Result<()> {
            Ok(self.sock.send_to(msg, self.server.addr)?)
        }

        fn recv(&mut self, timeout: Duration) -> Result<Option<Packet>> {
            let start = Instant::now();
            loop {
                let Some((len, from)) = self
                    .sock
                    .recv_from(&mut self.buf, timeout.saturating_sub(start.elapsed()))?
                else {
                    return Ok(None);
                };
                let data = self.buf.get(..len).unwrap_or_default();
                if from.port == PORT_CTRL {
                    if let Notice::Gone(server) = qrtr::decode_notice(data)?
                        && server.addr == self.server.addr
                    {
                        return Ok(Some(Packet::Gone));
                    }
                } else if from == self.server.addr {
                    return Ok(Some(Packet::Qmi(data.to_vec())));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pb::Encoder;
    use crate::testutil::{ACCEL, report, response};

    #[derive(Default)]
    struct Fake {
        sent: Vec<Vec<u8>>,
        incoming: VecDeque<Packet>,
    }

    impl Transport for Fake {
        fn send(&mut self, msg: &[u8]) -> Result<()> {
            self.sent.push(msg.to_vec());
            Ok(())
        }

        fn recv(&mut self, _timeout: Duration) -> Result<Option<Packet>> {
            Ok(self.incoming.pop_front())
        }
    }

    const SENSOR: Suid = ACCEL;
    const T: Duration = Duration::from_millis(10);

    fn client(incoming: Vec<Packet>) -> Client<Fake> {
        Client::new(Fake {
            sent: Vec::new(),
            incoming: incoming.into(),
        })
    }

    #[test]
    fn request_waits_for_its_own_transaction() {
        let mut c = client(vec![
            response(5, 0, 1),
            report(SENSOR, &[(1025, &[])]),
            response(1, 0, 77),
        ]);
        assert_eq!(c.request(SENSOR, id::SENSOR_CONFIG, &[], T).unwrap(), 77);
        let sent = c.transport().sent.first().unwrap();
        let sent = qmi::decode(sent).unwrap();
        assert_eq!((sent.kind, sent.txn, sent.id), (Kind::Request, 1, msg::CONTROL));
        let queued = c.next_report(T).unwrap().unwrap();
        assert_eq!((queued.suid, queued.events.first().unwrap().msg_id), (SENSOR, 1025));
        assert!(c.next_report(T).unwrap().is_none());
    }

    #[test]
    fn transaction_ids_skip_zero() {
        let mut c = client(Vec::new());
        c.txn = u16::MAX;
        assert!(matches!(
            c.request(SENSOR, id::DISABLE_REQ, &[], T),
            Err(Error::Timeout)
        ));
        assert_eq!(c.txn, 1);
    }

    #[test]
    fn failed_result_is_a_qmi_error() {
        let mut c = client(vec![response(1, 1, 0)]);
        assert!(matches!(
            c.request(SENSOR, id::DISABLE_REQ, &[], T),
            Err(Error::Qmi { result: 1, .. })
        ));
    }

    #[test]
    fn service_gone_ends_a_request() {
        let mut c = client(vec![Packet::Gone]);
        assert!(matches!(c.enable(SENSOR, 10.0, T), Err(Error::ServiceGone)));
    }

    #[test]
    fn find_returns_the_suids_of_its_data_type() {
        let other = Encoder::new().bytes(1, b"gyro").finish();
        let accel = Encoder::new()
            .bytes(1, b"accel")
            .bytes(2, &Encoder::new().fixed64(1, 1).fixed64(2, 2).finish())
            .finish();
        let mut c = client(vec![
            response(1, 0, 3),
            report(SENSOR, &[(id::SUID_EVENT, &accel)]),
            report(SUID_LOOKUP, &[(id::SUID_EVENT, &other)]),
            report(SUID_LOOKUP, &[(id::SUID_EVENT, &accel)]),
        ]);
        assert_eq!(c.find("accel", T).unwrap(), [SENSOR]);
    }

    #[test]
    fn find_without_sensors_is_not_found() {
        let none = Encoder::new().bytes(1, b"accel").finish();
        let mut c = client(vec![response(1, 0, 3), report(SUID_LOOKUP, &[(id::SUID_EVENT, &none)])]);
        assert!(matches!(c.find("accel", T), Err(Error::NotFound)));
    }

    #[test]
    fn attributes_come_from_the_attribute_event() {
        let name = Encoder::new().bytes(2, b"lsm6dst").finish();
        let value = Encoder::new().bytes(1, &name).finish();
        let attr = Encoder::new().varint(1, 0).bytes(2, &value).finish();
        let event = Encoder::new().bytes(1, &attr).finish();
        let mut c = client(vec![
            response(1, 0, 3),
            report(SENSOR, &[(id::SENSOR_EVENT, &[]), (id::ATTR_EVENT, &event)]),
        ]);
        let attrs = c.attributes(SENSOR, T).unwrap();
        assert_eq!(attrs.first().unwrap().name(), "NAME");
        assert_eq!(attrs.first().unwrap().values, [msg::AttrValue::Str("lsm6dst".into())]);
    }
}
