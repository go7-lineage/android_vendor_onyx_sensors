//! Sensor core packets and a fake sensor core service, for tests

use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use std::time::Duration;

use crate::client::{Packet, Transport};
use crate::error::{Error, Result};
use crate::msg::{self, SUID_LOOKUP, Suid, id};
use crate::pb::{Encoder, Fields};
use crate::qmi::{self, Kind};

pub const ACCEL: Suid = Suid { low: 1, high: 2 };

pub fn response(txn: u16, result: u16, client_id: u64) -> Packet {
    Packet::Qmi(
        qmi::encode(
            Kind::Response,
            txn,
            msg::CONTROL,
            &[
                (0x02, &[result.to_le_bytes(), [0, 0]].concat()),
                (0x10, &client_id.to_le_bytes()),
            ],
        )
        .unwrap(),
    )
}

pub fn report(suid: Suid, events: &[(u32, &[u8])]) -> Packet {
    let events: Vec<(u32, u64, &[u8])> = events.iter().map(|&(msg_id, payload)| (msg_id, 42, payload)).collect();
    report_at(suid, &events)
}

pub fn report_at(suid: Suid, events: &[(u32, u64, &[u8])]) -> Packet {
    let suid = Encoder::new().fixed64(1, suid.low).fixed64(2, suid.high).finish();
    let mut report = Encoder::new().bytes(1, &suid);
    for (msg_id, tick, payload) in events {
        let event = Encoder::new()
            .fixed32(1, *msg_id)
            .fixed64(2, *tick)
            .bytes(3, payload)
            .finish();
        report = report.bytes(2, &event);
    }
    let report = report.finish();
    let data = [&u16::try_from(report.len()).unwrap().to_le_bytes()[..], &report].concat();
    Packet::Qmi(
        qmi::encode(
            Kind::Indication,
            0,
            msg::REPORT_LARGE,
            &[(0x01, &9u64.to_le_bytes()), (0x02, &data)],
        )
        .unwrap(),
    )
}

pub fn sample(data: &[f32], status: u64) -> Vec<u8> {
    let mut sample = Encoder::new();
    for v in data {
        sample = sample.float(1, *v);
    }
    sample.varint(2, status).finish()
}

pub fn suid_event(data_type: &str, suids: &[Suid]) -> Vec<u8> {
    let mut event = Encoder::new().bytes(1, data_type.as_bytes());
    for suid in suids {
        event = event.bytes(2, &Encoder::new().fixed64(1, suid.low).fixed64(2, suid.high).finish());
    }
    event.finish()
}

/// A sensor core service: answers every request, and SUID lookups with [`ACCEL`] once `ready`
#[derive(Debug, Default)]
pub struct Service {
    pub ready: bool,
    pub refuse: u32,
    pub reject: Option<u32>,
    pub requests: Vec<(Suid, u32, Vec<u8>)>,
    pub incoming: VecDeque<Packet>,
}

impl Service {
    pub fn shared(ready: bool) -> Rc<RefCell<Self>> {
        Rc::new(RefCell::new(Self {
            ready,
            ..Self::default()
        }))
    }

    pub fn requests_of(&self, msg_id: u32) -> Vec<Vec<u8>> {
        self.requests
            .iter()
            .filter(|(_, m, _)| *m == msg_id)
            .map(|(_, _, payload)| payload.clone())
            .collect()
    }
}

/// A link to a [`Service`]
#[derive(Debug)]
pub struct Link(pub Rc<RefCell<Service>>);

impl Link {
    /// A new link, unless the service still refuses connections
    pub fn connect(service: &Rc<RefCell<Service>>) -> Result<Self> {
        let mut s = service.borrow_mut();
        if s.refuse > 0 {
            s.refuse = s.refuse.saturating_sub(1);
            return Err(Error::NotFound);
        }
        Ok(Self(Rc::clone(service)))
    }
}

impl Transport for Link {
    fn send(&mut self, bytes: &[u8]) -> Result<()> {
        let message = qmi::decode(bytes)?;
        let data = message.item(0x01)?.unwrap();
        let (suid, msg_id, payload) = parse_request(&data[2..]);
        let mut s = self.0.borrow_mut();
        let result = u16::from(s.reject == Some(msg_id));
        s.incoming.push_back(response(message.txn, result, 3));
        if msg_id == id::SUID_REQ {
            let suids: &[Suid] = if s.ready { &[ACCEL] } else { &[] };
            s.incoming
                .push_back(report(SUID_LOOKUP, &[(id::SUID_EVENT, &suid_event("accel", suids))]));
        }
        s.requests.push((suid, msg_id, payload));
        Ok(())
    }

    fn recv(&mut self, _timeout: Duration) -> Result<Option<Packet>> {
        Ok(self.0.borrow_mut().incoming.pop_front())
    }
}

fn parse_request(request: &[u8]) -> (Suid, u32, Vec<u8>) {
    let (mut suid, mut msg_id, mut payload) = (None, 0, Vec::new());
    for field in Fields::new(request) {
        match field.unwrap() {
            (1, v) => suid = Some(Suid::decode(v.bytes("suid").unwrap()).unwrap()),
            (2, v) => msg_id = v.fixed32("msg_id").unwrap(),
            (4, v) => {
                for inner in Fields::new(v.bytes("request").unwrap()) {
                    if let (2, p) = inner.unwrap() {
                        payload = p.bytes("payload").unwrap().to_vec();
                    }
                }
            }
            _ => {}
        }
    }
    (suid.unwrap(), msg_id, payload)
}
