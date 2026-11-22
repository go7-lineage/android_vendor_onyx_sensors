//! Sensor core messages: QMI service 400 and the protobuf messages it carries.
//! Field numbers are those of the stock `sns_client`, `sns_std`, `sns_std_sensor` and `sns_suid` protos.

use std::fmt;

use crate::error::{Error, Result};
use crate::pb::{Encoder, Fields};
use crate::qmi::{self, Kind, Message};

/// QMI service number of the sensor core
pub const SERVICE: u32 = 400;
/// QMI message id of a control request and its response
pub const CONTROL: u16 = 0x20;
/// QMI message id of a small report indication
pub const REPORT_SMALL: u16 = 0x21;
/// QMI message id of a large report indication
pub const REPORT_LARGE: u16 = 0x22;

const REPORT_TYPE_LARGE: u8 = 1;
const PROCESSOR_APSS: u64 = 1;
const DELIVERY_NO_WAKEUP: u64 = 1;
const MAX_ATTR_DEPTH: u8 = 4;

/// Message ids inside `sns_client_request_msg` and `sns_client_event_msg`
pub mod id {
    /// `SNS_STD_MSGID_SNS_STD_ATTR_REQ`
    pub const ATTR_REQ: u32 = 1;
    /// `SNS_CLIENT_MSGID_SNS_CLIENT_DISABLE_REQ`
    pub const DISABLE_REQ: u32 = 10;
    /// `SNS_STD_MSGID_SNS_STD_ATTR_EVENT`
    pub const ATTR_EVENT: u32 = 128;
    /// `SNS_STD_MSGID_SNS_STD_ERROR_EVENT`
    pub const ERROR_EVENT: u32 = 130;
    /// `SNS_SUID_MSGID_SNS_SUID_REQ`
    pub const SUID_REQ: u32 = 512;
    /// `SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG`
    pub const SENSOR_CONFIG: u32 = 513;
    /// `SNS_SUID_MSGID_SNS_SUID_EVENT`, from the SUID sensor
    pub const SUID_EVENT: u32 = 768;
    /// `SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT`, from a physical sensor
    pub const PHYSICAL_CONFIG_EVENT: u32 = 768;
    /// `SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT`
    pub const SENSOR_EVENT: u32 = 1025;
}

/// Sensor unique id
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Suid {
    /// Low 64 bits
    pub low: u64,
    /// High 64 bits
    pub high: u64,
}

/// The SUID sensor, which looks up the others
pub const SUID_LOOKUP: Suid = Suid {
    low: 0xabab_abab_abab_abab,
    high: 0xabab_abab_abab_abab,
};

impl Suid {
    fn encode(self) -> Vec<u8> {
        Encoder::new().fixed64(1, self.low).fixed64(2, self.high).finish()
    }

    fn decode(buf: &[u8]) -> Result<Self> {
        let mut suid = Self { low: 0, high: 0 };
        for field in Fields::new(buf) {
            match field? {
                (1, v) => suid.low = v.fixed64("SUID")?,
                (2, v) => suid.high = v.fixed64("SUID")?,
                _ => {}
            }
        }
        Ok(suid)
    }
}

impl fmt::Display for Suid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:016x}{:016x}", self.high, self.low)
    }
}

/// `sns_client_request_msg` for `msg_id` to `suid`, from the application processor, delivered without
/// waking it; an empty `payload` is left out
#[must_use]
pub fn client_request(suid: Suid, msg_id: u32, payload: &[u8]) -> Vec<u8> {
    let suspend = Encoder::new()
        .varint(1, PROCESSOR_APSS)
        .varint(2, DELIVERY_NO_WAKEUP)
        .finish();
    let request = if payload.is_empty() {
        Encoder::new()
    } else {
        Encoder::new().bytes(2, payload)
    }
    .finish();
    Encoder::new()
        .bytes(1, &suid.encode())
        .fixed32(2, msg_id)
        .bytes(3, &suspend)
        .bytes(4, &request)
        .finish()
}

/// `sns_suid_req` for the default sensor of `data_type` ("accel"), without update notices
#[must_use]
pub fn suid_request(data_type: &str) -> Vec<u8> {
    Encoder::new()
        .bytes(1, data_type.as_bytes())
        .bool(2, false)
        .bool(3, true)
        .finish()
}

/// `sns_std_sensor_config` for streaming at `rate_hz`
#[must_use]
pub fn sensor_config(rate_hz: f32) -> Vec<u8> {
    Encoder::new().float(1, rate_hz).finish()
}

/// QMI control request carrying `request` (a `sns_client_request_msg`), asking for large reports
///
/// # Errors
/// [`Error::Malformed`] if `request` is over 65533 bytes
pub fn control(txn: u16, request: &[u8]) -> Result<Vec<u8>> {
    let mut data = Vec::with_capacity(request.len().saturating_add(2));
    data.extend_from_slice(&qmi::len16(request.len())?.to_le_bytes());
    data.extend_from_slice(request);
    qmi::encode(
        Kind::Request,
        txn,
        CONTROL,
        &[(0x01, &data), (0x10, &[REPORT_TYPE_LARGE])],
    )
}

/// Client id in a control response
///
/// # Errors
/// [`Error::Qmi`] for a failure result, [`Error::Malformed`] if the client id is missing
pub fn control_response(msg: &Message<'_>) -> Result<u64> {
    msg.check_result()?;
    let value = msg.item(0x10)?.ok_or(Error::Malformed("SSC client id"))?;
    let (id, _) = value
        .split_first_chunk::<8>()
        .ok_or(Error::Malformed("SSC client id"))?;
    Ok(u64::from_le_bytes(*id))
}

/// The protobuf data (`sns_client_event_msg`) of a report indication
///
/// # Errors
/// [`Error::Malformed`] if the data item is missing or short
pub fn report_data<'a>(msg: &Message<'a>) -> Result<&'a [u8]> {
    let value = msg.item(0x02)?.ok_or(Error::Malformed("SSC report"))?;
    let (&[l0, l1], rest) = value.split_first_chunk::<2>().ok_or(Error::Malformed("SSC report"))?;
    rest.get(..usize::from(u16::from_le_bytes([l0, l1])))
        .ok_or(Error::Malformed("SSC report length"))
}

/// One event of a report
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Event {
    /// What the payload is
    pub msg_id: u32,
    /// `QTimer` ticks
    pub timestamp: u64,
    /// Protobuf message for `msg_id`
    pub payload: Vec<u8>,
}

/// `sns_client_event_msg`: events of one sensor
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Report {
    /// The sensor
    pub suid: Suid,
    /// Its events, oldest first
    pub events: Vec<Event>,
}

/// Decodes a `sns_client_event_msg`
///
/// # Errors
/// [`Error::Malformed`] for bad protobuf
pub fn decode_report(buf: &[u8]) -> Result<Report> {
    let mut suid = None;
    let mut events = Vec::new();
    for field in Fields::new(buf) {
        match field? {
            (1, v) => suid = Some(Suid::decode(v.bytes("SSC report SUID")?)?),
            (2, v) => events.push(decode_event(v.bytes("SSC event")?)?),
            _ => {}
        }
    }
    Ok(Report {
        suid: suid.ok_or(Error::Malformed("SSC report without SUID"))?,
        events,
    })
}

fn decode_event(buf: &[u8]) -> Result<Event> {
    let mut event = Event {
        msg_id: 0,
        timestamp: 0,
        payload: Vec::new(),
    };
    for field in Fields::new(buf) {
        match field? {
            (1, v) => event.msg_id = v.fixed32("SSC event id")?,
            (2, v) => event.timestamp = v.fixed64("SSC event timestamp")?,
            (3, v) => v.bytes("SSC event payload")?.clone_into(&mut event.payload),
            _ => {}
        }
    }
    Ok(event)
}

/// `sns_suid_event`: sensors of one data type
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SuidEvent {
    /// Data type that was looked up
    pub data_type: String,
    /// Its sensors, the default one first; empty if there are none (yet)
    pub suids: Vec<Suid>,
}

/// Decodes a `sns_suid_event`
///
/// # Errors
/// [`Error::Malformed`] for bad protobuf
pub fn decode_suid_event(buf: &[u8]) -> Result<SuidEvent> {
    let mut event = SuidEvent {
        data_type: String::new(),
        suids: Vec::new(),
    };
    for field in Fields::new(buf) {
        match field? {
            (1, v) => event.data_type = String::from_utf8_lossy(v.bytes("SUID data type")?).into_owned(),
            (2, v) => event.suids.push(Suid::decode(v.bytes("SUID")?)?),
            _ => {}
        }
    }
    Ok(event)
}

/// `sns_std_sensor_event`: one sample
#[derive(Debug, Clone, PartialEq)]
pub struct Sample {
    /// Values; x, y, z in m/s² for an accelerometer
    pub data: Vec<f32>,
    /// `sns_std_sensor_sample_status`: 0 unreliable, 1 low, 2 medium, 3 high accuracy
    pub status: u64,
}

/// Decodes a `sns_std_sensor_event`
///
/// # Errors
/// [`Error::Malformed`] for bad protobuf
pub fn decode_sample(buf: &[u8]) -> Result<Sample> {
    let mut sample = Sample {
        data: Vec::new(),
        status: 0,
    };
    for field in Fields::new(buf) {
        match field? {
            (1, v) => v.floats("sample data", &mut sample.data)?,
            (2, v) => sample.status = v.varint("sample status")?,
            _ => {}
        }
    }
    Ok(sample)
}

/// `sns_std_sensor_physical_config_event`: what the hardware is set to
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PhysicalConfig {
    /// Hz
    pub sample_rate: Option<f32>,
    /// Minimum and maximum
    pub range: Vec<f32>,
    /// Resolution in data units
    pub resolution: Option<f32>,
    /// Hardware operating mode
    pub operation_mode: Option<String>,
    /// µA
    pub active_current: Option<u64>,
}

/// Decodes a `sns_std_sensor_physical_config_event`
///
/// # Errors
/// [`Error::Malformed`] for bad protobuf
pub fn decode_physical_config(buf: &[u8]) -> Result<PhysicalConfig> {
    let mut config = PhysicalConfig::default();
    for field in Fields::new(buf) {
        match field? {
            (1, v) => config.sample_rate = Some(v.float("config rate")?),
            (3, v) => v.floats("config range", &mut config.range)?,
            (4, v) => config.resolution = Some(v.float("config resolution")?),
            (5, v) => config.operation_mode = Some(String::from_utf8_lossy(v.bytes("config mode")?).into_owned()),
            (6, v) => config.active_current = Some(v.varint("config current")?),
            _ => {}
        }
    }
    Ok(config)
}

/// Decodes a `sns_std_error_event` to its `sns_std_error` code
///
/// # Errors
/// [`Error::Malformed`] for bad protobuf
pub fn decode_error(buf: &[u8]) -> Result<u64> {
    let mut code = 0;
    for field in Fields::new(buf) {
        if let (1, v) = field? {
            code = v.varint("error code")?;
        }
    }
    Ok(code)
}

/// One value of an attribute
#[derive(Debug, Clone, PartialEq)]
pub enum AttrValue {
    /// Nested list (ranges, for one)
    List(Vec<Self>),
    /// String
    Str(String),
    /// Float
    Float(f32),
    /// Integer
    Int(i64),
    /// Boolean
    Bool(bool),
}

impl fmt::Display for AttrValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::List(values) => {
                f.write_str("[")?;
                for (n, v) in values.iter().enumerate() {
                    if n > 0 {
                        f.write_str(", ")?;
                    }
                    write!(f, "{v}")?;
                }
                f.write_str("]")
            }
            Self::Str(s) => write!(f, "{s:?}"),
            Self::Float(v) => write!(f, "{v}"),
            Self::Int(v) => write!(f, "{v}"),
            Self::Bool(v) => write!(f, "{v}"),
        }
    }
}

/// One attribute (`sns_std_attr`)
#[derive(Debug, Clone, PartialEq)]
pub struct Attr {
    /// `sns_std_sensor_attr_id`
    pub id: i64,
    /// Its values
    pub values: Vec<AttrValue>,
}

/// Decodes a `sns_std_attr_event`
///
/// # Errors
/// [`Error::Malformed`] for bad protobuf or values nested too deep
pub fn decode_attributes(buf: &[u8]) -> Result<Vec<Attr>> {
    let mut attrs = Vec::new();
    for field in Fields::new(buf) {
        if let (1, v) = field? {
            let mut attr = Attr {
                id: 0,
                values: Vec::new(),
            };
            for field in Fields::new(v.bytes("attribute")?) {
                match field? {
                    (1, v) => attr.id = v.varint("attribute id")?.cast_signed(),
                    (2, v) => attr.values = attr_values(v.bytes("attribute value")?, MAX_ATTR_DEPTH)?,
                    _ => {}
                }
            }
            attrs.push(attr);
        }
    }
    Ok(attrs)
}

fn attr_values(buf: &[u8], depth: u8) -> Result<Vec<AttrValue>> {
    let depth = depth.checked_sub(1).ok_or(Error::Malformed("attribute nesting"))?;
    let mut values = Vec::new();
    for field in Fields::new(buf) {
        if let (1, v) = field?
            && let Some(value) = attr_value(v.bytes("attribute data")?, depth)?
        {
            values.push(value);
        }
    }
    Ok(values)
}

fn attr_value(buf: &[u8], depth: u8) -> Result<Option<AttrValue>> {
    let mut value = None;
    for field in Fields::new(buf) {
        value = match field? {
            (1, v) => Some(AttrValue::List(attr_values(v.bytes("attribute list")?, depth)?)),
            (2, v) => Some(AttrValue::Str(
                String::from_utf8_lossy(v.bytes("attribute string")?).into_owned(),
            )),
            (3, v) => Some(AttrValue::Float(v.float("attribute float")?)),
            (4, v) => Some(AttrValue::Int(v.fixed64("attribute int")?.cast_signed())),
            (5, v) => Some(AttrValue::Bool(v.varint("attribute bool")? != 0)),
            _ => value,
        };
    }
    Ok(value)
}

/// Names of `sns_std_sensor_attr_id` 0..=28
pub const ATTR_NAMES: [&str; 29] = [
    "NAME",
    "VENDOR",
    "TYPE",
    "AVAILABLE",
    "VERSION",
    "API",
    "RATES",
    "RESOLUTIONS",
    "FIFO_SIZE",
    "ACTIVE_CURRENT",
    "SLEEP_CURRENT",
    "RANGES",
    "OP_MODES",
    "DRI",
    "STREAM_SYNC",
    "EVENT_SIZE",
    "STREAM_TYPE",
    "DYNAMIC",
    "HW_ID",
    "RIGID_BODY",
    "PLACEMENT",
    "PHYSICAL_SENSOR",
    "PHYSICAL_SENSOR_TESTS",
    "SELECTED_RESOLUTION",
    "SELECTED_RANGE",
    "ADDITIONAL_LOW_LATENCY_RATES",
    "PASSIVE_REQUEST",
    "DIRECT_CHANNEL_MAX_SAMPLE_RATE",
    "DIRECT_CHANNEL_MAX_REPORT_RATE",
];

impl Attr {
    /// Name of the attribute id, `"?"` if unknown
    #[must_use]
    pub fn name(&self) -> &'static str {
        usize::try_from(self.id)
            .ok()
            .and_then(|n| ATTR_NAMES.get(n))
            .copied()
            .unwrap_or("?")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(s: &str) -> Vec<u8> {
        s.as_bytes()
            .chunks(2)
            .map(|c| u8::from_str_radix(std::str::from_utf8(c).unwrap(), 16).unwrap())
            .collect()
    }

    const SENSOR: Suid = Suid {
        low: 0x1122_3344_5566_7788,
        high: 0x99aa_bbcc_ddee_ff00,
    };

    #[test]
    fn suid_lookup_request_matches_protoc() {
        assert_eq!(suid_request("accel"), hex("0a05616363656c10001801"));
        assert_eq!(
            client_request(SUID_LOOKUP, id::SUID_REQ, &suid_request("accel")),
            hex("0a1209abababababababab11abababababababab15000200001a0408011001220d120b0a05616363656c10001801")
        );
    }

    #[test]
    fn sensor_config_request_matches_protoc() {
        assert_eq!(
            client_request(SENSOR, id::SENSOR_CONFIG, &sensor_config(10.0)),
            hex("0a120988776655443322111100ffeeddccbbaa9915010200001a0408011001220712050d00002041")
        );
    }

    #[test]
    fn disable_request_matches_protoc() {
        assert_eq!(
            client_request(SENSOR, id::DISABLE_REQ, &[]),
            hex("0a120988776655443322111100ffeeddccbbaa99150a0000001a04080110012200")
        );
    }

    #[test]
    fn control_request_wraps_the_protobuf() {
        let buf = control(3, &[0xaa, 0xbb]).unwrap();
        assert_eq!(
            buf,
            [0, 3, 0, 0x20, 0, 11, 0, 0x01, 4, 0, 2, 0, 0xaa, 0xbb, 0x10, 1, 0, 1]
        );
    }

    #[test]
    fn control_response_gives_the_client_id() {
        let buf = qmi::encode(
            Kind::Response,
            3,
            CONTROL,
            &[
                (0x02, &[0, 0, 0, 0]),
                (0x10, &7u64.to_le_bytes()),
                (0x11, &[0, 0, 0, 0]),
            ],
        )
        .unwrap();
        assert_eq!(control_response(&qmi::decode(&buf).unwrap()).unwrap(), 7);
    }

    #[test]
    fn report_data_strips_the_length_prefix() {
        let buf = qmi::encode(
            Kind::Indication,
            0,
            REPORT_LARGE,
            &[(0x01, &7u64.to_le_bytes()), (0x02, &[2, 0, 9, 8])],
        )
        .unwrap();
        assert_eq!(report_data(&qmi::decode(&buf).unwrap()).unwrap(), [9, 8]);
    }

    #[test]
    fn decodes_a_report_from_protoc() {
        let report = decode_report(&hex(concat!(
            "0a120988776655443322111100ffeeddccbbaa9912310d0003000011e8030000000000001a210d0000504110011d00001dc2",
            "1d00001d42250ad7a33a2a064e4f524d414c30a00112210d010400001115cd5b07000000001a110d0000803e0d000000bf0d",
            "00001c411003"
        )))
        .unwrap();
        assert_eq!(report.suid, SENSOR);
        assert_eq!(report.events.len(), 2);
        let config = report.events.first().unwrap();
        assert_eq!((config.msg_id, config.timestamp), (id::PHYSICAL_CONFIG_EVENT, 1000));
        let config = decode_physical_config(&config.payload).unwrap();
        assert_eq!(config.sample_rate, Some(13.0));
        assert_eq!(config.range, [-39.25, 39.25]);
        assert_eq!(config.resolution, Some(0.00125));
        assert_eq!(config.operation_mode.as_deref(), Some("NORMAL"));
        assert_eq!(config.active_current, Some(160));
        let sample = report.events.get(1).unwrap();
        assert_eq!((sample.msg_id, sample.timestamp), (id::SENSOR_EVENT, 123_456_789));
        let sample = decode_sample(&sample.payload).unwrap();
        assert_eq!(sample.data, [0.25, -0.5, 9.75]);
        assert_eq!(sample.status, 3);
    }

    #[test]
    fn decodes_a_suid_event_from_protoc() {
        let event = decode_suid_event(&hex(
            "0a05616363656c12120988776655443322111100ffeeddccbbaa991212090807060504030201111817161514131211",
        ))
        .unwrap();
        assert_eq!(event.data_type, "accel");
        assert_eq!(
            event.suids,
            [
                SENSOR,
                Suid {
                    low: 0x0102_0304_0506_0708,
                    high: 0x1112_1314_1516_1718
                }
            ]
        );
        assert_eq!(
            event.suids.first().unwrap().to_string(),
            "99aabbccddeeff001122334455667788"
        );
    }

    #[test]
    fn decodes_attributes_from_protoc() {
        let attrs = decode_attributes(&hex(concat!(
            "0a0f0800120b0a0912076c736d366473740a19080612150a051d000050410a051d0000d0410a051d000050420a16080b1212",
            "0a100a0e0a051d00001dc20a051d00001d420a0f0810120b0a0921ffffffffffffffff0a08081112040a022801"
        )))
        .unwrap();
        let shown: Vec<String> = attrs
            .iter()
            .map(|a| format!("{} {}", a.name(), AttrValue::List(a.values.clone())))
            .collect();
        assert_eq!(
            shown,
            [
                "NAME [\"lsm6dst\"]",
                "RATES [13, 26, 52]",
                "RANGES [[-39.25, 39.25]]",
                "STREAM_TYPE [-1]",
                "DYNAMIC [true]"
            ]
        );
    }

    #[test]
    fn deep_attribute_nesting_is_refused() {
        let mut value = Encoder::new().bytes(1, &Encoder::new().bool(5, true).finish()).finish();
        for _ in 0..MAX_ATTR_DEPTH {
            value = Encoder::new()
                .bytes(1, &Encoder::new().bytes(1, &value).finish())
                .finish();
        }
        let attr = Encoder::new().varint(1, 0).bytes(2, &value).finish();
        assert!(decode_attributes(&Encoder::new().bytes(1, &attr).finish()).is_err());
    }

    #[test]
    fn error_event_gives_the_code() {
        assert_eq!(decode_error(&Encoder::new().varint(1, 5).finish()).unwrap(), 5);
    }
}
