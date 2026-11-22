//! Protobuf (proto2) wire format, as far as the sensor core messages use it

use crate::error::{Error, Result};

const VARINT: u64 = 0;
const FIXED64: u64 = 1;
const LEN: u64 = 2;
const FIXED32: u64 = 5;

/// Builds one message, fields in the order they are added
#[derive(Debug, Default)]
pub struct Encoder {
    buf: Vec<u8>,
}

impl Encoder {
    /// Empty message
    #[must_use]
    pub const fn new() -> Self {
        Self { buf: Vec::new() }
    }

    fn key(&mut self, field: u32, wire: u64) {
        self.raw_varint(u64::from(field).wrapping_shl(3) | wire);
    }

    fn raw_varint(&mut self, mut value: u64) {
        while value >= 0x80 {
            self.buf.push(low7(value) | 0x80);
            value = value.wrapping_shr(7);
        }
        self.buf.push(low7(value));
    }

    /// Varint field: `uint32`, `uint64`, enums, non-negative `int32`
    #[must_use]
    pub fn varint(mut self, field: u32, value: u64) -> Self {
        self.key(field, VARINT);
        self.raw_varint(value);
        self
    }

    /// `bool` field
    #[must_use]
    pub fn bool(self, field: u32, value: bool) -> Self {
        self.varint(field, u64::from(value))
    }

    /// `fixed32` field
    #[must_use]
    pub fn fixed32(mut self, field: u32, value: u32) -> Self {
        self.key(field, FIXED32);
        self.buf.extend_from_slice(&value.to_le_bytes());
        self
    }

    /// `fixed64` field
    #[must_use]
    pub fn fixed64(mut self, field: u32, value: u64) -> Self {
        self.key(field, FIXED64);
        self.buf.extend_from_slice(&value.to_le_bytes());
        self
    }

    /// `float` field
    #[must_use]
    pub fn float(self, field: u32, value: f32) -> Self {
        self.fixed32(field, value.to_bits())
    }

    /// `bytes`, `string` or embedded message field
    #[must_use]
    pub fn bytes(mut self, field: u32, value: &[u8]) -> Self {
        self.key(field, LEN);
        self.raw_varint(u64::try_from(value.len()).unwrap_or(u64::MAX));
        self.buf.extend_from_slice(value);
        self
    }

    /// The encoded message
    #[must_use]
    pub fn finish(self) -> Vec<u8> {
        self.buf
    }
}

const fn low7(value: u64) -> u8 {
    let [byte, ..] = (value & 0x7f).to_le_bytes();
    byte
}

/// One field's value as it is on the wire
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Value<'a> {
    /// Varint
    Varint(u64),
    /// 8 bytes, little-endian
    Fixed64(u64),
    /// Length-delimited
    Bytes(&'a [u8]),
    /// 4 bytes, little-endian
    Fixed32(u32),
}

impl<'a> Value<'a> {
    /// The varint
    ///
    /// # Errors
    /// [`Error::Malformed`] naming `what` if the field has another wire type
    pub const fn varint(self, what: &'static str) -> Result<u64> {
        match self {
            Self::Varint(v) => Ok(v),
            _ => Err(Error::Malformed(what)),
        }
    }

    /// The bytes
    ///
    /// # Errors
    /// [`Error::Malformed`] naming `what` if the field has another wire type
    pub const fn bytes(self, what: &'static str) -> Result<&'a [u8]> {
        match self {
            Self::Bytes(v) => Ok(v),
            _ => Err(Error::Malformed(what)),
        }
    }

    /// The `fixed32`
    ///
    /// # Errors
    /// [`Error::Malformed`] naming `what` if the field has another wire type
    pub const fn fixed32(self, what: &'static str) -> Result<u32> {
        match self {
            Self::Fixed32(v) => Ok(v),
            _ => Err(Error::Malformed(what)),
        }
    }

    /// The `fixed64`
    ///
    /// # Errors
    /// [`Error::Malformed`] naming `what` if the field has another wire type
    pub const fn fixed64(self, what: &'static str) -> Result<u64> {
        match self {
            Self::Fixed64(v) => Ok(v),
            _ => Err(Error::Malformed(what)),
        }
    }

    /// The `float`
    ///
    /// # Errors
    /// [`Error::Malformed`] naming `what` if the field has another wire type
    pub fn float(self, what: &'static str) -> Result<f32> {
        Ok(f32::from_bits(self.fixed32(what)?))
    }

    /// Adds the `float`s of a repeated field, packed or not, to `out`
    ///
    /// # Errors
    /// [`Error::Malformed`] naming `what` for another wire type or a packed length that isn't a multiple of 4
    pub fn floats(self, what: &'static str, out: &mut Vec<f32>) -> Result<()> {
        match self {
            Self::Fixed32(v) => out.push(f32::from_bits(v)),
            Self::Bytes(packed) => {
                let (words, rest) = packed.as_chunks::<4>();
                if !rest.is_empty() {
                    return Err(Error::Malformed(what));
                }
                out.extend(words.iter().map(|w| f32::from_le_bytes(*w)));
            }
            _ => return Err(Error::Malformed(what)),
        }
        Ok(())
    }
}

/// The fields of one message, in wire order
#[derive(Debug, Clone)]
pub struct Fields<'a> {
    rest: &'a [u8],
}

impl<'a> Fields<'a> {
    /// Fields of `buf`
    #[must_use]
    pub const fn new(buf: &'a [u8]) -> Self {
        Self { rest: buf }
    }

    fn field(&mut self) -> Result<(u32, Value<'a>)> {
        let key = varint(&mut self.rest)?;
        let number = u32::try_from(key.wrapping_shr(3)).map_err(|_| Error::Malformed("protobuf field number"))?;
        let value = match key & 7 {
            VARINT => Value::Varint(varint(&mut self.rest)?),
            FIXED64 => Value::Fixed64(u64::from_le_bytes(take(&mut self.rest)?)),
            LEN => {
                let len = usize::try_from(varint(&mut self.rest)?).map_err(|_| Error::Malformed("protobuf length"))?;
                let (value, rest) = self
                    .rest
                    .split_at_checked(len)
                    .ok_or(Error::Malformed("protobuf length"))?;
                self.rest = rest;
                Value::Bytes(value)
            }
            FIXED32 => Value::Fixed32(u32::from_le_bytes(take(&mut self.rest)?)),
            _ => return Err(Error::Malformed("protobuf wire type")),
        };
        Ok((number, value))
    }
}

impl<'a> Iterator for Fields<'a> {
    type Item = Result<(u32, Value<'a>)>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.rest.is_empty() {
            return None;
        }
        let field = self.field();
        if field.is_err() {
            self.rest = &[];
        }
        Some(field)
    }
}

fn varint(buf: &mut &[u8]) -> Result<u64> {
    let mut value = 0u64;
    let mut shift = 0u32;
    while shift < 64 {
        let (&byte, rest) = buf.split_first().ok_or(Error::Malformed("protobuf varint"))?;
        *buf = rest;
        value |= u64::from(byte & 0x7f).wrapping_shl(shift);
        if byte & 0x80 == 0 {
            return Ok(value);
        }
        shift = shift.wrapping_add(7);
    }
    Err(Error::Malformed("protobuf varint"))
}

fn take<const N: usize>(buf: &mut &[u8]) -> Result<[u8; N]> {
    let (head, rest) = buf
        .split_first_chunk::<N>()
        .ok_or(Error::Malformed("protobuf fixed field"))?;
    *buf = rest;
    Ok(*head)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fields(buf: &[u8]) -> Vec<(u32, Value<'_>)> {
        Fields::new(buf).collect::<Result<_>>().unwrap()
    }

    #[test]
    fn encodes_each_wire_type() {
        let buf = Encoder::new()
            .varint(1, 300)
            .bool(2, true)
            .fixed32(3, 0x0102_0304)
            .fixed64(4, 5)
            .float(5, 1.0)
            .bytes(6, b"ab")
            .finish();
        assert_eq!(
            buf,
            [
                0x08, 0xac, 0x02, 0x10, 0x01, 0x1d, 0x04, 0x03, 0x02, 0x01, 0x21, 5, 0, 0, 0, 0, 0, 0, 0, 0x2d, 0, 0,
                0x80, 0x3f, 0x32, 2, b'a', b'b'
            ]
        );
    }

    #[test]
    fn decodes_what_it_encodes() {
        let buf = Encoder::new()
            .varint(1, u64::MAX)
            .fixed64(2, 7)
            .bytes(3, b"xyz")
            .fixed32(4, 9)
            .finish();
        assert_eq!(
            fields(&buf),
            [
                (1, Value::Varint(u64::MAX)),
                (2, Value::Fixed64(7)),
                (3, Value::Bytes(b"xyz")),
                (4, Value::Fixed32(9))
            ]
        );
    }

    #[test]
    fn large_field_numbers_use_multibyte_keys() {
        let buf = Encoder::new().varint(1025, 1).finish();
        assert_eq!(buf, [0x88, 0x40, 0x01]);
        assert_eq!(fields(&buf), [(1025, Value::Varint(1))]);
    }

    #[test]
    fn floats_accepts_packed_and_unpacked() {
        let mut out = Vec::new();
        Value::Fixed32(1.5f32.to_bits()).floats("t", &mut out).unwrap();
        let packed: Vec<u8> = [2.5f32, -1.0].iter().flat_map(|f| f.to_le_bytes()).collect();
        Value::Bytes(&packed).floats("t", &mut out).unwrap();
        assert_eq!(out, [1.5, 2.5, -1.0]);
        assert!(Value::Bytes(&[0, 0, 0]).floats("t", &mut out).is_err());
    }

    #[test]
    fn truncated_input_is_an_error_not_a_panic() {
        for buf in [
            &[0x08][..],
            &[0x0a, 0x05, b'a'],
            &[0x0d, 1, 2],
            &[0x09, 1],
            &[0x80; 11],
            &[0x0b],
        ] {
            let mut f = Fields::new(buf);
            assert!(f.next().unwrap().is_err(), "{buf:?}");
            assert!(f.next().is_none());
        }
    }

    #[test]
    fn wrong_wire_type_names_the_field() {
        assert!(matches!(Value::Varint(1).bytes("suid"), Err(Error::Malformed("suid"))));
    }
}
