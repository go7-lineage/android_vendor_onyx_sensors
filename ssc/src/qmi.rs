//! QMI messages as QRTR carries them: a 7-byte header, then type-length-value items

use crate::error::{Error, Result};

const HEADER: usize = 7;
const RESULT: u8 = 0x02;

/// Message type in the header
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// From the client
    Request,
    /// Answer to a request with the same transaction id
    Response,
    /// Unsolicited, from the service
    Indication,
}

impl Kind {
    const fn byte(self) -> u8 {
        match self {
            Self::Request => 0,
            Self::Response => 2,
            Self::Indication => 4,
        }
    }

    const fn from_byte(byte: u8) -> Result<Self> {
        match byte {
            0 => Ok(Self::Request),
            2 => Ok(Self::Response),
            4 => Ok(Self::Indication),
            _ => Err(Error::Malformed("QMI message type")),
        }
    }
}

/// A received message; its items borrow from the receive buffer
#[derive(Debug, Clone, Copy)]
pub struct Message<'a> {
    /// Message type
    pub kind: Kind,
    /// Transaction id
    pub txn: u16,
    /// Message id
    pub id: u16,
    items: &'a [u8],
}

/// Length as the 16-bit field QMI uses
///
/// # Errors
/// [`Error::Malformed`] above 65535
pub fn len16(len: usize) -> Result<u16> {
    u16::try_from(len).map_err(|_| Error::Malformed("QMI length over 65535"))
}

/// A message of `kind` with `items` (type, value) in order
///
/// # Errors
/// [`Error::Malformed`] if an item or the whole message is over 65535 bytes
pub fn encode(kind: Kind, txn: u16, id: u16, items: &[(u8, &[u8])]) -> Result<Vec<u8>> {
    let mut body = Vec::new();
    for &(item, value) in items {
        body.push(item);
        body.extend_from_slice(&len16(value.len())?.to_le_bytes());
        body.extend_from_slice(value);
    }
    let mut out = Vec::with_capacity(body.len().saturating_add(HEADER));
    out.push(kind.byte());
    out.extend_from_slice(&txn.to_le_bytes());
    out.extend_from_slice(&id.to_le_bytes());
    out.extend_from_slice(&len16(body.len())?.to_le_bytes());
    out.extend_from_slice(&body);
    Ok(out)
}

/// Header and items of `buf`; bytes after the length in the header are ignored
///
/// # Errors
/// [`Error::Malformed`] for a short buffer or an unknown message type
pub fn decode(buf: &[u8]) -> Result<Message<'_>> {
    let (&[kind, t0, t1, i0, i1, l0, l1], rest) = buf
        .split_first_chunk::<HEADER>()
        .ok_or(Error::Malformed("QMI header"))?;
    let items = rest
        .get(..usize::from(u16::from_le_bytes([l0, l1])))
        .ok_or(Error::Malformed("QMI length"))?;
    Ok(Message {
        kind: Kind::from_byte(kind)?,
        txn: u16::from_le_bytes([t0, t1]),
        id: u16::from_le_bytes([i0, i1]),
        items,
    })
}

impl<'a> Message<'a> {
    /// Value of item `item`, if the message has it
    ///
    /// # Errors
    /// [`Error::Malformed`] if the items run past the message
    pub fn item(&self, item: u8) -> Result<Option<&'a [u8]>> {
        let mut rest = self.items;
        while let Some((&ty, after)) = rest.split_first() {
            let (&[l0, l1], after) = after.split_first_chunk::<2>().ok_or(Error::Malformed("QMI item"))?;
            let (value, after) = after
                .split_at_checked(usize::from(u16::from_le_bytes([l0, l1])))
                .ok_or(Error::Malformed("QMI item length"))?;
            if ty == item {
                return Ok(Some(value));
            }
            rest = after;
        }
        Ok(None)
    }

    /// The standard result item (0x02) of a response
    ///
    /// # Errors
    /// [`Error::Qmi`] for a failure result, [`Error::Malformed`] if the item is missing or short
    pub fn check_result(&self) -> Result<()> {
        let value = self.item(RESULT)?.ok_or(Error::Malformed("QMI result"))?;
        let (&[r0, r1, e0, e1], _) = value.split_first_chunk::<4>().ok_or(Error::Malformed("QMI result"))?;
        match (u16::from_le_bytes([r0, r1]), u16::from_le_bytes([e0, e1])) {
            (0, _) => Ok(()),
            (result, error) => Err(Error::Qmi { result, error }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encodes_header_then_items() {
        let buf = encode(Kind::Request, 0x0102, 0x0020, &[(0x01, &[0xaa, 0xbb]), (0x10, &[1])]).unwrap();
        assert_eq!(
            buf,
            [0, 0x02, 0x01, 0x20, 0, 9, 0, 0x01, 2, 0, 0xaa, 0xbb, 0x10, 1, 0, 1]
        );
    }

    #[test]
    fn decodes_items_by_type() {
        let buf = encode(Kind::Indication, 7, 0x22, &[(0x01, &[1, 2]), (0x02, &[3])]).unwrap();
        let msg = decode(&buf).unwrap();
        assert_eq!((msg.kind, msg.txn, msg.id), (Kind::Indication, 7, 0x22));
        assert_eq!(msg.item(0x02).unwrap(), Some(&[3][..]));
        assert_eq!(msg.item(0x01).unwrap(), Some(&[1, 2][..]));
        assert_eq!(msg.item(0x03).unwrap(), None);
    }

    #[test]
    fn result_item_maps_to_error() {
        let ok = encode(Kind::Response, 1, 0x20, &[(RESULT, &[0, 0, 0, 0])]).unwrap();
        decode(&ok).unwrap().check_result().unwrap();
        let failed = encode(Kind::Response, 1, 0x20, &[(RESULT, &[1, 0, 0x30, 0])]).unwrap();
        assert!(matches!(
            decode(&failed).unwrap().check_result(),
            Err(Error::Qmi { result: 1, error: 0x30 })
        ));
        let missing = encode(Kind::Response, 1, 0x20, &[]).unwrap();
        assert!(decode(&missing).unwrap().check_result().is_err());
    }

    #[test]
    fn short_or_bad_input_is_an_error() {
        assert!(decode(&[0, 1, 0, 0x20, 0, 5]).is_err());
        assert!(decode(&[0, 1, 0, 0x20, 0, 5, 0, 1]).is_err());
        assert!(decode(&[9, 1, 0, 0x20, 0, 0, 0]).is_err());
        let msg = decode(&[2, 1, 0, 0x20, 0, 4, 0, 0x02, 9, 0, 0]).unwrap();
        assert!(msg.item(0x02).is_err());
    }
}
