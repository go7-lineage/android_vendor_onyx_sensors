//! QRTR: datagram sockets to services on the DSPs, and the kernel's name service that lists those services

use crate::error::{Error, Result};

/// `AF_QIPCRTR`; the `libc` crate at the AOSP version doesn't have it
pub const AF_QIPCRTR: i32 = 42;
/// Port of the name service on the local node
pub const PORT_CTRL: u32 = 0xffff_fffe;

const NEW_SERVER: u32 = 4;
const DEL_SERVER: u32 = 5;
const NEW_LOOKUP: u32 = 10;

/// Node and port
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Addr {
    /// Node (processor)
    pub node: u32,
    /// Port on that node
    pub port: u32,
}

/// A service instance the name service announces
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Server {
    /// Service number
    pub service: u32,
    /// Instance: version in the low byte, instance above
    pub instance: u32,
    /// Where it listens
    pub addr: Addr,
}

/// What the name service sends after a lookup
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Notice {
    /// A server is there, or came up
    New(Server),
    /// A server went away
    Gone(Server),
    /// The list of servers that were up when the lookup started is complete
    EndOfList,
    /// Any other control packet
    Other,
}

/// `NEW_LOOKUP` for every instance of `service`
#[must_use]
pub fn lookup_packet(service: u32) -> [u8; 20] {
    let mut packet = [0; 20];
    for (word, value) in packet.as_chunks_mut::<4>().0.iter_mut().zip([NEW_LOOKUP, service]) {
        *word = value.to_le_bytes();
    }
    packet
}

/// Decodes a control packet from the name service
///
/// # Errors
/// [`Error::Malformed`] if it is shorter than its command needs
pub fn decode_notice(buf: &[u8]) -> Result<Notice> {
    let (words, _) = buf.as_chunks::<4>();
    let Some(cmd) = words.first() else {
        return Err(Error::Malformed("QRTR control packet"));
    };
    let cmd = u32::from_le_bytes(*cmd);
    if cmd != NEW_SERVER && cmd != DEL_SERVER {
        return Ok(Notice::Other);
    }
    let [_, service, instance, node, port, ..] = words else {
        return Err(Error::Malformed("QRTR server notice"));
    };
    let server = Server {
        service: u32::from_le_bytes(*service),
        instance: u32::from_le_bytes(*instance),
        addr: Addr {
            node: u32::from_le_bytes(*node),
            port: u32::from_le_bytes(*port),
        },
    };
    Ok(match cmd {
        NEW_SERVER if server.service == 0 && server.addr.node == 0 && server.addr.port == 0 => Notice::EndOfList,
        NEW_SERVER => Notice::New(server),
        _ => Notice::Gone(server),
    })
}

#[cfg(any(target_os = "linux", target_os = "android"))]
pub use socket::{Socket, lookup};

#[cfg(any(target_os = "linux", target_os = "android"))]
mod socket {
    use std::io;
    use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
    use std::time::{Duration, Instant};

    use super::{AF_QIPCRTR, Addr, Notice, PORT_CTRL, Server, decode_notice, lookup_packet};
    use crate::error::{Error, Result};

    #[repr(C)]
    struct SockaddrQrtr {
        family: libc::sa_family_t,
        node: u32,
        port: u32,
    }

    impl SockaddrQrtr {
        fn new(addr: Addr) -> Self {
            Self {
                family: libc::sa_family_t::try_from(AF_QIPCRTR).unwrap_or_default(),
                node: addr.node,
                port: addr.port,
            }
        }
    }

    fn addr_len() -> libc::socklen_t {
        libc::socklen_t::try_from(size_of::<SockaddrQrtr>()).unwrap_or_default()
    }

    /// `AF_QIPCRTR` datagram socket; the kernel binds a port on the first send
    #[derive(Debug)]
    pub struct Socket {
        fd: OwnedFd,
    }

    impl Socket {
        /// New socket
        ///
        /// # Errors
        /// `socket(2)` failure, such as a kernel without QRTR
        pub fn open() -> io::Result<Self> {
            // SAFETY: socket(2) with constant arguments has no memory effects
            let fd = unsafe { libc::socket(AF_QIPCRTR, libc::SOCK_DGRAM | libc::SOCK_CLOEXEC, 0) };
            if fd < 0 {
                return Err(io::Error::last_os_error());
            }
            // SAFETY: `fd` was just returned by socket(2) and nothing else owns it
            let fd = unsafe { OwnedFd::from_raw_fd(fd) };
            Ok(Self { fd })
        }

        /// This socket's address; the node is the local node before any send
        ///
        /// # Errors
        /// `getsockname(2)` failure
        pub fn local_addr(&self) -> io::Result<Addr> {
            let mut sa = SockaddrQrtr::new(Addr { node: 0, port: 0 });
            let mut len = addr_len();
            // SAFETY: `sa` and `len` describe one writable sockaddr_qrtr for the duration of the call
            if unsafe { libc::getsockname(self.fd.as_raw_fd(), (&raw mut sa).cast(), &raw mut len) } != 0 {
                return Err(io::Error::last_os_error());
            }
            Ok(Addr {
                node: sa.node,
                port: sa.port,
            })
        }

        /// Sends one datagram to `to`
        ///
        /// # Errors
        /// `sendto(2)` failure; `ENODEV` or `EHOSTUNREACH` once the remote is gone
        pub fn send_to(&self, buf: &[u8], to: Addr) -> io::Result<()> {
            let sa = SockaddrQrtr::new(to);
            // SAFETY: `buf` and `sa` are readable for their lengths for the duration of the call
            let sent = unsafe {
                libc::sendto(
                    self.fd.as_raw_fd(),
                    buf.as_ptr().cast(),
                    buf.len(),
                    0,
                    (&raw const sa).cast(),
                    addr_len(),
                )
            };
            if sent < 0 {
                return Err(io::Error::last_os_error());
            }
            Ok(())
        }

        /// Receives one datagram into `buf`, waiting at most `timeout`; `None` on timeout
        ///
        /// # Errors
        /// `poll(2)` or `recvfrom(2)` failure
        pub fn recv_from(&self, buf: &mut [u8], timeout: Duration) -> io::Result<Option<(usize, Addr)>> {
            let mut pfd = libc::pollfd {
                fd: self.fd.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            };
            let ms = libc::c_int::try_from(timeout.as_millis()).unwrap_or(libc::c_int::MAX);
            // SAFETY: one valid pollfd for the duration of the call
            let ready = unsafe { libc::poll(&raw mut pfd, 1, ms) };
            if ready < 0 {
                let e = io::Error::last_os_error();
                return if e.kind() == io::ErrorKind::Interrupted {
                    Ok(None)
                } else {
                    Err(e)
                };
            }
            if ready == 0 {
                return Ok(None);
            }
            let mut sa = SockaddrQrtr::new(Addr { node: 0, port: 0 });
            let mut len = addr_len();
            // SAFETY: `buf`, `sa` and `len` are writable for their lengths for the duration of the call
            let got = unsafe {
                libc::recvfrom(
                    self.fd.as_raw_fd(),
                    buf.as_mut_ptr().cast(),
                    buf.len(),
                    0,
                    (&raw mut sa).cast(),
                    &raw mut len,
                )
            };
            let got = usize::try_from(got).map_err(|_| io::Error::last_os_error())?;
            Ok(Some((
                got,
                Addr {
                    node: sa.node,
                    port: sa.port,
                },
            )))
        }
    }

    /// Every server of `service` the name service knows now. The lookup stays registered: the socket also gets
    /// [`Notice::New`] and [`Notice::Gone`] for that service later, from port [`PORT_CTRL`].
    ///
    /// # Errors
    /// Socket errors, a malformed notice, or [`Error::Timeout`] before the end of the list
    pub fn lookup(sock: &Socket, service: u32, timeout: Duration) -> Result<Vec<Server>> {
        let name_service = Addr {
            node: sock.local_addr()?.node,
            port: PORT_CTRL,
        };
        sock.send_to(&lookup_packet(service), name_service)?;
        let start = Instant::now();
        let mut servers = Vec::new();
        let mut buf = [0; 64];
        loop {
            let left = timeout.saturating_sub(start.elapsed());
            let Some((len, from)) = sock.recv_from(&mut buf, left)? else {
                return Err(Error::Timeout);
            };
            if from.port != PORT_CTRL {
                continue;
            }
            match decode_notice(buf.get(..len).unwrap_or_default())? {
                Notice::New(server) if server.service == service => servers.push(server),
                Notice::EndOfList => return Ok(servers),
                _ => {}
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn packet(words: [u32; 5]) -> Vec<u8> {
        words.iter().flat_map(|w| w.to_le_bytes()).collect()
    }

    #[test]
    fn lookup_packet_is_new_lookup_with_any_instance() {
        assert_eq!(lookup_packet(400).to_vec(), packet([10, 400, 0, 0, 0]));
    }

    #[test]
    fn decodes_server_notices() {
        let server = Server {
            service: 400,
            instance: 1,
            addr: Addr { node: 5, port: 0x4001 },
        };
        assert_eq!(
            decode_notice(&packet([4, 400, 1, 5, 0x4001])).unwrap(),
            Notice::New(server)
        );
        assert_eq!(
            decode_notice(&packet([5, 400, 1, 5, 0x4001])).unwrap(),
            Notice::Gone(server)
        );
        assert_eq!(decode_notice(&packet([4, 0, 0, 0, 0])).unwrap(), Notice::EndOfList);
        assert_eq!(decode_notice(&packet([6, 5, 9, 0, 0])).unwrap(), Notice::Other);
    }

    #[test]
    fn short_packets_are_errors() {
        assert!(decode_notice(&[4, 0]).is_err());
        assert!(decode_notice(&[4, 0, 0, 0, 1, 0, 0, 0]).is_err());
    }
}
