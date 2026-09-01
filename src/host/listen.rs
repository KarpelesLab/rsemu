//! Where a remote frontend listens, and the rule about it.
//!
//! Two frontends bind TCP ports — the gdbstub and the VNC server — and both of
//! them hand a stranger something they should not have: the debugger can read
//! and write every byte of guest memory, and the VNC server can watch the
//! screen and type at the keyboard. So the address rule is one rule, in one
//! place, rather than two that could drift apart:
//!
//! > **A bare port or a leading colon binds the loopback interface only.**
//! > Exposing the port to the network is a decision someone makes explicitly,
//! > by naming an address (`0.0.0.0:5900`).
//!
//! That is the whole of the protection, and it is deliberate: RFB's own
//! authentication (RFC 6143 §7.2.2) is a DES challenge over a password
//! truncated to eight characters, which is not security, and offering it would
//! invite someone to rely on it.

use std::io::ErrorKind;
use std::net::{SocketAddr, TcpListener, ToSocketAddrs};

/// Turn `5900`, `:5900` or `host:5900` into addresses to bind.
///
/// # Errors
///
/// An address that does not parse or resolve.
pub fn resolve(addr: &str) -> std::io::Result<Vec<SocketAddr>> {
    let spec = if addr.starts_with(':') {
        format!("127.0.0.1{addr}")
    } else if addr.chars().all(|c| c.is_ascii_digit()) && !addr.is_empty() {
        format!("127.0.0.1:{addr}")
    } else {
        addr.to_string()
    };
    let list: Vec<SocketAddr> = spec.to_socket_addrs()?.collect();
    if list.is_empty() {
        return Err(std::io::Error::new(
            ErrorKind::InvalidInput,
            format!("`{addr}` resolved to no address"),
        ));
    }
    Ok(list)
}

/// Bind `addr` and put the listener in non-blocking mode.
///
/// Non-blocking because a frontend shares its thread with the machine: an
/// `accept` that blocked would stop the guest until somebody connected
/// (`CLAUDE.md` — submit jobs, never spawn threads).
///
/// # Errors
///
/// As [`resolve`], plus a port that cannot be bound.
pub fn bind(addr: &str) -> std::io::Result<TcpListener> {
    let resolved = resolve(addr)?;
    let listener = TcpListener::bind(&resolved[..])?;
    listener.set_nonblocking(true)?;
    Ok(listener)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_bare_port_binds_the_loopback_only() {
        let addrs = resolve("5900").expect("a bare port");
        assert!(addrs.iter().all(|a| a.ip().is_loopback()), "{addrs:?}");
        let addrs = resolve(":5900").expect("a leading colon");
        assert!(addrs.iter().all(|a| a.ip().is_loopback()), "{addrs:?}");
        let addrs = resolve("0.0.0.0:5900").expect("an explicit address");
        assert!(addrs.iter().any(|a| a.ip().is_unspecified()), "{addrs:?}");
    }

    #[test]
    fn a_nonsense_address_is_an_error_not_a_panic() {
        assert!(resolve("").is_err());
        assert!(resolve("not a host name at all:1").is_err());
    }

    #[test]
    fn an_ephemeral_port_lands_somewhere_and_says_where() {
        let listener = bind(":0").expect("bind");
        let addr = listener.local_addr().expect("local_addr");
        assert!(addr.port() != 0);
        assert!(addr.ip().is_loopback());
    }
}
