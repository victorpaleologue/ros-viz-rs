//! Environment diagnostics shared by tests and startup logging.

use std::net::UdpSocket;
use std::time::Duration;

/// Whether DDS-style UDP multicast works on this host's default interface.
///
/// rustdds deliberately skips the loopback interface, so discovery and data
/// exchange ride the default route — which macOS blocks when the app lacks
/// the *Local Network* permission (send fails with "No route to host"), and
/// some networks/VPNs break in subtler ways. Tests that need real DDS call
/// this and skip with a message instead of failing on machine state.
///
/// The probe sends one datagram to an RTPS-style multicast group and waits
/// briefly for it to loop back.
pub fn dds_multicast_available() -> bool {
    fn probe() -> std::io::Result<bool> {
        const GROUP: std::net::Ipv4Addr = std::net::Ipv4Addr::new(239, 255, 0, 1);
        // An uncommon port, to not disturb real DDS on 7400+.
        const PORT: u16 = 17979;

        let receiver = UdpSocket::bind(("0.0.0.0", PORT))?;
        receiver.join_multicast_v4(&GROUP, &std::net::Ipv4Addr::UNSPECIFIED)?;
        receiver.set_read_timeout(Some(Duration::from_millis(800)))?;

        let sender = UdpSocket::bind(("0.0.0.0", 0))?;
        sender.set_multicast_loop_v4(true)?;
        sender.send_to(b"ros-viz-rs-mc-probe", (GROUP, PORT))?;

        let mut buf = [0u8; 32];
        match receiver.recv_from(&mut buf) {
            Ok((n, _)) => Ok(&buf[..n] == b"ros-viz-rs-mc-probe"),
            Err(_) => Ok(false),
        }
    }
    probe().unwrap_or(false)
}

/// Skip the current test when the host cannot do DDS multicast.
///
/// Expands to an early `return` with a loud explanation; environmental
/// preconditions should read as *skipped*, not as failures.
#[macro_export]
macro_rules! require_dds_multicast {
    () => {
        if !$crate::diagnostics::dds_multicast_available() {
            eprintln!(
                "SKIPPED: UDP multicast is unavailable on this host's default \
                 interface, so DDS cannot work (rustdds does not use loopback). \
                 On macOS, grant your terminal the 'Local Network' permission \
                 (System Settings > Privacy & Security > Local Network) and \
                 check the Wi-Fi/VPN state."
            );
            return;
        }
    };
}

#[cfg(test)]
mod tests {
    /// The probe must never panic or hang, whatever the network state.
    #[test]
    fn probe_completes() {
        let available = super::dds_multicast_available();
        eprintln!("dds_multicast_available = {available}");
    }
}
