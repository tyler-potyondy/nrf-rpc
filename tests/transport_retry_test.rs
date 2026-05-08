//! Tests for the two bugs fixed in `src/lib.rs`:
//!
//! **Fix 1 — `receive_packet` propagates transport errors with `?`**
//!
//! Before the fix, `receive_packet` had:
//! ```ignore
//! .await.expect("Failed to receive packet")  // panics on Err
//! ```
//! This made the `Err(_) => continue` retry branches in every `send_command_*`
//! method dead code — a transport read error would always panic before the
//! retry branch could fire.
//!
//! After the fix:
//! ```ignore
//! .await.map_err(|_| RpcError::Transport)?   // propagates as Err
//! ```
//! The retry branches are now live: transport errors are retried and the caller
//! receives `Err(RpcError::Timeout)` after the budget is exhausted.
//!
//! **Fix 2 — Retry budget 5 × 100 ms → 20 × 200 ms; `NoResponse` → `Timeout`**
//!
//! Before the fix, each `send_command_*` method retried at most 5 times (500 ms
//! total) and returned `Err(RpcError::NoResponse)` on exhaustion.
//! After the fix, 20 retries (up to 4 s) are attempted and exhaustion returns
//! `Err(RpcError::Timeout)`.

use embassy_futures::block_on;
use nrf_rpc::{RpcError, TransportError, ble::Ble, uart_transport::{Uart, UartTransport}};
use std::sync::{Arc, Mutex};

// ── shared error / helper ─────────────────────────────────────────────────────

#[derive(Debug)]
struct MockError;

impl core::fmt::Display for MockError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "mock transport error")
    }
}

impl TransportError for MockError {}

/// Compute CRC-16/CCITT (seed = 0xFFFF, poly = 0x8408) — matches the nRF RPC UART framing.
fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        let mut b = byte;
        for _ in 0..8 {
            if (crc ^ b as u16) & 0x0001 != 0 {
                crc = (crc >> 1) ^ 0x8408;
            } else {
                crc >>= 1;
            }
            b >>= 1;
        }
    }
    crc
}

/// Build a complete HDLC-framed NRF-RPC response frame that carries `value`
/// as a CBOR-encoded i32.  This is the wire format that `receive_packet` +
/// `decode_i32_response` expect when a command returns `int`.
///
/// Frame layout: 0x7E | raw_pkt | CRC_lo | CRC_hi | 0x7E
/// Raw packet:   [type=0x01, 0xFF, dst_ctx=0x00, src_grp=0x00, dst_grp=0x00, cbor_i32]
fn make_i32_response_frame(value: i32) -> Vec<u8> {
    // CBOR integer encoding for small non-negative values (0..=23) is a single byte.
    // bt_enable success (0) is the only value used in these tests.
    assert!(
        (0..=23).contains(&value),
        "make_i32_response_frame only handles values 0..=23 for this test helper"
    );
    let raw = vec![0x01u8, 0xFF, 0x00, 0x00, 0x00, value as u8];
    let crc = crc16_ccitt(&raw);
    let mut frame = vec![0x7E];
    frame.extend_from_slice(&raw);
    frame.extend_from_slice(&crc.to_le_bytes());
    frame.push(0x7E);
    frame
}

// ── mock transports ───────────────────────────────────────────────────────────

/// Every `read()` call returns `Err`.
///
/// **Before Fix 1**: `receive_packet` called `.expect()` on the result →
/// the test would **panic**.
///
/// **After Fix 1**: `receive_packet` returns `Err(RpcError::Transport)`;
/// the `Err(_) => continue` branch fires; after 20 retries the caller gets
/// `Err(RpcError::Timeout)` — no panic.
struct AlwaysErrTransport;

impl Uart for AlwaysErrTransport {
    type Error = MockError;
    async fn write(&mut self, data: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(data.len())
    }
    async fn read(&mut self, _buf: &mut [u8]) -> Result<usize, Self::Error> {
        Err(MockError)
    }
    async fn delay_ms(&mut self, _ms: u32) {}
}

/// `read()` returns `Err` for the first `fail_count` calls, then returns a
/// valid response frame.
///
/// **Before Fix 1**: panics on the very first `Err`.
/// **After Fix 1+2**: retries through the failures and succeeds.
struct FailThenSucceedTransport {
    remaining_failures: Arc<Mutex<usize>>,
    response: Vec<u8>,
}

impl FailThenSucceedTransport {
    fn new(fail_count: usize, response: Vec<u8>) -> Self {
        Self {
            remaining_failures: Arc::new(Mutex::new(fail_count)),
            response,
        }
    }
}

impl Uart for FailThenSucceedTransport {
    type Error = MockError;
    async fn write(&mut self, data: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(data.len())
    }
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut rem = self.remaining_failures.lock().unwrap();
        if *rem > 0 {
            *rem -= 1;
            return Err(MockError);
        }
        let n = core::cmp::min(buf.len(), self.response.len());
        buf[..n].copy_from_slice(&self.response[..n]);
        Ok(n)
    }
    async fn delay_ms(&mut self, _ms: u32) {}
}

/// `read()` always returns `Ok(0)` — no data available.
///
/// **Before Fix 2**: `send_command_and_get_i32` returned
/// `Err(RpcError::NoResponse)` after only 5 retries.
///
/// **After Fix 2**: returns `Err(RpcError::Timeout)` after 20 retries.
struct NoDataTransport;

impl Uart for NoDataTransport {
    type Error = MockError;
    async fn write(&mut self, data: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(data.len())
    }
    async fn read(&mut self, _buf: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(0)
    }
    async fn delay_ms(&mut self, _ms: u32) {}
}

/// `read()` returns `Ok(0)` for the first `empty_count` calls, then returns a
/// valid response frame.
///
/// **Before Fix 2** (5 retries): fails when `empty_count >= 5`.
/// **After Fix 2** (20 retries): succeeds when `empty_count < 20`.
struct EmptyThenSucceedTransport {
    remaining_empty: Arc<Mutex<usize>>,
    response: Vec<u8>,
}

impl EmptyThenSucceedTransport {
    fn new(empty_count: usize, response: Vec<u8>) -> Self {
        Self {
            remaining_empty: Arc::new(Mutex::new(empty_count)),
            response,
        }
    }
}

impl Uart for EmptyThenSucceedTransport {
    type Error = MockError;
    async fn write(&mut self, data: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(data.len())
    }
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut rem = self.remaining_empty.lock().unwrap();
        if *rem > 0 {
            *rem -= 1;
            return Ok(0);
        }
        let n = core::cmp::min(buf.len(), self.response.len());
        buf[..n].copy_from_slice(&self.response[..n]);
        Ok(n)
    }
    async fn delay_ms(&mut self, _ms: u32) {}
}

// ── tests ─────────────────────────────────────────────────────────────────────

/// **Fix 1** — transport read errors propagate as `RpcError::Timeout` instead
/// of panicking.
///
/// The server is unreachable: every `transport.read()` returns `Err`.
///
/// *Before the fix*: `receive_packet` called `.await.expect(…)` which panics
/// on `Err`, so this test would fail with a panic.
///
/// *After the fix*: `receive_packet` uses `?`, the `Err(_) => continue` branch
/// in `send_command_and_get_i32` fires on every attempt, and after 20 retries
/// the function returns `Err(RpcError::Timeout)` — no panic.
#[test]
fn test_transport_read_error_propagates_as_timeout_not_panic() {
    let mut ble =
        block_on(Ble::new(UartTransport::new(AlwaysErrTransport))).expect("Ble::new must succeed (init only writes)");

    let result = block_on(ble.bt_enable(None));

    assert!(
        result.is_err(),
        "bt_enable must fail when the transport always returns Err on read"
    );
    assert!(
        matches!(result.unwrap_err(), RpcError::Timeout),
        "expected RpcError::Timeout after retries exhausted, not a panic"
    );
}

/// **Fix 1** — the retry branch is now reachable and recovers after transient
/// transport errors.
///
/// The transport returns `Err` for the first 3 reads, simulating transient
/// failures (e.g., the server UART buffer not yet ready), then delivers a
/// valid i32=0 (success) response frame.
///
/// *Before the fix*: `receive_packet` panicked on the very first `Err` read,
/// so recovery was impossible.
///
/// *After the fix*: `receive_packet` propagates `Err(RpcError::Transport)`;
/// the retry loop continues; the 4th read succeeds and `bt_enable` returns
/// `Ok(0)`.
#[test]
fn test_transport_error_then_success_on_retry() {
    let response = make_i32_response_frame(0);
    let transport = FailThenSucceedTransport::new(3, response);
    let mut ble =
        block_on(Ble::new(UartTransport::new(transport))).expect("Ble::new must succeed (init only writes)");

    let result = block_on(ble.bt_enable(None));

    assert!(
        result.is_ok(),
        "bt_enable must succeed after retrying past initial transport errors, got: {:?}",
        result.err()
    );
    assert_eq!(result.unwrap(), 0, "bt_enable must return status 0 (success)");
}

/// **Fix 2** — exhausted retries return `RpcError::Timeout`, not
/// `RpcError::NoResponse`.
///
/// The transport always returns `Ok(0)` (no data), simulating a server that
/// never responds (e.g., still booting).
///
/// *Before the fix*: `send_command_and_get_i32` returned
/// `Err(RpcError::NoResponse)` after 5 retries.  A caller checking for
/// `RpcError::Timeout` (the natural timeout sentinel) would never match.
///
/// *After the fix*: the method retries 20 times and returns
/// `Err(RpcError::Timeout)`.  `RpcError::Timeout` was previously defined but
/// unreachable; it is now the canonical exhaustion error.
#[test]
fn test_exhausted_retries_return_timeout_not_no_response() {
    let mut ble =
        block_on(Ble::new(UartTransport::new(NoDataTransport))).expect("Ble::new must succeed (init only writes)");

    let result = block_on(ble.bt_enable(None));

    assert!(
        result.is_err(),
        "bt_enable must fail when the transport never returns data"
    );
    assert!(
        matches!(result.unwrap_err(), RpcError::Timeout),
        "expected RpcError::Timeout (not RpcError::NoResponse) after retries exhausted"
    );
}

/// **Fix 2** — a late response that arrives on retry 11 now succeeds inside
/// the expanded 20-retry window.
///
/// The transport returns 10 empty reads before delivering a valid response.
/// This models a simulation stack (e.g., BabbleSim) that is still
/// initialising and doesn't respond to the first several attempts.
///
/// *Before the fix* (5 retries): the 11th read is never attempted; the
/// function returns `Err` after the 5th empty read.
///
/// *After the fix* (20 retries): the loop reaches i=10, gets the valid frame,
/// and `bt_enable` returns `Ok(0)`.
#[test]
fn test_late_response_succeeds_within_expanded_retry_window() {
    let response = make_i32_response_frame(0);
    // 10 empty reads — beyond the old 5-retry limit, within the new 20-retry limit.
    let transport = EmptyThenSucceedTransport::new(10, response);
    let mut ble =
        block_on(Ble::new(UartTransport::new(transport))).expect("Ble::new must succeed (init only writes)");

    let result = block_on(ble.bt_enable(None));

    assert!(
        result.is_ok(),
        "bt_enable must succeed when the response arrives within the 20-retry window, got: {:?}",
        result.err()
    );
    assert_eq!(result.unwrap(), 0, "bt_enable must return status 0 (success)");
}
