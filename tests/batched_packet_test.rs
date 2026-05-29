//! Tests for the early-return bug in `send_command_and_get_i32` and its
//! sibling `send_command_and_get_i32_ack_events_u8`.
//!
//! **Bug**: When `receive_packet` returns a list containing both a Response
//! and a Command packet — as happens when two HDLC frames arrive inside a
//! single TCP/UART read — the `TypeField::Response` arm executes an early
//! `return` that exits the iteration loop before the remaining Command
//! packets are ever reached.  Those packets are silently dropped: never
//! enqueued in the pending-event queue and never ACKed.
//!
//! **Observable**: With the buggy code the mock transport's write buffer will
//! NOT contain a Response-type ACK frame (first content byte 0x01) for the
//! batched Command event.  With the fix the ACK frame is always present.
//!
//! Each test will **fail** on the unfixed code and **pass** after the fix is
//! applied (deferred `i32_result` pattern).

use embassy_futures::block_on;
use nrf_rpc::{
    TransportError,
    ble::{Ble, BtGattDiscoverParams, BtGattDiscoverType},
    uart_transport::{Uart, UartTransport},
};
use std::sync::{Arc, Mutex};

// ── helpers ───────────────────────────────────────────────────────────────────

#[derive(Debug)]
struct MockError;

impl core::fmt::Display for MockError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "mock transport error")
    }
}

impl TransportError for MockError {}

/// CRC-16/CCITT (seed 0xFFFF, poly 0x8408) — matches nRF RPC UART framing.
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

/// Build a complete HDLC-framed packet from raw (pre-escape) bytes.
///
/// Format: `0x7E | escaped(raw) | escaped(CRC_lo) | escaped(CRC_hi) | 0x7E`
fn make_hdlc_frame(raw: &[u8]) -> Vec<u8> {
    let crc = crc16_ccitt(raw);
    let mut out = vec![0x7Eu8];
    let mut push_escaped = |out: &mut Vec<u8>, b: u8| {
        if b == 0x7E || b == 0x7D {
            out.push(0x7D);
            out.push(b ^ 0x20);
        } else {
            out.push(b);
        }
    };
    for &b in raw {
        push_escaped(&mut out, b);
    }
    for &c in &crc.to_le_bytes() {
        push_escaped(&mut out, c);
    }
    out.push(0x7E);
    out
}

/// HDLC frame for a server-sent Response carrying a small non-negative i32.
///
/// Layout: `[type=0x01, cmd_id=0xFF, dst_ctx=0x00, src_grp=0x00,
///           dst_grp=0x00, cbor_int(value)]`
fn make_i32_response_frame(value: i32) -> Vec<u8> {
    assert!((0..=23).contains(&value), "helper only handles 0..=23");
    let raw = [0x01u8, 0xFF, 0x00, 0x00, 0x00, value as u8];
    make_hdlc_frame(&raw)
}

/// HDLC frame for a server-initiated Command (event) with an empty CBOR payload.
///
/// Layout: `[type=0x80|src_ctx, cmd_id, dst_ctx, src_grp, dst_grp, 0xF6]`
fn make_command_event_frame(
    src_ctx: u8,
    cmd_id: u8,
    dst_ctx: u8,
    src_grp: u8,
    dst_grp: u8,
) -> Vec<u8> {
    let raw = [0x80 | src_ctx, cmd_id, dst_ctx, src_grp, dst_grp, 0xF6u8];
    make_hdlc_frame(&raw)
}

/// Returns `true` when `buf` contains the two-byte sequence `[0x7E, 0x01]`,
/// which is the opening delimiter + type byte of a Response (ACK) frame.
///
/// Init frames start with `[0x7E, 0x04]` and outgoing commands with
/// `[0x7E, 0x80..]`, so this uniquely identifies a sent ACK in the write
/// buffer without decoding the full frame.
fn write_buffer_contains_response_ack(buf: &[u8]) -> bool {
    buf.windows(2).any(|w| w[0] == 0x7E && w[1] == 0x01)
}

// ── mock transport ────────────────────────────────────────────────────────────

/// A mock UART that:
/// - Returns `Ok(0)` for the first `skip_reads` `read()` calls so that the
///   `RpcClient::init()` drain step completes without consuming the test data.
/// - Delivers `read_data` on the `(skip_reads + 1)`-th `read()` call.
/// - Returns `Ok(0)` once all `read_data` bytes are exhausted.
/// - Records every `write()` call in the shared `writes` buffer.
struct DelayedOneShotTransport {
    skip_reads: usize,
    read_count: usize,
    read_data: Vec<u8>,
    read_pos: usize,
    writes: Arc<Mutex<Vec<u8>>>,
}

impl DelayedOneShotTransport {
    fn new(skip_reads: usize, read_data: Vec<u8>) -> (Self, Arc<Mutex<Vec<u8>>>) {
        let writes = Arc::new(Mutex::new(Vec::new()));
        let t = Self {
            skip_reads,
            read_count: 0,
            read_data,
            read_pos: 0,
            writes: Arc::clone(&writes),
        };
        (t, writes)
    }
}

impl Uart for DelayedOneShotTransport {
    type Error = MockError;

    async fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
        self.writes.lock().unwrap().extend_from_slice(data);
        Ok(data.len())
    }

    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_count += 1;
        if self.read_count <= self.skip_reads {
            return Ok(0);
        }
        if self.read_pos >= self.read_data.len() {
            return Ok(0);
        }
        let n = core::cmp::min(buf.len(), self.read_data.len() - self.read_pos);
        buf[..n].copy_from_slice(&self.read_data[self.read_pos..self.read_pos + n]);
        self.read_pos += n;
        Ok(n)
    }

    async fn delay_ms(&mut self, _ms: u32) {}

    fn has_buffered_data(&mut self) -> bool { false }
}

// ── tests ─────────────────────────────────────────────────────────────────────

/// **`send_command_and_get_i32`** — batched Command event after Response is dropped.
///
/// Scenario: `bt_enable(None)` is issued.  The server sends back a
/// `Response(i32=0)` and a `Command(cmd_id=42)` in the same read buffer
/// (two concatenated HDLC frames), with the Response appearing first.
///
/// *Bug*: the early `return self.decode_i32_response(...)` exits the
/// `for recv_packet in recv_packet_list` loop the moment the Response is
/// seen.  `recv_packet_list[1]` (the Command) is never reached; it is
/// dropped without being enqueued or ACKed.
///
/// *Observable*: No void-Response ACK (HDLC frame with type byte 0x01) is
/// written to the transport.
///
/// This test **fails** with the unfixed code and **passes** after applying
/// the deferred-`i32_result` fix.
#[test]
fn test_send_command_and_get_i32_drops_batched_command_event() {
    // Two HDLC frames delivered in a single read: Response(0) then Command(42).
    let mut read_data = make_i32_response_frame(0);
    read_data.extend(make_command_event_frame(
        /* src_ctx */ 0, /* cmd_id */ 42,
        /* dst_ctx */ 0, /* src_grp */ 0, /* dst_grp */ 0,
    ));

    // skip_reads=1: first inner read (init drain) returns Ok(0); second delivers data.
    let (transport, writes) = DelayedOneShotTransport::new(1, read_data);

    let mut ble =
        block_on(Ble::new(UartTransport::new(transport))).expect("Ble::new must succeed");

    let result = block_on(ble.bt_enable(None));

    assert!(
        result.is_ok(),
        "bt_enable must return Ok when response i32=0 is batched with a Command; got: {:?}",
        result.err()
    );
    assert_eq!(result.unwrap(), 0, "bt_enable must decode i32=0 from the response");

    let written = writes.lock().unwrap().clone();
    assert!(
        write_buffer_contains_response_ack(&written),
        "A void-Response ACK (0x7E 0x01 …) must be written for the batched Command event.\n\
         Bug: early `return` after TypeField::Response exits the recv_packet_list loop\n\
         before reaching the Command packet — it is dropped without being ACKed.\n\
         Written bytes: {:02X?}",
        written
    );
}

/// **`send_command_and_get_i32_ack_events_u8`** — same early-return bug.
///
/// Scenario: `bt_gatt_discover` is issued (uses `send_command_and_get_i32_ack_events_u8`
/// internally).  The server returns `Response(i32=0)` and `Command(cmd_id=7)`
/// batched in the same read buffer.
///
/// *Bug*: The early `return self.decode_i32_response(...)` in the
/// `TypeField::Response` arm drops the trailing Command without ACKing it.
///
/// *Observable*: No u8-Response ACK (HDLC frame with type byte 0x01) is
/// written to the transport.
///
/// This test **fails** with the unfixed code and **passes** after applying
/// the deferred-`i32_result` fix.
#[test]
fn test_send_command_and_get_i32_ack_events_u8_drops_batched_command_event() {
    // Two HDLC frames: Response(0) then Command(cmd_id=7).
    let mut read_data = make_i32_response_frame(0);
    read_data.extend(make_command_event_frame(
        /* src_ctx */ 0, /* cmd_id */ 7,
        /* dst_ctx */ 0, /* src_grp */ 0, /* dst_grp */ 0,
    ));

    // skip_reads=1 to let the init drain pass without consuming the test data.
    let (transport, writes) = DelayedOneShotTransport::new(1, read_data);

    let mut ble =
        block_on(Ble::new(UartTransport::new(transport))).expect("Ble::new must succeed");

    let params = BtGattDiscoverParams {
        // BT_UUID_GATT_CHRC (0x2803) encoded as Zephyr's bt_uuid_16 struct bytes:
        // [type=BT_UUID_TYPE_16=0x01, padding=0x00, val_lo=0x03, val_hi=0x28]
        uuid: [0x01, 0x00, 0x03, 0x28],
        start_handle: 1,
        end_handle: 0xFFFF,
        discover_type: BtGattDiscoverType::PrimaryService,
    };

    let result = block_on(ble.bt_gatt_discover(&params, 0xDEAD_BEEF));

    assert!(
        result.is_ok(),
        "bt_gatt_discover must return Ok when response i32=0 is batched with a Command; got: {:?}",
        result.err()
    );
    assert_eq!(
        result.unwrap(),
        0,
        "bt_gatt_discover must decode i32=0 from the response"
    );

    let written = writes.lock().unwrap().clone();
    assert!(
        write_buffer_contains_response_ack(&written),
        "A u8-Response ACK (0x7E 0x01 …) must be written for the batched Command event.\n\
         Bug: early `return` after TypeField::Response exits the recv_packet_list loop\n\
         before reaching the Command packet — it is dropped without being ACKed.\n\
         Written bytes: {:02X?}",
        written
    );
}
