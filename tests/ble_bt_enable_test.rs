use nrf_rpc::{AsyncTransport, TransportError, ble::Ble};
use std::sync::{Arc, Mutex};

/// Simple in-memory UART used to unit test higher-level BLE behavior without
/// involving the Zephyr server or Babble simulator.
#[derive(Debug, Clone)]
struct DummyState {
    sent_packets: Vec<Vec<u8>>,
    response: Vec<u8>,
    read_calls: usize,
}

#[derive(Debug, Clone)]
struct DummyUart {
    state: Arc<Mutex<DummyState>>,
}

#[derive(Debug)]
struct DummyError;

impl core::fmt::Display for DummyError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Dummy transport error")
    }
}

impl TransportError for DummyError {}

impl DummyUart {
    fn new(response: Vec<u8>) -> Self {
        Self {
            state: Arc::new(Mutex::new(DummyState {
                sent_packets: Vec::new(),
                response,
                read_calls: 0,
            })),
        }
    }

    fn state(&self) -> Arc<Mutex<DummyState>> {
        Arc::clone(&self.state)
    }
}

impl AsyncTransport for DummyUart {
    type Error = DummyError;
    type TxTransportPacket<'a> = nrf_rpc::uart_transport::UartTxTransport<'a>;
    type RxTransportPacket<'a> = nrf_rpc::uart_transport::UartRxTransport<'a>;

    async fn write(&mut self, data: &mut [u8]) -> Result<usize, Self::Error> {
        let mut state = self.state.lock().unwrap();
        state.sent_packets.push(data.to_vec());
        Ok(data.len())
    }

    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        let mut state = self.state.lock().unwrap();
        // Provide a valid response for the first two reads (bt_enable and
        // settings load), then behave like an empty read.
        if state.read_calls >= 2 {
            // No more data; behave like a non-blocking empty read.
            return Ok(0);
        }

        state.read_calls += 1;
        let n = core::cmp::min(buffer.len(), state.response.len());
        buffer[..n].copy_from_slice(&state.response[..n]);
        Ok(n)
    }
}

/// Minimal async executor for this test - same pattern as in integration_test.rs.
fn block_on<F: core::future::Future>(mut f: F) -> F::Output {
    use std::pin::Pin;
    use std::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    unsafe fn clone(_: *const ()) -> RawWaker {
        RawWaker::new(std::ptr::null(), &VTABLE)
    }
    unsafe fn wake(_: *const ()) {}
    unsafe fn wake_by_ref(_: *const ()) {}
    unsafe fn drop(_: *const ()) {}

    static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, wake, wake_by_ref, drop);

    let waker = unsafe { Waker::from_raw(RawWaker::new(std::ptr::null(), &VTABLE)) };
    let mut context = Context::from_waker(&waker);

    // Pin the future
    let mut pinned = unsafe { Pin::new_unchecked(&mut f) };

    loop {
        match pinned.as_mut().poll(&mut context) {
            Poll::Ready(output) => return output,
            Poll::Pending => {
                panic!("Future didn't complete immediately - tests need a real async runtime");
            }
        }
    }
}

/// CRC16-CCITT calculation matching the UART transport implementation.
/// Polynomial 0x8408, seed 0xffff, reflected input/output.
fn calculate_crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc = 0xffffu16;
    for &byte in data {
        crc ^= byte as u16;
        for _ in 0..8 {
            if (crc & 1) != 0 {
                crc = (crc >> 1) ^ 0x8408u16;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

#[test]
fn test_bt_enable_uses_enable_command_and_parses_status() {
    // Minimal nRF RPC response frame for bt_enable:
    // Raw packet (before UART framing):
    // 01: Type = response
    // FF: cmd/evt/cnt unused for responses
    // 00: dst ctx id
    // 00: src group id
    // 00: dst group id
    // 00: CBOR 0 (success status)

    // Calculate CRC16-CCITT for the raw packet
    let raw_packet = vec![0x01, 0xFF, 0x00, 0x00, 0x00, 0x00];
    let crc = calculate_crc16_ccitt(&raw_packet);

    // UART framing: 0x7e (delimiter) + raw_packet + crc (2 bytes, LE) + 0x7e (delimiter)
    let mut response = vec![0x7e]; // opening delimiter
    response.extend_from_slice(&raw_packet);
    response.extend_from_slice(&crc.to_le_bytes()); // CRC in little-endian
    response.push(0x7e); // closing delimiter

    let uart = DummyUart::new(response);
    let state_handle = uart.state();

    // Initialize BLE client without a real server behind the transport.
    let mut ble = block_on(Ble::new(uart)).expect("Failed to initialize BLE client");

    // Drop any packets sent during init; we only care about bt_enable traffic.
    {
        let mut state = state_handle.lock().unwrap();
        state.sent_packets.clear();
        state.read_calls = 0;
    }

    // bt_enable should complete successfully given a zero status response.
    block_on(ble.bt_enable(5)).expect("bt_enable RPC failed");

    // Ensure we sent exactly one command frame and performed at least one read.
    let state = state_handle.lock().unwrap();
    assert!(
        !state.sent_packets.is_empty(),
        "Expected at least one command frame from bt_enable"
    );
    assert!(
        state.read_calls >= 1,
        "Expected bt_enable to read a response frame"
    );

    // Inspect the first command frame, which should be BT_ENABLE_RPC_CMD.
    let frame = &state.sent_packets[0];

    // First byte: command packet (0x80 | ctx).
    assert!(
        !frame.is_empty() && (frame[1] & 0x80) == 0x80,
        "First byte should indicate a command packet (0x80 | ctx): got 0x{:02X}",
        frame[0]
    );

    // Second byte: command ID for bt_enable (BT_ENABLE_RPC_CMD == 0x01).
    assert_eq!(
        frame[2], 0x01,
        "Command ID for bt_enable should be 0x01 (BT_ENABLE_RPC_CMD)"
    );
}
