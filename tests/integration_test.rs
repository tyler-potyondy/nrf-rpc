//! Integration tests that verify BLE function calls produce correct packet bytes.
//! 
//! These tests verify that calling Ble methods with specific parameters generates
//! the exact byte sequences observed in the UART trace from:
//! nrf/samples/nrf_rpc/protocols_serialization/client/src/bt_test_shell.c

use nrf_rpc::ble::{
    Ble, BtData, BtLeAdvParam, BT_DATA_FLAGS, BT_DATA_NAME_COMPLETE, BT_LE_AD_GENERAL,
    BT_LE_AD_NO_BREDR,
};
use nrf_rpc::{AsyncTransport, TransportError};
use std::sync::{Arc, Mutex};

/// Mock error type
#[derive(Debug)]
struct MockError;

impl core::fmt::Display for MockError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Mock transport error")
    }
}

impl TransportError for MockError {}

/// Mock UART transport that records all written packets
#[derive(Clone)]
struct MockUart {
    sent_packets: Arc<Mutex<Vec<Vec<u8>>>>,
}

impl MockUart {
    fn new() -> Self {
        Self {
            sent_packets: Arc::new(Mutex::new(Vec::new())),
        }
    }

    fn get_sent_packets(&self) -> Vec<Vec<u8>> {
        self.sent_packets.lock().unwrap().clone()
    }

    fn clear_packets(&self) {
        self.sent_packets.lock().unwrap().clear();
    }
}

impl AsyncTransport for MockUart {
    type Error = MockError;

    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        // Log the packet being sent
        println!("MockUart: Sending {} bytes: {:02X?}", data.len(), data);
        self.sent_packets.lock().unwrap().push(data.to_vec());
        Ok(())
    }

    async fn read(&mut self, _buffer: &mut [u8]) -> Result<usize, Self::Error> {
        // For these tests, we don't simulate responses
        Ok(0)
    }
}

/// Helper to convert hex string to bytes
fn hex_to_bytes(hex: &str) -> Vec<u8> {
    hex.replace(",", "")
        .replace("{", "")
        .replace("}", "")
        .replace("0x", "")
        .trim()
        .split_whitespace()
        .map(|s| u8::from_str_radix(s, 16).unwrap())
        .collect()
}

/// Minimal async runtime for tests - just polls futures to completion
fn block_on<F: core::future::Future>(mut f: F) -> F::Output {
    use std::pin::Pin;
    use std::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    // Create a no-op waker
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
    
    // Poll until complete
    loop {
        match pinned.as_mut().poll(&mut context) {
            Poll::Ready(output) => return output,
            Poll::Pending => {
                // In real async runtime this would yield, but our futures complete immediately
                panic!("Future didn't complete immediately - tests need a real async runtime");
            }
        }
    }
}

#[test]
fn test_bt_enable_generates_correct_packet() {
    block_on(async {
        // From trace: bt_enable() generates this packet
        // Note: In real usage with responses, group IDs would be 0x00
        // but our mock doesn't simulate responses, so they stay at 0xFF
        let expected_packet = hex_to_bytes("80 00 FF FF FF 18 1C 18 1C F6");

        let uart = MockUart::new();
        let uart_clone = uart.clone(); // Keep a reference to check packets
        
        // new() automatically initializes RPC and sends 2 init packets
        let mut ble = Ble::new(uart).await.ok().unwrap();
        uart_clone.clear_packets();
        
        // Call bt_enable
        ble.bt_enable().await.ok();
        
        let packets = uart_clone.get_sent_packets();
        assert_eq!(packets.len(), 1, "Expected 1 packet from bt_enable");
        
        assert_eq!(packets[0], expected_packet,
            "bt_enable packet mismatch\nExpected: {:02X?}\nGot:      {:02X?}",
            expected_packet, packets[0]);
    });
}

#[test]
fn test_bt_le_adv_start_generates_correct_packet() {
    block_on(async {
        // From trace: "bt advertise on" command generates this packet
        // Note: In real usage with responses, group IDs would be 0x00
        // but our mock doesn't simulate responses, so they stay at 0xFF
        let expected_packet = hex_to_bytes(
            "80 04 FF FF FF 18 20 00 00 00 03 18 A0 18 F0 F6 \
             01 01 01 41 06 01 09 09 49 4E 6F 72 64 69 63 5F 50 53 F6"
        );

        let uart = MockUart::new();
        let uart_clone = uart.clone(); // Keep a reference to check packets
        
        // new() automatically initializes RPC and sends 2 init packets
        let mut ble = Ble::new(uart).await.ok().unwrap();
        uart_clone.clear_packets();
        
        // Call bt_le_adv_start with the same parameters as the trace
        let param = BtLeAdvParam {
            id: 0,
            sid: 0,
            secondary_max_skip: 0,
            options: 0x03,  // BT_LE_ADV_OPT_CONNECTABLE | connectable-something
            interval_min: 160,
            interval_max: 240,
            peer: None,
        };
        
        let ad = [BtData {
            data_type: BT_DATA_FLAGS,
            data: &[BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR],
        }];
        
        let sd = [BtData {
            data_type: BT_DATA_NAME_COMPLETE,
            data: b"Nordic_PS",
        }];
        
        ble.bt_le_adv_start(&param, &ad, &sd).await.ok();
        
        let packets = uart_clone.get_sent_packets();
        assert_eq!(packets.len(), 1, "Expected 1 packet from bt_le_adv_start");
        
        assert_eq!(packets[0], expected_packet,
            "bt_le_adv_start packet mismatch\nExpected: {:02X?}\nGot:      {:02X?}",
            expected_packet, packets[0]);
    });
}
