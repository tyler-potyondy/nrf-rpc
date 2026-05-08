extern crate alloc;

mod utils;
use alloc::{boxed::Box, rc::Rc, vec::Vec};
use core::cell::RefCell;
use nrf_rpc::{ble::Ble, uart_transport::UartTransport};

// #[test]
fn test_rpc_init_packets() {
    let mock_uart = utils::MockUart {
        transmitted: Rc::new(RefCell::new(Vec::new())),
    };

    let mock_uart_clone = mock_uart.clone();
    let result = utils::block_on(Box::pin(async { Ble::new(UartTransport::new(mock_uart_clone)).await }));

    assert!(
        result.is_ok(),
        "Ble::new() should succeed during init, error: {:?}",
        result.err().unwrap()
    );

    // Check that the packet is correct.
    // Correct init packet should be: {04 00 ff 00 ff 00 62 74  5f 72 70 63}
    let bt_rpc_init_packet = [
        0x04, 0x00, 0xFF, 0x00, 0xFF, 0x00, b'b', b't', b'_', b'r', b'p', b'c',
    ];
    let rpc_utils_init_packet = [
        0x04, 0x00, 0xFF, 0x01, 0xFF, 0x00, b'r', b'p', b'c', b'_', b'u', b't', b'i', b'l', b's',
    ];

    let expected_data = [
        bt_rpc_init_packet.as_slice(),
        rpc_utils_init_packet.as_slice(),
    ]
    .concat();

    let transmitted = mock_uart.transmitted.borrow();
    // Check that the transmitted packet matches the expected init packet.
    assert_eq!(
        transmitted.as_slice(),
        expected_data.as_slice(),
        "[Test Failure] - Transmitted packet should match expected init packet. 
        Expected: {:02x?}, 
        Got:      {:02x?}",
        expected_data.as_slice(),
        transmitted.as_slice()
    );
}
