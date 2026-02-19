//! Integration tests intended to evaluate the `nrf-rpc` crate's client implementation
//! against the zephyr `sample/nrf_rpc/protocol_serialization/server`.
//!
//! This test is fully run on the host without any hardware. To do this, we use
//! nordic's Babble Simulator. The testing infastructure will build the zephyr sample,
//! launch the Babble Simulator, and then bind the server's pseudo port used for uart
//! rx/tx to a unix socket. The test here then provides a mock transport layer that
//! directs client writes to this socket and polls for responses on the socket.

use nrf_rpc::{AsyncTransport, TransportError};
use std::collections::HashSet;
use std::io::{BufRead, Read};
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

    async fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
        // Log the packet being sent
        println!("MockUart: Sending {} bytes: {:02X?}", data.len(), data);
        self.sent_packets.lock().unwrap().push(data.to_vec());
        Ok(data.len())
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

const ZEPHY_RPC_SERVER_RUN_SCRIPT: &str = "run_bsim.sh";
const TEST_DIRECTORY_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/");

use std::io::BufReader;
use std::time::{Duration, Instant};
pub mod TestProcessInfra {
    use std::{
        io::{BufReader, Lines},
        process::ChildStdout,
    };

    type ZephyrServerProcess = std::process::Child;
    type SocatProcess = std::process::Child;

    pub struct TestProcesses {
        rpc_server: ZephyrServerProcess,
        rpc_server_stdout: Lines<BufReader<ChildStdout>>,
        socat: SocatProcess,
    }

    impl TestProcesses {
        pub fn new(
            rpc_server: ZephyrServerProcess,
            rpc_server_stdout: Lines<BufReader<ChildStdout>>,
            socat: SocatProcess,
        ) -> Self {
            Self {
                rpc_server,
                rpc_server_stdout,
                socat,
            }
        }

        /// Blocking call to get the next line of stdout from the RPC server process.
        pub fn get_rpc_server_stdout_line(&mut self) -> String {
            self.rpc_server_stdout
                .next()
                .expect("Failed to read stdout")
                .expect("Failed to read stdout")
        }

        fn kill(&mut self) {
            self.rpc_server.kill().ok();
            self.socat.kill().ok();
        }

        pub fn get_rpc_server(&mut self) -> &mut ZephyrServerProcess {
            &mut self.rpc_server
        }

        pub fn get_socat(&mut self) -> &mut SocatProcess {
            &mut self.socat
        }
    }

    impl Drop for TestProcesses {
        fn drop(&mut self) {
            self.kill();
        }
    }
}

use TestProcessInfra::TestProcesses;

/// Run the Zephyr RPC server script that launches the Babble Simulator
/// and runs the RPC Server app.
///
/// This outputs verbose output we capture and will process later to determine
/// if the client/server are working properly.
fn run_zephyr_rpc_server_exe() -> TestProcesses {
    use std::process::{Command, Stdio};

    let mut rpc_server = Command::new("sh")
        .current_dir(TEST_DIRECTORY_PATH) // Set working directory
        .arg(ZEPHY_RPC_SERVER_RUN_SCRIPT)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("Failed to start Zephyr RPC server");

    // See if process failed to start.
    if let Some(status) = rpc_server.try_wait().expect("Failed to wait on process") {
        panic!(
            "RPC server process exited immediately with status: {}",
            status
        );
    }

    // Block until see: "UART 0 (UARTE0) connected to pseudotty: /dev/pts/XX"
    let rpc_server_stdout = rpc_server.stdout.take().expect("Failed to capture stdout");
    let reader = BufReader::new(rpc_server_stdout);
    let mut lines = reader.lines();
    let interface = loop {
        let line = lines.next().expect("Failed to read stdout");
        if let Ok(line) = line {
            if line.contains("UART 0 (UARTE0) connected to pseudotty") {
                // Extract the interface name from the line
                if let Some(start) = line.find("/dev/pts/") {
                    let interface = line[start..].trim().to_string();
                    println!("Found interface: {}", interface);
                    break interface;
                }
            }
        } else {
            panic!("EOF or error before finding interface");
        }
    };

    let socat = create_socat_socket(&interface, "/tmp/nrf_rpc_socket");
    TestProcesses::new(rpc_server, lines, socat)
}

fn create_socat_socket(pty_port: &str, socket_path: &str) -> std::process::Child {
    use std::process::Command;
    Command::new("socat")
        .arg(format!("UNIX-LISTEN:{},fork", socket_path))
        .arg(format!("PTY,link={},raw,echo=0", pty_port))
        .spawn()
        .expect("Failed to start socat")
}

#[test]
/// Basic functionality test to launch server. No client interactions for this test.
fn test_zephyr_rpc_server() {
    println!("Starting Zephyr RPC server test to test that the server launches properly.");

    const TEST_DURATION: u64 = 10; // seconds

    let mut processes = run_zephyr_rpc_server_exe();
    let expected = [
        "<inf> nrf_ps_server: Initializing RPC server",
        "<dbg> NRF_RPC: Done initializing nRF RPC module",
        "<inf> nrf_ps_server: RPC server ready",
    ];

    let start = Instant::now();
    let mut expected_index = 0;

    // (todo) test might hang if server doesn't have an output since
    // reader is blocking.
    while Instant::now() - start < Duration::from_secs(TEST_DURATION) {
        if expected_index >= expected.len() {
            println!("Found all expected outputs!");
            return; // Test passed
        }

        // Read from stdout and see if we find any of the expected outputs.
        let line = processes.get_rpc_server_stdout_line();
        if expected_index < expected.len() && line.contains(expected[expected_index]) {
            expected_index += 1;
        }
    }

    panic!(
        "Found {}/{} expected outputs",
        expected_index,
        expected.len()
    );
}

/*
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

        assert_eq!(
            packets[0], expected_packet,
            "bt_enable packet mismatch\nExpected: {:02X?}\nGot:      {:02X?}",
            expected_packet, packets[0]
        );
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
             01 01 01 41 06 01 09 09 49 4E 6F 72 64 69 63 5F 50 53 F6",
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
            options: 0x03, // BT_LE_ADV_OPT_CONNECTABLE | connectable-something
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

        assert_eq!(
            packets[0], expected_packet,
            "bt_le_adv_start packet mismatch\nExpected: {:02X?}\nGot:      {:02X?}",
            expected_packet, packets[0]
        );
    });
}
*/
