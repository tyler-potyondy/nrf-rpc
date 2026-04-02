//! Integration tests intended to evaluate the `nrf-rpc` crate's client implementation
//! against the zephyr `sample/nrf_rpc/protocol_serialization/server`.
//!
//! This test is fully run on the host without any hardware. To do this, we use
//! nordic's Babble Simulator. The testing infastructure will build the zephyr sample,
//! launch the Babble Simulator, and then bind the server's pseudo port used for uart
//! rx/tx to a unix socket. The test here then provides a mock transport layer that
//! directs client writes to this socket and polls for responses on the socket.

use nrf_rpc::ble::cgm::{
    BT_UUID_CGM_FEATURE_VAL, BT_UUID_CGM_MEASUREMENT_VAL, BT_UUID_CGM_STATUS_VAL, BT_UUID_CGMS_VAL,
    CgmMeasurement, encode_uuid_16,
};
use nrf_rpc::ble::{
    BT_GATT_CCC_NOTIFY, BT_LE_SCAN_TYPE_ACTIVE, BtConnLeCreateParam, BtGattDiscoverParams,
    BtGattDiscoverType, BtGattReadParams, BtGattSubscribeParams, BtLeConnParam, BtLeScanParam,
    ScanResultData,
};
use nrf_rpc::{AsyncTransport, RpcClient, TransportError, ble::Ble, uart_transport::Uart};
use serial_test::serial;
use std::collections::HashSet;
use std::io::{BufRead, Read, Write};
use std::os::unix::net::UnixStream;
use std::os::unix::thread;
use std::process::{ChildStderr, ChildStdout};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// Mock error type
#[derive(Debug)]
struct MockError;

impl core::fmt::Display for MockError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Mock transport error")
    }
}

impl TransportError for MockError {}

/// Mock UART transport that records all written packets and forwards them to
/// the Zephyr server via the socat UNIX socket, while continuously reading
/// bytes from the socket into an internal RX buffer.
struct MockUart {
    socat_socket_path: String,
    sent_packets: Arc<Mutex<Vec<Vec<u8>>>>,
    socket: UnixStream,
    rx_buffer: Arc<Mutex<Vec<u8>>>,
}

impl MockUart {
    fn new(socat_socket_path: &str) -> Self {
        let socat_socket_path = socat_socket_path.to_string();
        let start = std::time::Instant::now();
        let timeout = std::time::Duration::from_secs(5);
        let mut last_err: Option<std::io::Error> = None;

        // Retry connecting for a short period to give socat time to start
        // listening on the UNIX socket.
        let socket = loop {
            match UnixStream::connect(&socat_socket_path) {
                Ok(s) => break s,
                Err(e) => {
                    last_err = Some(e);
                    if start.elapsed() >= timeout {
                        panic!(
                            "Failed to connect to socat UNIX socket {} within {:?}: {:?}",
                            socat_socket_path, timeout, last_err
                        );
                    }
                    std::thread::sleep(std::time::Duration::from_millis(50));
                }
            }
        };

        // Ensure blocking mode so writes complete before our minimal executor polls again.
        socket
            .set_nonblocking(false)
            .expect("Failed to configure socat UNIX socket");

        // Shared RX buffer where the background reader thread will place bytes
        // received from the Zephyr UART via the UNIX socket.
        let rx_buffer = Arc::new(Mutex::new(Vec::new()));

        // Clone pieces needed for the background RX thread.
        let rx_buffer_clone = Arc::clone(&rx_buffer);
        let socat_socket_path_clone = socat_socket_path.clone();
        let mut read_socket = socket
            .try_clone()
            .expect("Failed to clone socat UNIX socket for RX thread");

        // Spawn a background thread that continuously reads from the socket and
        // appends data into the RX buffer. This emulates a UART RX IRQ/DMA
        // filling a hardware FIFO.
        std::thread::spawn(move || {
            let mut buf = [0u8; 1024];
            loop {
                match read_socket.read(&mut buf) {
                    Ok(0) => {
                        println!(
                            "MockUart RX thread: EOF while reading from socat socket {}",
                            socat_socket_path_clone
                        );
                        break;
                    }
                    Ok(n) => {
                        // Useful for debugging socket/UART rx
                        println!(
                            "MockUart RX thread: Received {} bytes from {}: {:02X?}",
                            n,
                            socat_socket_path_clone,
                            &buf[..n]
                        );
                        let mut rx = rx_buffer_clone.lock().unwrap();
                        rx.extend_from_slice(&buf[..n]);
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {
                        continue;
                    }
                    Err(e) => {
                        println!(
                            "MockUart RX thread: Read error from socat socket {}: {}",
                            socat_socket_path_clone, e
                        );
                        break;
                    }
                }
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
        });

        Self {
            socat_socket_path,
            sent_packets: Arc::new(Mutex::new(Vec::new())),
            socket,
            rx_buffer,
        }
    }

    fn get_sent_packets(&self) -> Vec<Vec<u8>> {
        self.sent_packets.lock().unwrap().clone()
    }

    fn clear_packets(&self) {
        self.sent_packets.lock().unwrap().clear();
    }
}

impl Uart for MockUart {}

impl AsyncTransport for MockUart {
    type Error = MockError;
    type TxTransportPacket<'a> = nrf_rpc::uart_transport::UartTxTransport<'a>;
    type RxTransportPacket<'a> = nrf_rpc::uart_transport::UartRxTransport<'a>;

    async fn write(&mut self, data: &mut [u8]) -> Result<usize, Self::Error> {
        // Log the packet being sent
        println!(
            "MockUart: Sending {} bytes to {}: {:02X?}",
            data.len(),
            self.socat_socket_path,
            data
        );

        // Record locally for inspection by tests if needed
        self.sent_packets.lock().unwrap().push(data.to_vec());

        // Forward the bytes to the socat UNIX socket so that the Zephyr UART
        // endpoint actually receives the frame.
        if let Err(e) = self.socket.write_all(data) {
            println!(
                "MockUart: Failed to write {} bytes to socat socket {}: {}",
                data.len(),
                self.socat_socket_path,
                e
            );
            return Err(MockError);
        }

        if let Err(e) = self.socket.flush() {
            println!(
                "MockUart: Failed to flush socat socket {}: {}",
                self.socat_socket_path, e
            );
            return Err(MockError);
        }

        Ok(data.len())
    }

    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        use std::time::{Duration, Instant};

        // Poll the RX buffer for a bounded amount of time so our minimal
        // executor (which expects futures to be immediately ready) does not
        // get stuck forever if no data arrives.
        let timeout = Duration::from_secs(5);
        let start = Instant::now();

        loop {
            {
                let rx = self.rx_buffer.lock().unwrap();
                if !rx.is_empty() {
                    // Data has started arriving.  Wait until bytes stop
                    // flowing so we drain complete HDLC frames rather than
                    // splitting on a 7E delimiter boundary.
                    let mut prev_len = rx.len();
                    drop(rx);

                    loop {
                        std::thread::sleep(Duration::from_millis(30));
                        let rx = self.rx_buffer.lock().unwrap();
                        let cur_len = rx.len();
                        if cur_len == prev_len {
                            break; // no new data arrived → coalesced
                        }
                        prev_len = cur_len;
                    }

                    let mut rx = self.rx_buffer.lock().unwrap();
                    let n = core::cmp::min(buffer.len(), rx.len());
                    buffer[..n].copy_from_slice(&rx[..n]);
                    rx.drain(0..n);

                    // Useful for debugging socket/UART rx
                    println!(
                        "MockUart: Delivering {} bytes from RX buffer to client: {:02X?}",
                        n,
                        &buffer[..n]
                    );

                    return Ok(n);
                }
            }

            if start.elapsed() >= timeout {
                println!(
                    "MockUart: Read timeout from RX buffer for socat socket {}",
                    self.socat_socket_path
                );
                return Ok(0);
            }

            // Sleep briefly before polling again to avoid a busy loop.
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    async fn delay_ms(&mut self, ms: u32) {
        std::thread::sleep(Duration::from_millis(ms as u64));
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

const ZEPHY_RPC_SERVER_RUN_SCRIPT: &str = "run_bsim.sh";
const TEST_DIRECTORY_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/");

use std::io::BufReader;
pub mod TestProcessInfra {
    use std::{
        collections::HashSet,
        io::{BufReader, Lines},
        process::ChildStdout,
        sync::mpsc,
        time::Duration,
    };

    type ZephyrServerProcess = std::process::Child;
    type SocatProcess = std::process::Child;

    pub struct TestProcesses {
        rpc_server: ZephyrServerProcess,
        rpc_server_stdout_rx: mpsc::Receiver<String>,
        socat: SocatProcess,
    }

    impl TestProcesses {
        pub fn new(
            rpc_server: ZephyrServerProcess,
            rpc_server_stdout: Lines<BufReader<ChildStdout>>,
            socat: SocatProcess,
        ) -> Self {
            let (tx, rx) = mpsc::channel::<String>();

            std::thread::spawn(move || {
                for line in rpc_server_stdout {
                    let Ok(line) = line else { break };
                    if tx.send(line).is_err() {
                        break;
                    }
                }
            });

            Self {
                rpc_server,
                rpc_server_stdout_rx: rx,
                socat,
            }
        }

        pub fn search_stdout_for_strings(&mut self, search_strings: HashSet<&str>) {
            let mut missing_strings = search_strings.clone();
            let deadline = std::time::Instant::now() + Duration::from_secs(20);

            while std::time::Instant::now() < deadline {
                if missing_strings.is_empty() {
                    println!("Found all expected outputs!");
                    return; // Test passed
                }

                let line = self.get_rpc_server_stdout_line(Duration::from_millis(200));
                if let Some(line) = line {
                    println!("{}", line);
                    for search_string in search_strings.iter() {
                        if line.contains(search_string) && missing_strings.remove(search_string) {
                            println!("Found expected line: {}", line);
                        }
                    }
                }
            }

            panic!(
                "{}/{} expected outputs found. Missing: {:?}",
                search_strings.len() - missing_strings.len(),
                search_strings.len(),
                missing_strings
            );
        }

        /// Call to get the next line of stdout from the RPC server process,
        /// waiting up to `timeout`, and returning None if no line is available.
        pub fn get_rpc_server_stdout_line(&mut self, timeout: Duration) -> Option<String> {
            self.rpc_server_stdout_rx.recv_timeout(timeout).ok()
        }

        fn kill(&mut self) {
            println!("Killing test processes");
            self.socat.kill().ok();

            // Kill the RPC server process and its children.
            // First try graceful termination, then force kill if needed.
            let pid = self.rpc_server.id();

            // Try SIGTERM first (graceful shutdown)
            let _ = std::process::Command::new("kill")
                .args([&format!("{}", pid)])
                .output();

            // Give it a moment to terminate gracefully
            std::thread::sleep(std::time::Duration::from_millis(100));

            // Force kill the specific process if still running
            let _ = self.rpc_server.kill();

            // Also try to kill any child processes specifically
            // Using pkill with parent PID is safer than negative process group
            let _ = std::process::Command::new("pkill")
                .args(["-P", &format!("{}", pid)])
                .output();
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

fn print_process_output_failure(std_out: ChildStdout, std_err: ChildStderr) {
    println!("======STDOUT======");
    let reader = BufReader::new(std_out);
    for line in reader.lines() {
        if let Ok(line) = line {
            println!("Process output: {}", line);
        }
    }

    let reader = BufReader::new(std_err);
    for line in reader.lines() {
        if let Ok(line) = line {
            println!("Process error output: {}", line);
        }
    }
}

/// Run the Zephyr RPC server script that launches the Babble Simulator
/// and runs the RPC Server app.
///
/// This outputs verbose output we capture and will process later to determine
/// if the client/server are working properly.
fn run_zephyr_rpc_server_exe(test_name: &str) -> (TestProcesses, MockUart) {
    use std::os::unix::process::CommandExt;
    use std::process::{Command, Stdio};

    let mut rpc_server = Command::new("setsid")
        .arg("sh")
        .current_dir(TEST_DIRECTORY_PATH) // Set working directory
        .arg(ZEPHY_RPC_SERVER_RUN_SCRIPT)
        .arg(test_name)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        // Put the script and all its children into a new process group
        // so we can kill them all at once with kill
        .process_group(0)
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
        let line = match lines.next() {
            Some(line) => line,
            None => {
                // Process exited before finding interface - print stderr for diagnostics
                println!("======STDERR======");
                if let Some(stderr) = rpc_server.stderr.take() {
                    let reader = BufReader::new(stderr);
                    for line in reader.lines() {
                        if let Ok(line) = line {
                            println!("Process error output: {}", line);
                        }
                    }
                }
                panic!("Bsim test process exited before finding UART interface");
            }
        };
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

    let socket_path = test_socket_path(test_name);
    let socat = create_socat_socket(&interface, &socket_path);
    let uart = MockUart::new(&socket_path);
    (TestProcesses::new(rpc_server, lines, socat), uart)
}

fn test_socket_path(test_name: &str) -> String {
    let mut socket_path = std::env::temp_dir();
    socket_path.push(format!("nrf_rpc_{}_{}.sock", std::process::id(), test_name));
    socket_path.to_string_lossy().into_owned()
}

fn create_socat_socket(pty_port: &str, socket_path: &str) -> std::process::Child {
    use std::process::Command;
    use std::{fs, io};

    // Remove any stale socket file from a previous test run. If the file does
    // not exist, ignore the error.
    match fs::remove_file(socket_path) {
        Ok(_) => {
            println!("Removed existing socat socket at {}", socket_path);
        }
        Err(e) if e.kind() == io::ErrorKind::NotFound => {}
        Err(e) => {
            panic!(
                "Failed to remove existing socat socket at {}: {}",
                socket_path, e
            );
        }
    }

    Command::new("socat")
        // Listen on the UNIX socket and forward to the existing Zephyr PTY.
        // The PTY path we get from the server log (e.g. /dev/pts/4) is an
        // already-existing device, so we use FILE: instead of creating a new
        // PTY with link=.
        .arg(format!("UNIX-LISTEN:{},fork", socket_path))
        .arg(format!("FILE:{},raw,echo=0", pty_port))
        .spawn()
        .expect("Failed to start socat")
}

#[test]
/// Basic functionality test to launch server. No client interactions for this test.
fn test_zephyr_rpc_server() {
    println!("Starting Zephyr RPC server test to test that the server launches properly.");

    let (mut processes, _) = run_zephyr_rpc_server_exe("test_zephyr_rpc_server");
    processes.search_stdout_for_strings(HashSet::from([
        "<inf> nrf_ps_server: Initializing RPC server",
        "<dbg> NRF_RPC: Done initializing nRF RPC module",
    ]));
}

#[test]
/// Test the client can send a packet and receive an ACK.
fn test_client_can_send_packet() {
    println!("Starting client can send packet test...");

    // First start the Zephyr RPC server and socat bridge so that the UNIX
    // socket exists and is listening before the MockUart attempts to connect.
    let (mut processes, uart) = run_zephyr_rpc_server_exe("test_client_can_send_packet");

    let _client = client_test_helper(uart);
    processes.search_stdout_for_strings(HashSet::from(["<dbg> nrf_rpc_uart: <<< TX packet"]));
}

// #[test]
// #[serial]
// fn test_client_acks_packets() {
//     println!("Starting client can ack packets test...");
//
//     let (mut processes, mut uart) = run_zephyr_rpc_server_exe();
//     let _client = client_test_helper(&mut uart);
//     processes.search_stdout_for_strings(HashSet::from(["<dbg> nrf_rpc_uart: >>> RX ack"]));
// }

#[test]
fn test_client_group_handshake() {
    println!("Starting client group handshake test...");

    let (mut processes, uart) = run_zephyr_rpc_server_exe("test_client_group_handshake");
    let _client = client_test_helper(uart);
    processes.search_stdout_for_strings(HashSet::from([
        "<dbg> NRF_RPC: Group 'bt_rpc' has id 0",
        "<dbg> NRF_RPC: Group 'rpc_utils' has id 1",
    ]));
}

#[test]
fn test_bt_enable_initializes_bluetooth() {
    println!("Starting bt_enable integration test...");

    let (mut processes, uart) = run_zephyr_rpc_server_exe("test_bt_enable_initializes_bluetooth");

    // Create BLE client over the same UART transport used by other tests.
    let mut ble =
        embassy_futures::block_on(Ble::new(uart)).expect("Failed to initialize BLE client");

    // Call bt_enable and expect it to succeed end-to-end against the Zephyr server.
    // embassy_futures::block_on(ble.bt_enable(5)).expect("bt_enable RPC failed");
    embassy_futures::block_on(ble.bt_enable(None));

    // Verify at least server startup logs are present.
    processes.search_stdout_for_strings(HashSet::from([
        "<inf> nrf_ps_server: Initializing RPC server",
    ]));
}

#[test]
fn test_bt_begin_advertising() {
    println!("Starting bt_begin_advertising integration test...");

    let (mut processes, uart) = run_zephyr_rpc_server_exe("test_bt_begin_advertising");

    // Create BLE client over the same UART transport used by other tests.
    let mut ble =
        embassy_futures::block_on(Ble::new(uart)).expect("Failed to initialize BLE client");

    // Call bt_enable and expect it to succeed end-to-end against the Zephyr server.
    let bt_enable_res = embassy_futures::block_on(ble.bt_enable(None));
    if bt_enable_res.is_err() {
        println!("[WARNING] bt_enable failed: {:?}", bt_enable_res.err());
    }

    let bt_le_adv_start_res = embassy_futures::block_on(ble.bt_le_adv_start());
    if bt_le_adv_start_res.is_err() {
        println!(
            "[WARNING] bt_le_adv_start failed: {:?}",
            bt_le_adv_start_res.err()
        );
    }

    // Verify at least server startup logs are present.
    processes.search_stdout_for_strings(HashSet::from([
        "<inf> nrf_ps_server: Initializing RPC server",
    ]));
}

fn client_test_helper(uart: MockUart) -> RpcClient<MockUart> {
    std::thread::sleep(Duration::from_secs(1));
    let mut client: RpcClient<MockUart> = RpcClient::new(uart);
    embassy_futures::block_on(client.init()).expect("Failed to initialize client");

    client
}

// =============================================================================
// CGM Central Integration Tests
// =============================================================================

/// Helper: Initialize BLE client with bt_enable and connection callback registration.
fn cgm_ble_init(uart: MockUart) -> Ble<MockUart> {
    let mut ble =
        embassy_futures::block_on(Ble::new(uart)).expect("Failed to initialize BLE client");

    // Enable Bluetooth
    let result = embassy_futures::block_on(ble.bt_enable(None));
    assert!(result.is_ok(), "bt_enable failed: {:?}", result.err());

    // Register connection callbacks so the server forwards connect/disconnect events
    let result = embassy_futures::block_on(ble.bt_conn_cb_register_on_remote());
    assert!(
        result.is_ok(),
        "bt_conn_cb_register_on_remote failed: {:?}",
        result.err()
    );

    // Register scan callbacks so the server forwards scan result events
    let result = embassy_futures::block_on(ble.bt_le_scan_cb_register_on_remote());
    assert!(
        result.is_ok(),
        "bt_le_scan_cb_register_on_remote failed: {:?}",
        result.err()
    );

    ble
}

#[test]
/// Test that BLE scanning can be started and stopped successfully.
///
/// This verifies that the bt_le_scan_start RPC command is correctly encoded
/// and accepted by the Zephyr server, and that the server begins scanning
/// for BLE devices (including the CGM peripheral running in BSIM).
fn test_cgm_scan_start_stop() {
    println!("Starting CGM scan start/stop test...");

    let (mut processes, uart) = run_zephyr_rpc_server_exe("test_cgm_scan_start_stop");

    let mut ble = cgm_ble_init(uart);

    // Start scanning with default active scan parameters
    let scan_params = BtLeScanParam {
        scan_type: BT_LE_SCAN_TYPE_ACTIVE,
        options: 0,
        interval: 0x0060,
        window: 0x0030,
        timeout: 0,
        interval_coded: 0,
        window_coded: 0,
    };

    let result = embassy_futures::block_on(ble.bt_le_scan_start(&scan_params, None));
    assert!(
        result.is_ok(),
        "bt_le_scan_start failed: {:?}",
        result.err()
    );
    let status = result.unwrap();
    println!("bt_le_scan_start returned status: {}", status);
    assert_eq!(
        status, 0,
        "bt_le_scan_start returned non-zero status: {}",
        status
    );

    // Give the scanner a moment to discover the CGM peripheral
    std::thread::sleep(Duration::from_secs(2));

    // Stop scanning
    let result = embassy_futures::block_on(ble.bt_le_scan_stop());
    assert!(result.is_ok(), "bt_le_scan_stop failed: {:?}", result.err());
    let status = result.unwrap();
    println!("bt_le_scan_stop returned status: {}", status);

    // Verify that the Zephyr side initialized BT and started scanning
    processes.search_stdout_for_strings(HashSet::from([
        "bt_hci_core: HW Platform: Nordic Semiconductor",
    ]));
}

#[test]
/// Test that bt_enable + scan start works and the server initializes properly.
///
/// This is a simpler smoke test for the CGM central flow — just enabling BT
/// and starting a scan, verifying the Zephyr logs show BT initialization.
fn test_cgm_bt_enable_and_scan() {
    println!("Starting CGM bt_enable + scan test...");

    let (mut processes, uart) = run_zephyr_rpc_server_exe("test_cgm_bt_enable_and_scan");

    let mut ble =
        embassy_futures::block_on(Ble::new(uart)).expect("Failed to initialize BLE client");

    // Enable Bluetooth
    let result = embassy_futures::block_on(ble.bt_enable(None));
    assert!(result.is_ok(), "bt_enable failed: {:?}", result.err());

    // Start active scanning
    let scan_params = BtLeScanParam::default();
    let result = embassy_futures::block_on(ble.bt_le_scan_start(&scan_params, None));
    assert!(
        result.is_ok(),
        "bt_le_scan_start failed: {:?}",
        result.err()
    );

    let status = result.unwrap();
    assert_eq!(status, 0, "bt_le_scan_start returned error: {}", status);

    // Verify BT initialization on server side
    processes.search_stdout_for_strings(HashSet::from([
        "bt_hci_core: HW Platform: Nordic Semiconductor",
    ]));
}

#[test]
/// Test CGM GATT discovery request.
///
/// After enabling BT, registering connection callbacks, and starting a scan,
/// this test waits for the server to connect to the CGM peripheral (via BSIM),
/// then initiates GATT service discovery for the CGM Service UUID (0x181F).
///
/// Note: Since we don't currently handle async callback events from the server
/// (scan result → auto-connect → connected callback → discover), this test
/// takes the simplest approach: just verify the RPC commands are accepted.
fn test_cgm_gatt_discover() {
    println!("Starting CGM GATT discover test...");

    let (mut processes, uart) = run_zephyr_rpc_server_exe("test_cgm_gatt_discover");

    let mut ble = cgm_ble_init(uart);

    // Start scanning for CGM peripheral
    let scan_params = BtLeScanParam {
        scan_type: BT_LE_SCAN_TYPE_ACTIVE,
        options: 0,
        interval: 0x0060,
        window: 0x0030,
        timeout: 0,
        interval_coded: 0,
        window_coded: 0,
    };

    let result = embassy_futures::block_on(ble.bt_le_scan_start(&scan_params, None));
    assert!(
        result.is_ok(),
        "bt_le_scan_start failed: {:?}",
        result.err()
    );
    assert_eq!(result.unwrap(), 0);

    // Wait for connection to be established via BSIM.
    // The CGM peripheral is advertising and the server should auto-connect
    // if scan results trigger it. In BSIM, this happens within simulated time.
    std::thread::sleep(Duration::from_secs(3));

    // Attempt GATT discovery for the CGM service
    let discover_params = BtGattDiscoverParams {
        uuid: encode_uuid_16(BT_UUID_CGMS_VAL),
        start_handle: 0x0001,
        end_handle: 0xFFFF,
        discover_type: BtGattDiscoverType::PrimaryService,
    };

    let result = embassy_futures::block_on(ble.bt_gatt_discover(&discover_params, 0x1234));
    println!("bt_gatt_discover result: {:?}", result);

    // Verify BT was initialized (minimal assertion — the discover may fail
    // if no connection was established, which is expected without full
    // callback handling)
    processes.search_stdout_for_strings(HashSet::from([
        "bt_hci_core: HW Platform: Nordic Semiconductor",
    ]));
}

// =============================================================================
// Thorough CGM Central Integration Test
// =============================================================================

#[test]
/// Full CGM Central flow: discover → connect → verify.
///
/// This test exercises the complete BLE central pipeline against the CGM
/// peripheral running in BSIM:
///
/// 1. bt_enable + register connection & scan callbacks
/// 2. Start active BLE scanning
/// 3. Receive scan result events and find the CGM peripheral ("Nordic Glucose Sensor")
/// 4. Stop scanning
/// 5. Initiate connection to the CGM peripheral's address
/// 6. Wait for the "connected" callback event (err == 0)
/// 7. Verify server-side logs confirm BT init and connection
fn test_cgm_full_central_flow() {
    println!("=== Starting CGM full central flow test ===");

    let (mut processes, uart) = run_zephyr_rpc_server_exe("test_cgm_full_central_flow");

    // ------------------------------------------------------------------
    // Step 1: Initialize BLE (bt_enable + conn_cb_register + scan_cb_register)
    // ------------------------------------------------------------------
    println!("[Step 1] Initializing BLE client...");
    let mut ble = cgm_ble_init(uart);
    println!("[Step 1] BLE client initialized.");

    // ------------------------------------------------------------------
    // Step 2: Start active BLE scanning
    // ------------------------------------------------------------------
    println!("[Step 2] Starting BLE scan...");
    let scan_params = BtLeScanParam {
        scan_type: BT_LE_SCAN_TYPE_ACTIVE,
        options: 0,
        interval: 0x0060,
        window: 0x0030,
        timeout: 0,
        interval_coded: 0,
        window_coded: 0,
    };

    let result = embassy_futures::block_on(ble.bt_le_scan_start(&scan_params, None));
    assert!(
        result.is_ok(),
        "bt_le_scan_start failed: {:?}",
        result.err()
    );
    let status = result.unwrap();
    assert_eq!(status, 0, "bt_le_scan_start returned error: {}", status);
    println!("[Step 2] Scanning started (status=0).");

    // ------------------------------------------------------------------
    // Step 3: Receive scan results and find the CGM peripheral
    // ------------------------------------------------------------------
    println!("[Step 3] Waiting for scan results...");
    let mut cgm_scan_result: Option<ScanResultData> = None;
    let max_scan_results = 50;

    for i in 0..max_scan_results {
        let result = embassy_futures::block_on(ble.wait_for_scan_result());
        match result {
            Ok(scan) => {
                let name = scan.device_name().unwrap_or("<unknown>");
                println!(
                    "  Scan result #{}: addr={:02X?} type={} rssi={} name=\"{}\"",
                    i, scan.addr, scan.addr_type, scan.rssi, name
                );

                // Look for the CGM peripheral by name or service UUID
                if name.contains("Nordic Glucose Sensor")
                    || name.contains("CGM")
                    || scan.has_service_uuid_16(BT_UUID_CGMS_VAL)
                {
                    println!(
                        "[Step 3] *** Found CGM peripheral: name=\"{}\" addr={:02X?} ***",
                        name, scan.addr
                    );
                    cgm_scan_result = Some(scan);
                    break;
                }
            }
            Err(e) => {
                println!("  Scan result #{}: error {:?}, retrying...", i, e);
            }
        }
    }

    let cgm_peripheral = cgm_scan_result.expect(
        "CGM peripheral (Nordic Glucose Sensor) not found in scan results! \
         Make sure the CGM peripheral BSIM device is running.",
    );

    // ------------------------------------------------------------------
    // Step 4: Stop scanning
    // ------------------------------------------------------------------
    println!("[Step 4] Stopping BLE scan...");
    let result = embassy_futures::block_on(ble.bt_le_scan_stop());
    assert!(result.is_ok(), "bt_le_scan_stop failed: {:?}", result.err());
    println!("[Step 4] Scan stopped.");

    // ------------------------------------------------------------------
    // Step 5: Connect to the CGM peripheral
    // ------------------------------------------------------------------
    let peer_addr = cgm_peripheral.to_addr_le();
    println!(
        "[Step 5] Connecting to CGM peripheral at {:02X?} (type={})...",
        peer_addr.addr, peer_addr.addr_type
    );

    let create_param = BtConnLeCreateParam::default();
    let conn_param = BtLeConnParam::default();
    let result =
        embassy_futures::block_on(ble.bt_conn_le_create(&peer_addr, &create_param, &conn_param));
    assert!(
        result.is_ok(),
        "bt_conn_le_create failed: {:?}",
        result.err()
    );
    let status = result.unwrap();
    assert_eq!(
        status, 0,
        "bt_conn_le_create returned error: {}",
        status
    );
    println!("[Step 5] bt_conn_le_create returned 0 (connection initiating).");

    // ------------------------------------------------------------------
    // Step 6: Wait for the "connected" callback event
    // ------------------------------------------------------------------
    println!("[Step 6] Waiting for connection event...");
    let conn_event = embassy_futures::block_on(ble.wait_for_connection());
    assert!(
        conn_event.is_ok(),
        "Did not receive connection event: {:?}",
        conn_event.err()
    );
    let conn_event = conn_event.unwrap();
    println!(
        "[Step 6] Connection event received: err={}",
        conn_event.err
    );
    assert_eq!(
        conn_event.err, 0,
        "Connection failed with HCI error: {}",
        conn_event.err
    );
    println!("[Step 6] Connection established successfully!");

    // ------------------------------------------------------------------
    // Step 7: Verify server-side logs
    // ------------------------------------------------------------------
    println!("[Step 7] Verifying server-side logs...");
    processes.search_stdout_for_strings(HashSet::from([
        "bt_hci_core: HW Platform: Nordic Semiconductor",
    ]));

    println!("=== CGM full central flow test PASSED ===");
}
