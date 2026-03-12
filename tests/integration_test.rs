//! Integration tests intended to evaluate the `nrf-rpc` crate's client implementation
//! against the zephyr `sample/nrf_rpc/protocol_serialization/server`.
//!
//! This test is fully run on the host without any hardware. To do this, we use
//! nordic's Babble Simulator. The testing infastructure will build the zephyr sample,
//! launch the Babble Simulator, and then bind the server's pseudo port used for uart
//! rx/tx to a unix socket. The test here then provides a mock transport layer that
//! directs client writes to this socket and polls for responses on the socket.

use nrf_rpc::{AsyncTransport, RpcClient, TransportError, ble::Ble, uart_transport::Uart};
use serial_test::serial;
use std::collections::HashSet;
use std::io::{BufRead, Read, Write};
use std::os::unix::net::UnixStream;
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
    fn new() -> Self {
        let socat_socket_path = SOCAT_SOCKET_PATH.to_string();
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
                        // println!(
                        //     "MockUart RX thread: Received {} bytes from {}: {:02X?}",
                        //     n,
                        //     socat_socket_path_clone,
                        //     &buf[..n]
                        // );
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
    type TxTransportBuffer<'a, const N: usize> = nrf_rpc::uart_transport::UartTxTransport<'a, N>;
    type RxTransportBuffer<'a, const N: usize> = nrf_rpc::uart_transport::UartRxTransport<'a, N>;

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
                let mut rx = self.rx_buffer.lock().unwrap();
                if !rx.is_empty() {
                    let n = core::cmp::min(buffer.len(), rx.len());
                    buffer[..n].copy_from_slice(&rx[..n]);
                    rx.drain(0..n);

                    // Useful for debugging socket/UART rx
                    // println!(
                    //     "MockUart: Delivering {} bytes from RX buffer to client: {:02X?}",
                    //     n,
                    //     &buffer[..n]
                    // );

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
const SOCAT_SOCKET_PATH: &str = "/tmp/nrf_rpc_socket";

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
            let mut expected_index = 0;
            let mut missing_strings = search_strings.clone();
            for _ in 0..500 {
                if expected_index >= search_strings.len() {
                    println!("Found all expected outputs!");
                    return; // Test passed
                }

                let line = self.get_rpc_server_stdout_line(Duration::from_millis(5));
                if let Some(line) = line {
                    println!("{}", line);
                    for search_string in search_strings.iter() {
                        if line.contains(search_string) {
                            missing_strings.remove(search_string);
                            println!("Found expected line: {}", line);
                            expected_index += 1;
                            break;
                        }
                    }
                }
            }

            panic!(
                "{}/{} expected outputs found. Missing: {:?}",
                expected_index,
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

            // Kill the entire process group of the RPC server to ensure all its children are also killed.
            let pid = self.rpc_server.id();
            std::process::Command::new("kill")
                .args(["-9", &format!("-{}", pid)])
                .output()
                .ok();
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
fn run_zephyr_rpc_server_exe() -> (TestProcesses, MockUart) {
    use std::os::unix::process::CommandExt;
    use std::process::{Command, Stdio};

    let mut rpc_server = Command::new("setsid")
        .arg("sh")
        .current_dir(TEST_DIRECTORY_PATH) // Set working directory
        .arg(ZEPHY_RPC_SERVER_RUN_SCRIPT)
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

    let socat = create_socat_socket(&interface, SOCAT_SOCKET_PATH);
    let uart = MockUart::new();
    (TestProcesses::new(rpc_server, lines, socat), uart)
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
#[serial]
/// Basic functionality test to launch server. No client interactions for this test.
fn test_zephyr_rpc_server() {
    println!("Starting Zephyr RPC server test to test that the server launches properly.");

    const TEST_DURATION: u64 = 10; // seconds

    let (mut processes, _) = run_zephyr_rpc_server_exe();
    let expected = [
        "<inf> nrf_ps_server: Initializing RPC server",
        "<dbg> NRF_RPC: Done initializing nRF RPC module",
        "<inf> nrf_ps_server: RPC server ready",
    ];

    let start = Instant::now();
    let mut expected_index = 0;

    // Read 500 lines of stdout from the server, looking for the expected outputs.
    for _ in 0..500 {
        if expected_index >= expected.len() {
            println!("Found all expected outputs!");
            return; // Test passed
        }

        let line = processes.get_rpc_server_stdout_line(Duration::from_millis(500));
        if let Some(line) = line {
            if expected_index < expected.len() && line.contains(expected[expected_index]) {
                expected_index += 1;
            }
        }
    }

    panic!(
        "Found {}/{} expected outputs",
        expected_index,
        expected.len()
    );
}

#[test]
#[serial]
/// Test the client can send a packet and receive an ACK.
fn test_client_can_send_packet() {
    println!("Starting client can send packet test...");

    // First start the Zephyr RPC server and socat bridge so that the UNIX
    // socket exists and is listening before the MockUart attempts to connect.
    let (mut processes, mut uart) = run_zephyr_rpc_server_exe();

    let _client = client_test_helper(uart);
    processes.search_stdout_for_strings(HashSet::from(["<dbg> nrf_rpc_uart: >>> RX packet"]));
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
#[serial]
fn test_client_group_handshake() {
    println!("Starting client group handshake test...");

    let (mut processes, mut uart) = run_zephyr_rpc_server_exe();
    let _client = client_test_helper(uart);
    processes.search_stdout_for_strings(HashSet::from([
        "NRF_RPC: Found corresponding local group. Remote id: 0, Local id: 0",
        "NRF_RPC: Found corresponding local group. Remote id: 1, Local id: 1",
    ]));
}

#[test]
#[serial]
fn test_bt_enable_initializes_bluetooth() {
    println!("Starting bt_enable integration test...");

    let (mut processes, mut uart) = run_zephyr_rpc_server_exe();

    // Create BLE client over the same UART transport used by other tests.
    let mut ble = block_on(Ble::new(uart)).expect("Failed to initialize BLE client");

    // Call bt_enable and expect it to succeed end-to-end against the Zephyr server.
    block_on(ble.bt_enable()).expect("bt_enable RPC failed");

    // Verify that the Zephyr side reports Bluetooth initialization and settings load.
    processes.search_stdout_for_strings(HashSet::from([
        "bt_hci_core: HW Platform: Nordic Semiconductor",
    ]));
}

#[test]
#[serial]
fn test_bt_begin_advertising() {
    println!("Starting bt_begin_advertising integration test...");

    let (mut processes, mut uart) = run_zephyr_rpc_server_exe();

    // Create BLE client over the same UART transport used by other tests.
    let mut ble = block_on(Ble::new(uart)).expect("Failed to initialize BLE client");

    // Call bt_enable and expect it to succeed end-to-end against the Zephyr server.
    block_on(ble.bt_enable()).expect("bt_enable RPC failed");

    block_on(ble.bt_le_adv_start()).expect("bt_le_adv_start RPC failed");

    processes.search_stdout_for_strings(HashSet::from([
        "bt_hci_core: HW Platform: Nordic Semiconductor",
        "JINordic",
    ]));
}

fn client_test_helper(uart: MockUart) -> RpcClient<MockUart> {
    std::thread::sleep(Duration::from_secs(1));
    let mut client: RpcClient<MockUart> = RpcClient::new(uart);
    block_on(client.init()).expect("Failed to initialize client");

    client
}
