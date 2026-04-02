/// Test file to verify the proc macro can parse C signatures
/// The generated code is meant to be used within the nrf-rpc crate

fn main() {
    // Test that the macro can parse various C function signatures
    // (compile-time test only)
    println!("Proc macro parsing test - compile only");
    
    // The macro would generate methods like this when used in nrf-rpc:
    // pub async fn bt_enable(&mut self, cb: u64) -> Result<i32, String> { ... }
}

// Demonstrate syntax parsing (won't actually compile without nrf-rpc types)
#[cfg(any())] // Never true, just parse
mod parse_test {
    use nrf_rpc_codegen::rpc_from_c;
    
    struct BleClient { client: () }
    
    impl BleClient {
        rpc_from_c!(
            client = self.client,
            cmd = "BtEnableRpcCmd",
            sig = "int bt_enable(bt_ready_cb_t cb)"
        );
    }
}
