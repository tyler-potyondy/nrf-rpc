# nrf-rpc

Rust implementation of the NRF RPC protocol client.

This crate enables Rust applications to communicate with other Nordic processors
using the RPC protocol. Currently, the `nrf-rpc` crate provides RPC encodings for 
the Zephyr Bluetooth RPC function calls.

## Usage

### Basic Example

```rust
use nrf_rpc::ble::{Ble, BtLeAdvParam, BtData, BT_LE_AD_GENERAL, BT_LE_AD_NO_BREDR};

let transport = MyTransport::new();
let mut ble = Ble::new(transport).await?;

// Enable Bluetooth (executes on remote device)
ble.bt_enable().await?;

// Start advertising
let param = BtLeAdvParam::connectable();
let ad = [BtData::flags(&[BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR])];
let sd = [BtData::name_complete(b"MyDevice")];
ble.bt_le_adv_start(&param, &ad, &sd).await?;
```

## BLE Module

The `Ble` struct provides methods that execute Bluetooth commands on the remote device:

- `bt_enable()` - Enable Bluetooth subsystem
- `bt_le_adv_start()` - Start BLE advertising
- More commands coming soon...

## License

This project is licensed under the MIT License OR Apache License 2.0.