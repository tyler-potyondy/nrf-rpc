//! Bluetooth Low Energy (BLE) RPC commands
//!
//! This module provides BLE functions that execute on a remote NRF device via RPC.
//! The API mirrors the Zephyr Bluetooth API.
//!
//! # Usage
//! ```ignore
//! use nrf_rpc::ble::{Ble, BtLeAdvParam, BtData, BT_LE_AD_GENERAL, BT_LE_AD_NO_BREDR};
//!
//! // Create BLE client - automatically initializes RPC connection
//! let mut ble = Ble::new(transport).await?;
//!
//! // Enable Bluetooth
//! ble.bt_enable().await?;
//!
//! // Start advertising
//! let param = BtLeAdvParam::connectable();
//! let ad = [BtData::flags(&[BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR])];
//! let sd = [BtData::name_complete(b"MyDevice")];
//! ble.bt_le_adv_start(&param, &ad, &sd).await?;
//! ```

use crate::packet::{CborError, PacketBuilder};
use crate::{AsyncTransport, RpcClient, RpcError};

// ============================================================================
// Ble Struct
// ============================================================================

/// BLE RPC client
///
/// Encapsulates an RPC client for Bluetooth Low Energy operations.
pub struct Ble<T: AsyncTransport> {
    client: RpcClient<T>,
}

impl<T: AsyncTransport> Ble<T> {
    /// Create a new BLE client and initialize the RPC connection
    ///
    /// This constructor is async and will block until the RPC handshake completes.
    ///
    /// # Example
    /// ```ignore
    /// let mut ble = Ble::new(transport).await?;
    /// ```
    pub async fn new(transport: T) -> Result<Self, RpcError> {
        let mut client = RpcClient::new(transport);
        client.init().await?;
        Ok(Self { client })
    }

    /// Enable Bluetooth (TODO) add zephyr doc comments HERE
    ///
    /// # Example
    /// ```ignore
    /// ble.bt_enable().await?;
    /// ```
    pub async fn bt_enable(&mut self) -> Result<i32, RpcError> {
        let packet = PacketBuilder::<64>::new()
            .command(
                self.client.context_id(),
                BT_ENABLE_RPC_CMD,
                0xFF,
                self.client.bt_rpc_group_id(),
                self.client.bt_rpc_group_id(),
            )
            .cbor_uint(28)?
            .cbor_uint(28)?
            .cbor_null()?;

        self.client.send_command(packet.as_slice()).await
    }

    /// Start BLE advertising
    ///
    /// # Example
    /// ```ignore
    /// let param = BtLeAdvParam::connectable();
    /// let ad = [BtData::flags(&[BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR])];
    /// let sd = [BtData::name_complete(b"MyDevice")];
    /// ble.bt_le_adv_start(&param, &ad, &sd).await?;
    /// ```
    pub async fn bt_le_adv_start<'a>(
        &mut self,
        param: &BtLeAdvParam,
        ad: &[BtData<'a>],
        sd: &[BtData<'a>],
    ) -> Result<i32, RpcError> {
        let packet = encode_bt_le_adv_start::<256>(
            self.client.context_id(),
            self.client.bt_rpc_group_id(),
            self.client.bt_rpc_group_id(),
            param,
            ad,
            sd,
        )?;

        self.client.send_command(packet.as_slice()).await
    }
}

// ============================================================================
// Constants
// ============================================================================

/// BLE advertising options
pub const BT_LE_ADV_OPT_CONNECTABLE: u32 = 0x00000001;
pub const BT_LE_ADV_OPT_ONE_TIME: u32 = 0x00000002;

/// BLE advertising data types (from Zephyr bluetooth.h)
pub const BT_DATA_FLAGS: u8 = 0x01;
pub const BT_DATA_NAME_COMPLETE: u8 = 0x09;

/// BLE advertising flags
pub const BT_LE_AD_GENERAL: u8 = 0x02;
pub const BT_LE_AD_NO_BREDR: u8 = 0x04;

// ============================================================================
// Data Structures
// ============================================================================

/// BLE advertising parameters
///
/// Corresponds to `bt_le_adv_param` struct in Zephyr
#[derive(Debug, Clone, Copy)]
pub struct BtLeAdvParam {
    pub id: u8,
    pub sid: u8,
    pub secondary_max_skip: u8,
    pub options: u32,
    pub interval_min: u32,
    pub interval_max: u32,
    pub peer: Option<BtAddrLe>,
}

impl BtLeAdvParam {
    /// Create default connectable advertising parameters
    pub fn connectable() -> Self {
        Self {
            id: 0,
            sid: 0,
            secondary_max_skip: 0,
            options: BT_LE_ADV_OPT_CONNECTABLE,
            interval_min: 160, // 100ms in 0.625ms units
            interval_max: 240, // 150ms in 0.625ms units
            peer: None,
        }
    }
}

/// BLE advertising data
///
/// Corresponds to `bt_data` struct in Zephyr
#[derive(Debug, Clone)]
pub struct BtData<'a> {
    pub data_type: u8,
    pub data: &'a [u8],
}

impl<'a> BtData<'a> {
    /// Create BLE flags advertising data
    pub fn flags(flags: &'a [u8]) -> Self {
        Self {
            data_type: BT_DATA_FLAGS,
            data: flags,
        }
    }

    /// Create complete local name advertising data
    pub fn name_complete(name: &'a [u8]) -> Self {
        Self {
            data_type: BT_DATA_NAME_COMPLETE,
            data: name,
        }
    }
}

/// BLE address with type
#[derive(Debug, Clone, Copy)]
pub struct BtAddrLe {
    pub addr_type: u8,
    pub addr: [u8; 6],
}

// ============================================================================
// Command IDs
// ============================================================================

const BT_ENABLE_RPC_CMD: u8 = 0x00;
const BT_LE_ADV_START_RPC_CMD: u8 = 0x04;

// ============================================================================
// Internal Encoding Functions
// ============================================================================

/// Encode bt_le_adv_start command
///
/// This is exposed for testing purposes.
#[doc(hidden)]
pub fn encode_bt_le_adv_start<const N: usize>(
    src_ctx_id: u8,
    src_grp_id: u8,
    dst_grp_id: u8,
    param: &BtLeAdvParam,
    ad: &[BtData],
    sd: &[BtData],
) -> Result<PacketBuilder<N>, CborError> {
    let scratchpad_size = calculate_scratchpad_size(param, ad, sd);

    let mut builder = PacketBuilder::<N>::new()
        .command(
            src_ctx_id,
            BT_LE_ADV_START_RPC_CMD,
            0xFF,
            src_grp_id,
            dst_grp_id,
        )
        .cbor_uint(scratchpad_size as u64)?;

    // Encode bt_le_adv_param
    builder = builder
        .cbor_uint(param.id as u64)?
        .cbor_uint(param.sid as u64)?
        .cbor_uint(param.secondary_max_skip as u64)?
        .cbor_uint(param.options as u64)?
        .cbor_uint(param.interval_min as u64)?
        .cbor_uint(param.interval_max as u64)?;

    // Encode peer address (null if None)
    builder = builder.cbor_null()?;

    // Encode advertising data array
    builder = builder.cbor_uint(ad.len() as u64)?;
    for ad_item in ad {
        builder = encode_bt_data(builder, ad_item)?;
    }

    // Encode scan response data array
    builder = builder.cbor_uint(sd.len() as u64)?;
    for sd_item in sd {
        builder = encode_bt_data(builder, sd_item)?;
    }

    // Terminator
    builder = builder.cbor_null()?;

    Ok(builder)
}

/// Encode a single bt_data structure
fn encode_bt_data<const N: usize>(
    mut builder: PacketBuilder<N>,
    data: &BtData,
) -> Result<PacketBuilder<N>, CborError> {
    builder = builder
        .cbor_uint(data.data_type as u64)?
        .cbor_uint(data.data.len() as u64)?
        .cbor_bytes(data.data)?;
    Ok(builder)
}

/// Calculate scratchpad size for bt_le_adv_start
///
/// Based on C implementation in bt_rpc_gap_client.c:
/// - For each bt_data: NRF_RPC_SCRATCHPAD_ALIGN(sizeof(struct bt_data)) + NRF_RPC_SCRATCHPAD_ALIGN(data_len)
/// - bt_le_adv_param_sp_size(param) which is 0 if peer is None
fn calculate_scratchpad_size(param: &BtLeAdvParam, ad: &[BtData], sd: &[BtData]) -> usize {
    const BT_DATA_SIZE: usize = 8; // sizeof(struct bt_data) in C
    let mut size = 0;

    // bt_data structures for ad
    for ad_item in ad {
        size += align_to_4(BT_DATA_SIZE); // sizeof(struct bt_data)
        size += align_to_4(ad_item.data.len()); // actual data
    }

    // bt_data structures for sd
    for sd_item in sd {
        size += align_to_4(BT_DATA_SIZE); // sizeof(struct bt_data)
        size += align_to_4(sd_item.data.len()); // actual data
    }

    // peer address if present
    if param.peer.is_some() {
        size += align_to_4(7); // sizeof(bt_addr_le_t)
    }

    size
}

/// Align size to 4-byte boundary (required by NRF RPC scratchpad)
fn align_to_4(size: usize) -> usize {
    (size + 3) & !3
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bt_le_adv_start_encoding() {
        // Test case from raw_rpc trace
        let param = BtLeAdvParam {
            id: 0,
            sid: 0,
            secondary_max_skip: 0,
            options: 0x03,
            interval_min: 160,
            interval_max: 240,
            peer: None,
        };

        let ad_data = [BtData {
            data_type: BT_DATA_FLAGS,
            data: &[BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR],
        }];

        let sd_data = [BtData {
            data_type: BT_DATA_NAME_COMPLETE,
            data: b"Nordic_PS",
        }];

        let packet =
            encode_bt_le_adv_start::<256>(0x00, 0x00, 0x00, &param, &ad_data, &sd_data).unwrap();

        let expected = &[
            0x80, 0x04, 0xFF, 0x00, 0x00, 0x18, 0x20, 0x00, 0x00, 0x00, 0x03, 0x18, 0xA0, 0x18,
            0xF0, 0xF6, 0x01, 0x01, 0x01, 0x41, 0x06, 0x01, 0x09, 0x09, 0x49, 0x4E, 0x6F, 0x72,
            0x64, 0x69, 0x63, 0x5F, 0x50, 0x53, 0xF6,
        ];

        assert_eq!(packet.as_slice(), expected);
    }
}
