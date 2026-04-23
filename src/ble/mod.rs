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

pub mod ble_types;
pub mod bt_le_adv;
pub mod cgm;

pub use crate::ble::ble_types::{
    BT_LE_AD_GENERAL, BT_LE_AD_NO_BREDR, BtAddrLe, BtData, BtLeAdvParam,
};
pub use crate::ble::cgm::{
    BT_UUID_CGM_FEATURE_VAL, BT_UUID_CGM_MEASUREMENT_VAL, BT_UUID_CGM_STATUS_VAL, BT_UUID_CGMS_VAL,
    CgmMeasurement, SFloat, encode_uuid_16,
};
use crate::cbor_encoding::{CborError, CborPayloadBuilder};
use crate::packet::{
    self, CommandId, DestContextId, DstGroupId, NrfRpcPacket, SrcContextId, SrcGroupId,
};
use crate::{AsyncTransport, RpcClient, RpcError};

const BT_RPC_GROUP_ID: u8 = 0x0;
const RPC_UTILS_GROUP_ID: u8 = 0x1;

// ============================================================================
// Ble Struct
// ============================================================================
#[allow(dead_code)]
#[repr(u8)]
enum BleClientCommandId {
    /* bluetooth.h API */
    BtRpcGetCheckListRpcCmd,
    BtEnableRpcCmd,
    BtDisableRpcCmd,
    BtIsReadyRpcCmd,
    BtLeAdvStartRpcCmd,
    BtLeAdvStopRpcCmd,
    BtLeScanStartRpcCmd,
    BtSetNameRpcCmd,
    BtGetNameOutRpcCmd,
    BtGetAppearanceRpcCmd,
    BtSetAppearanceRpcCmd,
    BtSetIdAddrRpcCmd,
    BtIdGetRpcCmd,
    BtIdCreateRpcCmd,
    BtIdResetRpcCmd,
    BtIdDeleteRpcCmd,
    BtLeAdvUpdateDataRpcCmd,
    BtLeExtAdvCreateRpcCmd,
    BtLeExtAdvDeleteRpcCmd,
    BtLeExtAdvStartRpcCmd,
    BtLeExtAdvStopRpcCmd,
    BtLeExtAdvSetDataRpcCmd,
    BtLeExtAdvUpdateParamRpcCmd,
    BtLeExtAdvGetIndexRpcCmd,
    BtLeExtAdvGetInfoRpcCmd,
    BtLePerAdvSetParamRpcCmd,
    BtLePerAdvSetDataRpcCmd,
    BtLePerAdvStartRpcCmd,
    BtLePerAdvStopRpcCmd,
    BtLePerAdvSyncGetIndexRpcCmd,
    BtLePerAdvSyncCreateRpcCmd,
    BtLePerAdvSyncDeleteRpcCmd,
    BtLePerAdvSyncCbRegisterOnRemoteRpcCmd,
    BtLePerAdvSyncRecvEnableRpcCmd,
    BtLePerAdvSyncRecvDisableRpcCmd,
    BtLePerAdvSyncTransferRpcCmd,
    BtLePerAdvSetInfoTransferRpcCmd,
    BtLePerAdvSyncTransferSubscribeRpcCmd,
    BtLePerAdvSyncTransferUnsubscribeRpcCmd,
    BtLePerAdvListAddRpcCmd,
    BtLePerAdvListRemoveRpcCmd,
    BtLePerAdvListClearRpcCmd,
    BtLeScanStopRpcCmd,
    BtLeScanCbRegisterOnRemoteRpcCmd,
    BtLeFilterAcceptListAddRpcCmd,
    BtLeFilterAcceptListRemoveRpcCmd,
    BtLeAcceptListClearRpcCmd,
    BtLeSetChanMapRpcCmd,
    BtLeOobGetLocalRpcCmd,
    BtLeExtAdvOobGetLocalRpcCmd,
    BtUnpairRpcCmd,
    BtForeachBondRpcCmd,
    BtSettingsLoadRpcCmd,
    /* conn.h API */
    BtConnRemoteUpdateRefRpcCmd,
    BtConnGetInfoRpcCmd,
    BtConnGetRemoteInfoRpcCmd,
    BtConnLeParamUpdateRpcCmd,
    BtConnLeDataLenUpdateRpcCmd,
    BtConnLePhyUpdateRpcCmd,
    BtConnDisconnectRpcCmd,
    BtConnLeCreateRpcCmd,
    BtConnLeCreateAutoRpcCmd,
    BtConnCreateAutoStopRpcCmd,
    BtConnSetSecurityRpcCmd,
    BtConnGetSecurityRpcCmd,
    BtConnEncKeySizeRpcCmd,
    BtConnCbRegisterOnRemoteRpcCmd,
    BtConnCbUnregisterOnRemoteRpcCmd,
    BtSetBondableRpcCmd,
    BtLeOobSetLegacyFlagRpcCmd,
    BtLeOobSetScFlagRpcCmd,
    BtLeOobSetLegacyTkRpcCmd,
    BtLeOobSetScDataRpcCmd,
    BtLeOobGetScDataRpcCmd,
    BtPasskeySetRpcCmd,
    BtConnAuthCbRegisterOnRemoteRpcCmd,
    BtConnAuthInfoCbRegisterOnRemoteRpcCmd,
    BtConnAuthInfoCbUnregisterOnRemoteRpcCmd,
    BtConnAuthPasskeyEntryRpcCmd,
    BtConnAuthCancelRpcCmd,
    BtConnAuthPasskeyConfirmRpcCmd,
    BtConnAuthPairingConfirmRpcCmd,
    BtConnForeachRpcCmd,
    BtConnLookupAddrLeRpcCmd,
    BtConnGetDstOutRpcCmd,
    /* gatt.h API */
    BtRpcGattStartServiceRpcCmd,
    BtRpcGattSendSimpleAttrRpcCmd,
    BtRpcGattSendDescAttrRpcCmd,
    BtRpcGattEndServiceRpcCmd,
    BtRpcGattServiceUnregisterRpcCmd,
    BtGattNotifyCbRpcCmd,
    BtGattIndicateRpcCmd,
    BtGattIsSubscribedRpcCmd,
    BtGattGetMtuRpcCmd,
    BtGattAttrGetHandleRpcCmd,
    BtLeGattCbRegisterOnRemoteRpcCmd,
    BtGattExchangeMtuRpcCmd,
    BtGattDiscoverRpcCmd,
    BtGattReadRpcCmd,
    BtGattWriteRpcCmd,
    BtGattWriteWithoutResponseCbRpcCmd,
    BtGattSubscribeRpcCmd,
    BtGattResubscribeRpcCmd,
    BtGattUnsubscribeRpcCmd,
    BtRpcGattSubscribeFlagUpdateRpcCmd,
    /* crypto.h API */
    BtRandRpcCmd,
    BtEncryptLeRpcCmd,
    BtEncryptBeRpcCmd,
    BtCcmDecryptRpcCmd,
    BtCcmEncryptRpcCmd,
    /* internal.h API */
    BtAddrLeIsBondedCmd,
    BtHciCmdSendSyncRpcCmd,
}

/// Host commands IDs used in bluetooth API serialization.
/// These commands are sent from the host to the client.
#[allow(dead_code)]
#[repr(u8)]
enum BleHostCommandId {
    /* bluetooth.h API */
    BtLeScanCbTCallbackRpcCmd,
    BtLeExtAdvCbSentCallbackRpcCmd,
    BtLeExtAdvCbScannedCallbackRpcCmd,
    BtLeExtAdvCbConnectedCallbackRpcCmd,
    BtLeScanCbRecvRpcCmd,
    BtLeScanCbTimeoutRpcCmd,
    BtForeachBondCbCallbackRpcCmd,
    BtPerAdvSyncCbSyncedRpcCmd,
    BtPerAdvSyncCbTermRpcCmd,
    BtPerAdvSyncCbRecvRpcCmd,
    BtPerAdvSyncCbStateChangedRpcCmd,
    /* conn.h API */
    BtConnCbConnectedCallRpcCmd,
    BtConnCbDisconnectedCallRpcCmd,
    BtConnCbLeParamReqCallRpcCmd,
    BtConnCbLeParamUpdatedCallRpcCmd,
    BtConnCbLePhyUpdatedCallRpcCmd,
    BtConnCbLeDataLenUpdatedCallRpcCmd,
    BtConnCbIdentityResolvedCallRpcCmd,
    BtConnCbSecurityChangedCallRpcCmd,
    BtConnCbRemoteInfoAvailableCallRpcCmd,
    BtRpcAuthCbPairingAcceptRpcCmd,
    BtRpcAuthCbPasskeyDisplayRpcCmd,
    BtRpcAuthCbPasskeyEntryRpcCmd,
    BtRpcAuthCbPasskeyConfirmRpcCmd,
    BtRpcAuthCbOobDataRequestRpcCmd,
    BtRpcAuthCbCancelRpcCmd,
    BtRpcAuthCbPairingConfirmRpcCmd,
    BtRpcAuthCbPincodeEntryRpcCmd,
    BtRpcAuthInfoCbPairingCompleteRpcCmd,
    BtRpcAuthInfoCbPairingFailedRpcCmd,
    BtRpcAuthInfoCbBondDeletedRpcCmd,
    BtConnForeachCbCallbackRpcCmd,
    /* gatt.h API */
    BtRpcGattCbAttrReadRpcCmd,
    BtRpcGattCbAttrWriteRpcCmd,
    BtRpcGattCbCccCfgChangedRpcCmd,
    BtRpcGattCbCccCfgWriteRpcCmd,
    BtRpcGattCbCccCfgMatchRpcCmd,
    BtGattCompleteFuncTCallbackRpcCmd,
    BtGattIndicateFuncTCallbackRpcCmd,
    BtGattIndicateParamsDestroyTCallbackRpcCmd,
    BtGattCbAttMtuUpdateCallRpcCmd,
    BtGattExchangeMtuCallbackRpcCmd,
    BtGattDiscoverCallbackRpcCmd,
    BtGattReadCallbackRpcCmd,
    BtGattWriteCallbackRpcCmd,
    BtGattSubscribeParamsNotifyRpcCmd,
    BtGattSubscribeParamsSubscribeRpcCmd,
}

use crate as nrf_rpc;
use nrf_rpc_codegen::rpc_from_c;

// rpc_from_c!(
//     cmd = "BtEnableRpcCmd",
//     sig = "int bt_enable(bt_ready_cb_t cb)"
// );

/// BLE RPC client
///
/// Encapsulates an RPC client for Bluetooth Low Energy operations.
#[rpc_from_c((cmd = "BtEnableRpcCmd", sig = "int bt_enable(bt_ready_cb_t cb)"))]
pub struct Ble<T: AsyncTransport> {
    client: RpcClient<T>,
}

#[derive(Debug)]
pub enum BleError {
    /// Underlying nRF RPC transport or protocol error.
    RpcError,
    /// Local argument or encoding issue before sending a command.
    InvalidParameter,
}

impl<T: AsyncTransport> Ble<T> {
    /// Validate configuration between client and remote host.
    ///
    /// This mirrors the Zephyr `validate_config` helper, but currently acts as a
    /// no-op placeholder. The underlying C implementation uses a configuration
    /// \"check list\" to verify that client and host Kconfig options match; when
    /// that logic is ported, this method should issue the corresponding
    /// `BtRpcGetCheckListRpcCmd` command and validate the returned buffer.
    async fn validate_config(&mut self) -> Result<(), BleError> {
        // For now, we do not perform the full checklist exchange and validation.
        // The Zephyr client treats a mismatch as a reported error but does not
        // change the `bt_enable` return code, so this no-op keeps semantics
        // compatible for success paths.
        Ok(())
    }
    /// Create a new BLE client and initialize the RPC connection
    ///
    /// This constructor is async and will block until the RPC handshake completes.
    ///
    /// # Example
    /// ```ignore
    /// let mut ble = Ble::new(transport).await?;
    /// ```
    pub async fn new(transport: T) -> Result<Self, BleError> {
        let mut client = RpcClient::new(transport);
        client.init().await.map_err(|_| BleError::RpcError)?;
        Ok(Self { client })
    }

    /*
    /// Enable Bluetooth (TODO) add zephyr doc comments HERE
    ///
    /// # Example
    /// ```ignore
    /// ble.bt_enable().await?;
    /// ```
    pub async fn bt_enable(&mut self) -> Result<(), BleError> {
        // Match Zephyr behavior:
        // 1) Validate configuration (no-op placeholder for now).
        self.validate_config()
            .await
            .expect("Failed to validate configuration");

        // 2) Send BT_ENABLE_RPC_CMD and wait for i32 status.
        let mut buffer = [0u8; 16];
        // Zephyr encodes a callback slot for `bt_enable`. The Rust API does not
        // currently expose a callback, so we send an empty CBOR payload
        // (terminating null only), which the host interprets as no arguments.
        let cbor_args = CborPayloadBuilder::new(&mut buffer);
        let payload = cbor_args.build().expect("Failed to build CBOR payload");

        let packet = NrfRpcPacket::<packet::Command>::new(
            SrcContextId::try_from(self.client.context_id()).expect("Invalid source context ID"),
            // New conversation: destination context is unknown (0xFF) until the
            // peer assigns it in the response.
            DestContextId::try_from(0xFF).expect("Invalid destination context ID"),
            CommandId::try_from(BleClientCommandId::BtEnableRpcCmd as u8)
                .expect("Invalid command ID"),
            SrcGroupId::try_from(self.client.bt_rpc_group_id()).expect("Invalid source group ID"),
            // Destination group ID is updated during init once the host assigns
            // an ID for `bt_rpc`. For now we assume the same ID as our source.
            DstGroupId::try_from(self.client.bt_rpc_group_id())
                .expect("Invalid destination group ID"),
            payload,
        );

        let status = self
            .client
            .send_command_and_get_i32(packet)
            .await
            .expect("Failed to send command and get i32");

        if status != 0 {
            panic!("bt_enable failed with status: {}", status);
            return Err(BleError::RpcError);
        }

        // 3) Optionally load Bluetooth settings on the remote if supported.
        // Zephyr does this under CONFIG_BT_SETTINGS after a successful
        // bt_enable(). We unconditionally send the corresponding RPC; hosts
        // that do not implement settings will simply return an error.
        // let mut buffer = [0u8; 8];
        // let cbor_args = CborPayloadBuilder::new(&mut buffer);
        // let payload = cbor_args.build().expect("Failed to build CBOR payload");

        // let settings_packet = NrfRpcPacket::<packet::Command>::new(
        //     SrcContextId::try_from(self.client.context_id()).expect("Invalid source context ID"),
        //     DestContextId::try_from(0xFF).expect("Invalid destination context ID"),
        //     CommandId::try_from(BleCommandId::BtSettingsLoadRpcCmd as u8)
        //         .expect("Invalid command ID"),
        //     SrcGroupId::try_from(self.client.bt_rpc_group_id()).expect("Invalid source group ID"),
        //     DstGroupId::try_from(self.client.bt_rpc_group_id())
        //         .expect("Invalid destination group ID"),
        //     payload,
        // );

        // Ignore non-zero status from settings load for now; the C implementation
        // also treats this as a separate step after bt_enable succeeds.
        // let _ = self
        //     .client
        //     .send_command_and_get_i32(settings_packet)
        //     .await
        //     .expect("Failed to send command and get i32");

        Ok(())
    }*/

    pub async fn bt_le_adv_start(&mut self) -> Result<(), BleError> {
        // Mirror the Zephyr bt_le_adv_start RPC encoding.
        //
        // For now we hard-code a simple connectable advertisement that matches
        // the sniffed Zephyr example:
        // - flags: BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR (0x06)
        // - complete name: "INordic_PS"
        // - options = 3, interval_min = 160, interval_max = 240, no peer.

        // Helper matching NRF_RPC_SCRATCHPAD_ALIGN (round up to 4-byte boundary).
        fn scratchpad_align(size: usize) -> usize {
            (size + 3) & !3
        }

        const BT_DATA_STRUCT_SIZE_32BIT: usize = 8;
        const BT_ADDR_LE_SIZE: usize = 7;

        fn bt_data_sp_size(d: &BtData<'_>) -> usize {
            scratchpad_align(d.data.len())
        }

        fn bt_le_adv_param_sp_size(param: &BtLeAdvParam) -> usize {
            if param.peer.is_some() {
                scratchpad_align(BT_ADDR_LE_SIZE)
            } else {
                0
            }
        }

        // 1) Build logical parameters and AD/SD elements.
        let flags = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR; // 0x06
        let adv_param = BtLeAdvParam::new(
            0,    // id
            0,    // sid
            0,    // secondary_max_skip
            3,    // options (match Zephyr example)
            160,  // interval_min
            240,  // interval_max
            None, // peer
        );

        let flags_bytes = [flags];
        let ad = [BtData::flags(&flags_bytes)];
        let sd = [BtData::name_complete(b"INordic_PS")];

        // 2) Compute scratchpad_size exactly like the C client.
        let mut scratchpad_size: usize = 0;

        for d in &ad {
            scratchpad_size += scratchpad_align(BT_DATA_STRUCT_SIZE_32BIT);
            scratchpad_size += bt_data_sp_size(d);
        }

        for d in &sd {
            scratchpad_size += scratchpad_align(BT_DATA_STRUCT_SIZE_32BIT);
            scratchpad_size += bt_data_sp_size(d);
        }

        scratchpad_size += bt_le_adv_param_sp_size(&adv_param);

        // 3) Encode CBOR arguments in the exact order expected by the host:
        //    scratchpad_size (uint),
        //    adv_param fields,
        //    ad_len, each bt_data,
        //    sd_len, each bt_data.
        let mut cbor_buffer = [0u8; 128];
        let mut builder = CborPayloadBuilder::new(&mut cbor_buffer);

        // scratchpad_size
        builder = builder
            .encode_uint_64(scratchpad_size as u64)
            .expect("Failed to encode scratchpad_size");

        // bt_le_adv_param fields
        builder = builder
            .encode_uint_8(adv_param.id)
            .and_then(|b| b.encode_uint_8(adv_param.sid))
            .and_then(|b| b.encode_uint_8(adv_param.secondary_max_skip))
            .and_then(|b| b.encode_uint_32(adv_param.options))
            .and_then(|b| b.encode_uint_32(adv_param.interval_min))
            .and_then(|b| b.encode_uint_32(adv_param.interval_max))
            .expect("Failed to encode adv parameters");

        if let Some(peer) = &adv_param.peer {
            let mut bytes = [0u8; BT_ADDR_LE_SIZE];
            bytes[0] = peer.addr_type;
            bytes[1..].copy_from_slice(&peer.addr);
            builder = builder
                .cbor_bytes(&bytes)
                .expect("Failed to encode peer address");
        } else {
            builder = builder.cbor_null().expect("Failed to encode null peer");
        }

        // ad_len
        builder = builder
            .encode_uint_64(ad.len() as u64)
            .expect("Failed to encode ad_len");

        // Each ad element: type, data_len, data
        for d in &ad {
            builder = builder
                .encode_uint_8(d.type_)
                .and_then(|b| b.encode_uint_64(d.data.len() as u64))
                .and_then(|b| b.cbor_bytes(d.data))
                .expect("Failed to encode ad element");
        }

        // sd_len
        builder = builder
            .encode_uint_64(sd.len() as u64)
            .expect("Failed to encode sd_len");

        // Each sd element
        for d in &sd {
            builder = builder
                .encode_uint_8(d.type_)
                .and_then(|b| b.encode_uint_64(d.data.len() as u64))
                .and_then(|b| b.cbor_bytes(d.data))
                .expect("Failed to encode sd element");
        }

        let cbor_payload = builder.build().expect("Failed to build CBOR payload");

        let packet = NrfRpcPacket::<packet::Command>::new(
            SrcContextId::try_from(self.client.context_id()).expect("Invalid source context ID"),
            // New conversation: destination context is unknown (0xFF) until the
            // peer assigns it in the response.
            DestContextId::try_from(0xFF).expect("Invalid destination context ID"),
            CommandId::try_from(BleClientCommandId::BtLeAdvStartRpcCmd as u8)
                .expect("Invalid command ID"),
            SrcGroupId::try_from(self.client.bt_rpc_group_id()).expect("Invalid source group ID"),
            DstGroupId::try_from(self.client.bt_rpc_group_id())
                .expect("Invalid destination group ID"),
            cbor_payload,
        );

        let status = self
            .client
            .send_command_and_get_i32(packet)
            .await
            .map_err(|_| BleError::RpcError)?;

        if status != 0 {
            return Err(BleError::RpcError);
        }

        Ok(())
    }

    // ========================================================================
    // Scanning
    // ========================================================================

    /// Start BLE scanning.
    ///
    /// Mirrors `bt_le_scan_start(const struct bt_le_scan_param *param, bt_le_scan_cb_t cb)`.
    ///
    /// The callback is encoded as a CBOR int32 slot (or null if None).
    /// For CONFIG_BT_MAX_CONN=1, the server does not encode the connection
    /// object in scan results, so the callback is simplistic.
    ///
    /// Wire encoding order (matching the C client):
    ///   scan_type, options, interval, window, timeout, interval_coded, window_coded, callback
    pub async fn bt_le_scan_start(
        &mut self,
        param: &BtLeScanParam,
        callback_slot: Option<u32>,
    ) -> Result<i32, BleError> {
        let mut cbor_buffer = [0u8; 64];
        let builder = CborPayloadBuilder::new(&mut cbor_buffer);

        let builder = builder
            .encode_uint_8(param.scan_type)
            .map_err(|_| BleError::InvalidParameter)?
            .encode_uint_8(param.options)
            .map_err(|_| BleError::InvalidParameter)?
            .encode_uint_16(param.interval)
            .map_err(|_| BleError::InvalidParameter)?
            .encode_uint_16(param.window)
            .map_err(|_| BleError::InvalidParameter)?
            .encode_uint_16(param.timeout)
            .map_err(|_| BleError::InvalidParameter)?
            .encode_uint_16(param.interval_coded)
            .map_err(|_| BleError::InvalidParameter)?
            .encode_uint_16(param.window_coded)
            .map_err(|_| BleError::InvalidParameter)?;

        // Encode callback: None → CBOR null, Some(slot) → CBOR int32
        let builder = match callback_slot {
            None => builder
                .cbor_null()
                .map_err(|_| BleError::InvalidParameter)?,
            Some(slot) => builder
                .encode_int_32(slot as i32)
                .map_err(|_| BleError::InvalidParameter)?,
        };

        let payload = builder.build().map_err(|_| BleError::InvalidParameter)?;

        let packet = NrfRpcPacket::<packet::Command>::new(
            SrcContextId::try_from(self.client.context_id()).expect("Invalid source context ID"),
            DestContextId::try_from(0xFF).expect("Invalid destination context ID"),
            CommandId::try_from(BleClientCommandId::BtLeScanStartRpcCmd as u8)
                .expect("Invalid command ID"),
            SrcGroupId::try_from(self.client.bt_rpc_group_id()).expect("Invalid source group ID"),
            DstGroupId::try_from(self.client.bt_rpc_group_id())
                .expect("Invalid destination group ID"),
            payload,
        );

        let status = self
            .client
            .send_command_and_get_i32(packet)
            .await
            .map_err(|_| BleError::RpcError)?;

        Ok(status)
    }

    /// Stop BLE scanning.
    ///
    /// Wire encoding: empty CBOR payload (just null terminator).
    pub async fn bt_le_scan_stop(&mut self) -> Result<i32, BleError> {
        let mut cbor_buffer = [0u8; 8];
        let builder = CborPayloadBuilder::new(&mut cbor_buffer);
        let payload = builder.build().map_err(|_| BleError::InvalidParameter)?;

        let packet = NrfRpcPacket::<packet::Command>::new(
            SrcContextId::try_from(self.client.context_id()).expect("Invalid source context ID"),
            DestContextId::try_from(0xFF).expect("Invalid destination context ID"),
            CommandId::try_from(BleClientCommandId::BtLeScanStopRpcCmd as u8)
                .expect("Invalid command ID"),
            SrcGroupId::try_from(self.client.bt_rpc_group_id()).expect("Invalid source group ID"),
            DstGroupId::try_from(self.client.bt_rpc_group_id())
                .expect("Invalid destination group ID"),
            payload,
        );

        let status = self
            .client
            .send_command_and_get_i32(packet)
            .await
            .map_err(|_| BleError::RpcError)?;

        Ok(status)
    }

    // ========================================================================
    // Connection Callbacks
    // ========================================================================

    /// Register connection callbacks on the remote.
    ///
    /// Mirrors `bt_conn_cb_register_on_remote()`. This tells the server to
    /// start forwarding connection events (connected, disconnected, etc.)
    /// back to us. The payload is empty — no parameters.
    pub async fn bt_conn_cb_register_on_remote(&mut self) -> Result<(), BleError> {
        let mut cbor_buffer = [0u8; 8];
        let builder = CborPayloadBuilder::new(&mut cbor_buffer);
        let payload = builder.build().map_err(|_| BleError::InvalidParameter)?;

        let packet = NrfRpcPacket::<packet::Command>::new(
            SrcContextId::try_from(self.client.context_id()).expect("Invalid source context ID"),
            DestContextId::try_from(0xFF).expect("Invalid destination context ID"),
            CommandId::try_from(BleClientCommandId::BtConnCbRegisterOnRemoteRpcCmd as u8)
                .expect("Invalid command ID"),
            SrcGroupId::try_from(self.client.bt_rpc_group_id()).expect("Invalid source group ID"),
            DstGroupId::try_from(self.client.bt_rpc_group_id())
                .expect("Invalid destination group ID"),
            payload,
        );

        self.client
            .send_command_void(packet)
            .await
            .map_err(|_| BleError::RpcError)?;

        Ok(())
    }

    // ========================================================================
    // GATT Discovery
    // ========================================================================

    /// Start GATT service/characteristic discovery.
    ///
    /// Mirrors `bt_gatt_discover(struct bt_conn *conn, struct bt_gatt_discover_params *params)`.
    ///
    /// Wire encoding order:
    ///   [conn_index if MAX_CONN>1], uuid_buffer, start_handle, end_handle, type, params_pointer
    ///
    /// Since CONFIG_BT_MAX_CONN=1 in our BSIM setup, the conn object is NOT
    /// encoded (it's implicit — the single active connection).
    ///
    /// `params_ptr` is a synthetic pointer value the server uses to match
    /// the discovery callback back to these params. We pass a unique ID.
    pub async fn bt_gatt_discover(
        &mut self,
        params: &BtGattDiscoverParams,
        params_ptr: u64,
    ) -> Result<i32, BleError> {
        let mut cbor_buffer = [0u8; 64];
        let builder = CborPayloadBuilder::new(&mut cbor_buffer);

        // Encode UUID as a CBOR byte string (the raw struct bytes)
        let builder = builder
            .cbor_bytes(&params.uuid)
            .map_err(|_| BleError::InvalidParameter)?
            .encode_uint_16(params.start_handle)
            .map_err(|_| BleError::InvalidParameter)?
            .encode_uint_16(params.end_handle)
            .map_err(|_| BleError::InvalidParameter)?
            .encode_uint_8(params.discover_type as u8)
            .map_err(|_| BleError::InvalidParameter)?
            .encode_uint_64(params_ptr)
            .map_err(|_| BleError::InvalidParameter)?;

        let payload = builder.build().map_err(|_| BleError::InvalidParameter)?;

        let packet = NrfRpcPacket::<packet::Command>::new(
            SrcContextId::try_from(self.client.context_id()).expect("Invalid source context ID"),
            DestContextId::try_from(0xFF).expect("Invalid destination context ID"),
            CommandId::try_from(BleClientCommandId::BtGattDiscoverRpcCmd as u8)
                .expect("Invalid command ID"),
            SrcGroupId::try_from(self.client.bt_rpc_group_id()).expect("Invalid source group ID"),
            DstGroupId::try_from(self.client.bt_rpc_group_id())
                .expect("Invalid destination group ID"),
            payload,
        );

        let status = self
            .client
            .send_command_and_get_i32_ack_events_u8(packet, BT_GATT_ITER_CONTINUE)
            .await
            .map_err(|_| BleError::RpcError)?;

        Ok(status)
    }

    // ========================================================================
    // GATT Read
    // ========================================================================

    /// Read a GATT characteristic value (single handle mode, handle_count=1).
    ///
    /// Mirrors `bt_gatt_read(struct bt_conn *conn, struct bt_gatt_read_params *params)`.
    ///
    /// Wire encoding order (handle_count == 1):
    ///   [conn], handle_count(1), handle, offset, params_pointer
    ///
    /// With CONFIG_BT_MAX_CONN=1, conn is not encoded.
    pub async fn bt_gatt_read(
        &mut self,
        params: &BtGattReadParams,
        params_ptr: u64,
    ) -> Result<i32, BleError> {
        let mut cbor_buffer = [0u8; 64];
        let builder = CborPayloadBuilder::new(&mut cbor_buffer);

        let builder = builder
            // handle_count = 1 (single read mode)
            .encode_uint_64(1u64)
            .map_err(|_| BleError::InvalidParameter)?
            // single.handle
            .encode_uint_16(params.handle)
            .map_err(|_| BleError::InvalidParameter)?
            // single.offset
            .encode_uint_16(params.offset)
            .map_err(|_| BleError::InvalidParameter)?
            // params pointer (for callback matching)
            .encode_uint_64(params_ptr)
            .map_err(|_| BleError::InvalidParameter)?;

        let payload = builder.build().map_err(|_| BleError::InvalidParameter)?;

        let packet = NrfRpcPacket::<packet::Command>::new(
            SrcContextId::try_from(self.client.context_id()).expect("Invalid source context ID"),
            DestContextId::try_from(0xFF).expect("Invalid destination context ID"),
            CommandId::try_from(BleClientCommandId::BtGattReadRpcCmd as u8)
                .expect("Invalid command ID"),
            SrcGroupId::try_from(self.client.bt_rpc_group_id()).expect("Invalid source group ID"),
            DstGroupId::try_from(self.client.bt_rpc_group_id())
                .expect("Invalid destination group ID"),
            payload,
        );

        let status = self
            .client
            .send_command_and_get_i32(packet)
            .await
            .map_err(|_| BleError::RpcError)?;

        Ok(status)
    }

    // ========================================================================
    // GATT Subscribe
    // ========================================================================

    /// Subscribe to GATT notifications/indications.
    ///
    /// Mirrors `bt_gatt_subscribe(struct bt_conn *conn, struct bt_gatt_subscribe_params *params)`.
    ///
    /// Wire encoding order:
    ///   [conn], params_pointer, has_notify(bool), subscribe_callback(null),
    ///   value_handle, ccc_handle, value, flags
    ///
    /// With CONFIG_BT_MAX_CONN=1, conn is not encoded.
    pub async fn bt_gatt_subscribe(
        &mut self,
        params: &BtGattSubscribeParams,
        params_ptr: u64,
    ) -> Result<i32, BleError> {
        let mut cbor_buffer = [0u8; 64];
        let builder = CborPayloadBuilder::new(&mut cbor_buffer);

        let builder = builder
            // params pointer (uintptr_t)
            .encode_uint_64(params_ptr)
            .map_err(|_| BleError::InvalidParameter)?
            // has_notify (CBOR bool — nrf_rpc_encode_bool)
            .cbor_bool(params.has_notify)
            .map_err(|_| BleError::InvalidParameter)?
            // subscribe callback (null = no callback — nrf_rpc_encode_callback)
            .cbor_null()
            .map_err(|_| BleError::InvalidParameter)?
            // value_handle
            .encode_uint_16(params.value_handle)
            .map_err(|_| BleError::InvalidParameter)?
            // ccc_handle
            .encode_uint_16(params.ccc_handle)
            .map_err(|_| BleError::InvalidParameter)?
            // value (notification/indication flags)
            .encode_uint_16(params.value)
            .map_err(|_| BleError::InvalidParameter)?
            // min_security (bt_security_t, CONFIG_BT_SMP)
            .encode_uint_8(params.min_security)
            .map_err(|_| BleError::InvalidParameter)?
            // flags (atomic_t, typically 0)
            .encode_uint_16(params.flags)
            .map_err(|_| BleError::InvalidParameter)?;

        let payload = builder.build().map_err(|_| BleError::InvalidParameter)?;

        let packet = NrfRpcPacket::<packet::Command>::new(
            SrcContextId::try_from(self.client.context_id()).expect("Invalid source context ID"),
            DestContextId::try_from(0xFF).expect("Invalid destination context ID"),
            CommandId::try_from(BleClientCommandId::BtGattSubscribeRpcCmd as u8)
                .expect("Invalid command ID"),
            SrcGroupId::try_from(self.client.bt_rpc_group_id()).expect("Invalid source group ID"),
            DstGroupId::try_from(self.client.bt_rpc_group_id())
                .expect("Invalid destination group ID"),
            payload,
        );

        let status = self
            .client
            .send_command_and_get_i32_ack_events_u8(packet, BT_GATT_ITER_CONTINUE)
            .await
            .map_err(|_| BleError::RpcError)?;

        Ok(status)
    }

    // ========================================================================
    // Scan callback registration
    // ========================================================================

    /// Register the scan callback on the remote (server).
    ///
    /// Mirrors `bt_le_scan_cb_register_on_remote()`. This tells the server to
    /// start forwarding scan result events (`BtLeScanCbRecvRpcCmd`) to us
    /// when BLE scanning is active.
    ///
    /// Must be called before `bt_le_scan_start()` for the client to receive
    /// scan results.
    pub async fn bt_le_scan_cb_register_on_remote(&mut self) -> Result<(), BleError> {
        let mut cbor_buffer = [0u8; 8];
        let builder = CborPayloadBuilder::new(&mut cbor_buffer);
        let payload = builder.build().map_err(|_| BleError::InvalidParameter)?;

        let packet = NrfRpcPacket::<packet::Command>::new(
            SrcContextId::try_from(self.client.context_id()).expect("Invalid source context ID"),
            DestContextId::try_from(0xFF).expect("Invalid destination context ID"),
            CommandId::try_from(BleClientCommandId::BtLeScanCbRegisterOnRemoteRpcCmd as u8)
                .expect("Invalid command ID"),
            SrcGroupId::try_from(self.client.bt_rpc_group_id()).expect("Invalid source group ID"),
            DstGroupId::try_from(self.client.bt_rpc_group_id())
                .expect("Invalid destination group ID"),
            payload,
        );

        self.client
            .send_command_void(packet)
            .await
            .map_err(|_| BleError::RpcError)?;

        Ok(())
    }

    // ========================================================================
    // Auth callback registration
    // ========================================================================

    /// Set the security level for a connection.
    ///
    /// Mirrors `bt_conn_set_security(struct bt_conn *conn, bt_security_t sec)`.
    /// With CONFIG_BT_MAX_CONN=1, the conn encoding is empty; only the security
    /// level is sent.
    ///
    /// Security levels:
    /// - 0: BT_SECURITY_L0 (no security)
    /// - 1: BT_SECURITY_L1 (no encryption / no authentication)
    /// - 2: BT_SECURITY_L2 (encryption / no authentication)
    /// - 3: BT_SECURITY_L3 (encryption / authentication)
    /// - 4: BT_SECURITY_L4 (128-bit key / authenticated SC)
    pub async fn bt_conn_set_security(&mut self, sec: u8) -> Result<i32, BleError> {
        let mut cbor_buffer = [0u8; 16];
        let builder = CborPayloadBuilder::new(&mut cbor_buffer);

        let builder = builder
            .encode_uint_8(sec)
            .map_err(|_| BleError::InvalidParameter)?;

        let payload = builder.build().map_err(|_| BleError::InvalidParameter)?;

        let packet = NrfRpcPacket::<packet::Command>::new(
            SrcContextId::try_from(self.client.context_id()).expect("Invalid source context ID"),
            DestContextId::try_from(0xFF).expect("Invalid destination context ID"),
            CommandId::try_from(BleClientCommandId::BtConnSetSecurityRpcCmd as u8)
                .expect("Invalid command ID"),
            SrcGroupId::try_from(self.client.bt_rpc_group_id()).expect("Invalid source group ID"),
            DstGroupId::try_from(self.client.bt_rpc_group_id())
                .expect("Invalid destination group ID"),
            payload,
        );

        // Use send_command_and_get_i32 which routes through ack_event for
        // proper dispatch: bool ACK for le_param_req (via bool_ack_cmd_id),
        // void ACK + auto-confirm for passkey_confirm, void for others.
        let status = self
            .client
            .send_command_and_get_i32(packet)
            .await
            .map_err(|_| BleError::RpcError)?;

        Ok(status)
    }

    /// Register authentication callbacks on the remote (server).
    ///
    /// Mirrors `bt_conn_auth_cb_register(const struct bt_conn_auth_cb *cb)`.
    ///
    /// `flags` is a bitmask of FLAG_*_PRESENT constants indicating which
    /// auth callbacks are provided. The server then forwards the corresponding
    /// auth events (passkey display, passkey confirm, etc.) back to this client.
    ///
    /// Common flags:
    /// - FLAG_PASSKEY_DISPLAY_PRESENT  = 0x02
    /// - FLAG_PASSKEY_CONFIRM_PRESENT  = 0x08
    /// - FLAG_CANCEL_PRESENT           = 0x20
    /// - FLAG_PAIRING_CONFIRM_PRESENT  = 0x40
    pub async fn bt_conn_auth_cb_register_on_remote(
        &mut self,
        flags: u16,
    ) -> Result<i32, BleError> {
        let mut cbor_buffer = [0u8; 16];
        let builder = CborPayloadBuilder::new(&mut cbor_buffer);

        let builder = builder
            .encode_uint_16(flags)
            .map_err(|_| BleError::InvalidParameter)?;

        let payload = builder.build().map_err(|_| BleError::InvalidParameter)?;

        let packet = NrfRpcPacket::<packet::Command>::new(
            SrcContextId::try_from(self.client.context_id()).expect("Invalid source context ID"),
            DestContextId::try_from(0xFF).expect("Invalid destination context ID"),
            CommandId::try_from(BleClientCommandId::BtConnAuthCbRegisterOnRemoteRpcCmd as u8)
                .expect("Invalid command ID"),
            SrcGroupId::try_from(self.client.bt_rpc_group_id()).expect("Invalid source group ID"),
            DstGroupId::try_from(self.client.bt_rpc_group_id())
                .expect("Invalid destination group ID"),
            payload,
        );

        let status = self
            .client
            .send_command_and_get_i32(packet)
            .await
            .map_err(|_| BleError::RpcError)?;

        // Now that auth callbacks are registered, the server may send
        // le_param_req events (cmd_id 0x0D) which require a bool ACK.
        // Tell the RPC client about this so all event ACK paths handle it.
        self.client
            .set_bool_ack_cmd_id(BleHostCommandId::BtConnCbLeParamReqCallRpcCmd as u8);

        // If passkey_confirm is registered (flag 0x08), set up auto-confirm so
        // that when the server sends a passkey_confirm event during GATT operations,
        // the client automatically void-ACKs AND sends bt_conn_auth_passkey_confirm().
        if flags & 0x08 != 0 {
            self.client.set_auto_confirm(
                BleHostCommandId::BtRpcAuthCbPasskeyConfirmRpcCmd as u8,
                BleClientCommandId::BtConnAuthPasskeyConfirmRpcCmd as u8,
            );
        }

        Ok(status)
    }

    /// Confirm a passkey for Secure Connections pairing.
    ///
    /// Mirrors `bt_conn_auth_passkey_confirm(struct bt_conn *conn)`.
    /// With CONFIG_BT_MAX_CONN=1, the conn is not encoded.
    pub async fn bt_conn_auth_passkey_confirm(&mut self) -> Result<i32, BleError> {
        let mut cbor_buffer = [0u8; 8];
        let builder = CborPayloadBuilder::new(&mut cbor_buffer);
        let payload = builder.build().map_err(|_| BleError::InvalidParameter)?;

        let packet = NrfRpcPacket::<packet::Command>::new(
            SrcContextId::try_from(self.client.context_id()).expect("Invalid source context ID"),
            DestContextId::try_from(0xFF).expect("Invalid destination context ID"),
            CommandId::try_from(BleClientCommandId::BtConnAuthPasskeyConfirmRpcCmd as u8)
                .expect("Invalid command ID"),
            SrcGroupId::try_from(self.client.bt_rpc_group_id()).expect("Invalid source group ID"),
            DstGroupId::try_from(self.client.bt_rpc_group_id())
                .expect("Invalid destination group ID"),
            payload,
        );

        let status = self
            .client
            .send_command_and_get_i32(packet)
            .await
            .map_err(|_| BleError::RpcError)?;

        Ok(status)
    }

    /// Confirm a pairing request ("Do you want to pair?").
    ///
    /// Mirrors `bt_conn_auth_pairing_confirm(struct bt_conn *conn)`.
    /// With CONFIG_BT_MAX_CONN=1, the conn is not encoded.
    pub async fn bt_conn_auth_pairing_confirm(&mut self) -> Result<i32, BleError> {
        let mut cbor_buffer = [0u8; 8];
        let builder = CborPayloadBuilder::new(&mut cbor_buffer);
        let payload = builder.build().map_err(|_| BleError::InvalidParameter)?;

        let packet = NrfRpcPacket::<packet::Command>::new(
            SrcContextId::try_from(self.client.context_id()).expect("Invalid source context ID"),
            DestContextId::try_from(0xFF).expect("Invalid destination context ID"),
            CommandId::try_from(BleClientCommandId::BtConnAuthPairingConfirmRpcCmd as u8)
                .expect("Invalid command ID"),
            SrcGroupId::try_from(self.client.bt_rpc_group_id()).expect("Invalid source group ID"),
            DstGroupId::try_from(self.client.bt_rpc_group_id())
                .expect("Invalid destination group ID"),
            payload,
        );

        let status = self
            .client
            .send_command_and_get_i32(packet)
            .await
            .map_err(|_| BleError::RpcError)?;

        Ok(status)
    }

    /// Wait for a passkey confirm event and automatically confirm it.
    ///
    /// After connecting to a peripheral that requires Secure Connections
    /// pairing, the server will send a `BtRpcAuthCbPasskeyConfirmRpcCmd`
    /// event with the passkey. This method waits for that event, ACKs it,
    /// then calls `bt_conn_auth_passkey_confirm` to complete pairing.
    ///
    /// Also handles the `pairing_confirm` event that precedes passkey_confirm
    /// in Numeric Comparison flows, and recognises `security_changed` (err=0)
    /// as an indication that pairing completed via Just Works.
    ///
    /// Expected event sequence for SC Numeric Comparison:
    ///   1. `BtRpcAuthCbPairingConfirmRpcCmd` → we reply with `bt_conn_auth_pairing_confirm`
    ///   2. `BtRpcAuthCbPasskeyConfirmRpcCmd` (contains passkey) → we reply with `bt_conn_auth_passkey_confirm`
    ///   3. `BtConnCbSecurityChangedCallRpcCmd` (level, err=0) → pairing done
    ///
    /// Other events (le_param_updated, etc.) are consumed and skipped.
    pub async fn wait_for_passkey_confirm_and_accept(&mut self) -> Result<u32, BleError> {
        #[cfg(test)]
        extern crate std;

        let timeout_retries = 30; // generous — pairing involves several round-trips
        let mut passkey_value: Option<u32> = None;

        for _i in 0..timeout_retries {
            let mut event_buf = [0u8; 256];
            let (cmd_id, payload_len) = match self.client.receive_server_event(&mut event_buf).await
            {
                Ok(result) => result,
                Err(_) => continue,
            };

            #[cfg(test)]
            std::println!(
                "  [pairing] event cmd_id=0x{:02X}, payload_len={}, raw={:02X?}",
                cmd_id,
                payload_len,
                &event_buf[..payload_len],
            );

            // ---- pairing_confirm ("Do you want to pair?") ----
            if cmd_id == BleHostCommandId::BtRpcAuthCbPairingConfirmRpcCmd as u8 {
                #[cfg(test)]
                std::println!("  [pairing] Got pairing_confirm — accepting.");
                let result = self.bt_conn_auth_pairing_confirm().await?;
                #[cfg(test)]
                std::println!("  [pairing] bt_conn_auth_pairing_confirm returned {}.", result);
                if result != 0 {
                    return Err(BleError::RpcError);
                }
                continue; // next: passkey_confirm
            }

            // ---- passkey_confirm (numeric comparison) ----
            if cmd_id == BleHostCommandId::BtRpcAuthCbPasskeyConfirmRpcCmd as u8 {
                let payload = &event_buf[..payload_len];
                let mut d = minicbor::decode::Decoder::new(payload);
                let passkey = d.u32().unwrap_or(0);

                #[cfg(test)]
                std::println!("  [pairing] Got passkey_confirm: passkey={} — confirming.", passkey);

                let result = self.bt_conn_auth_passkey_confirm().await?;
                #[cfg(test)]
                std::println!("  [pairing] bt_conn_auth_passkey_confirm returned {}.", result);
                if result != 0 {
                    return Err(BleError::RpcError);
                }
                passkey_value = Some(passkey);
                // After confirming, wait for security_changed to know pairing succeeded.
                continue;
            }

            // ---- security_changed ----
            if cmd_id == BleHostCommandId::BtConnCbSecurityChangedCallRpcCmd as u8 {
                let payload = &event_buf[..payload_len];
                let mut d = minicbor::decode::Decoder::new(payload);
                let level = d.u8().unwrap_or(0);
                let err = d.u8().unwrap_or(0xFF);

                #[cfg(test)]
                std::println!("  [pairing] Got security_changed: level={}, err={}", level, err);

                if err == 0 {
                    return Ok(passkey_value.unwrap_or(0));
                }
                // err != 0 with no passkey yet: pairing attempt failed, keep looping
                // (the real passkey_confirm may still arrive on a retry)
                if passkey_value.is_some() {
                    // We already confirmed passkey but got an error — fatal
                    return Err(BleError::RpcError);
                }
                continue;
            }

            // ---- any other event: consume and keep looping ----
            #[cfg(test)]
            std::println!("  [pairing] Ignoring unrelated event 0x{:02X}", cmd_id);
        }

        Err(BleError::RpcError)
    }

    /// Wait until the connection security level reaches at least `target_level`.
    ///
    /// Consumes server events (ACKing them properly, including auto-confirm
    /// for passkey exchange) until a `BtConnCbSecurityChangedCallRpcCmd`
    /// arrives with `err == 0` and `level >= target_level`.
    ///
    /// This is used after `bt_conn_set_security(4)` to wait for the SMP
    /// Numeric Comparison passkey exchange to complete.
    pub async fn wait_for_security_level(&mut self, target_level: u8) -> Result<u8, BleError> {
        #[cfg(test)]
        extern crate std;

        let timeout_retries = 30;
        for _i in 0..timeout_retries {
            let mut event_buf = [0u8; 256];
            let (cmd_id, payload_len) = match self.client.receive_server_event(&mut event_buf).await
            {
                Ok(result) => result,
                Err(_) => continue,
            };

            #[cfg(test)]
            std::println!(
                "  [wait_for_security] got event cmd_id=0x{:02X} (expect 0x{:02X}), payload_len={}",
                cmd_id,
                BleHostCommandId::BtConnCbSecurityChangedCallRpcCmd as u8,
                payload_len,
            );

            if cmd_id == BleHostCommandId::BtConnCbSecurityChangedCallRpcCmd as u8 {
                let payload = &event_buf[..payload_len];
                let mut d = minicbor::decode::Decoder::new(payload);
                let level = d.u8().unwrap_or(0);
                let err = d.u8().unwrap_or(0xFF);

                #[cfg(test)]
                std::println!(
                    "  [wait_for_security] security_changed: level={}, err={}",
                    level, err,
                );

                if err == 0 && level >= target_level {
                    return Ok(level);
                }
                // err != 0 or level too low — keep waiting (SMP might still be in progress)
                continue;
            }

            // Other events consumed and discarded (auto-confirm handles passkey inline)
            #[cfg(test)]
            std::println!("  [wait_for_security] consumed non-security event 0x{:02X}", cmd_id);
        }

        Err(BleError::RpcError)
    }

    // ========================================================================
    // Connection creation
    // ========================================================================

    /// Create a BLE connection to a peer.
    ///
    /// Mirrors `bt_conn_le_create(const bt_addr_le_t *peer,
    ///     const struct bt_conn_le_create_param *create_param,
    ///     const struct bt_le_conn_param *conn_param,
    ///     struct bt_conn **conn)`.
    ///
    /// Wire encoding order:
    ///   addr_type, addr_bytes, options, interval, window,
    ///   interval_coded, window_coded, timeout,
    ///   interval_min, interval_max, latency, conn_timeout
    ///
    /// With CONFIG_BT_MAX_CONN=1, the conn object is not returned over the wire.
    /// Returns the i32 result code (0 = success, initiating connection).
    pub async fn bt_conn_le_create(
        &mut self,
        peer: &BtAddrLe,
        create_param: &BtConnLeCreateParam,
        conn_param: &BtLeConnParam,
    ) -> Result<i32, BleError> {
        let mut cbor_buffer = [0u8; 128];
        let builder = CborPayloadBuilder::new(&mut cbor_buffer);

        // Encode bt_addr_le_t as a single 7-byte buffer: [type, addr[0..6]]
        let mut addr_le_buf = [0u8; 7];
        addr_le_buf[0] = peer.addr_type;
        addr_le_buf[1..7].copy_from_slice(&peer.addr);

        let builder = builder
            // Peer address as 7-byte buffer
            .cbor_bytes(&addr_le_buf)
            .map_err(|_| BleError::InvalidParameter)?
            // Create parameters
            .encode_uint_32(create_param.options)
            .map_err(|_| BleError::InvalidParameter)?
            .encode_uint_16(create_param.interval)
            .map_err(|_| BleError::InvalidParameter)?
            .encode_uint_16(create_param.window)
            .map_err(|_| BleError::InvalidParameter)?
            .encode_uint_16(create_param.interval_coded)
            .map_err(|_| BleError::InvalidParameter)?
            .encode_uint_16(create_param.window_coded)
            .map_err(|_| BleError::InvalidParameter)?
            .encode_uint_16(create_param.timeout)
            .map_err(|_| BleError::InvalidParameter)?
            // Connection parameters
            .encode_uint_16(conn_param.interval_min)
            .map_err(|_| BleError::InvalidParameter)?
            .encode_uint_16(conn_param.interval_max)
            .map_err(|_| BleError::InvalidParameter)?
            .encode_uint_16(conn_param.latency)
            .map_err(|_| BleError::InvalidParameter)?
            .encode_uint_16(conn_param.timeout)
            .map_err(|_| BleError::InvalidParameter)?;

        let payload = builder.build().map_err(|_| BleError::InvalidParameter)?;

        let packet = NrfRpcPacket::<packet::Command>::new(
            SrcContextId::try_from(self.client.context_id()).expect("Invalid source context ID"),
            DestContextId::try_from(0xFF).expect("Invalid destination context ID"),
            CommandId::try_from(BleClientCommandId::BtConnLeCreateRpcCmd as u8)
                .expect("Invalid command ID"),
            SrcGroupId::try_from(self.client.bt_rpc_group_id()).expect("Invalid source group ID"),
            DstGroupId::try_from(self.client.bt_rpc_group_id())
                .expect("Invalid destination group ID"),
            payload,
        );

        let status = self
            .client
            .send_command_and_get_i32(packet)
            .await
            .map_err(|_| BleError::RpcError)?;

        Ok(status)
    }

    // ========================================================================
    // Event waiting methods
    // ========================================================================

    /// Wait for and decode a scan result event from the server.
    ///
    /// Blocks until a `BtLeScanCbRecvRpcCmd` Command arrives from the server.
    /// Other event types are ACKed and skipped. Returns the decoded scan result.
    pub async fn wait_for_scan_result(&mut self) -> Result<ScanResultData, BleError> {
        // Each attempt may block for the transport read timeout (~5s).
        // 10 retries → ~50s max wait.
        let timeout_retries = 10;
        for _i in 0..timeout_retries {
            let mut event_buf = [0u8; 256];
            let (cmd_id, payload_len) = match self.client.receive_server_event(&mut event_buf).await
            {
                Ok(result) => result,
                Err(_) => continue,
            };

            #[cfg(test)]
            extern crate std;
            #[cfg(test)]
            std::println!(
                "  [wait_for_scan_result] got event cmd_id={} (expect {}), payload_len={}",
                cmd_id,
                BleHostCommandId::BtLeScanCbRecvRpcCmd as u8,
                payload_len,
            );

            if cmd_id == BleHostCommandId::BtLeScanCbRecvRpcCmd as u8 {
                let payload = &event_buf[..payload_len];
                return Self::decode_scan_result(payload);
            }
            // Not the event we're looking for — it was already ACKed, keep waiting.
        }

        Err(BleError::RpcError)
    }

    /// Wait for a connection event from the server.
    ///
    /// Blocks until a `BtConnCbConnectedCallRpcCmd` Command arrives. Other
    /// events (e.g., scan results still flowing) are ACKed and skipped.
    pub async fn wait_for_connection(&mut self) -> Result<ConnectionEvent, BleError> {
        // Each attempt may block for the transport read timeout (~5s).
        // 10 retries → ~50s max wait.
        let timeout_retries = 10;
        for _i in 0..timeout_retries {
            let mut event_buf = [0u8; 256];
            let (cmd_id, payload_len) = match self.client.receive_server_event(&mut event_buf).await
            {
                Ok(result) => result,
                Err(_) => continue,
            };

            #[cfg(test)]
            extern crate std;
            #[cfg(test)]
            std::println!(
                "  [wait_for_connection] got event cmd_id={} (expect {}), payload_len={}",
                cmd_id,
                BleHostCommandId::BtConnCbConnectedCallRpcCmd as u8,
                payload_len,
            );

            if cmd_id == BleHostCommandId::BtConnCbConnectedCallRpcCmd as u8 {
                let payload = &event_buf[..payload_len];
                let mut decoder = minicbor::decode::Decoder::new(payload);
                // With CONFIG_BT_MAX_CONN=1, bt_rpc_encode_bt_conn encodes
                // nothing (conn index is implicit). Only the err field is present.
                let err = decoder.u8().map_err(|_| BleError::RpcError)?;
                return Ok(ConnectionEvent { err });
            }
            // Not a connection event — already ACKed, continue waiting.
        }

        Err(BleError::RpcError)
    }

    // ========================================================================
    // GATT Discovery event waiting
    // ========================================================================

    /// Wait for and decode a single GATT discovery callback event from the server.
    ///
    /// The server sends `BtGattDiscoverCallbackRpcCmd` for each discovered
    /// attribute, and a final one with attr=NULL to signal completion.
    ///
    /// This method responds with `BT_GATT_ITER_CONTINUE` so the server keeps
    /// iterating. Call repeatedly until `GattDiscoverResult::Complete` is returned.
    pub async fn wait_for_gatt_discover_result(
        &mut self,
    ) -> Result<GattDiscoverResult, BleError> {
        let timeout_retries = 20;
        for _i in 0..timeout_retries {
            let mut event_buf = [0u8; 256];
            let (cmd_id, payload_len) = match self
                .client
                .receive_server_event_with_u8_response(
                    &mut event_buf,
                    BT_GATT_ITER_CONTINUE,
                )
                .await
            {
                Ok(result) => result,
                Err(_) => continue,
            };

            #[cfg(test)]
            extern crate std;
            #[cfg(test)]
            std::println!(
                "  [wait_for_gatt_discover] got event cmd_id={} (expect {}), payload_len={}",
                cmd_id,
                BleHostCommandId::BtGattDiscoverCallbackRpcCmd as u8,
                payload_len,
            );

            if cmd_id == BleHostCommandId::BtGattDiscoverCallbackRpcCmd as u8 {
                let payload = &event_buf[..payload_len];
                return Self::decode_discover_callback(payload);
            }
            // Not the event we need — keep waiting.
        }

        Err(BleError::RpcError)
    }

    /// Wait for and decode a GATT notification event from the server.
    ///
    /// The server sends `BtGattSubscribeParamsNotifyRpcCmd` each time a
    /// notification is received from the peripheral.
    ///
    /// Responds with `BT_GATT_ITER_CONTINUE` so the server keeps forwarding.
    pub async fn wait_for_gatt_notification(
        &mut self,
    ) -> Result<GattNotificationData, BleError> {
        let timeout_retries = 20;
        for _i in 0..timeout_retries {
            let mut event_buf = [0u8; 256];
            let (cmd_id, payload_len) = match self
                .client
                .receive_server_event_with_u8_response(
                    &mut event_buf,
                    BT_GATT_ITER_CONTINUE,
                )
                .await
            {
                Ok(result) => result,
                Err(_) => continue,
            };

            #[cfg(test)]
            extern crate std;
            #[cfg(test)]
            std::println!(
                "  [wait_for_gatt_notification] got event cmd_id={} (expect {}), payload_len={}",
                cmd_id,
                BleHostCommandId::BtGattSubscribeParamsNotifyRpcCmd as u8,
                payload_len,
            );

            if cmd_id == BleHostCommandId::BtGattSubscribeParamsNotifyRpcCmd as u8 {
                let payload = &event_buf[..payload_len];
                return Self::decode_notification(payload);
            }
            // Not the event we need — keep waiting.
        }

        Err(BleError::RpcError)
    }

    // ========================================================================
    // Internal event decoders
    // ========================================================================

    /// Decode a GATT discovery callback from raw CBOR payload.
    ///
    /// Wire format (CONFIG_BT_MAX_CONN=1, so no conn encoded):
    ///   params_ptr(uint),
    ///   then either:
    ///     null            → discovery complete
    ///     OR:
    ///       uuid(bstr)    → attribute UUID
    ///       handle(uint)  → attribute handle
    ///       user_data:
    ///         null                                → no user_data
    ///         OR for primary/secondary service:
    ///           service_uuid(bstr), end_handle(uint)
    ///         OR for characteristic:
    ///           char_uuid(bstr), value_handle(uint), properties(uint)
    fn decode_discover_callback(payload: &[u8]) -> Result<GattDiscoverResult, BleError> {
        let mut d = minicbor::decode::Decoder::new(payload);

        // params_ptr — we don't need it currently, but must consume it.
        let _params_ptr = d.u64().map_err(|_| BleError::RpcError)?;

        // Check for null (discovery complete).
        // Use datatype() to peek without consuming, because d.null() advances
        // the decoder position even on failure.
        if d.datatype().map_err(|_| BleError::RpcError)? == minicbor::data::Type::Null {
            let _ = d.null();
            return Ok(GattDiscoverResult::Complete);
        }

        // attr != NULL: decode uuid, handle, user_data
        let uuid_bytes = d.bytes().map_err(|_| BleError::RpcError)?;
        let handle = d.u16().map_err(|_| BleError::RpcError)?;

        // Extract the 16-bit UUID value from the attr's uuid bytes.
        // C struct bt_uuid_16 layout: [type(1), pad(1), val_lo, val_hi]
        let attr_uuid_16 = if uuid_bytes.len() >= 4 && uuid_bytes[0] == cgm::BT_UUID_TYPE_16 {
            u16::from_le_bytes([uuid_bytes[2], uuid_bytes[3]])
        } else {
            0
        };

        // Check if user_data is null.
        // Peek with datatype() to avoid corrupting decoder position.
        if d.datatype().map_err(|_| BleError::RpcError)? == minicbor::data::Type::Null {
            let _ = d.null();
            return Ok(GattDiscoverResult::Descriptor {
                handle,
                uuid_16: attr_uuid_16,
            });
        }

        // user_data is not null — branch on attr_uuid_16
        match attr_uuid_16 {
            BT_UUID_GATT_PRIMARY_VAL | BT_UUID_GATT_SECONDARY_VAL => {
                // Service: service_uuid(bstr), end_handle(uint)
                let svc_uuid_bytes = d.bytes().map_err(|_| BleError::RpcError)?;
                let end_handle = d.u16().map_err(|_| BleError::RpcError)?;

                let svc_uuid_16 =
                    if svc_uuid_bytes.len() >= 4 && svc_uuid_bytes[0] == cgm::BT_UUID_TYPE_16 {
                        u16::from_le_bytes([svc_uuid_bytes[2], svc_uuid_bytes[3]])
                    } else {
                        0
                    };

                Ok(GattDiscoverResult::Service {
                    handle,
                    service_uuid_16: svc_uuid_16,
                    end_handle,
                })
            }
            BT_UUID_GATT_CHRC_VAL => {
                // Characteristic: char_uuid(bstr), value_handle(uint), properties(uint)
                let char_uuid_bytes = d.bytes().map_err(|_| BleError::RpcError)?;
                let value_handle = d.u16().map_err(|_| BleError::RpcError)?;
                let properties = d.u8().map_err(|_| BleError::RpcError)?;

                let char_uuid_16 =
                    if char_uuid_bytes.len() >= 4 && char_uuid_bytes[0] == cgm::BT_UUID_TYPE_16 {
                        u16::from_le_bytes([char_uuid_bytes[2], char_uuid_bytes[3]])
                    } else {
                        0
                    };

                Ok(GattDiscoverResult::Characteristic {
                    handle,
                    char_uuid_16,
                    value_handle,
                    properties,
                })
            }
            _ => {
                // Include or unknown — treat as descriptor
                Ok(GattDiscoverResult::Descriptor {
                    handle,
                    uuid_16: attr_uuid_16,
                })
            }
        }
    }

    /// Decode a GATT notification from raw CBOR payload.
    ///
    /// Wire format (CONFIG_BT_MAX_CONN=1, so no conn encoded):
    ///   scratchpad_size(uint), params_ptr(uint), data(bstr | null)
    ///
    /// When data is CBOR null, it means the subscription was terminated
    /// (e.g., peripheral disconnected or unsubscribed). In that case,
    /// `data_len` is set to 0.
    fn decode_notification(payload: &[u8]) -> Result<GattNotificationData, BleError> {
        let mut d = minicbor::decode::Decoder::new(payload);

        let _scratchpad_size = d.u32().map_err(|_| BleError::RpcError)?;
        let params_ptr = d.u64().map_err(|_| BleError::RpcError)?;

        let mut data = [0u8; 128];
        let data_len;

        // data can be CBOR null (subscription ended) or a byte string.
        if d.datatype().map_err(|_| BleError::RpcError)? == minicbor::data::Type::Null {
            let _ = d.null();
            data_len = 0;
        } else {
            let data_bytes = d.bytes().map_err(|_| BleError::RpcError)?;
            data_len = core::cmp::min(data_bytes.len(), data.len());
            data[..data_len].copy_from_slice(&data_bytes[..data_len]);
        }

        Ok(GattNotificationData {
            params_ptr,
            data,
            data_len,
        })
    }

    /// Decode a scan result from raw CBOR payload.
    ///
    /// Wire format (encoded by the server):
    ///   scratchpad_size(uint), bt_addr_le_t(bytes[7]: type+addr),
    ///   sid(u8), rssi(i8), tx_power(i8), adv_type(u8), adv_props(u16),
    ///   interval(u16), primary_phy(u8), secondary_phy(u8), ad_data(bytes)
    fn decode_scan_result(payload: &[u8]) -> Result<ScanResultData, BleError> {
        let mut d = minicbor::decode::Decoder::new(payload);

        // First field is scratchpad_size — skip it.
        let _scratchpad_size = d.u32().map_err(|_| BleError::RpcError)?;

        // bt_addr_le_t is encoded as a 7-byte buffer: [type, addr[0..6]]
        let addr_le_bytes = d.bytes().map_err(|_| BleError::RpcError)?;
        if addr_le_bytes.len() < 7 {
            return Err(BleError::RpcError);
        }
        let addr_type = addr_le_bytes[0];
        let mut addr = [0u8; 6];
        addr.copy_from_slice(&addr_le_bytes[1..7]);

        let sid = d.u8().map_err(|_| BleError::RpcError)?;
        let rssi = d.i8().map_err(|_| BleError::RpcError)?;
        let tx_power = d.i8().map_err(|_| BleError::RpcError)?;
        let adv_type = d.u8().map_err(|_| BleError::RpcError)?;
        let adv_props = d.u16().map_err(|_| BleError::RpcError)?;
        let interval = d.u16().map_err(|_| BleError::RpcError)?;
        let primary_phy = d.u8().map_err(|_| BleError::RpcError)?;
        let secondary_phy = d.u8().map_err(|_| BleError::RpcError)?;
        let ad_data_bytes = d.bytes().map_err(|_| BleError::RpcError)?;

        let mut ad_data = [0u8; 64];
        let ad_len = core::cmp::min(ad_data_bytes.len(), 64);
        ad_data[..ad_len].copy_from_slice(&ad_data_bytes[..ad_len]);

        Ok(ScanResultData {
            addr_type,
            addr,
            sid,
            rssi,
            tx_power,
            adv_type,
            adv_props,
            interval,
            primary_phy,
            secondary_phy,
            ad_data,
            ad_data_len: ad_len,
        })
    }

    // ========================================================================
    // Raw packet receive (for receiving callback events from server)
    // ========================================================================

    /// Receive and return raw bytes from the transport.
    ///
    /// This is a low-level helper for tests that need to observe server-initiated
    /// events (e.g., scan results, connection events, GATT notifications) which
    /// arrive asynchronously. Returns the number of bytes read.
    pub async fn receive_raw(&mut self, buffer: &mut [u8]) -> Result<usize, BleError> {
        self.client
            .transport_read(buffer)
            .await
            .map_err(|_| BleError::RpcError)
    }
}

/// LE scan parameters matching Zephyr's `struct bt_le_scan_param`.
#[derive(Debug, Clone, Copy)]
pub struct BtLeScanParam {
    /// Scan type: 0 = passive, 1 = active.
    pub scan_type: u8,
    /// Bit-field of scanning options.
    pub options: u8,
    /// Scan interval (N * 0.625 ms).
    pub interval: u16,
    /// Scan window (N * 0.625 ms).
    pub window: u16,
    /// Scan timeout (N * 10 ms). 0 = no timeout.
    pub timeout: u16,
    /// Scan interval LE Coded PHY (N * 0.625 ms). 0 = same as 1M.
    pub interval_coded: u16,
    /// Scan window LE Coded PHY (N * 0.625 ms). 0 = same as 1M.
    pub window_coded: u16,
}

impl Default for BtLeScanParam {
    fn default() -> Self {
        Self {
            scan_type: 1, // Active scan
            options: 0,
            interval: 0x0060, // 60ms
            window: 0x0030,   // 30ms
            timeout: 0,
            interval_coded: 0,
            window_coded: 0,
        }
    }
}

/// BLE scan type constants
pub const BT_LE_SCAN_TYPE_PASSIVE: u8 = 0;
pub const BT_LE_SCAN_TYPE_ACTIVE: u8 = 1;

/// GATT discover types matching Zephyr's `enum bt_gatt_discover_type`.
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum BtGattDiscoverType {
    /// Discover Primary Services
    PrimaryService = 0,
    /// Discover Secondary Services
    SecondaryService = 1,
    /// Discover Included Services
    Include = 2,
    /// Discover Characteristics
    Characteristic = 3,
    /// Discover Descriptors
    Descriptor = 4,
    /// Discover Attributes (any type)
    Attribute = 5,
    /// Discover Standard Characteristic Descriptor
    StdCharDesc = 6,
}

/// Parameters for GATT discovery.
pub struct BtGattDiscoverParams {
    /// UUID to discover. Encoded as raw bytes (Zephyr C struct format).
    /// For 16-bit UUIDs: [type, padding(0x00), val_lo, val_hi] — matches `struct bt_uuid_16`.
    pub uuid: [u8; 4],
    /// Start handle for discovery range.
    pub start_handle: u16,
    /// End handle for discovery range.
    pub end_handle: u16,
    /// Discovery type.
    pub discover_type: BtGattDiscoverType,
}

/// Parameters for GATT read (single handle mode).
pub struct BtGattReadParams {
    /// Handle to read from.
    pub handle: u16,
    /// Offset within the attribute value.
    pub offset: u16,
}

/// Parameters for GATT subscribe (notifications/indications).
pub struct BtGattSubscribeParams {
    /// Whether a notify callback is present.
    pub has_notify: bool,
    /// Value handle of the characteristic.
    pub value_handle: u16,
    /// CCC descriptor handle.
    pub ccc_handle: u16,
    /// Subscription value: 1 = notifications, 2 = indications.
    pub value: u16,
    /// Minimum required security level (bt_security_t).
    /// Only encoded when the server has CONFIG_BT_SMP enabled.
    /// 0 = BT_SECURITY_L0 (no security), 1 = BT_SECURITY_L1, etc.
    pub min_security: u8,
    /// Atomic flags (typically 0).
    pub flags: u16,
}

/// CCC value for enabling notifications
pub const BT_GATT_CCC_NOTIFY: u16 = 0x0001;

/// CCC value for enabling indications
pub const BT_GATT_CCC_INDICATE: u16 = 0x0002;

// ============================================================================
// BLE Event types (decoded from server-initiated Command packets)
// ============================================================================

/// Scan result data decoded from a BtLeScanCbRecvRpcCmd event.
#[derive(Debug, Clone)]
pub struct ScanResultData {
    /// Address type (e.g., 0x00 = public, 0x01 = random).
    pub addr_type: u8,
    /// 6-byte BLE address (little-endian, as on the wire).
    pub addr: [u8; 6],
    /// SID (advertising set identifier).
    pub sid: u8,
    /// Received signal strength (dBm).
    pub rssi: i8,
    /// Transmit power (dBm).
    pub tx_power: i8,
    /// Advertising PDU type.
    pub adv_type: u8,
    /// Advertising properties bitmask.
    pub adv_props: u16,
    /// Advertising interval.
    pub interval: u16,
    /// Primary PHY.
    pub primary_phy: u8,
    /// Secondary PHY.
    pub secondary_phy: u8,
    /// Raw advertising data (AD structures).
    pub ad_data: [u8; 64],
    /// Number of valid bytes in `ad_data`.
    pub ad_data_len: usize,
}

impl ScanResultData {
    /// Parse the device name from advertising data (AD type 0x08 = Shortened, 0x09 = Complete Local Name).
    pub fn device_name(&self) -> Option<&str> {
        let data = &self.ad_data[..self.ad_data_len];
        let mut i = 0;
        while i < data.len() {
            let len = data[i] as usize;
            if len == 0 || i + 1 + len > data.len() {
                break;
            }
            let ad_type = data[i + 1];
            if ad_type == 0x08 || ad_type == 0x09 {
                return core::str::from_utf8(&data[i + 2..i + 1 + len]).ok();
            }
            i += len + 1;
        }
        None
    }

    /// Check if a 16-bit service UUID is present in the advertising data
    /// (AD types 0x02 = Incomplete or 0x03 = Complete List of 16-bit Service UUIDs).
    pub fn has_service_uuid_16(&self, uuid: u16) -> bool {
        let data = &self.ad_data[..self.ad_data_len];
        let mut i = 0;
        while i < data.len() {
            let len = data[i] as usize;
            if len == 0 || i + 1 + len > data.len() {
                break;
            }
            let ad_type = data[i + 1];
            if ad_type == 0x02 || ad_type == 0x03 {
                // Each UUID is 2 bytes (little-endian)
                let uuid_data = &data[i + 2..i + 1 + len];
                let mut j = 0;
                while j + 1 < uuid_data.len() {
                    let found = u16::from_le_bytes([uuid_data[j], uuid_data[j + 1]]);
                    if found == uuid {
                        return true;
                    }
                    j += 2;
                }
            }
            i += len + 1;
        }
        false
    }

    /// Convert the address into a `BtAddrLe` for use with `bt_conn_le_create`.
    pub fn to_addr_le(&self) -> BtAddrLe {
        BtAddrLe {
            addr_type: self.addr_type,
            addr: self.addr,
        }
    }
}

/// Connection event data from BtConnCbConnectedCallRpcCmd.
#[derive(Debug, Clone)]
pub struct ConnectionEvent {
    /// HCI error code. 0 = success.
    pub err: u8,
}

/// Disconnection event data from BtConnCbDisconnectedCallRpcCmd.
#[derive(Debug, Clone)]
pub struct DisconnectionEvent {
    /// HCI reason code.
    pub reason: u8,
}

// ============================================================================
// GATT Event types
// ============================================================================

/// BT_GATT_ITER_STOP — returned by the client to stop iteration.
pub const BT_GATT_ITER_STOP: u8 = 0;

/// BT_GATT_ITER_CONTINUE — returned by the client to continue iteration.
pub const BT_GATT_ITER_CONTINUE: u8 = 1;

/// Attribute UUID types used to distinguish service vs characteristic in
/// GATT discovery callbacks.
const BT_UUID_GATT_PRIMARY_VAL: u16 = 0x2800;
const BT_UUID_GATT_SECONDARY_VAL: u16 = 0x2801;
const BT_UUID_GATT_INCLUDE_VAL: u16 = 0x2802;
const BT_UUID_GATT_CHRC_VAL: u16 = 0x2803;

/// Decoded GATT discovery result from a `BtGattDiscoverCallbackRpcCmd` event.
#[derive(Debug, Clone)]
pub enum GattDiscoverResult {
    /// Discovery complete (attr == NULL from server).
    Complete,
    /// Primary or secondary service found.
    Service {
        /// Attribute handle.
        handle: u16,
        /// Service UUID (raw 16-bit value, or 0 if 128-bit).
        service_uuid_16: u16,
        /// End handle of the service group.
        end_handle: u16,
    },
    /// Characteristic declaration found.
    Characteristic {
        /// Attribute handle (declaration handle).
        handle: u16,
        /// Characteristic UUID (raw 16-bit value, or 0 if 128-bit).
        char_uuid_16: u16,
        /// Value handle of the characteristic.
        value_handle: u16,
        /// Characteristic properties (read, write, notify, etc.).
        properties: u8,
    },
    /// Descriptor found (e.g., CCC).
    Descriptor {
        /// Attribute handle.
        handle: u16,
        /// Descriptor UUID (raw 16-bit value).
        uuid_16: u16,
    },
}

/// Decoded GATT notification data from a `BtGattSubscribeParamsNotifyRpcCmd` event.
#[derive(Debug, Clone)]
pub struct GattNotificationData {
    /// The params pointer echoed back from the server (for matching subscriptions).
    pub params_ptr: u64,
    /// Raw notification payload bytes.
    pub data: [u8; 128],
    /// Number of valid bytes in `data`.
    pub data_len: usize,
}

// ============================================================================
// Connection create parameters
// ============================================================================

/// Parameters for `bt_conn_le_create` — controls scanning behavior during connection.
#[derive(Debug, Clone, Copy)]
pub struct BtConnLeCreateParam {
    /// Options bitmask (BT_CONN_LE_OPT_*).
    pub options: u32,
    /// Scan interval (N * 0.625 ms).
    pub interval: u16,
    /// Scan window (N * 0.625 ms).
    pub window: u16,
    /// Scan interval for LE Coded PHY. 0 = same as 1M.
    pub interval_coded: u16,
    /// Scan window for LE Coded PHY. 0 = same as 1M.
    pub window_coded: u16,
    /// Connection initiation timeout (N * 10 ms). 0 = no timeout.
    pub timeout: u16,
}

impl Default for BtConnLeCreateParam {
    fn default() -> Self {
        Self {
            options: 0,
            interval: 0x0060, // 60ms
            window: 0x0030,   // 30ms
            interval_coded: 0,
            window_coded: 0,
            timeout: 0,
        }
    }
}

/// LE connection parameters for `bt_conn_le_create`.
#[derive(Debug, Clone, Copy)]
pub struct BtLeConnParam {
    /// Minimum connection interval (N * 1.25 ms).
    pub interval_min: u16,
    /// Maximum connection interval (N * 1.25 ms).
    pub interval_max: u16,
    /// Peripheral latency (number of connection events to skip).
    pub latency: u16,
    /// Supervision timeout (N * 10 ms).
    pub timeout: u16,
}

impl Default for BtLeConnParam {
    fn default() -> Self {
        Self {
            interval_min: 24, // 30 ms
            interval_max: 40, // 50 ms
            latency: 0,
            timeout: 400, // 4 s
        }
    }
}
