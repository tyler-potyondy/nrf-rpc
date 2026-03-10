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

pub use crate::ble_types::{BT_LE_AD_GENERAL, BT_LE_AD_NO_BREDR, BtAddrLe, BtData, BtLeAdvParam};
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

/// BLE RPC client
///
/// Encapsulates an RPC client for Bluetooth Low Energy operations.
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
    }

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
            .expect("Failed to send command and get i32");

        if status != 0 {
            return Err(BleError::RpcError);
        }

        Ok(())
    }
}

/** LE scan parameters */
struct bt_le_scan_param {
    /** Scan type. @ref BT_LE_SCAN_TYPE_ACTIVE or @ref BT_LE_SCAN_TYPE_PASSIVE. */
    pub scan_type: u8,

    /** Bit-field of scanning options. */
    pub options: u8,

    /** Scan interval (N * 0.625 ms).
     *
     * @note When @kconfig{CONFIG_BT_SCAN_AND_INITIATE_IN_PARALLEL} is enabled
     *       and the application wants to scan and connect in parallel,
     *       the Bluetooth Controller may require the scan interval used
     *       for scanning and connection establishment to be equal to
     *       obtain the best performance.
     */
    pub interval: u16,

    /** Scan window (N * 0.625 ms)
     *
     * @note When @kconfig{CONFIG_BT_SCAN_AND_INITIATE_IN_PARALLEL} is enabled
     *       and the application wants to scan and connect in parallel,
     *       the Bluetooth Controller may require the scan window used
     *       for scanning and connection establishment to be equal to
     *       obtain the best performance.
     */
    pub window: u16,

    /**
     * @brief Scan timeout (N * 10 ms)
     *
     * Application will be notified by the scan timeout callback.
     * Set zero to disable timeout.
     */
    pub timeout: u16,

    /**
     * @brief Scan interval LE Coded PHY (N * 0.625 MS)
     *
     * Set zero to use same as LE 1M PHY scan interval.
     */
    pub interval_coded: u16,

    /**
     * @brief Scan window LE Coded PHY (N * 0.625 MS)
     *
     * Set zero to use same as LE 1M PHY scan window.
     */
    pub window_coded: u16,
}
