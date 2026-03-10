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
enum BleCommandId {
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
            CommandId::try_from(BleCommandId::BtEnableRpcCmd as u8).expect("Invalid command ID"),
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

    pub async fn bt_begin_advertising(&mut self) -> Result<(), BleError> {
        Ok(())
    }
}
