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
use crate::packet::{self, CommandId, NrfRpcPacket};
use crate::{AsyncTransport, RpcClient, RpcError};

const BT_RPC_GROUP_ID: u8 = 0x0;
const RPC_UTILS_GROUP_ID: u8 = 0x1;

// ============================================================================
// Ble Struct
// ============================================================================

#[repr(C)]
#[derive(code_gen_helpers::CommandId)]
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
    RpcError,
    InvalidParameter,
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
        let mut buffer = [0u8; 16]; // Allocate a buffer for CBOR encoding (adjust size as needed)
        let cbor_args = CborPayloadBuilder::new(&mut buffer); // No arguments for bt_enable
        let payload = cbor_args.build().map_err(|_| BleError::InvalidParameter)?;

        let packet = NrfRpcPacket::<packet::Command<BtEnableRpcCmd>>::new(0, 0, 0, 0, payload);
        self.client
            .send_packet(packet)
            .await
            .map_err(|_| BleError::RpcError)
    }
}
