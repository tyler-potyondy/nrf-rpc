#![no_std]

#[cfg(test)]
extern crate std;

pub mod ble;
mod cbor_encoding;
mod decoding;
#[doc(hidden)]
pub mod packet;
pub mod transport;
pub mod uart_transport;

pub use transport::{AsyncTransport, RpcRxTransportPacket, RpcTxTransportPacket, TransportError};

use cbor_encoding::CborError;

use crate::{
    decoding::{ParsedNrfRpcPacket, ParsedPayload},
    packet::{DstGroupId, MaxVersion, MinVersion, NrfRpcPacket, SrcGroupId},
};

/// RPC client errors
#[derive(Debug)]
pub enum RpcError {
    Transport,
    Cbor(CborError),
    InvalidResponse,
    Timeout,
    NoResponse,
}

const NRF_RPC_PROTOCOL_VERSION_MIN: MinVersion = MinVersion::new(0);

const NRF_RPC_PROTOCOL_VERSION_MAX: MaxVersion = MaxVersion::new(0);

impl core::fmt::Display for RpcError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            RpcError::Transport => write!(f, "Transport error"),
            RpcError::Cbor(e) => write!(f, "CBOR error: {}", e),
            RpcError::InvalidResponse => write!(f, "Invalid response"),
            RpcError::Timeout => write!(f, "Timeout"),
            RpcError::NoResponse => write!(f, "No response received"),
        }
    }
}

impl From<CborError> for RpcError {
    fn from(e: CborError) -> Self {
        RpcError::Cbor(e)
    }
}

/// NRF RPC Client
///
/// Generic over a transport. The transport can be any implementation
/// of the AsyncTransport trait (e.g., UART, IPC, USB).
pub struct RpcClient<T: AsyncTransport> {
    transport: T,
    bt_rpc_group_id: u8,
    rpc_utils_group_id: u8,
    context_id: u8,
}

impl<T: AsyncTransport> RpcClient<T> {
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            bt_rpc_group_id: 0x0,
            rpc_utils_group_id: 0x1,
            context_id: 0,
        }
    }

    /// Initialize RPC client by registering bt_rpc and rpc_utils groups
    pub async fn init(&mut self) -> Result<(), RpcError> {
        let mut buffer = [0u8; 64];
        let bt_rpc_init_packet_payload = packet::InitPacketPayload::new(
            &mut buffer,
            NRF_RPC_PROTOCOL_VERSION_MAX,
            NRF_RPC_PROTOCOL_VERSION_MIN,
            "bt_rpc",
        )
        .expect("Failed to build bt_rpc init packet");

        let unspecified_dst_group_id =
            DstGroupId::try_from(0xFF).expect("Invalid destination group ID");

        let bt_rpc_group_id =
            SrcGroupId::try_from(self.bt_rpc_group_id).expect("Invalid source group ID");

        let rpc_utils_group_id =
            SrcGroupId::try_from(self.rpc_utils_group_id).expect("Invalid source group ID");

        let bt_rpc_init_packet = packet::NrfRpcPacket::<'_, packet::Init>::new(
            bt_rpc_group_id,
            unspecified_dst_group_id,
            bt_rpc_init_packet_payload,
        );

        self.send_packet(bt_rpc_init_packet)
            .await
            .expect("Failed to send bt_rpc init packet");

        let bt_rpc_init_packet_payload = packet::InitPacketPayload::new(
            &mut buffer,
            NRF_RPC_PROTOCOL_VERSION_MAX,
            NRF_RPC_PROTOCOL_VERSION_MIN,
            "rpc_utils",
        )
        .expect("Failed to build bt_rpc init packet");

        let bt_rpc_init_packet = packet::NrfRpcPacket::<'_, packet::Init>::new(
            rpc_utils_group_id,
            unspecified_dst_group_id,
            bt_rpc_init_packet_payload,
        );

        self.send_packet(bt_rpc_init_packet)
            .await
            .expect("Failed to send rpc_utils init packet");

        Ok(())
    }

    // Accessor methods for internal use by command modules
    pub(crate) fn context_id(&self) -> u8 {
        self.context_id
    }

    pub(crate) fn bt_rpc_group_id(&self) -> u8 {
        self.bt_rpc_group_id
    }

    pub(crate) async fn send_packet<'a, P: crate::packet::NrfRpcPacketType<'a>>(
        &mut self,
        packet: NrfRpcPacket<'a, P>,
    ) -> Result<(), RpcError> {
        let mut buffer = [0u8; 256];
        let mut rpc_transport_buf = T::TxTransportPacket::<'_>::new(&mut buffer);

        packet
            .write_into(&mut rpc_transport_buf)
            .expect("Failed to write packet into transport buffer");

        let encoded_buf = rpc_transport_buf
            .encode_packet()
            .expect("Failed to encode packet");

        let encoded_buf_slice: &mut [u8] = encoded_buf.into();

        self.transport
            .write(encoded_buf_slice)
            .await
            .expect("Failed to write packet to transport");

        Ok(())
    }

    /// Send a command packet and decode an i32 CBOR return value from the response payload.
    pub(crate) async fn send_command_and_get_i32(
        &mut self,
        packet: NrfRpcPacket<'_, crate::packet::Command>,
    ) -> Result<i32, RpcError> {
        // Send the command
        self.send_packet(packet)
            .await
            .expect("Failed to send packet");

        let mut retry_count = 3;

        for _ in 0..retry_count {
            // Receive the corresponding response
            let mut buffer = [0u8; 256];
            let recv_packet_list = self.receive_packet(&mut buffer).await?;

            for recv_packet in recv_packet_list.into_iter().flatten() {
                if let ParsedPayload::Cbor(payload) = recv_packet.payload {
                    return Ok(self
                        .decode_i32_response(payload.into())
                        .expect("Failed to decode i32 response"));
                } else {
                    // continue
                }
            }
        }

        Err(RpcError::NoResponse)
    }

    pub(crate) async fn receive_packet<'a>(
        &mut self,
        output: &'a mut [u8; 256],
    ) -> Result<[Option<ParsedNrfRpcPacket<'a>>; 5], RpcError> {
        // Get the raw packet from the "wire"
        let len = self
            .transport
            .read(output)
            .await
            .expect("Failed to receive packet");

        let mut output_pkt_list: [Option<ParsedNrfRpcPacket<'a>>; 5] = [const { None }; 5];
        let mut packet_index = 0;

        let mut remaining_buffer = &mut output[..len];
        while remaining_buffer.len() > 0 {
            let (raw_packet, next_remaining_buffer) =
                T::RxTransportPacket::new(remaining_buffer).map_err(|_| RpcError::Transport)?;

            let decoded_packet = raw_packet
                .decode_raw_packet()
                .map_err(|_| RpcError::InvalidResponse)?;

            let decoded_packet_slice = decoded_packet.into();
            let parsed_packet = ParsedNrfRpcPacket::try_from(decoded_packet_slice)
                .map_err(|_| RpcError::InvalidResponse)?;

            output_pkt_list[packet_index] = Some(parsed_packet);
            packet_index += 1;

            if let Some(next_buffer) = next_remaining_buffer {
                remaining_buffer = next_buffer;
            } else {
                break;
            }
        }
        Ok(output_pkt_list)
    }

    // pub(crate) async fn send_command(&mut self, packet: &[u8]) -> Result<i32, RpcError> {
    //     self.send_packet(packet).await?;

    //     let mut response_buf = [0u8; 256];
    //     let len = self.receive_packet(&mut response_buf).await?;

    //     //if len < 5 {
    //     //    return Err(RpcError::InvalidResponse);
    //     //}
    //     Ok(5)
    //     // let packet_type = response_buf[0] & 0x7F;
    //     // if packet_type != 0x01 {
    //     //     return Err(RpcError::InvalidResponse);
    //     // }

    //     // let payload = &response_buf[5..len];
    //     // self.decode_i32_response(payload)
    // }

    fn decode_i32_response(&self, payload: &[u8]) -> Result<i32, RpcError> {
        use minicbor::decode::Decoder;

        let mut decoder = Decoder::new(payload);
        decoder.i32().map_err(|_| RpcError::InvalidResponse)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate alloc;
    use alloc::format;

    #[test]
    fn test_rpc_error_display() {
        let err = RpcError::Transport;
        assert_eq!(format!("{}", err), "Transport error");
    }
}
