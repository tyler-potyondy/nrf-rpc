#![no_std]

#[cfg(test)]
extern crate std;

pub mod ble;
mod cbor_encoding;
mod decoding;
#[doc(hidden)]
pub mod packet;
mod transport;
mod uart_transport;
pub use uart_transport::NrfRpcUartTransport;
pub use uart_transport::UartTransport;

pub use transport::{AsyncTransport, TransportError};

use cbor_encoding::CborError;

use crate::packet::{DstGroupId, MaxVersion, MinVersion, NrfRpcPacket, SrcGroupId};

/// RPC client errors
#[derive(Debug)]
pub enum RpcError {
    Transport,
    Cbor(CborError),
    InvalidResponse,
    Timeout,
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
        }
    }
}

impl From<CborError> for RpcError {
    fn from(e: CborError) -> Self {
        RpcError::Cbor(e)
    }
}

pub struct RpcTransportBuffer<'a> {
    buffer: &'a mut [u8],
    pos: usize,
}

impl<'a> From<RpcTransportBuffer<'a>> for &'a [u8] {
    fn from(value: RpcTransportBuffer<'a>) -> Self {
        &value.buffer[..value.pos]
    }
}

impl<'a> RpcTransportBuffer<'a> {
    pub fn new(buffer: &'a mut [u8]) -> Self {
        Self { buffer, pos: 0 }
    }

    pub fn remaining_len(&self) -> usize {
        self.buffer.len() - self.pos
    }

    pub fn write_slice_into_or_err(&mut self, data: &[u8]) -> Result<(), ()> {
        if self.pos + data.len() > self.buffer.len() {
            return Err(());
        }
        self.buffer[self.pos..self.pos + data.len()].copy_from_slice(data);
        self.pos += data.len();
        Ok(())
    }

    pub fn write_byte_into_or_err(&mut self, data: u8) -> Result<(), ()> {
        if self.pos + 1 > self.buffer.len() {
            return Err(());
        }
        self.buffer[self.pos] = data;
        self.pos += 1;
        Ok(())
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

        self.send_packet(bt_rpc_init_packet).await?;

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

        self.send_packet(bt_rpc_init_packet).await
    }

    // Accessor methods for internal use by command modules
    pub(crate) fn context_id(&self) -> u8 {
        self.context_id
    }

    pub(crate) fn bt_rpc_group_id(&self) -> u8 {
        self.bt_rpc_group_id
    }

    pub(crate) async fn send_packet<'a, P: crate::packet::NrfRpcPacketType>(
        &mut self,
        packet: NrfRpcPacket<'a, P>,
    ) -> Result<(), RpcError> {
        let mut buffer = [0u8; 256];
        let mut rpc_transport_buf = RpcTransportBuffer::new(&mut buffer);

        packet
            .write_into(&mut rpc_transport_buf)
            .map_err(|_| RpcError::InvalidResponse)?;

        self.transport
            .write(rpc_transport_buf.into())
            .await
            .map_err(|_| RpcError::Transport)?;

        // (todo) error handling
        Ok(())
    }

    pub(crate) async fn receive_packet(&mut self, output: &mut [u8]) -> Result<usize, RpcError> {
        self.transport
            .read(output)
            .await
            .map_err(|_| RpcError::Transport)
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
