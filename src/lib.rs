pub mod ble;
#[doc(hidden)]
pub mod packet;
mod transport;

pub use transport::{AsyncTransport, TransportError};

use packet::{CborError, PacketBuilder};

/// RPC client errors
#[derive(Debug)]
pub enum RpcError {
    Transport,
    Cbor(CborError),
    InvalidResponse,
    Timeout,
}

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

/// NRF RPC Client
///
/// Generic over a transport. The transport can be any implementation
/// of the AsyncTransport trait (e.g., UART, IPC, USB).
pub(crate) struct RpcClient<T: AsyncTransport> {
    transport: T,
    bt_rpc_group_id: u8,
    rpc_utils_group_id: u8,
    context_id: u8,
}

impl<T: AsyncTransport> RpcClient<T> {
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            bt_rpc_group_id: 0xFF,
            rpc_utils_group_id: 0xFF,
            context_id: 0,
        }
    }

    /// Initialize RPC client by registering bt_rpc and rpc_utils groups
    pub async fn init(&mut self) -> Result<(), RpcError> {
        let bt_rpc_init = PacketBuilder::<64>::new().init(0x00, "bt_rpc");
        self.send_packet(bt_rpc_init.as_slice()).await?;

        let rpc_utils_init = PacketBuilder::<64>::new().init(0x01, "rpc_utils");
        self.send_packet(rpc_utils_init.as_slice()).await?;

        let mut response_buf = [0u8; 256];
        
        let len = self.receive_packet(&mut response_buf).await?;
        if len >= 5 && response_buf[0] == 0x04 {
            self.bt_rpc_group_id = response_buf[4];
        }

        let len = self.receive_packet(&mut response_buf).await?;
        if len >= 5 && response_buf[0] == 0x04 {
            self.rpc_utils_group_id = response_buf[4];
        }

        Ok(())
    }

    // Accessor methods for internal use by command modules
    pub(crate) fn context_id(&self) -> u8 {
        self.context_id
    }

    pub(crate) fn bt_rpc_group_id(&self) -> u8 {
        self.bt_rpc_group_id
    }

    pub(crate) async fn send_packet(&mut self, packet: &[u8]) -> Result<(), RpcError> {
        self.transport.write(packet).await.map_err(|_| RpcError::Transport)?;
        Ok(())
    }

    pub(crate) async fn receive_packet(&mut self, output: &mut [u8]) -> Result<usize, RpcError> {
        self.transport.read(output).await.map_err(|_| RpcError::Transport)
    }

    pub(crate) async fn send_command(&mut self, packet: &[u8]) -> Result<i32, RpcError> {
        self.send_packet(packet).await?;

        let mut response_buf = [0u8; 256];
        let len = self.receive_packet(&mut response_buf).await?;

        if len < 5 {
            return Err(RpcError::InvalidResponse);
        }

        let packet_type = response_buf[0] & 0x7F;
        if packet_type != 0x01 {
            return Err(RpcError::InvalidResponse);
        }

        let payload = &response_buf[5..len];
        self.decode_i32_response(payload)
    }

    fn decode_i32_response(&self, payload: &[u8]) -> Result<i32, RpcError> {
        use minicbor::decode::Decoder;
        
        let mut decoder = Decoder::new(payload);
        decoder.i32().map_err(|_| RpcError::InvalidResponse)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rpc_error_display() {
        let err = RpcError::Transport;
        assert_eq!(format!("{}", err), "Transport error");
    }
}
