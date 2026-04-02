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

use cbor_encoding::{CborError, CborPayloadBuilder};

use crate::{
    decoding::{ParsedNrfRpcPacket, ParsedPayload},
    packet::{
        DestContextId, DstGroupId, MaxVersion, MinVersion, NrfRpcPacket, SrcGroupId, TypeField,
    },
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

/// A server-initiated event that was received while processing a command response.
/// Stored internally so it can be retrieved later by `receive_server_event`.
struct PendingEvent {
    cmd_id: u8,
    payload: [u8; 128],
    payload_len: usize,
}

const MAX_PENDING_EVENTS: usize = 4;

/// NRF RPC Client
///
/// Generic over a transport. The transport can be any implementation
/// of the AsyncTransport trait (e.g., UART, IPC, USB).
pub struct RpcClient<T: AsyncTransport> {
    transport: T,
    bt_rpc_group_id: u8,
    rpc_utils_group_id: u8,
    context_id: u8,
    /// Ring buffer of pending server events that arrived while we were waiting
    /// for a command response.
    pending_events: [Option<PendingEvent>; MAX_PENDING_EVENTS],
    pending_head: usize,
    pending_count: usize,
}

impl<T: AsyncTransport> RpcClient<T> {
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            bt_rpc_group_id: 0x0,
            rpc_utils_group_id: 0x1,
            context_id: 0,
            pending_events: [const { None }; MAX_PENDING_EVENTS],
            pending_head: 0,
            pending_count: 0,
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

    /// Low-level transport read for observing raw bytes from the server.
    pub(crate) async fn transport_read(&mut self, buffer: &mut [u8]) -> Result<usize, RpcError> {
        self.transport
            .read(buffer)
            .await
            .map_err(|_| RpcError::Transport)
    }

    /// Store a server-initiated Command event for later retrieval.
    fn enqueue_event(&mut self, cmd_id: u8, payload: &[u8]) {
        if self.pending_count >= MAX_PENDING_EVENTS {
            // Queue full — drop the oldest event.
            self.pending_head = (self.pending_head + 1) % MAX_PENDING_EVENTS;
            self.pending_count -= 1;
        }
        let idx = (self.pending_head + self.pending_count) % MAX_PENDING_EVENTS;
        let mut buf = [0u8; 128];
        let len = core::cmp::min(payload.len(), buf.len());
        buf[..len].copy_from_slice(&payload[..len]);
        self.pending_events[idx] = Some(PendingEvent {
            cmd_id,
            payload: buf,
            payload_len: len,
        });
        self.pending_count += 1;
    }

    /// Pop the oldest pending event, if any.
    fn dequeue_event(&mut self) -> Option<(u8, [u8; 128], usize)> {
        if self.pending_count == 0 {
            return None;
        }
        let idx = self.pending_head;
        if let Some(evt) = self.pending_events[idx].take() {
            self.pending_head = (self.pending_head + 1) % MAX_PENDING_EVENTS;
            self.pending_count -= 1;
            Some((evt.cmd_id, evt.payload, evt.payload_len))
        } else {
            None
        }
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

    /// Send a void response (ACK) for a server-initiated Command.
    ///
    /// When the server sends a Command (e.g., scan result, connection event),
    /// it expects a Response packet with an empty CBOR payload back.
    ///
    /// `dst_context_id` = server's source context ID from the incoming Command.
    /// `our_group_id` = our group ID (incoming Command's dst_group_id).
    /// `server_group_id` = server's group ID (incoming Command's src_group_id).
    pub(crate) async fn send_void_response(
        &mut self,
        dst_context_id: u8,
        our_group_id: u8,
        server_group_id: u8,
    ) -> Result<(), RpcError> {
        let mut cbor_buffer = [0u8; 8];
        let builder = CborPayloadBuilder::new(&mut cbor_buffer);
        let payload = builder.build().expect("Failed to build empty CBOR payload");

        let packet = NrfRpcPacket::<crate::packet::Response>::new(
            DestContextId::try_from(dst_context_id).expect("Invalid dest context ID"),
            SrcGroupId::try_from(our_group_id).expect("Invalid source group ID"),
            DstGroupId::try_from(server_group_id).expect("Invalid dest group ID"),
            payload,
        );

        self.send_packet(packet).await
    }

    /// Send a command packet and expect a void response (no CBOR return value).
    ///
    /// The server sends `nrf_rpc_rsp_send_void` which is a response packet with
    /// an empty CBOR payload (just the null terminator). We consume it and return Ok.
    ///
    /// If server-initiated Command packets (events) arrive interleaved with the
    /// response, they are ACKed with void responses and skipped.
    pub(crate) async fn send_command_void(
        &mut self,
        packet: NrfRpcPacket<'_, crate::packet::Command>,
    ) -> Result<(), RpcError> {
        // Send the command
        self.send_packet(packet)
            .await
            .expect("Failed to send packet");

        let retry_count = 5;
        for i in 0..retry_count {
            if i > 0 {
                self.transport.delay_ms(100).await;
            }

            let mut buffer = [0u8; 256];
            let recv_packet_list = match self.receive_packet(&mut buffer).await {
                Ok(list) => list,
                Err(_) => {
                    // Transport or parse error (e.g. partial HDLC frame).
                    // Retry – the rest of the data may arrive on the next read.
                    continue;
                }
            };

            let mut got_void_response = false;

            for recv_packet in recv_packet_list.into_iter().flatten() {
                match recv_packet.packet_type {
                    TypeField::Command => {
                        // Server-initiated event arrived while waiting for our response.
                        // ACK it with a void response and save for later retrieval.
                        let cmd_id: u8 = recv_packet.command_id.into();
                        let src_ctx: u8 = recv_packet.src_context_id.into();
                        let src_grp: u8 = recv_packet.src_group_id.into();
                        let dst_grp: u8 = recv_packet.dst_group_id.into();
                        if let ParsedPayload::Cbor(payload) = recv_packet.payload {
                            let payload_bytes: &[u8] = payload.into();
                            self.enqueue_event(cmd_id, payload_bytes);
                        }
                        let _ = self.send_void_response(src_ctx, dst_grp, src_grp).await;
                    }
                    TypeField::Response => {
                        if let ParsedPayload::Cbor(_) = recv_packet.payload {
                            got_void_response = true;
                        }
                    }
                    _ => {}
                }
            }

            if got_void_response {
                return Ok(());
            }
        }

        Err(RpcError::NoResponse)
    }

    /// Send a command packet and decode an i32 CBOR return value from the response payload.
    ///
    /// If server-initiated Command packets (events) arrive interleaved with the
    /// response, they are ACKed with void responses and skipped.
    pub(crate) async fn send_command_and_get_i32(
        &mut self,
        packet: NrfRpcPacket<'_, crate::packet::Command>,
    ) -> Result<i32, RpcError> {
        // Send the command
        self.send_packet(packet)
            .await
            .expect("Failed to send packet");

        let retry_count = 5;
        for i in 0..retry_count {
            // Wait before retrying (except for the first attempt)
            if i > 0 {
                self.transport.delay_ms(100).await;
            }

            // Receive the corresponding response
            let mut buffer = [0u8; 256];
            let recv_packet_list = match self.receive_packet(&mut buffer).await {
                Ok(list) => list,
                Err(_) => {
                    // Transport or parse error (e.g. partial HDLC frame).
                    // Retry – the rest of the data may arrive on the next read.
                    continue;
                }
            };

            let mut response_value: Option<i32> = None;

            for recv_packet in recv_packet_list.into_iter().flatten() {
                match recv_packet.packet_type {
                    TypeField::Command => {
                        // Server-initiated event arrived while waiting for our response.
                        // ACK it with a void response and save for later retrieval.
                        let cmd_id: u8 = recv_packet.command_id.into();
                        let src_ctx: u8 = recv_packet.src_context_id.into();
                        let src_grp: u8 = recv_packet.src_group_id.into();
                        let dst_grp: u8 = recv_packet.dst_group_id.into();
                        if let ParsedPayload::Cbor(payload) = recv_packet.payload {
                            let payload_bytes: &[u8] = payload.into();
                            self.enqueue_event(cmd_id, payload_bytes);
                        }
                        let _ = self.send_void_response(src_ctx, dst_grp, src_grp).await;
                    }
                    TypeField::Response => {
                        if response_value.is_none() {
                            if let ParsedPayload::Cbor(payload) = recv_packet.payload {
                                response_value = Some(self
                                    .decode_i32_response(payload.into())
                                    .expect("Failed to decode i32 response"));
                            }
                        }
                    }
                    _ => {}
                }
            }

            if let Some(val) = response_value {
                return Ok(val);
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

    /// Receive a server-initiated event (Command packet), ACK it, and return the
    /// event's command ID and CBOR payload.
    ///
    /// The payload is copied into `event_payload_out`. Returns `(command_id, payload_len)`.
    ///
    /// First checks the internal pending-event queue (populated when events arrive
    /// bundled with command responses). If nothing is queued, performs a single
    /// transport read (which may block for the transport's built-in timeout).
    /// Callers should implement their own retry/timeout loop.
    pub(crate) async fn receive_server_event(
        &mut self,
        event_payload_out: &mut [u8],
    ) -> Result<(u8, usize), RpcError> {
        // Check the pending-event queue first.
        if let Some((cmd_id, payload_buf, payload_len)) = self.dequeue_event() {
            let len = core::cmp::min(payload_len, event_payload_out.len());
            event_payload_out[..len].copy_from_slice(&payload_buf[..len]);
            return Ok((cmd_id, len));
        }

        // Nothing queued — do a live transport read.
        let mut buffer = [0u8; 256];
        let recv_packet_list = self.receive_packet(&mut buffer).await?;

        let mut found: Option<(u8, usize)> = None;

        for recv_packet in recv_packet_list.into_iter().flatten() {
            if recv_packet.packet_type == TypeField::Command {
                let cmd_id: u8 = recv_packet.command_id.into();
                let src_ctx: u8 = recv_packet.src_context_id.into();
                let src_grp: u8 = recv_packet.src_group_id.into();
                let dst_grp: u8 = recv_packet.dst_group_id.into();

                if let ParsedPayload::Cbor(payload) = recv_packet.payload {
                    let payload_bytes: &[u8] = payload.into();

                    if found.is_none() {
                        // First Command — return it directly.
                        let len = core::cmp::min(payload_bytes.len(), event_payload_out.len());
                        event_payload_out[..len].copy_from_slice(&payload_bytes[..len]);
                        found = Some((cmd_id, len));
                    } else {
                        // Additional Command — enqueue for later.
                        self.enqueue_event(cmd_id, payload_bytes);
                    }

                    // ACK the event with a void response
                    let _ = self.send_void_response(src_ctx, dst_grp, src_grp).await;
                }
            }
        }

        found.ok_or(RpcError::NoResponse)
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
