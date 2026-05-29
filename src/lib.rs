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

// ============================================================================
// Generic event dispatch traits
// ============================================================================

/// The ACK type to send in response to a server-initiated command.
///
/// The correct value is protocol-specific and must be returned by
/// [`RpcEventDecoder::ack_type`] for each `cmd_id`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AckType {
    /// No-payload void response (most events).
    Void,
    /// Single-byte `u8` response (e.g., `BT_GATT_ITER_CONTINUE = 1` for GATT discovery).
    U8(u8),
    /// Boolean response (e.g., `true` to accept LE parameter update requests).
    Bool(bool),
}

/// Decode server-initiated events for an nRF RPC protocol layer.
///
/// Implement this trait for your protocol's event type, then call
/// [`RpcClient::next_event`] to receive, ACK, and decode events without
/// duplicating transport/queue handling.
///
/// # Example
///
/// ```ignore
/// struct MyDecoder;
/// impl RpcEventDecoder for MyDecoder {
///     type Event = MyEvent;
///     type Error = MyError; // must impl From<RpcError>
///
///     fn ack_type(cmd_id: u8) -> AckType { AckType::Void }
///
///     fn decode(cmd_id: u8, payload: &[u8]) -> Result<Option<MyEvent>, MyError> {
///         match cmd_id {
///             1 => Ok(Some(MyEvent::Foo)),
///             _ => Ok(None), // silently skip unknown commands
///         }
///     }
/// }
///
/// let event = client.next_event::<MyDecoder>().await?;
/// ```
pub trait RpcEventDecoder {
    /// The decoded event type returned to the caller.
    type Event;
    /// Error type; must convert from [`RpcError`] so transport errors propagate.
    type Error: From<RpcError>;

    /// Return the ACK type the protocol requires for `cmd_id`.
    fn ack_type(cmd_id: u8) -> AckType;

    /// Decode the raw CBOR `payload` for `cmd_id`.
    ///
    /// - `Ok(Some(e))` — yield the event to the caller.
    /// - `Ok(None)` — silently consume this event and wait for the next one.
    /// - `Err(e)` — propagate a decode error.
    fn decode(cmd_id: u8, payload: &[u8]) -> Result<Option<Self::Event>, Self::Error>;
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
    /// Optional: a server event cmd_id that requires a bool `true` ACK
    /// instead of the default (void or u8). Set by the BLE module for
    /// `le_param_req` (0x0D) which expects `nrf_rpc_rsp_decode_bool`.
    bool_ack_cmd_id: Option<u8>,
    /// Optional: auto-confirm configuration for SMP passkey_confirm events.
    /// When set, upon receiving a server event matching `auto_confirm_event_cmd_id`,
    /// the client will:
    ///   1. ACK the event with void (normal behavior)
    ///   2. Immediately send an RPC command (`auto_confirm_action_cmd_id`) to the
    ///      server to confirm the passkey/pairing, mimicking the C RPC client's
    ///      inline behavior where the app callback calls bt_conn_auth_passkey_confirm().
    auto_confirm_event_cmd_id: Option<u8>,
    auto_confirm_action_cmd_id: Option<u8>,
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
            bool_ack_cmd_id: None,
            auto_confirm_event_cmd_id: None,
            auto_confirm_action_cmd_id: None,
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

        // The server echoes both Init packets back as part of the nRF RPC group-
        // registration handshake. Drain them now so they don't sit in the RX
        // buffer and get consumed by the first real command (e.g. bt_enable).
        // One receive_packet call is sufficient: the transport typically delivers
        // both echoes together in the same read, and the HDLC accumulation loop
        // will capture all complete frames in that burst.
        let mut drain_buf = [0u8; 256];
        let _ = self.receive_packet(&mut drain_buf).await;

        Ok(())
    }

    // Accessor methods for internal use by command modules
    pub(crate) fn context_id(&self) -> u8 {
        self.context_id
    }

    pub(crate) fn bt_rpc_group_id(&self) -> u8 {
        self.bt_rpc_group_id
    }

    /// Set a server event cmd_id that should be ACKed with `bool true`
    /// instead of the default void/u8 response.
    ///
    /// This is called by the BLE module after auth callbacks are registered,
    /// because `le_param_req` (cmd_id 0x0D) requires a bool response.
    pub(crate) fn set_bool_ack_cmd_id(&mut self, cmd_id: u8) {
        self.bool_ack_cmd_id = Some(cmd_id);
    }

    /// Configure auto-confirm behavior for SMP passkey/pairing events.
    ///
    /// When a server event with `event_cmd_id` is received, the client will:
    ///   1. ACK it normally (void)
    ///   2. Immediately send an RPC command with `action_cmd_id` (empty CBOR
    ///      payload, expects i32 response) to the server.
    ///
    /// This mimics the C RPC client's behavior where the `passkey_confirm`
    /// callback calls `bt_conn_auth_passkey_confirm()` inline.
    pub(crate) fn set_auto_confirm(&mut self, event_cmd_id: u8, action_cmd_id: u8) {
        self.auto_confirm_event_cmd_id = Some(event_cmd_id);
        self.auto_confirm_action_cmd_id = Some(action_cmd_id);
    }

    /// Internal helper: ACK a server event with the appropriate response type.
    ///
    /// If `override_u8` is Some, use u8 as the default ACK type (for GATT callbacks).
    /// If the event's cmd_id matches `self.bool_ack_cmd_id`, always use bool true.
    /// Otherwise, use void.
    ///
    /// If the event's cmd_id matches `self.auto_confirm_event_cmd_id`, after
    /// ACKing, immediately send the confirm command to the server.
    async fn ack_event(
        &mut self,
        cmd_id: u8,
        src_ctx: u8,
        dst_grp: u8,
        src_grp: u8,
        override_u8: Option<u8>,
    ) -> Result<(), RpcError> {
        // Choose ACK type
        if self.bool_ack_cmd_id == Some(cmd_id) {
            self.send_bool_response(src_ctx, dst_grp, src_grp, true)
                .await?;
        } else if let Some(val) = override_u8 {
            self.send_u8_response(src_ctx, dst_grp, src_grp, val)
                .await?;
        } else {
            self.send_void_response(src_ctx, dst_grp, src_grp).await?;
        }

        // Auto-confirm: if this event is the passkey/pairing confirm event,
        // immediately send the confirm command to the server.
        if let (Some(evt_id), Some(action_id)) = (
            self.auto_confirm_event_cmd_id,
            self.auto_confirm_action_cmd_id,
        ) {
            if cmd_id == evt_id {
                // Build an empty-payload command for the confirm action
                let mut cbor_buffer = [0u8; 8];
                let builder = CborPayloadBuilder::new(&mut cbor_buffer);
                let payload = builder.build().expect("Failed to build empty CBOR payload");

                let packet = NrfRpcPacket::<crate::packet::Command>::new(
                    crate::packet::SrcContextId::try_from(self.context_id)
                        .expect("Invalid source context ID"),
                    DestContextId::try_from(0xFF).expect("Invalid dest context ID"),
                    crate::packet::CommandId::try_from(action_id).expect("Invalid command ID"),
                    SrcGroupId::try_from(self.bt_rpc_group_id).expect("Invalid source group ID"),
                    DstGroupId::try_from(self.bt_rpc_group_id).expect("Invalid dest group ID"),
                    payload,
                );

                // Send the confirm command and get i32 result
                // We can't call send_command_and_get_i32 recursively, so do a
                // simple send + receive inline.
                self.send_packet(packet).await?;

                // Read the i32 response (with retry)
                for _retry in 0..5 {
                    let mut buffer = [0u8; 256];
                    let recv_packet_list = match self.receive_packet(&mut buffer).await {
                        Ok(list) => list,
                        Err(_) => continue,
                    };

                    for recv_packet in recv_packet_list.into_iter().flatten() {
                        match recv_packet.packet_type {
                            TypeField::Command => {
                                // Interleaved event during confirm — ACK it
                                let evt_cmd_id: u8 = recv_packet.command_id.into();
                                let evt_src_ctx: u8 = recv_packet.src_context_id.into();
                                let evt_src_grp: u8 = recv_packet.src_group_id.into();
                                let evt_dst_grp: u8 = recv_packet.dst_group_id.into();
                                if let ParsedPayload::Cbor(payload) = recv_packet.payload {
                                    let payload_bytes: &[u8] = payload.into();
                                    self.enqueue_event(evt_cmd_id, payload_bytes);
                                }
                                // ACK appropriately (but don't recurse into auto-confirm)
                                if self.bool_ack_cmd_id == Some(evt_cmd_id) {
                                    let _ = self
                                        .send_bool_response(
                                            evt_src_ctx,
                                            evt_dst_grp,
                                            evt_src_grp,
                                            true,
                                        )
                                        .await;
                                } else {
                                    let _ = self
                                        .send_void_response(evt_src_ctx, evt_dst_grp, evt_src_grp)
                                        .await;
                                }
                            }
                            TypeField::Response => {
                                // Got the confirm result — done
                                break;
                            }
                            _ => {}
                        }
                    }
                    break;
                }
            }
        }

        Ok(())
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

    /// Send a response with a single CBOR uint8 value for a server-initiated Command.
    ///
    /// Some server callbacks (e.g., GATT discover, GATT notify) expect the client
    /// to return a uint8 result (e.g., `BT_GATT_ITER_CONTINUE` or `BT_GATT_ITER_STOP`).
    pub(crate) async fn send_u8_response(
        &mut self,
        dst_context_id: u8,
        our_group_id: u8,
        server_group_id: u8,
        value: u8,
    ) -> Result<(), RpcError> {
        let mut cbor_buffer = [0u8; 16];
        let builder = CborPayloadBuilder::new(&mut cbor_buffer);
        let payload = builder
            .encode_uint_8(value)
            .expect("Failed to encode u8 response")
            .build()
            .expect("Failed to build u8 CBOR payload");

        let packet = NrfRpcPacket::<crate::packet::Response>::new(
            DestContextId::try_from(dst_context_id).expect("Invalid dest context ID"),
            SrcGroupId::try_from(our_group_id).expect("Invalid source group ID"),
            DstGroupId::try_from(server_group_id).expect("Invalid dest group ID"),
            payload,
        );

        self.send_packet(packet).await
    }

    /// Send a response with a CBOR bool value for a server-initiated Command.
    ///
    /// Some server callbacks (e.g., `le_param_req`) expect the client to return
    /// a boolean value. CBOR true = 0xF5, false = 0xF4.
    pub(crate) async fn send_bool_response(
        &mut self,
        dst_context_id: u8,
        our_group_id: u8,
        server_group_id: u8,
        value: bool,
    ) -> Result<(), RpcError> {
        let mut cbor_buffer = [0u8; 16];
        let builder = CborPayloadBuilder::new(&mut cbor_buffer);
        let payload = builder
            .cbor_bool(value)
            .expect("Failed to encode bool response")
            .build()
            .expect("Failed to build bool CBOR payload");

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
        self.send_packet(packet)
            .await
            .expect("Failed to send packet");

        // Read frames until we find the void Response. The server may send
        // interleaved Init or Command frames before the Response (e.g. nRF RPC
        // group re-advertisements after bt_enable). Each receive_packet call
        // blocks on the transport until data arrives — no artificial delay.
        // Transient transport errors (e.g. server not yet ready) are retried;
        // after 20 attempts without a Response we return Err(RpcError::Timeout).
        for _ in 0..20 {
            let mut buffer = [0u8; 256];
            let recv_packet_list = match self.receive_packet(&mut buffer).await {
                Ok(list) => list,
                Err(_) => continue,
            };

            for recv_packet in recv_packet_list.into_iter().flatten() {
                match recv_packet.packet_type {
                    TypeField::Command => {
                        let cmd_id: u8 = recv_packet.command_id.into();
                        let src_ctx: u8 = recv_packet.src_context_id.into();
                        let src_grp: u8 = recv_packet.src_group_id.into();
                        let dst_grp: u8 = recv_packet.dst_group_id.into();
                        if let ParsedPayload::Cbor(payload) = recv_packet.payload {
                            let payload_bytes: &[u8] = payload.into();
                            self.enqueue_event(cmd_id, payload_bytes);
                        }
                        let _ = self
                            .ack_event(cmd_id, src_ctx, dst_grp, src_grp, None)
                            .await;
                    }
                    TypeField::Response => {
                        if let ParsedPayload::Cbor(_) = recv_packet.payload {
                            return Ok(());
                        }
                    }
                    _ => {}
                }
            }
        }

        Err(RpcError::Timeout)
    }

    /// Send a command packet and decode an i32 CBOR return value from the response payload.
    ///
    /// If server-initiated Command packets (events) arrive interleaved with the
    /// response, they are ACKed with void responses and skipped.
    pub(crate) async fn send_command_and_get_i32(
        &mut self,
        packet: NrfRpcPacket<'_, crate::packet::Command>,
    ) -> Result<i32, RpcError> {
        self.send_packet(packet)
            .await
            .expect("Failed to send packet");

        for _ in 0..20 {
            let mut buffer = [0u8; 256];
            let recv_packet_list = match self.receive_packet(&mut buffer).await {
                Ok(list) => list,
                Err(_) => continue,
            };

            let mut i32_result: Option<Result<i32, RpcError>> = None;
            for recv_packet in recv_packet_list.into_iter().flatten() {
                match recv_packet.packet_type {
                    TypeField::Command => {
                        let cmd_id: u8 = recv_packet.command_id.into();
                        let src_ctx: u8 = recv_packet.src_context_id.into();
                        let src_grp: u8 = recv_packet.src_group_id.into();
                        let dst_grp: u8 = recv_packet.dst_group_id.into();
                        if let ParsedPayload::Cbor(payload) = recv_packet.payload {
                            let payload_bytes: &[u8] = payload.into();
                            self.enqueue_event(cmd_id, payload_bytes);
                        }
                        let _ = self
                            .ack_event(cmd_id, src_ctx, dst_grp, src_grp, None)
                            .await;
                    }
                    TypeField::Response => {
                        if i32_result.is_none() {
                            if let ParsedPayload::Cbor(payload) = recv_packet.payload {
                                i32_result = Some(self.decode_i32_response(payload.into()));
                            }
                        }
                    }
                    _ => {}
                }
            }
            if let Some(result) = i32_result {
                return result;
            }
        }

        Err(RpcError::Timeout)
    }

    /// Like `send_command_and_get_i32`, but ACKs any interleaved server Command
    /// events with a CBOR uint8 `event_ack_value` instead of a void response.
    ///
    /// This is needed when starting GATT discovery or subscribe — the server
    /// may send callback events interleaved with the i32 result, and those
    /// callbacks expect a uint8 return code (e.g. BT_GATT_ITER_CONTINUE).
    pub(crate) async fn send_command_and_get_i32_ack_events_u8(
        &mut self,
        packet: NrfRpcPacket<'_, crate::packet::Command>,
        event_ack_value: u8,
    ) -> Result<i32, RpcError> {
        self.send_packet(packet)
            .await
            .expect("Failed to send packet");

        for _ in 0..20 {
            let mut buffer = [0u8; 256];
            let recv_packet_list = match self.receive_packet(&mut buffer).await {
                Ok(list) => list,
                Err(_) => continue,
            };

            let mut i32_result: Option<Result<i32, RpcError>> = None;
            for recv_packet in recv_packet_list.into_iter().flatten() {
                match recv_packet.packet_type {
                    TypeField::Command => {
                        let cmd_id: u8 = recv_packet.command_id.into();
                        let src_ctx: u8 = recv_packet.src_context_id.into();
                        let src_grp: u8 = recv_packet.src_group_id.into();
                        let dst_grp: u8 = recv_packet.dst_group_id.into();
                        if let ParsedPayload::Cbor(payload) = recv_packet.payload {
                            let payload_bytes: &[u8] = payload.into();
                            self.enqueue_event(cmd_id, payload_bytes);
                        }
                        let _ = self
                            .ack_event(cmd_id, src_ctx, dst_grp, src_grp, Some(event_ack_value))
                            .await;
                    }
                    TypeField::Response => {
                        if i32_result.is_none() {
                            if let ParsedPayload::Cbor(payload) = recv_packet.payload {
                                i32_result = Some(self.decode_i32_response(payload.into()));
                            }
                        }
                    }
                    _ => {}
                }
            }
            if let Some(result) = i32_result {
                return result;
            }
        }

        Err(RpcError::Timeout)
    }

    /// Like `send_command_and_get_i32`, but ACKs any interleaved server Command
    /// events with a CBOR **bool** response.
    ///
    /// This is needed for `bt_conn_set_security` — the server may send a
    /// `le_param_req` callback event that expects a bool return (true = accept).
    pub(crate) async fn send_command_and_get_i32_ack_events_bool(
        &mut self,
        packet: NrfRpcPacket<'_, crate::packet::Command>,
        event_ack_value: bool,
    ) -> Result<i32, RpcError> {
        self.send_packet(packet)
            .await
            .expect("Failed to send packet");

        for _ in 0..20 {
            let mut buffer = [0u8; 256];
            let recv_packet_list = match self.receive_packet(&mut buffer).await {
                Ok(list) => list,
                Err(_) => continue,
            };

            let mut i32_result: Option<Result<i32, RpcError>> = None;
            for recv_packet in recv_packet_list.into_iter().flatten() {
                match recv_packet.packet_type {
                    TypeField::Command => {
                        let cmd_id: u8 = recv_packet.command_id.into();
                        let src_ctx: u8 = recv_packet.src_context_id.into();
                        let src_grp: u8 = recv_packet.src_group_id.into();
                        let dst_grp: u8 = recv_packet.dst_group_id.into();
                        if let ParsedPayload::Cbor(payload) = recv_packet.payload {
                            let payload_bytes: &[u8] = payload.into();
                            self.enqueue_event(cmd_id, payload_bytes);
                        }
                        let _ = self
                            .send_bool_response(src_ctx, dst_grp, src_grp, event_ack_value)
                            .await;
                    }
                    TypeField::Response => {
                        if i32_result.is_none() {
                            if let ParsedPayload::Cbor(payload) = recv_packet.payload {
                                i32_result = Some(self.decode_i32_response(payload.into()));
                            }
                        }
                    }
                    _ => {}
                }
            }
            if let Some(result) = i32_result {
                return result;
            }
        }

        Err(RpcError::Timeout)
    }

    /// Smart ACK variant of `send_command_and_get_i32`.
    ///
    /// Sends a command and decodes an i32 response, while ACKing interleaved
    /// server events with the correct response type based on their command ID.
    ///
    /// - Events matching `bool_ack_cmd_id` are ACKed with `bool true` (e.g., `le_param_req`)
    /// - Other events are ACKed with `default_ack_u8` (u8 value) if `Some`, or void if `None`
    ///
    /// This avoids EBADMSG errors caused by ACKing `le_param_req` (expects bool)
    /// with the wrong CBOR type (void or u8).
    pub(crate) async fn send_command_and_get_i32_smart_ack(
        &mut self,
        packet: NrfRpcPacket<'_, crate::packet::Command>,
        default_ack_u8: Option<u8>,
        bool_ack_cmd_id: Option<u8>,
    ) -> Result<i32, RpcError> {
        self.send_packet(packet)
            .await
            .expect("Failed to send packet");

        for _ in 0..20 {
            let mut buffer = [0u8; 256];
            let recv_packet_list = match self.receive_packet(&mut buffer).await {
                Ok(list) => list,
                Err(_) => continue,
            };

            let mut i32_result: Option<Result<i32, RpcError>> = None;
            for recv_packet in recv_packet_list.into_iter().flatten() {
                match recv_packet.packet_type {
                    TypeField::Command => {
                        let cmd_id: u8 = recv_packet.command_id.into();
                        let src_ctx: u8 = recv_packet.src_context_id.into();
                        let src_grp: u8 = recv_packet.src_group_id.into();
                        let dst_grp: u8 = recv_packet.dst_group_id.into();
                        if let ParsedPayload::Cbor(payload) = recv_packet.payload {
                            let payload_bytes: &[u8] = payload.into();
                            self.enqueue_event(cmd_id, payload_bytes);
                        }
                        if bool_ack_cmd_id == Some(cmd_id) {
                            let _ = self
                                .send_bool_response(src_ctx, dst_grp, src_grp, true)
                                .await;
                        } else if let Some(u8_val) = default_ack_u8 {
                            let _ = self
                                .send_u8_response(src_ctx, dst_grp, src_grp, u8_val)
                                .await;
                        } else {
                            let _ = self.send_void_response(src_ctx, dst_grp, src_grp).await;
                        }
                    }
                    TypeField::Response => {
                        if i32_result.is_none() {
                            if let ParsedPayload::Cbor(payload) = recv_packet.payload {
                                i32_result = Some(self.decode_i32_response(payload.into()));
                            }
                        }
                    }
                    _ => {}
                }
            }
            if let Some(result) = i32_result {
                return result;
            }
        }

        Err(RpcError::Timeout)
    }

    pub(crate) async fn receive_packet<'a>(
        &mut self,
        output: &'a mut [u8; 256],
    ) -> Result<[Option<ParsedNrfRpcPacket<'a>>; 5], RpcError> {
        // Accumulate raw bytes until at least one complete HDLC frame (opening
        // and closing 0x7e delimiter) is present in the buffer. This guards
        // against partial reads from the transport layer; the caller does not
        // need to retry on partial frames.
        let mut total = 0;
        loop {
            let n = self
                .transport
                .read(&mut output[total..])
                .await
                .map_err(|_| RpcError::Transport)?;
            if n == 0 {
                return Err(RpcError::Transport);
            }
            total += n;
            if crate::uart_transport::hdlc_frame_complete(&output[..total]) {
                break;
            }
        }

        let mut output_pkt_list: [Option<ParsedNrfRpcPacket<'a>>; 5] = [const { None }; 5];
        let mut packet_index = 0;

        let mut remaining_buffer = &mut output[..total];
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

                    // ACK the event appropriately (bool for le_param_req, void otherwise)
                    let _ = self
                        .ack_event(cmd_id, src_ctx, dst_grp, src_grp, None)
                        .await;
                }
            }
        }

        found.ok_or(RpcError::NoResponse)
    }

    /// Receive a server-initiated event and respond with a uint8 value.
    ///
    /// Like `receive_server_event`, but sends the given `response_value` as a
    /// CBOR uint8 in the response packet instead of an empty (void) payload.
    /// This is needed for GATT callbacks that expect a return code
    /// (e.g., `BT_GATT_ITER_CONTINUE` or `BT_GATT_ITER_STOP`).
    pub(crate) async fn receive_server_event_with_u8_response(
        &mut self,
        event_payload_out: &mut [u8],
        response_value: u8,
    ) -> Result<(u8, usize), RpcError> {
        // Check the pending-event queue first.
        // Note: queued events were already ACKed with void when they were first received.
        // This is a known limitation — only live events get the u8 response.
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
                        // Respond with u8 unless this is a bool-ack event
                        let _ = self
                            .ack_event(cmd_id, src_ctx, dst_grp, src_grp, Some(response_value))
                            .await;
                    } else {
                        // Additional Command — enqueue for later, ACK appropriately
                        self.enqueue_event(cmd_id, payload_bytes);
                        let _ = self
                            .ack_event(cmd_id, src_ctx, dst_grp, src_grp, None)
                            .await;
                    }
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

    /// Returns `true` if there is anything to process — either an event already
    /// in the internal queue, or bytes buffered in the transport that haven't
    /// been read yet.
    ///
    /// When `true`, [`next_event`](Self::next_event) is guaranteed to make
    /// progress quickly without waiting for new wire activity. Use this in a
    /// two-task embassy setup to avoid holding a mutex while blocking:
    ///
    /// ```ignore
    /// loop {
    ///     let mut ble = ble_mutex.lock().await;
    ///     if ble.has_data() {
    ///         let evt = ble.next_event().await?;
    ///         drop(ble);
    ///         handle(evt);
    ///     } else {
    ///         drop(ble); // nothing ready — release so command task can run
    ///         embassy_futures::yield_now().await;
    ///     }
    /// }
    /// ```
    pub fn has_data(&mut self) -> bool {
        self.pending_count > 0 || self.transport.has_buffered_data()
    }

    /// Returns `true` if at least one server-initiated event is already buffered
    /// in the internal queue.
    ///
    /// When `true`, [`try_next_event`](Self::try_next_event) will return
    /// `Ok(Some(_))` without reading the transport. When `false`,
    /// [`next_event`](Self::next_event) will suspend until transport data arrives.
    ///
    /// This is useful in embassy tasks that must also service a channel, letting
    /// you drain buffered events before blocking:
    ///
    /// ```ignore
    /// loop {
    ///     // Drain any already-buffered events first.
    ///     while client.has_pending_event() {
    ///         if let Some(evt) = client.try_next_event::<MyDecoder>()? {
    ///             handle(evt);
    ///         }
    ///     }
    ///     // Now select without worrying about next_event blocking forever.
    ///     match select(channel.receive(), client.next_event::<MyDecoder>()).await {
    ///         Either::First(cmd) => handle_cmd(cmd),
    ///         Either::Second(Ok(evt)) => handle(evt),
    ///         Either::Second(Err(e)) => { /* … */ }
    ///     }
    /// }
    /// ```
    pub fn has_pending_event(&self) -> bool {
        self.pending_count > 0
    }

    /// Return the next already-queued event without reading the transport.
    ///
    /// Dequeues one entry from the internal ring buffer, decodes it via
    /// `D::decode`, and returns the result. Returns `Ok(None)` immediately
    /// when the queue is empty — it **never suspends**.
    ///
    /// Queued events were already ACKed when they arrived, so no ACK is sent.
    /// Unknown `cmd_id`s for which `D::decode` returns `Ok(None)` are skipped
    /// and the next queued entry is tried; the function only returns `Ok(None)`
    /// once the queue is fully drained.
    ///
    /// ```ignore
    /// while let Some(evt) = client.try_next_event::<MyDecoder>()? {
    ///     handle(evt);
    /// }
    /// ```
    pub fn try_next_event<D: RpcEventDecoder>(&mut self) -> Result<Option<D::Event>, D::Error> {
        loop {
            let (cmd_id, payload_buf, payload_len) = match self.dequeue_event() {
                Some(e) => e,
                None => return Ok(None),
            };
            let payload = &payload_buf[..payload_len];
            match D::decode(cmd_id, payload) {
                Ok(Some(event)) => return Ok(Some(event)),
                Ok(None) => continue,
                Err(e) => return Err(e),
            }
        }
    }

    /// Receive and decode the next server-initiated event using an [`RpcEventDecoder`].
    ///
    /// Handles the full event receive cycle:
    /// 1. Reads the next raw event (queue first, then transport).
    /// 2. Sends the ACK type returned by `D::ack_type(cmd_id)`.
    /// 3. Decodes the payload via `D::decode(cmd_id, payload)`.
    ///    - `Ok(None)` means "skip this event" — loops automatically.
    ///    - `Ok(Some(e))` returns the event.
    ///    - `Err(e)` propagates the error.
    ///
    /// Protocol layers built on top of [`RpcClient`] should implement
    /// [`RpcEventDecoder`] and call this rather than [`Self::next_raw_event`]
    /// directly.
    pub async fn next_event<D: RpcEventDecoder>(&mut self) -> Result<D::Event, D::Error> {
        loop {
            let mut buf = [0u8; 256];
            let (cmd_id, payload_len, ack_routing) =
                self.next_raw_event(&mut buf).await.map_err(Into::into)?;
            let payload = &buf[..payload_len];

            if let Some((src_ctx, dst_grp, src_grp)) = ack_routing {
                match D::ack_type(cmd_id) {
                    AckType::Void => {
                        self.send_void_response(src_ctx, dst_grp, src_grp)
                            .await
                            .map_err(Into::into)?;
                    }
                    AckType::U8(v) => {
                        self.send_u8_response(src_ctx, dst_grp, src_grp, v)
                            .await
                            .map_err(Into::into)?;
                    }
                    AckType::Bool(b) => {
                        self.send_bool_response(src_ctx, dst_grp, src_grp, b)
                            .await
                            .map_err(Into::into)?;
                    }
                }
            }

            match D::decode(cmd_id, payload) {
                Ok(Some(event)) => return Ok(event),
                Ok(None) => continue,
                Err(e) => return Err(e),
            }
        }
    }

    /// Receive the next raw server-initiated event (Command packet).
    ///
    /// Checks the pending-event queue first (already ACKed). For live events
    /// from the transport, the payload is written into `buf` and routing info
    /// `(src_ctx, dst_grp, src_grp)` is returned so the caller can send the
    /// correct ACK type. Queue events return `None` for routing info.
    ///
    /// Additional Command packets that arrive in the same transport frame are
    /// ACKed with void and enqueued for subsequent calls.
    pub(crate) async fn next_raw_event(
        &mut self,
        buf: &mut [u8],
    ) -> Result<(u8, usize, Option<(u8, u8, u8)>), RpcError> {
        // Drain queue first — those events were already ACKed on arrival.
        if let Some((cmd_id, payload_buf, payload_len)) = self.dequeue_event() {
            let len = core::cmp::min(payload_len, buf.len());
            buf[..len].copy_from_slice(&payload_buf[..len]);
            return Ok((cmd_id, len, None));
        }

        // Nothing queued — do a live transport read.
        let mut raw_buf = [0u8; 256];
        let recv_packet_list = self.receive_packet(&mut raw_buf).await?;

        let mut found: Option<(u8, usize, u8, u8, u8)> = None;

        for recv_packet in recv_packet_list.into_iter().flatten() {
            if recv_packet.packet_type == TypeField::Command {
                let cmd_id: u8 = recv_packet.command_id.into();
                let src_ctx: u8 = recv_packet.src_context_id.into();
                let src_grp: u8 = recv_packet.src_group_id.into();
                let dst_grp: u8 = recv_packet.dst_group_id.into();

                if let ParsedPayload::Cbor(payload) = recv_packet.payload {
                    let payload_bytes: &[u8] = payload.into();

                    if found.is_none() {
                        // First event — return it to the caller for ACKing.
                        let len = core::cmp::min(payload_bytes.len(), buf.len());
                        buf[..len].copy_from_slice(&payload_bytes[..len]);
                        found = Some((cmd_id, len, src_ctx, dst_grp, src_grp));
                    } else {
                        // Additional event in the same frame — enqueue and ACK with void now.
                        self.enqueue_event(cmd_id, payload_bytes);
                        let _ = self
                            .ack_event(cmd_id, src_ctx, dst_grp, src_grp, None)
                            .await;
                    }
                }
            }
        }

        if let Some((cmd_id, len, src_ctx, dst_grp, src_grp)) = found {
            Ok((cmd_id, len, Some((src_ctx, dst_grp, src_grp))))
        } else {
            Err(RpcError::NoResponse)
        }
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
    extern crate alloc;
    use alloc::format;
    use alloc::vec::Vec;

    #[test]
    fn test_rpc_error_display() {
        let err = RpcError::Transport;
        assert_eq!(format!("{}", err), "Transport error");
    }

    // ── helpers shared by the batched-packet bug tests ────────────────────────

    #[derive(Debug)]
    struct InternalMockError;
    impl core::fmt::Display for InternalMockError {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            write!(f, "internal mock error")
        }
    }
    impl crate::TransportError for InternalMockError {}

    fn crc16_ccitt_internal(data: &[u8]) -> u16 {
        let mut crc: u16 = 0xFFFF;
        for &byte in data {
            let mut b = byte;
            for _ in 0..8 {
                if (crc ^ b as u16) & 0x0001 != 0 {
                    crc = (crc >> 1) ^ 0x8408;
                } else {
                    crc >>= 1;
                }
                b >>= 1;
            }
        }
        crc
    }

    fn make_hdlc_frame_internal(raw: &[u8]) -> Vec<u8> {
        let crc = crc16_ccitt_internal(raw);
        let mut out = alloc::vec![0x7Eu8];
        let mut push_escaped = |out: &mut Vec<u8>, b: u8| {
            if b == 0x7E || b == 0x7D {
                out.push(0x7D);
                out.push(b ^ 0x20);
            } else {
                out.push(b);
            }
        };
        for &b in raw {
            push_escaped(&mut out, b);
        }
        for &c in &crc.to_le_bytes() {
            push_escaped(&mut out, c);
        }
        out.push(0x7E);
        out
    }

    fn make_i32_response_frame_internal(value: i32) -> Vec<u8> {
        assert!((0..=23).contains(&value));
        let raw = [0x01u8, 0xFF, 0x00, 0x00, 0x00, value as u8];
        make_hdlc_frame_internal(&raw)
    }

    fn make_command_event_frame_internal(
        src_ctx: u8,
        cmd_id: u8,
        dst_ctx: u8,
        src_grp: u8,
        dst_grp: u8,
    ) -> Vec<u8> {
        let raw = [0x80 | src_ctx, cmd_id, dst_ctx, src_grp, dst_grp, 0xF6u8];
        make_hdlc_frame_internal(&raw)
    }

    fn write_buffer_contains_response_ack_internal(buf: &[u8]) -> bool {
        buf.windows(2).any(|w| w[0] == 0x7E && w[1] == 0x01)
    }

    /// Mock inner UART used directly inside the crate's own test module so
    /// that `pub(crate)` methods on `RpcClient` are accessible.
    struct InternalOneShotUart {
        skip_reads: usize,
        read_count: usize,
        read_data: Vec<u8>,
        read_pos: usize,
        writes: std::sync::Arc<std::sync::Mutex<Vec<u8>>>,
    }

    impl InternalOneShotUart {
        fn new(
            skip_reads: usize,
            read_data: Vec<u8>,
        ) -> (Self, std::sync::Arc<std::sync::Mutex<Vec<u8>>>) {
            let writes = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
            let uart = Self {
                skip_reads,
                read_count: 0,
                read_data,
                read_pos: 0,
                writes: std::sync::Arc::clone(&writes),
            };
            (uart, writes)
        }
    }

    impl crate::uart_transport::Uart for InternalOneShotUart {
        type Error = InternalMockError;

        async fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
            self.writes.lock().unwrap().extend_from_slice(data);
            Ok(data.len())
        }

        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            self.read_count += 1;
            if self.read_count <= self.skip_reads {
                return Ok(0);
            }
            if self.read_pos >= self.read_data.len() {
                return Ok(0);
            }
            let n = core::cmp::min(buf.len(), self.read_data.len() - self.read_pos);
            buf[..n].copy_from_slice(&self.read_data[self.read_pos..self.read_pos + n]);
            self.read_pos += n;
            Ok(n)
        }

        async fn delay_ms(&mut self, _ms: u32) {}

        fn has_buffered_data(&mut self) -> bool { false }
    }

    // ── `send_command_and_get_i32_ack_events_bool` ───────────────────────────

    /// **`send_command_and_get_i32_ack_events_bool`** — batched Command after
    /// Response is dropped by the early return.
    ///
    /// This variant is not exposed through any public `Ble` method, so it is
    /// tested here where `pub(crate)` access is available.
    ///
    /// *Observable*: No bool-Response ACK frame written to the transport.
    ///
    /// Fails with unfixed code; passes after the deferred-`i32_result` fix.
    #[test]
    fn test_send_command_and_get_i32_ack_events_bool_drops_batched_command_event() {
        use embassy_futures::block_on;

        let mut read_data = make_i32_response_frame_internal(0);
        read_data.extend(make_command_event_frame_internal(0, 55, 0, 0, 0));

        let (uart, writes) = InternalOneShotUart::new(0, read_data);
        let mut client = RpcClient::new(crate::uart_transport::UartTransport::new(uart));

        // Build a minimal Command packet with empty CBOR payload.
        let mut cbor_buf = [0u8; 8];
        let payload = crate::cbor_encoding::CborPayloadBuilder::new(&mut cbor_buf)
            .build()
            .expect("CBOR build");

        let packet = packet::NrfRpcPacket::<packet::Command>::new(
            packet::SrcContextId::try_from(0).unwrap(),
            packet::DestContextId::try_from(0xFF).unwrap(),
            packet::CommandId::try_from(0x01).unwrap(),
            packet::SrcGroupId::try_from(0).unwrap(),
            packet::DstGroupId::try_from(0).unwrap(),
            payload,
        );

        let result = block_on(client.send_command_and_get_i32_ack_events_bool(packet, true));

        assert!(
            result.is_ok(),
            "must return Ok(0) when response carries i32=0; got: {:?}",
            result.err()
        );
        assert_eq!(result.unwrap(), 0);

        let written = writes.lock().unwrap().clone();
        assert!(
            write_buffer_contains_response_ack_internal(&written),
            "A bool-Response ACK (0x7E 0x01 …) must be written for the batched Command.\n\
             Bug: early return in send_command_and_get_i32_ack_events_bool drops it.\n\
             Written bytes: {:02X?}",
            written
        );
    }

    // ── `send_command_and_get_i32_smart_ack` ─────────────────────────────────

    /// **`send_command_and_get_i32_smart_ack`** — same early-return bug.
    ///
    /// Tested directly here since this variant has no public `Ble` wrapper.
    ///
    /// *Observable*: No Response ACK frame written to the transport.
    ///
    /// Fails with unfixed code; passes after the deferred-`i32_result` fix.
    #[test]
    fn test_send_command_and_get_i32_smart_ack_drops_batched_command_event() {
        use embassy_futures::block_on;

        let mut read_data = make_i32_response_frame_internal(0);
        read_data.extend(make_command_event_frame_internal(0, 77, 0, 0, 0));

        let (uart, writes) = InternalOneShotUart::new(0, read_data);
        let mut client = RpcClient::new(crate::uart_transport::UartTransport::new(uart));

        let mut cbor_buf = [0u8; 8];
        let payload = crate::cbor_encoding::CborPayloadBuilder::new(&mut cbor_buf)
            .build()
            .expect("CBOR build");

        let packet = packet::NrfRpcPacket::<packet::Command>::new(
            packet::SrcContextId::try_from(0).unwrap(),
            packet::DestContextId::try_from(0xFF).unwrap(),
            packet::CommandId::try_from(0x02).unwrap(),
            packet::SrcGroupId::try_from(0).unwrap(),
            packet::DstGroupId::try_from(0).unwrap(),
            payload,
        );

        let result = block_on(client.send_command_and_get_i32_smart_ack(
            packet,
            Some(0), // default_ack_u8
            None,    // bool_ack_cmd_id
        ));

        assert!(
            result.is_ok(),
            "must return Ok(0) when response carries i32=0; got: {:?}",
            result.err()
        );
        assert_eq!(result.unwrap(), 0);

        let written = writes.lock().unwrap().clone();
        assert!(
            write_buffer_contains_response_ack_internal(&written),
            "A Response ACK (0x7E 0x01 …) must be written for the batched Command.\n\
             Bug: early return in send_command_and_get_i32_smart_ack drops it.\n\
             Written bytes: {:02X?}",
            written
        );
    }
}
