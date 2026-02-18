//! nRF RPC protocol specification
//!
//! Two processors that communicate with each other using the remote procedure call
//! (nRF RPC) library follow the nRF RPC protocol. The nRF RPC protocol specifies
//! the binary format and rules for constructing packets that are exchanged within an
//! nRF RPC communication.
//!
//! The nRF RPC packets that are constructed by the nRF RPC core library are relayed
//! to the selected transport layer, where they can be additionally encoded to ensure
//! a reliable delivery of the packet to the other processor using the selected medium.
//! The nRF RPC transport's specification is outside the scope of this document.
//!
//! Source: https://github.com/nrfconnect/sdk-nrfxlib/blob/6204e5fcdac22b4309c72b990857fcc28d8c3095/nrf_rpc/doc/protocol_specification.rst

use core::marker::PhantomData;

const EVENT_PACKET_TYPE: u8 = 0x00;
const RESPONSE_PACKET_TYPE: u8 = 0x01;
const EVENT_ACK_PACKET_TYPE: u8 = 0x02;
const ERROR_REPORT_PACKET_TYPE: u8 = 0x03;
const INIT_PACKET_TYPE: u8 = 0x04;
const COMMAND_PACKET_TYPE_BASE: u8 = 0x80;

/// Command ID field value indicating that the field is unused
/// (e.g., for response and init packets).
const COMMAND_ID_FIELD_UNUSED: u8 = 0xFF;

/// nRF RPC packet format
///
/// Each nRF RPC packet consists of a 5-byte header and an optional, variable-length payload:
///   +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
///   |0                              |1                              |
///   +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
///   |0  |1  |2  |3  |4  |5  |6  |7  |0  |1  |2  |3  |4  |5  |6  |7  |
///   +===+===+===+===+===+===+===+===+===+===+===+===+===+===+===+===+
///   | Type [\| Source Context ID]   | Command ID                    |
///   +-------------------------------+-------------------------------+
///   | Destination Context ID        | Source Group ID               |
///   +-------------------------------+-------------------------------+
///   | Destination Group ID          | [Payload...]                  |
///   +-------------------------------+-------------------------------+
///   |                             [...]                             |
///   +---------------------------------------------------------------+
///
/// Type [| Source Context ID]: 8 bits
/// ==================================
/// The packet type determines the function of the packet and it can be
/// one of the following values:
///   - 0x00: event
///   - 0x01: response
///   - 0x02: event acknowledgment
///   - 0x03: error report
///   - 0x04: initialization packet
///   - 0x80: command
///
/// If the packet type is 0x80 (command), this field is additionally bitwise
/// ORed with the source context ID.
///
/// The source context ID is a numeric identifier of the conversation to which
/// the packet is associated, chosen by the packet sender.
///
/// The source context ID is a feature of the nRF RPC protocol that facilitates
/// concurrent conversations. When two threads on the local processor want to
/// start an nRF RPC conversation at the same time, they shall use distinct source
/// context IDs when constructing a packet to the remote processor. The remote
/// processor is then obliged to use the source context ID as the destination context
/// ID in the response packet. This ensures that responses and any packets that
/// follow within each conversation are correctly routed to the initiating thread.
///
/// The exact source context ID allocation pattern is implementation-defined, meaning
/// that when the packet sender initiates a new conversation or responds to the
/// initiating packet, it is free to allocate any unused source context ID for the new
/// conversation.
///
/// Command ID: 8 bits
/// ==================
/// Identifies an individual command or event within an nRF RPC group.
///
/// If the packet is a response or an initialization packet, this field has no meaning
/// and shall be set to 0xff.
///
/// Destination Context ID: 8 bits
/// ==============================
/// A numeric identifier of the conversation to which the packet is associated,
/// chosen by the packet receiver.
///
/// In a packet that starts a new conversation, this field shall be assigned
/// the value 0xff (indicating it is unknown). In all subsequent packets within
/// the conversation, the sender of the packet shall carry over the source context
/// ID that was included in the last packet received from the peer.
///
/// Source Group ID: 8 bits
/// =======================
/// A numeric identifier of the nRF RPC group associated with the packet, chosen
/// by the packet sender.
///
/// Each processor that uses the nRF RPC protocol chooses unique numeric identifiers
/// for all nRF RPC groups that it supports. During the nRF RPC protocol initialization,
/// it then communicates its own mapping of the pre-shared string group identifiers to
/// these unique numeric identifiers.
///
/// Destination Group ID: 8 bits
/// ============================
/// A numeric identifier of the nRF RPC group associated with the packet, chosen by
/// the packet receiver.
///
/// The sender learns this identifier by receiving an initialization packet from the
/// peer during the nRF RPC protocol initialization.
///
/// Payload: variable length
/// ========================
/// The payload format depends on the packet type:
///   - event acknowledgment: the payload is empty.
///   - error report: the payload is a 32-bit integer representing an error code, in
///     little-endian byte order.
///   - initialization packet: the payload has the following format:
///     ```text
///     +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
///     |0                              |1                              |
///     +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
///     |0  |1  |2  |3  |4  |5  |6  |7  |0  |1  |2  |3  |4  |5  |6  |7  |
///     +===+===+===+===+===+===+===+===+===+===+===+===+===+===+===+===+
///     | Max Version   | Min Version   | Group name....                |
///     +---------------+---------------+-------------------------------+
///     |                              ...                              |
///     +---------------------------------------------------------------+
///     ```
///     The Min Version and Max Version fields indicate the minimum and maximum
///     version of the nRF RPC protocol supported by the sender. The Group name
///     field has a variable length and contains the string identifier of the nRF
///     RPC group to which this packet is associated with, without the null terminator.
///   - event, response, command: the payload contains remote procedure call
///     arguments or return values, represented in an implementation-defined format. If
///     the nRF RPC protocol is used together with the CBOR encoding, then the
///     arguments and return values are represented as a sequence of CBOR data items,
///     terminated by the null data item (0xf6).
///
///     For example, if a packet is an nRF RPC command that represents the C function
///     call `foo(100, "bar")`, the packet might look as follows:
///       80 01 ff 00 00 18 64 63 62 61 72 f6
///
///       80: Command | Source Context ID (0)
///       01: Command ID (1)
///       ff: Destination Context ID (unknown)
///       00: Source Group ID (0)
///       00: Destination Group ID (0)
///       18 64: CBOR unsigned int (100)
///       63 62 61 72: CBOR text string ("bar")
///       f6: CBOR null
struct NrfRpcPacket<T: NrfRpcPacketType, const N: usize> {
    src_context_id: Option<u8>, // Only set for command packets
    dst_context_id: u8,
    src_group_id: u8,
    dst_group_id: u8,
    payload: [u8; N],
    associated_packet_type: PhantomData<T>,
}

struct CBorPayload<const N: usize>([u8; N]);

impl<const N: usize, C: CommandId> NrfRpcPacket<Event<C>, N> {
    fn new(
        dst_context_id: u8,
        src_group_id: u8,
        dst_group_id: u8,
        cbor_encoded_payload: CBorPayload<N>,
    ) -> Self {
        Self {
            src_context_id: None,
            dst_context_id,
            src_group_id,
            dst_group_id,
            payload: cbor_encoded_payload.0,
            associated_packet_type: PhantomData,
        }
    }
}

impl<const N: usize, C: CommandId> NrfRpcPacket<Response<C>, N> {
    fn new(
        dst_context_id: u8,
        src_group_id: u8,
        dst_group_id: u8,
        cbor_encoded_payload: CBorPayload<N>,
    ) -> Self {
        Self {
            src_context_id: None,
            dst_context_id,
            src_group_id,
            dst_group_id,
            payload: cbor_encoded_payload.0,
            associated_packet_type: PhantomData,
        }
    }
}

impl<C: CommandId> NrfRpcPacket<EventAck<C>, 0> {
    fn new(dst_context_id: u8, src_group_id: u8, dst_group_id: u8) -> Self {
        Self {
            src_context_id: None,
            dst_context_id,
            src_group_id,
            dst_group_id,
            payload: [],
            associated_packet_type: PhantomData,
        }
    }
}

// ErrorReport packets have a fixed payload size of 4 bytes (32-bit error code)
impl NrfRpcPacket<ErrorReport, 4> {
    // Error report: the payload is a 32-bit integer representing an error code, in
    // little-endian byte order.
    fn new(dst_context_id: u8, src_group_id: u8, dst_group_id: u8, error_code: u32) -> Self {
        Self {
            src_context_id: None,
            dst_context_id,
            src_group_id,
            dst_group_id,
            payload: error_code.to_le_bytes(),
            associated_packet_type: PhantomData,
        }
    }
}

impl<const N: usize, C: CommandId> NrfRpcPacket<Command<C>, N> {
    fn new(
        src_context_id: u8,
        dst_context_id: u8,
        src_group_id: u8,
        dst_group_id: u8,
        payload: [u8; N],
    ) -> Self {
        Self {
            src_context_id: Some(src_context_id),
            dst_context_id,
            src_group_id,
            dst_group_id,
            payload,
            associated_packet_type: PhantomData,
        }
    }
}

struct MinVersion(u8);
struct MaxVersion(u8);

struct InitPacketPayload<const N: usize> {
    max_version: MaxVersion,
    min_version: MinVersion,
    group_name: [u8; N],
}

impl<const N: usize> NrfRpcPacket<Init, N> {
    // (todo) use flux or nightly feature for const generic arith
    // to add this to the type system (instead of just payload array here).
    // Max Version: bits 0-3 (byte 0)
    // Min Version: bits 4-7 (byte 0)
    // Group name: byte 1 to N
    const fn new(src_group_id: u8, dst_group_id: u8, payload: [u8; N]) -> Self {
        Self {
            src_context_id: None,
            dst_context_id: 0xFF, // Unknown destination context ID for init packets
            src_group_id,
            dst_group_id,
            payload: payload,
            associated_packet_type: PhantomData,
        }
    }
}

impl<const N: usize, P: NrfRpcPacketType> NrfRpcPacket<P, N> {
    fn form_header(&self) -> [u8; 5] {
        let type_byte = P::TypeField | self.src_context_id.unwrap_or(0);
        let command_id_byte = P::CommandIdField;
        [
            type_byte,
            command_id_byte,
            self.dst_context_id,
            self.src_group_id,
            self.dst_group_id,
        ]
    }

    fn form_payload(&self) -> [u8; N] {
        self.payload
    }
}

trait CommandId {
    const COMMAND_ID: u8;
}

trait NrfRpcPacketType {
    const TypeField: u8;
    const CommandIdField: u8;
}

struct Event<C: CommandId>(PhantomData<C>);
impl<C: CommandId> NrfRpcPacketType for Event<C> {
    const CommandIdField: u8 = C::COMMAND_ID;
    const TypeField: u8 = 0x00;
}

struct Response<C: CommandId>(PhantomData<C>);
impl<C: CommandId> NrfRpcPacketType for Response<C> {
    const CommandIdField: u8 = C::COMMAND_ID;
    const TypeField: u8 = RESPONSE_PACKET_TYPE;
}

struct EventAck<C: CommandId>(PhantomData<C>);
impl<C: CommandId> NrfRpcPacketType for EventAck<C> {
    const CommandIdField: u8 = C::COMMAND_ID;
    const TypeField: u8 = EVENT_ACK_PACKET_TYPE;
}

struct ErrorReport;
impl NrfRpcPacketType for ErrorReport {
    const CommandIdField: u8 = unimplemented!();
    const TypeField: u8 = ERROR_REPORT_PACKET_TYPE;
}

struct Init;
impl NrfRpcPacketType for Init {
    const CommandIdField: u8 = COMMAND_ID_FIELD_UNUSED;
    const TypeField: u8 = INIT_PACKET_TYPE;
}

struct Command<C: CommandId>(PhantomData<C>);
impl<C: CommandId> NrfRpcPacketType for Command<C> {
    const CommandIdField: u8 = C::COMMAND_ID;
    const TypeField: u8 = COMMAND_PACKET_TYPE_BASE;
}
