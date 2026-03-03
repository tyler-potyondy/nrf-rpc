//! nRF RPC UART transport
//!
//! The nRF RPC UART transport allows you to use the `nrf_rpc` protocol to execute
//! procedures on a remote processor that is connected with the local processor using
//! the UART interface.

use crate::{AsyncTransport, RpcTransportBuffer, TransportError};

/// nRF RPC UART Escape Byte.
const ESCAPE: u8 = 0x7d;
/// nRF RPC UART Frame Delimiter Byte.
const DELIMITER: u8 = 0x7e;

/// Frame encoding for the nRF RPC UART transport.
///
/// =========================================================================
/// (NOTE) The current frame format is experimental and is a subject to change.
/// =========================================================================
///
/// An nRF RPC packet that is sent using the nRF RPC UART transport is encoded
/// within a frame whose format resembles the one used by the HDLC protocol:
///
/// - Each frame shall start and end with the delimiter octet (0b01111110 = 0x7e).
/// - Each two subsequent frames may be separated by more than one delimiter octet.
/// - Each byte of the nRF RPC packet shall be encoded according to the following rules:
///     - If the byte matches one of the nrf_rpc_uart special octets, it shall be encoded
///       as the following two octets:
///         - the escape octet (0x7d),
///         - the input byte XORed with 0x20.
///     - Otherwise, the byte shall be encoded into a frame’s octet without changes.
/// - The last two bytes of the frame contain the nRF RPC packet checksum, in
///   little-endian byte order. The checksum is calculated using the CRC16_CCITT
///   function with the initial value ``0xffff``.
///
/// Special octets
/// ==============
///
/// The following octets transmitted over the UART interface have a special meaning:
///
/// +-----------+-----------+
/// | Value     | Meaning   |
/// +===========+===========+
/// |   0x7d    | escape    |
/// +-----------+-----------+
/// |   0x7e    | delimiter |
/// +-----------+-----------+
///
/// Encoding example
/// ================
///
/// If the following nRF RPC packet is sent using the nRF RPC UART transport:
///   80 01 ff 00 00 61 7e f6
///
/// Then the following octets are transmitted over the UART interface:
///                    |-2 byte checksum-||--delimiter byte--|
///                                  v  v  v
///   7e 80 01 ff 00 00 61 7d 5e f6 6d 72 7e
///                         ^  ^  
///              |-escape octet, XORed byte-|  
///
/// Reliability
/// ===========
///
/// The nRF RPC UART transport may optionally enable a reliability feature.
///
/// The reliability feature introduces the following changes to the transport protocol:
///   - The receiver of a valid frame acknowledges the frame by replying to the
///     sender with the frame's checksum field.
///   - If a sender has not received an acknowledgment within a certain time, it
///     retransmits the frame.
///     The time (in milliseconds) is defined using XXXXXXXX.
///   - If the sender has not received an acknowledgment after a certain number of
///     attempts, it gives up and reports the transmission error. The number of attempts
///     is defined using XXXXX.
///   - The frame's checksum field is composed of two values:
///       - the most significant bit is the sequence bit that is flipped by the sender
///         for each new transmission.
///       - the remaining bits are 15 least significant bits of the nRF RPC packet checksum.
///   - If the received frame has the same checksum field as the previous one, it is
///     rejected as a duplicate.
// struct UartFrame<const N: usize, M: Mode> {
//     base_frame: [u8; N],
// }
pub struct NrfRpcUartTransport<'a, U: UartTransport> {
    uart: &'a mut U,
}

/// Calculate CRC16_CCITT with initial value 0xffff.
///
/// Matches Zephyr's `crc16_ccitt` configuration, which uses reversed
/// input and output. This is equivalent to a reflected CRC-16/CCITT
/// with polynomial 0x8408, seed 0xffff, and no final XOR.
fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;

    for &byte in data {
        crc ^= byte as u16;
        for _ in 0..8 {
            if (crc & 1) != 0 {
                crc = (crc >> 1) ^ 0x8408;
            } else {
                crc >>= 1;
            }
        }
    }

    crc
}

impl<'a, U: UartTransport> NrfRpcUartTransport<'a, U> {
    pub fn new(uart: &'a mut U) -> Self {
        Self { uart }
    }

    /// Write a single byte into the frame, applying UART escape rules.
    fn write_escaped_byte(&self, buffer: &mut RpcTransportBuffer, byte: u8) -> Result<(), ()> {
        if byte == ESCAPE || byte == DELIMITER {
            buffer.write_byte_into_or_err(ESCAPE)?;
            buffer.write_byte_into_or_err(byte ^ 0x20)
        } else {
            buffer.write_byte_into_or_err(byte)
        }
    }

    fn encode_uart_frame(
        &mut self,
        data: &[u8],
        buffer: &mut RpcTransportBuffer,
    ) -> Result<(), ()> {
        // Add deliminter byte.
        buffer.write_byte_into_or_err(DELIMITER)?;

        // Add copy data bytes into buffer, checking for special bytes (escape, delimiter).
        // If find a special byte, add the escape byte and the XORed byte.
        for byte in data {
            self.write_escaped_byte(buffer, *byte)?;
        }

        // Add checksum bytes.
        let checksum = crc16_ccitt(data).to_le_bytes();
        self.write_escaped_byte(buffer, checksum[0])?;
        self.write_escaped_byte(buffer, checksum[1])?;

        // Add deliminter byte.
        buffer.write_byte_into_or_err(DELIMITER)?;

        Ok(())
    }

    fn decode_uart_frame(
        &mut self,
        input_buffer: &mut RpcTransportBuffer,
        output_buffer: &mut RpcTransportBuffer,
    ) -> Result<(), ()> {
        // Remove delimiter byte.
        if input_buffer.read_byte_or_err()? != DELIMITER {
            return Err(()); // Expected delimiter byte not found.
        }

        // Read everything except last byte (delimiter byte).
        while input_buffer.remaining_len() > 1 {
            let byte = input_buffer.read_byte_or_err()?;

            if byte == ESCAPE {
                let next_byte = input_buffer.read_byte_or_err()?;
                // Undo the XORing of the escape byte.
                output_buffer.write_byte_into_or_err(next_byte ^ 0x20)?;
            } else {
                output_buffer.write_byte_into_or_err(byte)?;
            }
        }

        // Read last byte (delimiter byte).
        if input_buffer.read_byte_or_err()? != DELIMITER {
            return Err(()); // Expected delimiter byte not found.
        }

        Ok(())
    }
}

impl<'a, U: UartTransport> AsyncTransport for NrfRpcUartTransport<'a, U> {
    type Error = U::Error;

    async fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
        let mut base_buffer = [0; 200];
        let mut buffer = RpcTransportBuffer::new(&mut base_buffer);

        // Encode UartFrame, then write over Uart.
        self.encode_uart_frame(data, &mut buffer)
            .expect("Failed to encode UART frame");
        self.uart.write(buffer.into()).await
    }

    /// Read from Uart, decode the frame, ACK (send received checksum, no encoding needed).
    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        let mut read_buffer: &mut [u8] = &mut [0; 200];
        self.uart.read(read_buffer).await?;

        let mut read_output_buffer = RpcTransportBuffer::new(read_buffer);

        // Decode packet from buffer.
        let mut read_input_buffer = RpcTransportBuffer::new(buffer);

        self.decode_uart_frame(&mut read_input_buffer, &mut read_output_buffer)
            .expect("Failed to decode UART frame");

        // Send ACK (last 2 bytes of the frame).
        let output_read_buffer = read_input_buffer.into();
        let mut ack_buffer = [0; 2];
        ack_buffer.copy_from_slice(output_read_buffer);
        self.uart
            .write(&ack_buffer)
            .await
            .expect("Failed to send ACK");

        Ok(output_read_buffer.len())
    }
}

pub trait UartTransport {
    type Error: TransportError;

    async fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error>;

    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error>;
}
