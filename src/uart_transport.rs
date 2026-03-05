//! nRF RPC UART transport
//!
//! The nRF RPC UART transport allows you to use the `nrf_rpc` protocol to execute
//! procedures on a remote processor that is connected with the local processor using
//! the UART interface.

use crate::{
    AsyncTransport, TransportError,
    transport::{RpcTransportBuffer, TransportBuffer},
};

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

/*
pub struct NrfRpcUartTransport<'a, U: UartTransport> {
    uart: &'a mut U,
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

    fn encode_uart_frame<const N: usize>(
        &mut self,
        buffer: RpcTransportBuffer<'a, N>,
    ) -> Result<RpcTransportBuffer<'a, N>, ()> {
        // Add deliminter byte.
        buffer.write_byte_into_or_err(DELIMITER).expect("Failed to write delimiter byte");

        // Add copy data bytes into buffer, checking for special bytes (escape, delimiter).
        // If find a special byte, add the escape byte and the XORed byte.
        for byte in data {
            self.write_escaped_byte(buffer, *byte).expect("Failed to write escaped byte");
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
            panic!(
                "Expected delimiter byte not found {:02x?}",
                input_buffer.buffer
            );
            return Err(()); // Expected delimiter byte not found.
        }

        // Read everything except last byte (delimiter byte).
        while input_buffer.start_pos + 1 < input_buffer.end_pos
            && input_buffer.read_byte_or_err()? != DELIMITER
        {
            let byte = input_buffer
                .read_byte_or_err()
                .expect("Failed to read byte");

            if byte == ESCAPE {
                let next_byte = input_buffer
                    .read_byte_or_err()
                    .expect("Failed to read byte");
                // Undo the XORing of the escape byte.
                output_buffer
                    .write_byte_into_or_err(next_byte ^ 0x20)
                    .expect("Failed to write byte");
            } else {
                output_buffer
                    .write_byte_into_or_err(byte)
                    .expect("Failed to write byte");
            }
        }

        // Read last byte (delimiter byte).
        if input_buffer.read_byte_or_err()? != DELIMITER {
            panic!(
                "Expected delimiter byte at end not found {:02x?}, start_pos: {}, end_pos: {}",
                input_buffer.buffer, input_buffer.start_pos, input_buffer.end_pos
            );
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
        let output_buffer = &mut [0; 200];
        let len = self.uart.read(read_buffer).await?;
        read_buffer = &mut read_buffer[..len];

        let mut read_output_buffer = RpcTransportBuffer::new(output_buffer);

        // Decode packet from buffer.
        let mut read_input_buffer = RpcTransportBuffer::from(read_buffer);

        self.decode_uart_frame(&mut read_input_buffer, &mut read_output_buffer)
            .expect("Failed to decode UART frame");

        // Send ACK (last 2 bytes of the frame).
        let output_read_buffer: &[u8] = read_input_buffer.into();
        let mut ack_buffer = [0; 2];
        ack_buffer.copy_from_slice(&output_read_buffer[output_read_buffer.len() - 2..]);
        self.uart
            .write(&ack_buffer)
            .await
            .expect("Failed to send ACK");

        // Copy read/processed buffer to provided output buffer.
        buffer[..read_output_buffer.end_pos - read_output_buffer.start_pos]
            .copy_from_slice(read_output_buffer.into());
        Ok(output_read_buffer.len()RpcTransportBuffer)
    }
}

//pub trait UartTransport {
//    type Error: TransportError;
//
//    async fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error>;
//
//    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error>;
//}


struct UartTransport<'a, T: Uart> {
    transport: &'a T,
}

impl<'a, T: Uart> UartTransport<'a, T> {
    pub fn new(transport: &'a T) -> Self {
        Self { transport }
    }

    async fn write<const N: usize>(&mut self, buf: RpcTransportBuffer<'a, N>) -> Result<(), ()> {
        unimplemented!()
    }

    async fn read<const N: usize>(
        &mut self,
        buf: RpcTransportBuffer<'a, N>,
    ) -> Result<RpcTransportBuffer<'a, N>, ()> {
        unimplemented!()
    }
}

*/
pub trait Uart: AsyncTransport {}

pub trait UartTransportBufferStatus {}

struct InProgress;
struct Encoded;

impl UartTransportBufferStatus for InProgress {}
impl UartTransportBufferStatus for Encoded {}

pub struct UartTransportBuffer<'a, const N: usize, S: UartTransportBufferStatus> {
    buf: TransportBuffer<'a, N>,
    crc: u16,
    status: core::marker::PhantomData<S>,
}

impl<'a, const N: usize> UartTransportBuffer<'a, N, InProgress> {
    /// Create a new InProgress UartTransportBuffer.
    ///
    /// Initialize the CRC to 0xffff and the status to InProgress.
    pub fn new(buf: &'a mut [u8; N]) -> Self {
        let mut output = Self {
            buf: TransportBuffer::new(buf),
            crc: 0xffff,
            status: core::marker::PhantomData,
        };

        // Input buffer of N = 0 is technically, valid but will be
        // useless. It is nice to not error here, so if the
        // UartTransportBuffer is empty and writing the delimiter byte
        // errors, we just ignore the error. May be yet another useful
        // place for flux.
        let _ = output.write_delimiter_byte();

        output
    }

    /// Write slice to underlying buffer, updating the CRC.
    ///
    /// May fail if there is not enough space in the buffer.
    fn write_slice_into_or_err(&mut self, data: &[u8]) -> Result<(), ()> {
        for byte in data {
            self.write_byte_into_or_err(*byte)?;
        }
        Ok(())
    }

    /// Write byte to underlying buffer, updating the CRC.
    ///
    /// May fail if there is not enough space in the buffer.
    fn write_byte_into_or_err(&mut self, data: u8) -> Result<(), ()> {
        self.crc = crc16_ccitt(self.crc, data);
        if data == ESCAPE || data == DELIMITER {
            self.buf.write_byte_into_or_err(ESCAPE)?;
            self.buf.write_byte_into_or_err(data ^ 0x20)
        } else {
            self.buf.write_byte_into_or_err(data)
        }
    }

    /// Helper method for writing delimiter byte to underlying buffer.
    ///
    /// Delimiter byte does not contribute to the CRC calculation, hence
    /// unique method rather than `write_byte_into_or_err`.
    fn write_delimiter_byte(&mut self) -> Result<(), ()> {
        self.buf.write_byte_into_or_err(DELIMITER)
    }

    /// Write checksum to underlying buffer.
    ///
    /// May fail if there is not enough space in the buffer.
    fn write_checksum_bytes(&mut self) -> Result<(), ()> {
        let checksum = self.crc.to_le_bytes();
        self.buf.write_byte_into_or_err(checksum[0])?;
        self.buf.write_byte_into_or_err(checksum[1])?;
        Ok(())
    }
}

impl<'a, const N: usize> RpcTransportBuffer<'a, N> for UartTransportBuffer<'a, N, InProgress> {
    fn write_slice_into_or_err(&mut self, data: &[u8]) -> Result<(), ()> {
        self.write_slice_into_or_err(data)
    }

    fn write_byte_into_or_err(&mut self, data: u8) -> Result<(), ()> {
        self.write_byte_into_or_err(data)
    }

    fn new(buffer: &'a mut [u8; N]) -> Self {
        Self::new(buffer)
    }
}

/// Attempt to convert InProgress UartTransportBuffer to Encoded UartTransportBuffer.
///
/// Write checksum bytes and delimiter byte to the buffer; may fail if there is
/// not enough space in the buffer.
impl<'a, const N: usize> TryFrom<UartTransportBuffer<'a, N, InProgress>>
    for UartTransportBuffer<'a, N, Encoded>
{
    type Error = ();
    fn try_from(mut value: UartTransportBuffer<'a, N, InProgress>) -> Result<Self, Self::Error> {
        value.write_checksum_bytes()?;
        value.write_delimiter_byte()?;

        Ok(Self {
            buf: value.buf,
            crc: value.crc,
            status: core::marker::PhantomData,
        })
    }
}
/// Consume Encoded UartTransportBuffer to slice of bytes.
impl<'a, const N: usize> From<UartTransportBuffer<'a, N, Encoded>> for &'a mut [u8] {
    fn from(value: UartTransportBuffer<'a, N, Encoded>) -> Self {
        value.buf.into()
    }
}

/// Attempt to convert InProgress UartTransportBuffer to slice of bytes via first
/// converting to Encoded UartTransportBuffer.
impl<'a, const N: usize> TryInto<&'a mut [u8]> for UartTransportBuffer<'a, N, InProgress> {
    type Error = ();

    fn try_into(self) -> Result<&'a mut [u8], Self::Error> {
        let ready_buffer: UartTransportBuffer<'a, N, Encoded> = self.try_into()?;
        Ok(ready_buffer.into())
    }
}

/// Calculate CRC16_CCITT with seed.
///
/// Matches Zephyr's `crc16_ccitt` configuration, which uses reversed
/// input and output. This is equivalent to a reflected CRC-16/CCITT
/// with polynomial 0x8408, seed 0xffff (used for the first byte), and no final XOR.
fn crc16_ccitt(seed: u16, data: u8) -> u16 {
    let mut crc = seed;
    crc ^= data as u16;
    for _ in 0..8 {
        if (crc & 1) != 0 {
            crc = (crc >> 1) ^ 0x8408u16;
        } else {
            crc >>= 1;
        }
    }

    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16_ccitt() {
        let data = [0x80, 0x01, 0xff, 0x00, 0x00, 0x61, 0x7e, 0xf6];

        let mut crc = 0xffff;
        for byte in data {
            crc = crc16_ccitt(crc, byte);
        }
        assert_eq!(crc, 0x726d);
    }

    #[test]
    fn uart_transport_buffer_encoding() {
        /*
        Test based on the doc comment:

        If the following nRF RPC packet is sent using the nRF RPC UART transport:
          80 01 ff 00 00 61 7e f6
          --------------------------------
          Then the following octets are transmitted over the UART interface:
                           |-2 byte checksum-||--delimiter byte--|
                                         v  v  v
          7e 80 01 ff 00 00 61 7d 5e f6 6d 72 7e
                                ^  ^
                     |-escape octet, XORed byte-|

         */
        let mut buf = [0; 200];
        let mut transport_buffer = UartTransportBuffer::new(&mut buf);
        transport_buffer
            .write_slice_into_or_err(&[0x80, 0x01, 0xff, 0x00, 0x00, 0x61, 0x7e, 0xf6])
            .expect("Failed to write slice");

        let ready_buffer: UartTransportBuffer<'_, 200, Encoded> = transport_buffer
            .try_into()
            .expect("Failed to convert to ready buffer");

        let ready_buffer_slice: &mut [u8] = ready_buffer.into();
        assert_eq!(
            ready_buffer_slice,
            [
                0x7e, 0x80, 0x01, 0xff, 0x00, 0x00, 0x61, 0x7d, 0x5e, 0xf6, 0x6d, 0x72, 0x7e
            ]
        );
    }

    #[test]
    fn uart_transport_buffer_decoding() {
        let mut buf: [u8; 13] = [
            0x7e, 0x80, 0x01, 0xff, 0x00, 0x00, 0x61, 0x7d, 0x5e, 0xf6, 0x6d, 0x72, 0x7e,
        ];
    }
}
