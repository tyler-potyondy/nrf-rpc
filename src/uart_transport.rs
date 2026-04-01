//! nRF RPC UART transport
//!
//! The nRF RPC UART transport allows you to use the `nrf_rpc` protocol to execute
//! procedures on a remote processor that is connected with the local processor using
//! the UART interface.
//!
//! Below docs from Nordic:
//!
//! Frame encoding for the nRF RPC UART transport.
//
//! =========================================================================
//! (NOTE) The current frame format is experimental and is a subject to change.
//! =========================================================================
//
//! An nRF RPC packet that is sent using the nRF RPC UART transport is encoded
//! within a frame whose format resembles the one used by the HDLC protocol:
//
//! - Each frame shall start and end with the delimiter octet (0b01111110 = 0x7e).
//! - Each two subsequent frames may be separated by more than one delimiter octet.
//! - Each byte of the nRF RPC packet shall be encoded according to the following rules:
//!     - If the byte matches one of the nrf_rpc_uart special octets, it shall be encoded
//!       as the following two octets:
//!         - the escape octet (0x7d),
//!         - the input byte XORed with 0x20.
//!     - Otherwise, the byte shall be encoded into a frame’s octet without changes.
//! - The last two bytes of the frame contain the nRF RPC packet checksum, in
//!   little-endian byte order. The checksum is calculated using the CRC16_CCITT
//!   function with the initial value ``0xffff``.
//
//! Special octets
//! ==============
//
//! The following octets transmitted over the UART interface have a special meaning:
//
//! +-----------+-----------+
//! | Value     | Meaning   |
//! +===========+===========+
//! |   0x7d    | escape    |
//! +-----------+-----------+
//! |   0x7e    | delimiter |
//! +-----------+-----------+
//
//! Encoding example
//! ================
//
//! If the following nRF RPC packet is sent using the nRF RPC UART transport:
//!   80 01 ff 00 00 61 7e f6
//
//! Then the following octets are transmitted over the UART interface:
//!                    |-2 byte checksum-||--delimiter byte--|
//!                                  v  v  v
//!   7e 80 01 ff 00 00 61 7d 5e f6 6d 72 7e
//!                         ^  ^  
//!              |-escape octet, XORed byte-|  
//
//! Reliability
//! ===========
//
//! The nRF RPC UART transport may optionally enable a reliability feature.
//
//! The reliability feature introduces the following changes to the transport protocol:
//!   - The receiver of a valid frame acknowledges the frame by replying to the
//!     sender with the frame's checksum field.
//!   - If a sender has not received an acknowledgment within a certain time, it
//!     retransmits the frame.
//!     The time (in milliseconds) is defined using XXXXXXXX.
//!   - If the sender has not received an acknowledgment after a certain number of
//!     attempts, it gives up and reports the transmission error. The number of attempts
//!     is defined using XXXXX.
//!   - The frame's checksum field is composed of two values:
//!       - the most significant bit is the sequence bit that is flipped by the sender
//!         for each new transmission.
//!       - the remaining bits are 15 least significant bits of the nRF RPC packet checksum.
//!   - If the received frame has the same checksum field as the previous one, it is
//!     rejected as a duplicate.

use crate::{
    AsyncTransport, TransportError,
    transport::{RpcRxTransportBuffer, RpcTxTransportBuffer, TransportBuffer},
};

/// nRF RPC UART Escape Byte.
const ESCAPE: u8 = 0x7d;
/// nRF RPC UART Frame Delimiter Byte.
const DELIMITER: u8 = 0x7e;

pub trait Uart: AsyncTransport {}

pub trait UartTransportBufferStatus {}

struct EncodingInProgress;
struct Encoded;
struct Decoded;
struct DecodingInProgress;
struct UncheckedEncoded;

pub struct UartTxTransport<'a, const N: usize> {
    inner: UartTransportBuffer<'a, N, EncodingInProgress>,
}

impl<'a, const N: usize> RpcTxTransportBuffer<'a, N> for UartTxTransport<'a, N> {
    fn write_slice_into_or_err(&mut self, data: &[u8]) -> Result<(), ()> {
        self.inner.write_slice_into_or_err(data)
    }

    fn write_byte_into_or_err(&mut self, data: u8) -> Result<(), ()> {
        self.inner.write_byte_into_or_err(data)
    }

    fn new(buffer: &'a mut [u8; N]) -> Self {
        Self {
            inner: UartTransportBuffer::<'_, N, EncodingInProgress>::new(buffer),
        }
    }
}

impl<'a, const N: usize> TryFrom<UartTxTransport<'a, N>> for &'a mut [u8] {
    type Error = ();

    fn try_from(value: UartTxTransport<'a, N>) -> Result<Self, Self::Error> {
        value.inner.try_into()
    }
}

pub struct UartRxTransport<'a, const N: usize> {
    inner: UartTransportBuffer<'a, N, UncheckedEncoded>,
}

impl<'a, const N: usize> RpcRxTransportBuffer<'a, N> for UartRxTransport<'a, N> {
    fn new(buffer: &'a mut [u8; N]) -> Self {
        Self {
            inner: UartTransportBuffer::<'_, N, UncheckedEncoded>::new(buffer),
        }
    }
}

impl<'a, const N: usize> TryFrom<UartRxTransport<'a, N>> for &'a mut [u8] {
    type Error = ();

    fn try_from(value: UartRxTransport<'a, N>) -> Result<Self, Self::Error> {
        value.inner.try_into()
    }
}

impl UartTransportBufferStatus for EncodingInProgress {}
impl UartTransportBufferStatus for Encoded {}
impl UartTransportBufferStatus for Decoded {}
impl UartTransportBufferStatus for DecodingInProgress {}
impl UartTransportBufferStatus for UncheckedEncoded {}

struct UartTransportBuffer<'a, const N: usize, S: UartTransportBufferStatus> {
    buf: TransportBuffer<'a, N>,
    crc: u16,
    status: core::marker::PhantomData<S>,
}

impl<'a, const N: usize> UartTransportBuffer<'a, N, EncodingInProgress> {
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
        let previous_crc = self.crc;

        self.write_byte_into_or_err(checksum[0])?;
        self.write_byte_into_or_err(checksum[1])?;

        // Note, using the write_byte_into_or_err method
        // results in the crc being updated with the
        // checksum bytes. We store/restore the original
        // value here. This is not strictly necessary, but
        // hopefully avoids a potential footgun. In a multithread
        // context, this save/restore may also result in a race
        // condition.
        self.crc = previous_crc;

        Ok(())
    }
}

impl<'a, const N: usize> TryFrom<UartTransportBuffer<'a, N, UncheckedEncoded>>
    for UartTransportBuffer<'a, N, Decoded>
{
    type Error = ();

    fn try_from(value: UartTransportBuffer<'a, N, UncheckedEncoded>) -> Result<Self, Self::Error> {
        let encoding_in_progress = value.consume_opening_delimiter_byte()?;
        encoding_in_progress.try_into()
    }
}

impl<'a, const N: usize> TryFrom<UartTransportBuffer<'a, N, DecodingInProgress>>
    for UartTransportBuffer<'a, N, Decoded>
{
    type Error = ();

    fn try_from(
        mut value: UartTransportBuffer<'a, N, DecodingInProgress>,
    ) -> Result<Self, Self::Error> {
        let mut output = [0; N];
        let mut write_pos = 0;

        // This is for all intensive purposes, a while loop until the
        // break, but we know that we should never read more than N bytes,
        // so to prevent a potential infinite loop, we use a for loop instead.
        for _ in 0..N {
            let byte = value.buf.read_byte_or_err()?;
            if byte == ESCAPE {
                let next_byte = value.buf.read_byte_or_err()?;
                output[write_pos] = next_byte ^ 0x20;
            } else if byte == DELIMITER {
                break;
            } else {
                output[write_pos] = byte;
            }
            write_pos += 1;
        }

        let output_len = write_pos;

        // Packets must have at least 2 bytes for the CRC
        if output_len < 2 {
            return Err(());
        }

        // Crc is stored in the last 2 bytes of the buffer. Calculate
        // the Crc of the received data and compare it to the received CRC.
        let mut crc = 0xffff;
        for byte in &output[..output_len - 2] {
            crc = crc16_ccitt(crc, *byte);
        }

        let received_crc: u16 =
            u16::from_le_bytes([output[output_len - 2], output[output_len - 1]]);

        if crc != received_crc {
            // todo: remove panic and add error handling
            panic!(
                "CRC mismatch: expected 0x{:04x}, got 0x{:04x}",
                crc, received_crc
            );
            return Err(());
        }

        let buf = value
            .buf
            .reset_with_new_slice(&output[..output_len - 2])
            .expect("Failed to reset buffer");

        Ok(UartTransportBuffer {
            buf: buf,
            crc: 0xffff,
            status: core::marker::PhantomData,
        })
    }
}

impl<'a, const N: usize> From<UartTransportBuffer<'a, N, Decoded>> for &'a mut [u8] {
    fn from(value: UartTransportBuffer<'a, N, Decoded>) -> Self {
        value.buf.into()
    }
}

impl<'a, const N: usize> UartTransportBuffer<'a, N, UncheckedEncoded> {
    pub fn new(input_buffer: &'a mut [u8; N]) -> Self {
        Self {
            buf: TransportBuffer::from(input_buffer),
            crc: 0xffff,
            status: core::marker::PhantomData,
        }
    }

    fn consume_opening_delimiter_byte(
        mut self,
    ) -> Result<UartTransportBuffer<'a, N, DecodingInProgress>, ()> {
        if self.buf.read_byte_or_err()? != DELIMITER {
            return Err(());
        }

        Ok(UartTransportBuffer {
            buf: self.buf,
            crc: 0xffff,
            status: core::marker::PhantomData,
        })
    }
}

impl<'a, const N: usize> RpcTxTransportBuffer<'a, N>
    for UartTransportBuffer<'a, N, EncodingInProgress>
{
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

impl<'a, const N: usize> TryFrom<UartTransportBuffer<'a, N, EncodingInProgress>>
    for UartTransportBuffer<'a, N, Encoded>
{
    type Error = ();

    /// Attempt to convert InProgress UartTransportBuffer to Encoded UartTransportBuffer.
    ///
    /// Write checksum bytes and delimiter byte to the buffer; may fail if there is
    /// not enough space in the buffer.
    fn try_from(
        mut value: UartTransportBuffer<'a, N, EncodingInProgress>,
    ) -> Result<Self, Self::Error> {
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
impl<'a, const N: usize> TryInto<&'a mut [u8]> for UartTransportBuffer<'a, N, EncodingInProgress> {
    type Error = ();

    fn try_into(self) -> Result<&'a mut [u8], Self::Error> {
        let ready_buffer: UartTransportBuffer<'a, N, Encoded> = self.try_into()?;
        Ok(ready_buffer.into())
    }
}

impl<'a, const N: usize> TryInto<&'a mut [u8]> for UartTransportBuffer<'a, N, UncheckedEncoded> {
    type Error = ();

    fn try_into(self) -> Result<&'a mut [u8], Self::Error> {
        let decoded_buffer: UartTransportBuffer<'a, N, Decoded> = self.try_into()?;
        Ok(decoded_buffer.into())
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

    /// CRC regression for `bt_le_adv_start` packet from Zephyr sniff.
    #[test]
    fn test_crc16_bt_le_adv_start_packet() {
        let data: [u8; 36] = [
            0x80, 0x04, 0xff, 0x00, 0x00, 0x18, 0x20, 0x00, 0x00, 0x00, 0x03, 0x18, 0xa0, 0x18,
            0xf0, 0xf6, 0x01, 0x01, 0x01, 0x41, 0x06, 0x01, 0x09, 0x0a, 0x4a, 0x49, 0x4e, 0x6f,
            0x72, 0x64, 0x69, 0x63, 0x5f, 0x50, 0x53, 0xf6,
        ];

        let mut crc = 0xffff;
        for byte in data {
            crc = crc16_ccitt(crc, byte);
        }

        assert_eq!(crc, 0x447e);
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
        let mut transport_buffer =
            UartTransportBuffer::<'_, 200, EncodingInProgress>::new(&mut buf);
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
        const EXPECTED_DECODED_BUFFER: [u8; 8] = [0x80, 0x01, 0xff, 0x00, 0x00, 0x61, 0x7e, 0xf6];

        let mut encoded_buffer: [u8; 13] = [
            0x7e, 0x80, 0x01, 0xff, 0x00, 0x00, 0x61, 0x7d, 0x5e, 0xf6, 0x6d, 0x72, 0x7e,
        ];

        let input_buffer: UartTransportBuffer<'_, 13, UncheckedEncoded> =
            UartTransportBuffer::<'_, 13, UncheckedEncoded>::new(&mut encoded_buffer);

        let decoded_buffer: UartTransportBuffer<'_, 13, Decoded> = input_buffer
            .try_into()
            .expect("Failed to convert to decoded buffer");

        let decoded_buffer_slice: &mut [u8] = decoded_buffer.into();
        assert_eq!(decoded_buffer_slice, EXPECTED_DECODED_BUFFER);
    }

    #[test]
    fn uart_transport_buffer_decoding_crc_with_delimiter() {
        let mut input_buffer: [u8; 41] = [
            0x7E, 0x80, 0x04, 0xff, 0x00, 0x00, 0x18, 0x20, 0x00, 0x00, 0x00, 0x03, 0x18, 0xA0,
            0x18, 0xF0, 0xF6, 0x01, 0x01, 0x01, 0x41, 0x06, 0x01, 0x09, 0x0A, 0x4A, 0x49, 0x4E,
            0x6F, 0x72, 0x64, 0x69, 0x63, 0x5F, 0x50, 0x53, 0xF6, 0x7d, 0x5e, 0x44, 0x7E,
        ];

        let input_buffer: UartTransportBuffer<'_, 41, UncheckedEncoded> =
            UartTransportBuffer::<'_, 41, UncheckedEncoded>::new(&mut input_buffer);

        let decoded_buffer_slice: &mut [u8] = input_buffer
            .try_into()
            .expect("Failed to convert to decoded buffer");

        const EXPECTED_DECODED_BUFFER: [u8; 36] = [
            0x80, 0x04, 0xff, 0x00, 0x00, 0x18, 0x20, 0x00, 0x00, 0x00, 0x03, 0x18, 0xA0, 0x18,
            0xF0, 0xF6, 0x01, 0x01, 0x01, 0x41, 0x06, 0x01, 0x09, 0x0A, 0x4A, 0x49, 0x4E, 0x6F,
            0x72, 0x64, 0x69, 0x63, 0x5F, 0x50, 0x53, 0xF6,
        ];
        assert_eq!(decoded_buffer_slice, EXPECTED_DECODED_BUFFER);
    }
}
