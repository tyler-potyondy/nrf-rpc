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
    AsyncTransport,
    transport::{
        DecodedTransportPacket, EncodedTransportPacket, RpcRxTransportPacket, RpcTxTransportPacket,
        TransportBuffer, TransportError,
    },
};

/// nRF RPC UART Escape Byte.
const ESCAPE: u8 = 0x7d;
/// nRF RPC UART Frame Delimiter Byte.
const DELIMITER: u8 = 0x7e;

/// Raw UART byte source — the primitive building block for UART transport.
///
/// Implement this trait for your UART hardware (e.g. an Embassy UART peripheral).
/// Wrap the implementation in [`UartTransport`] to get HDLC frame accumulation
/// for free; callers of [`AsyncTransport::read`] will always receive a complete,
/// parseable HDLC frame.
#[allow(async_fn_in_trait)]
pub trait Uart {
    type Error: TransportError;

    /// Read raw bytes from the UART into `buf`.
    ///
    /// May return fewer bytes than `buf.len()` and may return `Ok(0)` when no
    /// data is immediately available. Partial frames are fine — [`UartTransport`]
    /// accumulates until a complete HDLC frame is present.
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error>;

    /// Write raw bytes to the UART.
    async fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error>;

    /// Delay for the given number of milliseconds.
    async fn delay_ms(&mut self, ms: u32);

    /// Returns `true` if at least one byte is already sitting in the
    /// hardware/DMA ring buffer and can be returned by [`read`](Self::read)
    /// without suspending.
    ///
    /// Implementations **must** be backed by a persistent ring buffer (e.g. a
    /// DMA-filled circular buffer or an interrupt-driven FIFO). The check must
    /// be synchronous and must not consume any bytes.
    ///
    /// For Embassy targets, delegate to [`embedded_io::ReadReady`] which checks
    /// the DMA ring buffer's read/write pointers without consuming bytes:
    ///
    /// ```ignore
    /// fn has_buffered_data(&mut self) -> bool {
    ///     use embedded_io::ReadReady;
    ///     self.uart.read_ready().unwrap_or(false)
    /// }
    /// ```
    ///
    /// If `ReadReady` is not available for your peripheral, return `false` as a
    /// safe conservative fallback — the two-task pattern remains correct, it
    /// just polls via `yield_now()` on every loop iteration.
    fn has_buffered_data(&mut self) -> bool;
}

/// UART transport wrapper that implements [`AsyncTransport`] on top of a [`Uart`].
///
/// [`UartTransport::read`] accumulates raw bytes internally until a complete HDLC
/// frame (two `0x7E` delimiters) is present, then delivers the frame to the caller.
/// This means every implementor of [`Uart`] automatically gets correct
/// frame-boundary semantics without having to know anything about HDLC.
pub struct UartTransport<Inner: Uart> {
    inner: Inner,
    /// Internal accumulation buffer for partial HDLC frames.
    rx_buf: [u8; 512],
    rx_len: usize,
}

impl<Inner: Uart> UartTransport<Inner> {
    pub fn new(inner: Inner) -> Self {
        Self {
            inner,
            rx_buf: [0u8; 512],
            rx_len: 0,
        }
    }
}

impl<Inner: Uart> AsyncTransport for UartTransport<Inner> {
    type Error = Inner::Error;
    type TxTransportPacket<'a> = UartTxTransport<'a>;
    type RxTransportPacket<'a> = UartRxTransport<'a>;

    async fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
        self.inner.write(data).await
    }

    /// Returns `true` if a complete HDLC frame is already accumulated in the
    /// internal buffer, or if the underlying [`Uart`] has bytes ready.
    fn has_buffered_data(&mut self) -> bool {
        hdlc_frame_complete(&self.rx_buf[..self.rx_len]) || self.inner.has_buffered_data()
    }

    /// Accumulate raw bytes from the inner transport until a complete HDLC frame
    /// is present, then deliver all bytes up through the closing delimiter.
    ///
    /// Returns `Ok(0)` when the inner transport has no data yet (or after a
    /// timeout), so the caller can retry later. Any bytes already accumulated
    /// are preserved across calls.
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        loop {
            if self.rx_len > 0 && hdlc_frame_complete(&self.rx_buf[..self.rx_len]) {
                break;
            }
            if self.rx_len >= self.rx_buf.len() {
                // Internal buffer full with no complete frame — discard stale data
                // and let the caller retry.
                self.rx_len = 0;
                return Ok(0);
            }
            let n = self.inner.read(&mut self.rx_buf[self.rx_len..]).await?;
            if n == 0 {
                // Inner has no data yet; preserve accumulated bytes and return 0
                // so the caller's retry loop can yield before trying again.
                return Ok(0);
            }
            self.rx_len += n;
        }

        // Find the closing delimiter of the last complete frame.
        //
        // Toggle between "inside frame" and "between frames" on each 0x7e.
        // The closing 0x7e of frame N is also the opening of frame N+1 in
        // HDLC, so we must not mark an opening delimiter as last_end.
        // Example: [7E content 7E  7E content 7E  7E]
        //                     ^15  ^16        ^35  ^36
        //   • toggle at 0  → inside
        //   • toggle at 15 → outside, last_end = 15  (frame A closed)
        //   • toggle at 16 → inside                  (frame B opened)
        //   • toggle at 35 → outside, last_end = 35  (frame B closed)
        //   • toggle at 36 → inside                  (frame C opened, not yet closed)
        // Deliver 0..=35, keep [7E] at 36 in rx_buf for next call.
        let mut last_end = 0usize;
        let mut found_last = false;
        let mut inside_frame = false;
        for (i, &byte) in self.rx_buf[..self.rx_len].iter().enumerate() {
            if byte == DELIMITER {
                if inside_frame {
                    // This 7e closes the current frame.
                    last_end = i;
                    found_last = true;
                    inside_frame = false;
                } else {
                    // This 7e opens a new frame.
                    inside_frame = true;
                }
            }
        }
        if !found_last {
            // hdlc_frame_complete guarantees this can't happen.
            self.rx_len = 0;
            return Ok(0);
        }

        let n = core::cmp::min(buf.len(), last_end + 1);
        buf[..n].copy_from_slice(&self.rx_buf[..n]);

        // Shift any remaining bytes (start of next frame) to the front.
        let remaining = self.rx_len - n;
        self.rx_buf.copy_within(n..self.rx_len, 0);
        self.rx_len = remaining;

        Ok(n)
    }

    async fn delay_ms(&mut self, ms: u32) {
        self.inner.delay_ms(ms).await;
    }
}

pub trait UartTransportBufferStatus {}

struct EncodingInProgress;
pub struct Encoded;
pub struct Decoded;
struct DecodingInProgress;
struct UncheckedEncoded;

pub struct UartTxTransport<'a> {
    inner: UartTransportBuffer<'a, EncodingInProgress>,
}

impl<'a> UartTxTransport<'a> {
    fn new(buffer: &'a mut [u8]) -> Self {
        Self {
            inner: UartTransportBuffer::<'_, EncodingInProgress>::new(buffer),
        }
    }

    fn encode(self) -> Result<UartTransportBuffer<'a, Encoded>, ()> {
        self.inner.complete_encoding()
    }
}

impl<'a> From<UartTransportBuffer<'a, Encoded>> for &'a mut [u8] {
    fn from(value: UartTransportBuffer<'a, Encoded>) -> Self {
        value.buf.into()
    }
}

impl<'a> EncodedTransportPacket<'a> for UartTransportBuffer<'a, Encoded> {}

impl<'a> RpcTxTransportPacket<'a> for UartTxTransport<'a> {
    type EncodedTransportPacket = UartTransportBuffer<'a, Encoded>;
    fn write_slice_into_or_err(&mut self, data: &[u8]) -> Result<(), ()> {
        self.inner.write_slice_into_or_err(data)
    }

    fn write_byte_into_or_err(&mut self, data: u8) -> Result<(), ()> {
        self.inner.write_byte_into_or_err(data)
    }

    fn new(buffer: &'a mut [u8]) -> Self {
        UartTxTransport::new(buffer)
    }

    fn encode_packet(self) -> Result<Self::EncodedTransportPacket, ()> {
        self.encode()
    }
}

pub struct UartRxTransport<'a> {
    inner: UartTransportBuffer<'a, UncheckedEncoded>,
}

impl<'a> UartRxTransport<'a> {
    fn new(buffer: &'a mut [u8]) -> Self {
        Self {
            inner: UartTransportBuffer::<'_, UncheckedEncoded>::new(buffer),
        }
    }

    fn decode(self) -> Result<UartTransportBuffer<'a, Decoded>, ()> {
        self.inner.decode()
    }
}

impl<'a> DecodedTransportPacket<'a> for UartTransportBuffer<'a, Decoded> {}
impl<'a> From<UartTransportBuffer<'a, Decoded>> for &'a mut [u8] {
    fn from(value: UartTransportBuffer<'a, Decoded>) -> Self {
        value.buf.into()
    }
}

impl<'a> RpcRxTransportPacket<'a> for UartRxTransport<'a> {
    type DecodedTransportPacket = UartTransportBuffer<'a, Decoded>;
    fn new(buffer: &'a mut [u8]) -> Result<(Self, Option<&'a mut [u8]>), (&'a mut [u8], ())> {
        // Consume from opening delimiter byte to closing delimiter byte.
        let mut start_ind = 0;
        let mut closing_ind = 0;
        let mut found_start = false;
        let mut found_end = false;

        for (index, byte) in buffer.iter().enumerate() {
            if *byte == DELIMITER {
                if found_start {
                    closing_ind = index;
                    found_end = true;
                    break;
                } else {
                    start_ind = index;
                    found_start = true;
                }
            }
        }

        if !found_start || !found_end {
            // No complete frame (missing opening or closing delimiter); return error with original buffer.
            return Err((buffer, ()));
        }

        // Split buffer into two slices: the packet (start_ind..=closing_ind) and the remaining data after.
        let (packet_and_before, remaining_buffer) = buffer.split_at_mut(closing_ind + 1);
        let processing_buffer: &mut [u8] = &mut packet_and_before[start_ind..];

        let transport_buf = UartRxTransport::new(processing_buffer);

        Ok((transport_buf, Some(remaining_buffer)))
    }

    fn decode_raw_packet(self) -> Result<Self::DecodedTransportPacket, ()> {
        self.decode()
    }
}

impl UartTransportBufferStatus for EncodingInProgress {}
impl UartTransportBufferStatus for Encoded {}
impl UartTransportBufferStatus for Decoded {}
impl UartTransportBufferStatus for DecodingInProgress {}
impl UartTransportBufferStatus for UncheckedEncoded {}

pub struct UartTransportBuffer<'a, S: UartTransportBufferStatus> {
    buf: TransportBuffer<'a>,
    crc: u16,
    status: core::marker::PhantomData<S>,
}

impl<'a> UartTransportBuffer<'a, EncodingInProgress> {
    /// Create a new InProgress UartTransportBuffer.
    ///
    /// Initialize the CRC to 0xffff and the status to InProgress.
    pub fn new(buf: &'a mut [u8]) -> Self {
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

impl<'a> UartTransportBuffer<'a, UncheckedEncoded> {
    fn decode(self) -> Result<UartTransportBuffer<'a, Decoded>, ()> {
        let decoding_in_progress = self.consume_opening_delimiter_byte()?;
        decoding_in_progress.complete_decoding()
    }
}

impl<'a> UartTransportBuffer<'a, DecodingInProgress> {
    fn complete_decoding(mut self) -> Result<UartTransportBuffer<'a, Decoded>, ()> {
        let mut output = [0u8; 256];
        let mut write_pos = 0;

        // This is for all intensive purposes, a while loop until the
        // break, but we know that we should never read more than N bytes,
        // so to prevent a potential infinite loop, we use a for loop instead.
        for _ in 0..self.buf.full_len() {
            let byte = self.buf.read_byte_or_err()?;
            if byte == ESCAPE {
                let next_byte = self.buf.read_byte_or_err()?;
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
            // todo: add error handling
            return Err(());
        }

        let buf = self
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

impl<'a> UartTransportBuffer<'a, UncheckedEncoded> {
    pub fn new(input_buffer: &'a mut [u8]) -> Self {
        Self {
            buf: TransportBuffer::from(input_buffer),
            crc: 0xffff,
            status: core::marker::PhantomData,
        }
    }

    fn consume_opening_delimiter_byte(
        mut self,
    ) -> Result<UartTransportBuffer<'a, DecodingInProgress>, ()> {
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

/*
impl<'a> RpcTxTransportPacket<'a> for UartTransportBuffer<'a, EncodingInProgress> {
    fn write_slice_into_or_err(&mut self, data: &[u8]) -> Result<(), ()> {
        self.write_slice_into_or_err(data)
    }

    fn write_byte_into_or_err(&mut self, data: u8) -> Result<(), ()> {
        self.write_byte_into_or_err(data)
    }

    fn new(buffer: &'a mut [u8]) -> Self {
        Self::new(buffer)
    }


}*/

impl<'a> UartTransportBuffer<'a, EncodingInProgress> {
    /// Attempt to convert InProgress UartTransportBuffer to Encoded UartTransportBuffer.
    ///
    /// Write checksum bytes and delimiter byte to the buffer; may fail if there is
    /// not enough space in the buffer.
    fn complete_encoding(mut self) -> Result<UartTransportBuffer<'a, Encoded>, ()> {
        self.write_checksum_bytes()?;
        self.write_delimiter_byte()?;

        Ok(UartTransportBuffer {
            buf: self.buf,
            crc: self.crc,
            status: core::marker::PhantomData,
        })
    }
}

/// Returns `true` once `buf` contains both an opening and a closing `0x7e`
/// delimiter, indicating that at least one complete HDLC frame is present.
///
/// Used by `RpcClient::receive_packet` to determine when enough bytes have been
/// accumulated to attempt HDLC frame parsing. Exposed as `pub` so external
/// transport implementations can use it as well.
pub fn hdlc_frame_complete(buf: &[u8]) -> bool {
    let mut found_start = false;
    for &byte in buf {
        if byte == DELIMITER {
            if found_start {
                return true;
            }
            found_start = true;
        }
    }
    false
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
        let mut transport_buffer = UartTransportBuffer::<'_, EncodingInProgress>::new(&mut buf);
        transport_buffer
            .write_slice_into_or_err(&[0x80, 0x01, 0xff, 0x00, 0x00, 0x61, 0x7e, 0xf6])
            .expect("Failed to write slice");

        let ready_buffer: UartTransportBuffer<'_, Encoded> = transport_buffer
            .complete_encoding()
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

        let input_buffer: UartTransportBuffer<'_, UncheckedEncoded> =
            UartTransportBuffer::<'_, UncheckedEncoded>::new(&mut encoded_buffer);

        let decoded_buffer: UartTransportBuffer<'_, Decoded> = input_buffer
            .decode()
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

        let input_buffer: UartTransportBuffer<'_, UncheckedEncoded> =
            UartTransportBuffer::<'_, UncheckedEncoded>::new(&mut input_buffer);

        let decoded_buffer = input_buffer
            .decode()
            .expect("Failed to convert to decoded buffer");

        let decoded_buffer_slice: &mut [u8] = decoded_buffer.into();

        const EXPECTED_DECODED_BUFFER: [u8; 36] = [
            0x80, 0x04, 0xff, 0x00, 0x00, 0x18, 0x20, 0x00, 0x00, 0x00, 0x03, 0x18, 0xA0, 0x18,
            0xF0, 0xF6, 0x01, 0x01, 0x01, 0x41, 0x06, 0x01, 0x09, 0x0A, 0x4A, 0x49, 0x4E, 0x6F,
            0x72, 0x64, 0x69, 0x63, 0x5F, 0x50, 0x53, 0xF6,
        ];
        assert_eq!(decoded_buffer_slice, EXPECTED_DECODED_BUFFER);
    }
}
