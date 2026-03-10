extern crate alloc;

use alloc::rc::Rc;
use alloc::vec::Vec;
use core::cell::RefCell;
use core::future::Future;
use core::pin::Pin;
use core::ptr;
use core::task::{Context, Poll, RawWaker, RawWakerVTable};
use nrf_rpc::{AsyncTransport, TransportError};

/// Simple blocking executor for tests - only works for futures that are immediately ready
pub fn block_on<F: Future + Unpin>(mut fut: F) -> F::Output {
    unsafe fn noop_clone(_: *const ()) -> RawWaker {
        noop_raw_waker()
    }

    unsafe fn noop(_: *const ()) {}

    fn noop_raw_waker() -> RawWaker {
        const VTABLE: RawWakerVTable = RawWakerVTable::new(noop_clone, noop, noop, noop);
        RawWaker::new(ptr::null(), &VTABLE)
    }

    let waker = unsafe { core::task::Waker::from_raw(noop_raw_waker()) };
    let mut ctx = Context::from_waker(&waker);

    loop {
        match Pin::new(&mut fut).poll(&mut ctx) {
            Poll::Ready(val) => return val,
            Poll::Pending => panic!("Future should be immediately ready in tests"),
        }
    }
}

/// Mock UART transport for testing - returns immediately
#[derive(Clone)]
pub struct MockUart {
    pub transmitted: Rc<RefCell<Vec<u8>>>,
}

#[derive(Debug)]
pub struct MockError;

impl TransportError for MockError {}

impl AsyncTransport for MockUart {
    type Error = MockError;
    type TxTransportBuffer<'a, const N: usize> = nrf_rpc::uart_transport::UartTxTransport<'a, N>;
    type RxTransportBuffer<'a, const N: usize> = nrf_rpc::uart_transport::UartRxTransport<'a, N>;

    async fn write(&mut self, data: &mut [u8]) -> Result<usize, Self::Error> {
        self.transmitted.borrow_mut().extend_from_slice(data);
        Ok(data.len())
    }

    async fn read(&mut self, _buffer: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(0)
    }
}
