use crate::time::Hertz;
use embassy::{util::Unborrow, waitqueue::WakerRegistration};

use super::*;

pub struct StateInner<'d, T: super::Basic16bitInstance> {
    timer: T,
    phantom: PhantomData<&'d T>,
    update_waker: WakerRegistration,
    ready: bool,
}

unsafe impl<'d, T: super::Basic16bitInstance> Send for StateInner<'d, T> {}
unsafe impl<'d, T: super::Basic16bitInstance> Sync for StateInner<'d, T> {}

impl<'d, T: super::Basic16bitInstance> PeripheralState for StateInner<'d, T>
where
    Self: 'd,
{
    type Interrupt = T::Interrupt;

    fn on_interrupt(&mut self) {
        self.update_waker.wake();
        self.ready = self.timer.clear_update_interrupt();
    }
}

pub struct State<'d, T: super::Basic16bitInstance>(StateStorage<StateInner<'d, T>>);

impl<'d, T: super::Basic16bitInstance> State<'d, T> {
    pub fn new() -> Self {
        Self(StateStorage::new())
    }
}

pub struct Timer<'d, T: super::Basic16bitInstance> {
    inner: PeripheralMutex<'d, StateInner<'d, T>>,
}

impl<'d, T> Timer<'d, T>
where
    T: super::Basic16bitInstance,
{
    pub fn new(
        state: &'d mut State<'d, T>,
        peri: impl Unborrow<Target = T> + 'd,
        irq: impl Unborrow<Target = T::Interrupt> + 'd,
    ) -> Self {
        unborrow!(peri, irq);
        T::enable();
        <T as crate::rcc::sealed::RccPeripheral>::reset();

        unsafe {
            Self {
                inner: PeripheralMutex::new_unchecked(irq, &mut state.0, move || StateInner {
                    timer: peri,
                    phantom: PhantomData,
                    update_waker: WakerRegistration::new(),
                    ready: false,
                }),
            }
        }
    }

    pub fn start<F: Into<Hertz>>(&mut self, frequency: F) {
        self.inner.with(|state| {
            state.timer.stop();
            state.timer.reset();
            state.timer.set_frequency(frequency);
            state.timer.enable_update_interrupt(true);
            state.timer.start();
        })
    }

    pub fn stop(&mut self) {
        self.inner.with(|state| {
            state.timer.stop();
        })
    }

    pub async fn tick(&mut self) {
        poll_fn(|cx| {
            self.inner.with(|inner| {
                inner.update_waker.register(cx.waker());

                if inner.ready {
                    inner.ready = false;
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
        })
        .await
    }
}
