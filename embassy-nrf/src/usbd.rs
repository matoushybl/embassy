use embassy::util::Unborrow;

use crate::peripherals::USBD;
use core::marker::PhantomData;

pub struct UsbPeripheral<'d> {
    _usbd: PhantomData<&'d USBD>,
}

// TODO check for external crystal clock
impl<'d> UsbPeripheral<'d> {
    pub fn new(_usbd: impl Unborrow<Target = USBD> + 'd) -> Self {
        Self { _usbd: PhantomData }
    }
}

unsafe impl<'a> nrf_usbd::UsbPeripheral for UsbPeripheral<'a> {
    const REGISTERS: *const () = crate::chip::pac::USBD::ptr() as *const _;
}
