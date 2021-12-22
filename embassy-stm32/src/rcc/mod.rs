#![macro_use]

use crate::peripherals;
use crate::time::Hertz;
use core::mem::MaybeUninit;

#[derive(Clone, Copy)]
pub struct Clocks {
    pub sys: Hertz,

    #[cfg(rcc_g0)]
    pub apb: Hertz,
    #[cfg(rcc_g0)]
    pub apb_tim: Hertz,

    #[cfg(not(rcc_g0))]
    pub apb1: Hertz,
    #[cfg(not(rcc_g0))]
    pub apb1_tim: Hertz,

    #[cfg(not(rcc_g0))]
    pub apb2: Hertz,
    #[cfg(not(rcc_g0))]
    pub apb2_tim: Hertz,

    #[cfg(any(rcc_wl5, rcc_u5))]
    pub apb3: Hertz,

    #[cfg(any(rcc_l0, rcc_l1, rcc_f0, rcc_f1, rcc_f3, rcc_f0x0, rcc_g0))]
    pub ahb: Hertz,

    #[cfg(any(rcc_l4, rcc_f4, rcc_f7, rcc_h7, rcc_g4, rcc_u5, rcc_wb, rcc_wl5))]
    pub ahb1: Hertz,

    #[cfg(any(rcc_l4, rcc_f4, rcc_f7, rcc_h7, rcc_g4, rcc_u5, rcc_wb, rcc_wl5))]
    pub ahb2: Hertz,

    #[cfg(any(rcc_l4, rcc_f4, rcc_f7, rcc_h7, rcc_u5, rcc_wb, rcc_wl5))]
    pub ahb3: Hertz,

    #[cfg(any(rcc_h7))]
    pub ahb4: Hertz,

    #[cfg(any(rcc_h7))]
    pub apb4: Hertz,

    #[cfg(rcc_f4)]
    pub pll48: Option<Hertz>,
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
static mut CLOCK_FREQS: MaybeUninit<Clocks> = MaybeUninit::uninit();

/// Sets the clock frequencies
///
/// Safety: Sets a mutable global.
pub unsafe fn set_freqs(freqs: Clocks) {
    CLOCK_FREQS.as_mut_ptr().write(freqs);
}

/// Safety: Reads a mutable global.
pub unsafe fn get_freqs() -> &'static Clocks {
    &*CLOCK_FREQS.as_ptr()
}

cfg_if::cfg_if! {
    if #[cfg(rcc_h7)] {
        mod h7;
        pub use h7::*;
    } else if #[cfg(rcc_l0)] {
        mod l0;
        pub use l0::*;
    } else if #[cfg(rcc_l1)] {
        mod l1;
        pub use l1::*;
    } else if #[cfg(rcc_l4)] {
        mod l4;
        pub use l4::*;
    } else if #[cfg(rcc_f1)] {
        mod f1;
        pub use f1::*;
    } else if #[cfg(rcc_f3)] {
        mod f3;
        pub use f3::*;
    } else if #[cfg(rcc_f4)] {
        mod f4;
        pub use f4::*;
    } else if #[cfg(rcc_f7)] {
        mod f7;
        pub use f7::*;
    } else if #[cfg(rcc_wb)] {
        mod wb;
        pub use wb::*;
    } else if #[cfg(rcc_wl5)] {
        mod wl5x;
        pub use wl5x::*;
    } else if #[cfg(any(rcc_f0, rcc_f0x0))] {
        mod f0;
        pub use f0::*;
    } else if #[cfg(any(rcc_g0))] {
        mod g0;
        pub use g0::*;
    } else if #[cfg(any(rcc_g4))] {
        mod g4;
        pub use g4::*;
    } else if #[cfg(any(rcc_u5))] {
        mod u5;
        pub use u5::*;
    }
}

#[cfg(feature = "unstable-pac")]
pub mod low_level {
    pub use super::sealed::*;
}

pub(crate) mod sealed {
    pub trait RccPeripheral {
        fn frequency() -> crate::time::Hertz;
        fn reset();
        fn enable();
        fn disable();
    }
}

pub trait RccPeripheral: sealed::RccPeripheral + 'static {}

crate::pac::peripheral_rcc!(
    ($inst:ident, gpio, GPIO, $clk:ident, $en:tt, $rst:tt) => {};
    ($inst:ident, $module:ident, $block:ident, $clk:ident, ($en_reg:ident, $en_field:ident, $en_set_field:ident), ($rst_reg:ident, $rst_field:ident, $rst_set_field:ident)) => {
        impl sealed::RccPeripheral for peripherals::$inst {
            fn frequency() -> crate::time::Hertz {
                critical_section::with(|_| {
                    unsafe { get_freqs().$clk }
                })
            }
            fn enable() {
                critical_section::with(|_| {
                    unsafe {
                        crate::pac::RCC.$en_reg().modify(|w| w.$en_set_field(true));
                    }
                })
            }
            fn disable() {
                critical_section::with(|_| {
                    unsafe {
                        crate::pac::RCC.$en_reg().modify(|w| w.$en_set_field(false));
                    }
                })
            }
            fn reset() {
                critical_section::with(|_| {
                    unsafe {
                        crate::pac::RCC.$rst_reg().modify(|w| w.$rst_set_field(true));
                        crate::pac::RCC.$rst_reg().modify(|w| w.$rst_set_field(false));
                    }
                })
            }
        }

        impl RccPeripheral for peripherals::$inst {}
    };
    ($inst:ident, $module:ident, $block:ident, $clk:ident, ($en_reg:ident, $en_field:ident, $en_set_field:ident), _) => {
        impl sealed::RccPeripheral for peripherals::$inst {
            fn frequency() -> crate::time::Hertz {
                critical_section::with(|_| {
                    unsafe { get_freqs().$clk }
                })
            }
            fn enable() {
                critical_section::with(|_| {
                    unsafe {
                        crate::pac::RCC.$en_reg().modify(|w| w.$en_set_field(true));
                    }
                })
            }
            fn disable() {
                critical_section::with(|_| {
                    unsafe {
                        crate::pac::RCC.$en_reg().modify(|w| w.$en_set_field(false));
                    }
                })
            }
            fn reset() {}
        }

        impl RccPeripheral for peripherals::$inst {}
    };
);
