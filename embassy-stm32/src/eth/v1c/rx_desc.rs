use core::sync::atomic::{fence, Ordering};

use embassy_net::{Packet, PacketBox, PacketBoxExt, PacketBuf};
use stm32_metapac::eth::vals::{DmaomrSr, Rpd};
use vcell::VolatileCell;

use crate::pac::ETH;

mod rx_consts {
    /// Owned by DMA engine
    pub const RXDESC_0_OWN: u32 = 1 << 31;
    /// First descriptor
    pub const RXDESC_0_FS: u32 = 1 << 9;
    /// Last descriptor
    pub const RXDESC_0_LS: u32 = 1 << 8;
    /// Error summary
    pub const RXDESC_0_ES: u32 = 1 << 15;
    /// Frame length
    pub const RXDESC_0_FL_MASK: u32 = 0x3FFF;
    pub const RXDESC_0_FL_SHIFT: usize = 16;

    pub const RXDESC_1_RBS_SHIFT: usize = 0;
    pub const RXDESC_1_RBS_MASK: u32 = 0x0fff << RXDESC_1_RBS_SHIFT;
    /// Second address chained
    pub const RXDESC_1_RCH: u32 = 1 << 14;
    /// End Of Ring
    pub const RXDESC_1_RER: u32 = 1 << 15;
}

use rx_consts::*;

/// Receive Descriptor representation
///
/// * rdes0: OWN and Status
/// * rdes1: byte counts
/// * rdes2: buffer 1 address
/// * rdes3: next descriptor address
#[repr(C)]
struct RDes {
    rdes0: VolatileCell<u32>,
    rdes1: VolatileCell<u32>,
    rdes2: VolatileCell<u32>,
    rdes3: VolatileCell<u32>,
}

impl RDes {
    pub const fn new() -> Self {
        Self {
            rdes0: VolatileCell::new(0),
            rdes1: VolatileCell::new(0),
            rdes2: VolatileCell::new(0),
            rdes3: VolatileCell::new(0),
        }
    }

    /// Return true if this RDes is acceptable to us
    #[inline(always)]
    pub fn valid(&self) -> bool {
        // Write-back descriptor is valid if:
        //
        // Contains first buffer of packet AND contains last buf of
        // packet AND no errors AND not a context descriptor
        self.rdes0.get() & (RXDESC_0_FS | RXDESC_0_LS | RXDESC_0_ES) == (RXDESC_0_FS | RXDESC_0_LS)
    }

    /// Return true if this RDes is not currently owned by the DMA
    #[inline(always)]
    pub fn available(&self) -> bool {
        self.rdes0.get() & RXDESC_0_OWN == 0 // Owned by us
    }

    // #[inline(always)]
    // pub fn set_ready(&mut self, buf_addr: u32) {
    //     self.set_buffer1(buffer, len)
    //     self.set_owned();
    // }

    /// Is owned by the DMA engine?
    fn is_owned(&self) -> bool {
        (self.rdes0.get() & RXDESC_0_OWN) == RXDESC_0_OWN
    }

    /// Pass ownership to the DMA engine
    fn set_owned(&mut self) {
        // "Preceding reads and writes cannot be moved past subsequent writes."

        fence(Ordering::Release);
        self.rdes0.set(self.rdes0.get() | RXDESC_0_OWN);

        // Used to flush the store buffer as fast as possible to make the buffer available for the
        // DMA.
        fence(Ordering::SeqCst);
    }

    fn has_error(&self) -> bool {
        (self.rdes0.get() & RXDESC_0_ES) == RXDESC_0_ES
    }

    /// Descriptor contains first buffer of frame
    fn is_first(&self) -> bool {
        (self.rdes0.get() & RXDESC_0_FS) == RXDESC_0_FS
    }

    /// Descriptor contains last buffers of frame
    fn is_last(&self) -> bool {
        (self.rdes0.get() & RXDESC_0_LS) == RXDESC_0_LS
    }

    fn set_buffer1(&mut self, buffer: *const u8) {
        self.rdes2.set(buffer as u32);
    }

    fn set_buffer1_len(&mut self, len: usize) {
        self.rdes1
            .set((self.rdes1.get() & !RXDESC_1_RBS_MASK) | ((len as u32) << RXDESC_1_RBS_SHIFT));
    }

    // points to next descriptor (RCH)
    fn set_buffer2(&mut self, buffer: *const u8) {
        self.rdes3.set(buffer as u32);
    }

    fn set_end_of_ring(&mut self) {
        self.rdes1.set(self.rdes1.get() | RXDESC_1_RER);
    }

    fn get_frame_len(&self) -> usize {
        ((self.rdes0.get() >> RXDESC_0_FL_SHIFT) & RXDESC_0_FL_MASK) as usize
    }

    pub fn setup(&mut self, next: Option<&Self>) {
        // Defer this initialization to this function, so we can have `RingEntry` on bss.

        self.rdes1.set(RXDESC_1_RCH);
        match next {
            Some(next) => self.set_buffer2(next as *const _ as *const u8),
            None => {
                self.set_buffer2(0 as *const u8);
                self.set_end_of_ring();
            }
        }
    }
}

/// Rx ring of descriptors and packets
///
/// This ring has three major locations that work in lock-step. The DMA will never write to the tail
/// index, so the `read_index` must never pass the tail index. The `next_tail_index` is always 1
/// slot ahead of the real tail index, and it must never pass the `read_index` or it could overwrite
/// a packet still to be passed to the application.
///
///                                                                   nt can't pass r (no alloc)
/// +---+---+---+---+  Read ok       +---+---+---+---+ No Read       +---+---+---+---+
/// |   |   |   |   |  ------------> |   |   |   |   | ------------> |   |   |   |   |
/// +---+---+---+---+  Allocation ok +---+---+---+---+               +---+---+---+---+
///   ^           ^t                   ^t  ^                           ^t  ^
///   |r                                   |r                              |r
///   |nt                                  |nt                             |nt
///
///
/// +---+---+---+---+  Read ok         +---+---+---+---+ Can't read    +---+---+---+---+
/// |   |   |   |   |  ------------>   |   |   |   |   | ------------> |   |   |   |   |
/// +---+---+---+---+  Allocation fail +---+---+---+---+ Allocation ok +---+---+---+---+
///       ^   ^t  ^                              ^t  ^                   ^       ^   ^t
///       |r      |                              |r  |                   |       |r
///               |nt                                |nt                 |nt
///
pub(crate) struct RDesRing<const N: usize> {
    descriptors: [RDes; N],
    buffers: [Option<PacketBox>; N],
    next_idx: usize,
    next_tail_idx: usize,
}

impl<const N: usize> RDesRing<N> {
    pub const fn new() -> Self {
        const RDES: RDes = RDes::new();
        const BUFFERS: Option<PacketBox> = None;

        Self {
            descriptors: [RDES; N],
            buffers: [BUFFERS; N],
            next_idx: 0,
            next_tail_idx: 0,
        }
    }

    pub(crate) fn init(&mut self) {
        assert!(N > 1);
        let mut previous: Option<&mut RDes> = None;
        let mut last_index = 0;
        for (index, buf) in self.buffers.iter_mut().enumerate() {
            let pkt = match PacketBox::new(Packet::new()) {
                Some(p) => p,
                None => {
                    if index == 0 {
                        panic!("Could not allocate at least one buffer for Ethernet receiving");
                    } else {
                        break;
                    }
                }
            };

            self.descriptors[index].set_buffer1(pkt.as_ptr() as *const u8);
            self.descriptors[index].set_owned();
            *buf = Some(pkt);
            last_index = index;
        }
        self.next_tail_idx = (last_index + 1) % N;

        // not sure if this is supposed to span all of the descriptor or just those that contain buffers
        {
            let mut previous: Option<&mut RDes> = None;
            for entry in self.descriptors.iter_mut() {
                if let Some(prev) = &mut previous {
                    prev.setup(Some(entry));
                }
                previous = Some(entry);
            }

            if let Some(entry) = &mut previous {
                entry.setup(None);
            }
        }

        // Register txdescriptor start
        // NOTE (unsafe) Used for atomic writes
        unsafe {
            ETH.ethernet_dma()
                .dmardlar()
                .write(|w| w.0 = &self.descriptors as *const _ as u32);
        };
        // We already have fences in `set_owned`, which is called in `setup`

        // Start receive
        unsafe {
            ETH.ethernet_dma()
                .dmaomr()
                .modify(|w| w.set_sr(DmaomrSr::STARTED))
        };

        self.demand_poll();
    }

    fn demand_poll(&self) {
        unsafe { ETH.ethernet_dma().dmarpdr().write(|w| w.set_rpd(Rpd::POLL)) };
    }

    pub(crate) fn on_interrupt(&mut self) {
        // XXX: Do we need to do anything here ? Maybe we should try to advance the tail ptr, but it
        // would soon hit the read ptr anyway, and we will wake smoltcp's stack on the interrupt
        // which should try to pop a packet...
    }

    pub(crate) fn pop_packet(&mut self) -> Option<PacketBuf> {
        // Not sure if the contents of the write buffer on the M7 can affects reads, so we are using
        // a DMB here just in case, it also serves as a hint to the compiler that we're syncing the
        // buffer (I think .-.)
        fence(Ordering::SeqCst);

        let read_available = self.descriptors[self.next_idx].available();
        let tail_index = (self.next_tail_idx + N - 1) % N;

        let pkt = if read_available && self.next_idx != tail_index {
            let pkt = self.buffers[self.next_idx].take();
            let len = (self.descriptors[self.next_idx].rdes1.get() & RXDESC_1_RBS_MASK) as usize;

            assert!(pkt.is_some());
            let valid = self.descriptors[self.next_idx].valid();

            self.next_idx = (self.next_idx + 1) % N;
            if valid {
                pkt.map(|p| p.slice(0..len))
            } else {
                None
            }
        } else {
            None
        };

        // Try to advance the tail_idx
        if self.next_tail_idx != self.next_idx {
            match PacketBox::new(Packet::new()) {
                Some(b) => {
                    self.descriptors[self.next_tail_idx].set_buffer1(b.as_ptr() as *const u8);
                    self.buffers[self.next_tail_idx].replace(b);
                    self.descriptors[self.next_tail_idx].set_owned();

                    // "Preceding reads and writes cannot be moved past subsequent writes."
                    fence(Ordering::Release);

                    self.next_tail_idx = (self.next_tail_idx + 1) % N;
                }
                None => {}
            }
        }
        pkt
    }
}
