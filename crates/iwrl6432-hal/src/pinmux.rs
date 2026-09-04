use crate::pac;

#[inline]
fn base() -> usize {
    pac::TopIoMux::PTR as usize
}

const KICK0_UNLOCK: u32 = 0x83E7_0B13;
const KICK1_UNLOCK: u32 = 0x95A4_F1E0;

/// `PADxx_CFG_REG.PIN_PULL_DISABLE` (bit 8).
pub const PIN_PULL_DISABLE: u32 = 1 << 8;

/// A device pad, named for its `PAD_Ax` ball. The `PADxx_CFG_REG` registers are
/// a contiguous u32 array from `PADAA`, so the discriminant is the index.
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Pad {
    Aa = 0, Ab, Ac, Ad, Ae, Af, Ag, Ah, Ai, Aj, Ak, Al,
    Am, An, Ao, Ap, Aq, Ar, As, At, Au, Av, Aw, Ax,
}

/// Pad config value for function `mode` with pulls disabled.
#[inline]
pub const fn pad_cfg(mode: u32) -> u32 {
    mode | PIN_PULL_DISABLE
}

/// Write `value` to a pad's `PADxx_CFG_REG`. Writes are ignored unless the two
/// KICK registers hold the unlock magics, so all writes must go through here.
pub fn configure(dp: &pac::Peripherals, pad: Pad, value: u32) {
    let reg = (base() + (pad as usize) * 4) as *mut u32;
    dp.top_io_mux
        .iocfgkick0()
        .write(|w| unsafe { w.bits(KICK0_UNLOCK) });
    dp.top_io_mux
        .iocfgkick1()
        .write(|w| unsafe { w.bits(KICK1_UNLOCK) });
    unsafe { core::ptr::write_volatile(reg, value) };
    dp.top_io_mux.iocfgkick1().write(|w| unsafe { w.bits(0) });
    dp.top_io_mux.iocfgkick0().write(|w| unsafe { w.bits(0) });
}

/// Read a pad's current `PADxx_CFG_REG` (no unlock needed for reads).
pub fn read(pad: Pad) -> u32 {
    let reg = (base() + (pad as usize) * 4) as *const u32;
    unsafe { core::ptr::read_volatile(reg) }
}
