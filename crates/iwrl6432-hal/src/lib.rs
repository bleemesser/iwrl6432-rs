#![no_std]

pub use cortex_m;
pub use xwrl64xx_pac as pac;

pub mod capture;
pub mod clock;
pub mod fecss;
pub mod gpio;
pub mod mcspi;
pub mod pinmux;
pub mod radar;
pub mod sensor;
pub mod time;
pub mod uart;

pub use uart::Uart;

/// Undo the non-default state the ROM bootloader hands off in. Call first thing
/// in `main`, before enabling any interrupt.
///
/// Clears `CCR.UNALIGN_TRP` and `DIV_0_TRP`, `FAULTMASK`, the WIC enable and its
/// latched wakeup, and stray NVIC/SysTick enables and pendings. Only needed on
/// flash boots.
#[inline]
pub fn normalize_after_bootloader() {
    const CCR: usize = 0xE000_ED14; // UNALIGN_TRP=b3, DIV_0_TRP=b4
    const SYST_CSR: usize = 0xE000_E010;
    const ICSR: usize = 0xE000_ED04; // PENDSTCLR (b25) | PENDSVCLR (b27)
    const NVIC_ICER: usize = 0xE000_E180;
    const NVIC_ICPR: usize = 0xE000_E280;
    const WIC_STAT_CLR: usize = 0x5606_0384; // APP_CTRL: clear latched wakeup
    const WICEN: usize = 0x5606_038C; // APP_CTRL: WIC enable
    unsafe {
        let ccr = core::ptr::read_volatile(CCR as *const u32);
        core::ptr::write_volatile(CCR as *mut u32, ccr & !0x18);
        core::ptr::write_volatile(WIC_STAT_CLR as *mut u32, 0xFFFF_FFFF);
        core::ptr::write_volatile(WICEN as *mut u32, 0);
        core::ptr::write_volatile(SYST_CSR as *mut u32, 0);
        core::ptr::write_volatile(ICSR as *mut u32, (1 << 25) | (1 << 27));
        // disable and clear-pending all 64 external interrupt lines
        for i in 0..2 {
            core::ptr::write_volatile((NVIC_ICER + i * 4) as *mut u32, 0xFFFF_FFFF);
            core::ptr::write_volatile((NVIC_ICPR + i * 4) as *mut u32, 0xFFFF_FFFF);
        }
        core::arch::asm!("cpsie i", "cpsie f", options(nomem, nostack, preserves_flags));
    }
}
