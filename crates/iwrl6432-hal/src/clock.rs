// the 3-bit per-peripheral clock-gate fields are inverted: 0x0 enables, 0x7 (reset) gates

use crate::pac;

/// Board crystal (40 MHz): source for the UART and RTI functional clocks.
pub const XTAL_HZ: u32 = 40_000_000;
/// M4F core clock after [`init_core`] locks the PLL and switches to it.
pub const CORE_CLK_HZ: u32 = 200_000_000;
/// UART functional clock (the 40 MHz XTAL, /1).
pub const UART_CLK_HZ: u32 = XTAL_HZ;
/// RTI timer functional clock (the 40 MHz OSC, /1).
pub const RTI_CLK_HZ: u32 = XTAL_HZ;

/// Lock the digital PLL and run the M4 (and its peripheral clock tree) from it.
/// Call once, before anything else. Peripheral accesses bus-fault until it runs.
pub fn init_core(dp: &pac::Peripherals) {
    const LOCKMON: u32 = 1 << 8; // PLLDIG_STATUS lock bit
    if dp.plldig_ctrl.plldig_status().read().bits() & LOCKMON == 0 {
        dp.plldig_ctrl.plldig_en().write(|w| unsafe { w.bits(0x0) });
        dp.plldig_ctrl
            .plldig_mdiv_ndiv()
            .write(|w| unsafe { w.bits(0x0014_00A0) });
        dp.plldig_ctrl
            .plldig_mode_en()
            .write(|w| unsafe { w.bits(0x0001_0001) });
        dp.plldig_ctrl
            .plldig_ctrl()
            .write(|w| unsafe { w.bits(0x0004_5100) });
        dp.plldig_ctrl.plldig_en().write(|w| unsafe { w.bits(0x7) });
        while dp.plldig_ctrl.plldig_status().read().bits() & LOCKMON == 0 {}
    }

    // M4 clock source to PLL, then wait for CURRCLK to read 0x8 (APLL/DPLL)
    dp.app_rcm
        .app_cpu_clkctl()
        .modify(|r, w| unsafe { w.bits((r.bits() & !0x0000_FFF0) | (0x333 << 4)) });
    while (dp.app_rcm.app_cpu_clkstat().read().bits() & 0x0000_0FF0) >> 4 != 0x8 {}

    errata_ana52_ldo();
}

/// Un-gate UARTB (UART1): `IPCFGCLKGATE1` [5:3] and `APP_UART_1_CLKCTL` [3:0]
/// -> 0, divider [27:16] -> 0 (/1, 40 MHz).
pub fn enable_uart1(dp: &pac::Peripherals) {
    dp.app_rcm
        .ipcfgclkgate1()
        .modify(|r, w| unsafe { w.bits(r.bits() & !0x0000_0038) });
    dp.app_rcm
        .app_uart_1_clkctl()
        .modify(|r, w| unsafe { w.bits(r.bits() & !0x0FFF_000F) });
}

/// Un-gate the GIO (GPIO) controller: `IPCFGCLKGATE2.GIO` [2:0] -> 0.
pub fn enable_gpio(dp: &pac::Peripherals) {
    dp.app_rcm
        .ipcfgclkgate2()
        .modify(|r, w| unsafe { w.bits(r.bits() & !0x0000_0007) });
}

/// Un-gate the HWASS functional clock, which backs the ADCBUF. While gated the
/// config bus still works but the DFE's ADC writes never commit. Pair with
/// [`crate::sensor::hwass_mem_init`] before capturing.
pub fn enable_hwass(dp: &pac::Peripherals) {
    // two separate enables, and the IPCFGCLKGATE2 field must not be pulsed
    dp.app_rcm
        .ipcfgclkgate2()
        .modify(|r, w| unsafe { w.bits(r.bits() & !0x0000_01C0) });
    const HWASS_CLK_GATE: *mut u32 = 0x5606_03B8 as *mut u32;
    unsafe { core::ptr::write_volatile(HWASS_CLK_GATE, 0x7) };
}

/// Clock the RTI timer from the 40 MHz OSC (/1) and un-gate it
/// (`APP_RTI_CLKCTL` -> 0, `IPCFGCLKGATE0.APP_RTI` [20:18] -> 0).
pub fn enable_rti(dp: &pac::Peripherals) {
    dp.app_rcm.app_rti_clkctl().write(|w| unsafe { w.bits(0) });
    dp.app_rcm
        .ipcfgclkgate0()
        .modify(|r, w| unsafe { w.bits(r.bits() & !0x001C_0000) });
}

/// Apply errata ANA-52: clear the Slicer LDO TLOAD once the oscillator is up.
/// PG1 silicon only, a no-op elsewhere.
fn errata_ana52_ldo() {
    const EFUSE0_ROW_18: *const u32 = (0x5A02_0000 + 0x38) as *const u32;
    const LDO_CLKTOP: *mut u32 = 0x5A04_0520 as *mut u32;
    let pg_version = unsafe { core::ptr::read_volatile(EFUSE0_ROW_18) } & 0xF;
    if pg_version == 1 {
        unsafe {
            let v = core::ptr::read_volatile(LDO_CLKTOP);
            core::ptr::write_volatile(LDO_CLKTOP, (v & !0xE000) | 0x2000);
        }
    }
}
