use cortex_m::asm;

// Peripheral bases.
const TOP_PRCM: usize = 0x5A04_0000; // FEC power domain
const FEC_CTRL: usize = 0x5200_0000; // FECSS control (meminit, IPC, M3 halt)
const FEC_RCM: usize = 0x5202_0000; // FECSS clock/reset
const RFS_IPC: usize = 0x2120_0000; // RFS <-> M4 shared-RAM IPC window
const APP_FEC_CLK_GATE: usize = 0x5606_0394; // APPSS-side FECSS root clock gate

// TOP_PRCM registers.
const FEC_PWR_REQ_PARAM: usize = TOP_PRCM + 0x00C;
const FEC_CORE_SYSRESET_PARAM: usize = TOP_PRCM + 0x018;
const PSCON_FEC_PD_EN: usize = TOP_PRCM + 0x458;
/// FRC (Frame-Reference-Counter) OSC clock gate (0=running, 7=gated).
/// Un-gating this is what delivers frame-start events into the FECSS.
const FRC_OSC_CLK_GATE: usize = TOP_PRCM + 0x504;
const PSCON_FEC_PD_RAM_STATE: usize = TOP_PRCM + 0x474;
const PSCON_FEC_PD_GRP4_STATE: usize = TOP_PRCM + 0x478;

const PWR_MODE_MANUAL: u32 = 1 << 11; // FEC_PWR_REQ_PARAM.MODE (resets to auto)
const PWR_WAKEUP_OUT: u32 = 1 << 12;
const PD_ON_STATUS_MASK: u32 = 0x300; // PSCON_FEC_PD_EN [9:8]
const PD_ON_STATUS_UP: u32 = 0x100;

// FEC_CTRL registers (meminit + M3 halt handshake).
const FECSS_MEM_INIT_SLICE_SEL: usize = FEC_CTRL + 0x030;
const FECSS_RAM_MEM_INIT: usize = FEC_CTRL + 0x034;
const FECSS_RAM_MEM_DONE: usize = FEC_CTRL + 0x038;
const FECSS_TIMING_ENGINE_MEM_INIT: usize = FEC_CTRL + 0x04C;
const FECSS_TIMING_ENGINE_MEM_DONE: usize = FEC_CTRL + 0x050;
const CM3_CPU_HALT_HANDSHAKE: usize = FEC_CTRL + 0x100;

const MEM_DONE: u32 = 1 << 0;

// FEC_RCM registers (FECSS clocks).
const FEC_SYS_CLKCTL: usize = FEC_RCM + 0x004;
const FEC_IPCFGCLKGATE0: usize = FEC_RCM + 0x034;
const FEC_IPCFGCLKGATE1: usize = FEC_RCM + 0x038;
/// sensor::start re-asserts the DFE un-gate: the RFS reprograms GATE1 during
/// boot, so the power_on-time write does not survive.
pub(crate) const FEC_IPCFGCLKGATE1_PUB: usize = FEC_IPCFGCLKGATE1;

/// Ungate DFE + chirp-timer IP clocks (fe_driver.c).
const IPCFGCLKGATE0_ON: u32 = 0x381C_0038;
/// GATE1 value that ungates FecDfe[8:6]; the reset default leaves it gated,
/// and a gated DFE never writes the ADCBUF.
const IPCFGCLKGATE1_ON: u32 = 0x7007;

// RFS IPC shared-RAM offsets.
const RFS_MAILBOX: usize = RFS_IPC + 0x000; // w_IpcMailbox[32], 128 B
const RFS_BOOT_INFO: usize = RFS_IPC + 0x100;
const RFS_BOOT_STS: usize = RFS_IPC + 0x108;
const RFS_CPU_FAULT_STS: usize = RFS_IPC + 0x120;
/// Temp-measurement config is a plain RAM write (z_TempMeasCfgCmd), not a
/// mailbox command.
const RFS_TEMP_CFG: usize = RFS_IPC + 0x19C;

// Mailbox trigger/handshake registers (rfs_rfsMbInit).
const IPC_CMD_TRIG: usize = 0x5606_0028; // M4 -> FECSS command doorbell
const FECSS_IPC_RFS: usize = FEC_CTRL + 0x05C; // FECSS -> M4 response status
const FECSS_IPC_BUSY_INT0: usize = FEC_CTRL + 0x060; // busy/handshake flag

// Mailbox handshake constants (rfs_driver.h).
const MB_SEND_TRIG: u32 = 0xA1;
const MB_READ_ACK: u32 = 0x50;
const MB_MAX_PAYLOAD_WORDS: usize = 31; // 124 B minus header

// RFS command IDs (rfs_command.h).
const CMD_RF_PWR_ONOFF: u16 = 0x0001;
const CMD_FACT_CAL: u16 = 0x0002;
const CMD_APLL_CLK_CTRL: u16 = 0x0003;
const CMD_RUNTIME_CAL: u16 = 0x0004;
const CMD_TEMP_TRIG: u16 = 0x0007;

// APLL_CLK_CTRL field values (dfp_device.h).
/// c_DevClkCtrl: FECSS clock source = fast clock.
pub const DEV_CLK_FCLK: u8 = 0x0A;
/// c_DevClkCtrl: FECSS clock source = XTAL (40 MHz boot clock).
pub const DEV_CLK_XTAL: u8 = 0x00;
/// c_FtClkCtrl: frame-timer clock source = XTAL.
pub const FT_CLK_XTAL: u8 = 0x00;
/// c_ApllClkCtrl: APLL on + calibrate (first boot).
pub const APLL_ON_CAL: u8 = 0xAA;
/// c_ApllClkCtrl: APLL on without recalibrating (warm re-enable).
pub const APLL_ON: u8 = 0x0A;

// FEC_SYS_CLKCTL FCLK config (fe_fecssClockSwitch): divider [27:16]=0x111,
// source [15:4]=0x333 (DIG_PLL /2 = 80 MHz); divider written first.
const FEC_FCLK_DIVR: u32 = 0x111;
const FEC_FCLK_SRCSEL: u32 = 0x333;

/// Calibration mask the SDK cold boot runs (VCO, PD, LODIST, RX-gain,
/// TX-power); FACT_CAL and RUNTIME_CAL echo it in run_status on success.
pub const CAL_MASK_COLDBOOT: u16 = 0x00CE;

/// c_TempBinIndex values for RUNTIME_CAL (dfp_device.h).
pub const TEMP_BIN_LOW: u8 = 0x00; // < 0 deg C
pub const TEMP_BIN_MID: u8 = 0x08; // 0..84 deg C
pub const TEMP_BIN_HIGH: u8 = 0x10; // >= 85 deg C

/// Boot-info word0: h_ClkFreq=0x5000 (80 MHz FCLK), c_PwrModeAndXtalType=0x30
/// (cold boot, 40 MHz XTAL), c_RfsApiCtrl=0x02 (RFS debug logger on; log
/// appears at RFS_IPC+0x7C0).
const BOOT_INFO_W0_FCLK: u32 = 0x0230_5000;
/// Boot-info h_ClkFreq for 80 MHz FCLK (rfs_rfsClkCfg).
const RFS_CLK_FREQ_80M: u32 = 20480;

/// M3 un-halt magic (M_FE_RFS_CM3_FW_UNHALT_VAL).
const CM3_UNHALT_MAGIC: u32 = 0xF5A3_6A17;
/// FEC_CORE_SYSRESET_PARAM value that releases the M3.
const FEC_CORE_UNHALT: u32 = 0x0002_17FF;

// Busy-delay cycle counts at 200 MHz.
const CYC_300US: u32 = 60_000;
const CYC_500US: u32 = 100_000;

#[inline]
fn rd(addr: usize) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}
#[inline]
fn wr(addr: usize, v: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, v) };
}
#[inline]
fn rmw(addr: usize, clear: u32, set: u32) {
    wr(addr, (rd(addr) & !clear) | set);
}

/// Poll `addr` until `(read & mask) == want`; false on timeout.
#[inline]
fn wait_until(addr: usize, mask: u32, want: u32, tries: u32) -> bool {
    for _ in 0..tries {
        if rd(addr) & mask == want {
            return true;
        }
    }
    false
}

/// RFS M3 boot status (0x21200108 low byte).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum BootStatus {
    /// 0x00: still booting (or never released).
    Uninit,
    /// 0x0A: booted OK.
    Pass,
    /// 0x05: self-test failed.
    Fail,
    Other(u8),
}

impl BootStatus {
    fn read() -> Self {
        match (rd(RFS_BOOT_STS) & 0xFF) as u8 {
            0x00 => BootStatus::Uninit,
            0x0A => BootStatus::Pass,
            0x05 => BootStatus::Fail,
            b => BootStatus::Other(b),
        }
    }
}

/// Which phase of [`power_on`] timed out.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PowerError {
    PowerDown,
    PowerUp,
    MemInit,
}

// Domain transitions take ~hundreds of us; allow a few ms.
const POLL_TRIES: u32 = 1_000_000;

/// Power the FEC domain, ungate its clocks, switch it to 80 MHz FCLK, and
/// initialise the FECSS RAMs (fe_fecssOpen). Idempotent: power-cycles the
/// domain to a known state. The RFS M3 stays halted; call [`rfs_boot`] next.
pub fn power_on() -> Result<(), PowerError> {
    // Manual power mode, no RAM retention (cold start).
    rmw(FEC_PWR_REQ_PARAM, PWR_MODE_MANUAL, 0);
    rmw(PSCON_FEC_PD_RAM_STATE, 0x1, 0);
    rmw(PSCON_FEC_PD_GRP4_STATE, 0x3, 0);
    // Power OFF, wait down; then ON, wait up.
    rmw(FEC_PWR_REQ_PARAM, PWR_WAKEUP_OUT, 0);
    if !wait_until(PSCON_FEC_PD_EN, 0x200, 0x200, POLL_TRIES) {
        return Err(PowerError::PowerDown);
    }
    rmw(FEC_PWR_REQ_PARAM, 0, PWR_WAKEUP_OUT);
    if !wait_until(
        PSCON_FEC_PD_EN,
        PD_ON_STATUS_MASK,
        PD_ON_STATUS_UP,
        POLL_TRIES,
    ) {
        return Err(PowerError::PowerUp);
    }
    asm::delay(CYC_300US);

    // Ungate the FECSS root clock, then the IP clocks.
    wr(APP_FEC_CLK_GATE, 0);
    asm::delay(CYC_500US);
    wr(FEC_IPCFGCLKGATE0, IPCFGCLKGATE0_ON);
    wr(FEC_IPCFGCLKGATE1, IPCFGCLKGATE1_ON);
    // Switch the FECSS system clock to FCLK BEFORE mem init and the RFS boot
    // (as fe_fecssOpen does) so the RFS boots and runs its frame-service
    // timing at 80 MHz. Divider field first, then source.
    rmw(FEC_SYS_CLKCTL, 0x0FFF_0000, FEC_FCLK_DIVR << 16);
    rmw(FEC_SYS_CLKCTL, 0x0000_FFF0, FEC_FCLK_SRCSEL << 4);

    // Initialise the FECSS data + chirp-timer RAMs.
    wr(FECSS_TIMING_ENGINE_MEM_DONE, MEM_DONE); // clear done
    wr(FECSS_RAM_MEM_DONE, MEM_DONE);
    wr(FECSS_MEM_INIT_SLICE_SEL, 0x1);
    wr(FECSS_RAM_MEM_INIT, 0x1);
    wr(FECSS_TIMING_ENGINE_MEM_INIT, 0x1);
    for _ in 0..POLL_TRIES {
        if rd(FECSS_RAM_MEM_DONE) & rd(FECSS_TIMING_ENGINE_MEM_DONE) & MEM_DONE != 0 {
            return Ok(());
        }
    }
    Err(PowerError::MemInit)
}

/// Read the RFS M3 boot-status word.
pub fn boot_status() -> BootStatus {
    BootStatus::read()
}

/// Decoded RFS fault block (T_RL_API_RFS_FAULT_STS_GET_RSP at 0x21200120).
/// h_RfsFwState: 0x8008 idle, 0x4004 booting, a cmd id while that command
/// executes, 0xFFFF dead.
pub struct RfsFault {
    pub fw_state: u16,
    pub patch: u8,
    /// 0x81 CPU fault, 0x82 FW assert.
    pub fault_type: u8,
    /// e.g. 0x81 stray MBOX interrupt, 0x83 frame during cal/monitor.
    pub error_code: u8,
    pub line: u16,
    pub abort_pc: u32,
    pub abort_lr: u32,
    pub cfsr: u32,
    pub hfsr: u32,
    pub exception_count: u8,
}

impl RfsFault {
    /// A faulted RFS is dead: it will never service frames again.
    pub fn is_faulted(&self) -> bool {
        self.fault_type != 0 || self.exception_count != 0
    }
}

pub fn rfs_fault_status() -> RfsFault {
    let w0 = rd(RFS_IPC + 0x120);
    let w2 = rd(RFS_IPC + 0x128);
    RfsFault {
        fw_state: (w0 & 0xFFFF) as u16,
        patch: ((w0 >> 16) & 0xFF) as u8,
        fault_type: (w2 & 0xFF) as u8,
        error_code: ((w2 >> 8) & 0xFF) as u8,
        line: ((w2 >> 16) & 0xFFFF) as u16,
        abort_pc: rd(RFS_IPC + 0x12C),
        abort_lr: rd(RFS_IPC + 0x130),
        cfsr: rd(RFS_IPC + 0x13C),
        hfsr: rd(RFS_IPC + 0x140),
        exception_count: (rd(RFS_IPC + 0x150) & 0xFF) as u8,
    }
}

/// Init the RFS mailbox, write boot-info, un-halt the RFS M3, poll boot
/// status. Returns [`BootStatus::Uninit`] on timeout. Requires the RFS patch
/// RPRC to be resident at 0x21204000 (the flash appimage bundles it; the ROM
/// self-test never reaches Pass without it).
pub fn rfs_boot() -> BootStatus {
    // Zero the 2 KB RFS IPC RAM: the RFS reads config mirrors from it.
    for i in 0..(0x800 / 4) {
        wr(RFS_IPC + i * 4, 0);
    }

    // rfs_rfsMbInit: zero the mailbox, clear the trigger/busy regs.
    for i in 0..32 {
        wr(RFS_MAILBOX + i * 4, 0);
    }
    rmw(IPC_CMD_TRIG, 0xFF, 0);
    rmw(FECSS_IPC_RFS, 0xFF, 0);
    wr(FECSS_IPC_BUSY_INT0, 0x1);

    wr(RFS_BOOT_INFO, BOOT_INFO_W0_FCLK);
    wr(RFS_BOOT_INFO + 4, 0);
    // Clear stale fault/boot status so an old 0xA can't pass the poll.
    rmw(RFS_CPU_FAULT_STS, 0x0000_FFFF, 0);
    rmw(RFS_BOOT_STS, 0x0000_00FF, 0);
    // Un-halt the M3.
    wr(CM3_CPU_HALT_HANDSHAKE, CM3_UNHALT_MAGIC);
    wr(FEC_CORE_SYSRESET_PARAM, FEC_CORE_UNHALT);

    // Boot takes ~120 us typical; budget ~2 ms.
    for _ in 0..200 {
        match BootStatus::read() {
            BootStatus::Uninit => asm::delay(2_000), // ~10 us
            other => return other,
        }
    }
    BootStatus::read()
}

/// Why a mailbox round-trip failed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum MailboxError {
    /// Busy flag set before send: a prior response was never consumed.
    Busy,
    /// Payload exceeds the 124-byte mailbox.
    TooLarge,
    /// No response within the poll budget.
    Timeout,
    /// RFS error response (cmd id echoed as 0), with its xw_CmdErrorCode.
    CmdError(i32),
    /// Response cmd id didn't match the command sent.
    Protocol { got: u16, resp_size: u16 },
}

/// One command over the RFS mailbox, polled (no IRQ 18): busy pre-check ->
/// header + payload -> doorbell -> wait for COMPLETION -> read -> ack.
///
/// COMPLETION, not accept: both the busy bit and the 0x5200005C status byte
/// are raised when the RFS *accepts* a command, but slow commands (cals,
/// temp) finish tens of ms later. Completion = status byte set AND
/// h_RfsFwState no longer holds this cmd id. Reading + acking at accept
/// returns garbage and leaves the real completion unconsumed, after which the
/// RFS FW-asserts ("stray MBOX interrupt") on the next doorbell or frame-start
/// event.
///
/// On success up to `resp.len()` response words are copied out and the
/// response payload size (bytes) is returned.
pub fn mailbox_command(
    cmd_id: u16,
    payload: &[u32],
    resp: &mut [u32],
) -> Result<u16, MailboxError> {
    if payload.len() > MB_MAX_PAYLOAD_WORDS {
        return Err(MailboxError::TooLarge);
    }
    if rd(FECSS_IPC_BUSY_INT0) & 0x1 != 0 {
        return Err(MailboxError::Busy);
    }

    // Header word0 = cmdId | (payload_bytes << 16), then the payload.
    let size_bytes = (payload.len() * 4) as u32;
    wr(RFS_MAILBOX, cmd_id as u32 | (size_bytes << 16));
    for (i, &w) in payload.iter().enumerate() {
        wr(RFS_MAILBOX + 4 + i * 4, w);
    }

    // Clear stale response status, ring the doorbell (both RMW low byte).
    rmw(FECSS_IPC_RFS, 0xFF, 0);
    rmw(IPC_CMD_TRIG, 0xFF, MB_SEND_TRIG);

    // Wait for completion; ~2 s budget (calibrations are the slow ones).
    let mut posted = false;
    for _ in 0..200_000 {
        if rd(FECSS_IPC_RFS) & 0xFF != 0 && rd(RFS_IPC + 0x120) & 0xFFFF != cmd_id as u32 {
            posted = true;
            break;
        }
        asm::delay(2_000); // ~10 us
    }
    if !posted {
        return Err(MailboxError::Timeout);
    }
    if rd(FECSS_IPC_BUSY_INT0) & 0x1 == 0 {
        return Err(MailboxError::Timeout); // response-ready precondition (rfs_rfsResGet)
    }

    // RFS echoes cmd_id on success, writes 0x0000 on error.
    let w0 = rd(RFS_MAILBOX);
    let got = (w0 & 0xFFFF) as u16;
    let resp_size = ((w0 >> 16) & 0xFFFF) as u16;
    let result = if got == 0x0000 {
        Err(MailboxError::CmdError(rd(RFS_MAILBOX + 4) as i32))
    } else if got != cmd_id {
        Err(MailboxError::Protocol { got, resp_size })
    } else {
        let words = (resp_size as usize / 4).min(resp.len());
        for (i, slot) in resp.iter_mut().take(words).enumerate() {
            *slot = rd(RFS_MAILBOX + 4 + i * 4);
        }
        Ok(resp_size)
    };

    // Ack and clear busy so the next command can be sent.
    rmw(IPC_CMD_TRIG, 0xFF, MB_READ_ACK);
    wr(FECSS_IPC_BUSY_INT0, 0x1);
    result
}

/// RF_PWR_ONOFF (0x0001): `rx_mask` bits[2:0] arm the RX chains, `tx_mask`
/// bits[1:0] the TX chains, `misc` = c_MiscCtrl (bit0 RDIF clock - needs the
/// APLL up first; bit1 1V LDO bypass). All-zero powers the RF off.
pub fn rf_pwr_onoff(rx_mask: u16, tx_mask: u16, misc: u8) -> Result<(), MailboxError> {
    // T_RL_API_FECSS_RF_PWR_CFG_CMD (12 B).
    let payload = [
        rx_mask as u32 | ((tx_mask as u32) << 16),
        (misc as u32) << 8,
        0,
    ];
    mailbox_command(CMD_RF_PWR_ONOFF, &payload, &mut []).map(|_| ())
}

/// Select the on-die temperature sensors for [`temp_trig`]. Plain RAM write
/// of T_RL_API_FECSS_TEMP_MEAS_CMD; SDK uses 0x311 = RX + TX + PM + DIG.
pub fn temp_config(ctrl_mask: u16) {
    wr(RFS_TEMP_CFG, ctrl_mask as u32);
    wr(RFS_TEMP_CFG + 4, 0); // struct is 8 B; remainder reserved
}

/// Result of a [`temp_trig`] measurement.
pub struct TempMeas {
    /// h_TempStsBitMask: 2 status bits per sensor (0b01 = valid).
    pub status: u16,
    /// Per-sensor deg C. Indices: RX0..2 = 0..2, TX0..1 = 4..5, PM = 8, DIG = 9.
    pub temp_c: [i16; 10],
}

/// TEMP_TRIG (0x0007): measure the sensors selected by [`temp_config`].
/// Needs the RF/PM analog powered for meaningful numbers.
pub fn temp_trig() -> Result<TempMeas, MailboxError> {
    let mut resp = [0u32; 7]; // T_RL_API_FECSS_TEMP_MEAS_RSP = 28 B
    mailbox_command(CMD_TEMP_TRIG, &[], &mut resp)?;
    let mut temp_c = [0i16; 10];
    for (i, t) in temp_c.iter_mut().enumerate() {
        let word = resp[1 + i / 2];
        let half = if i % 2 == 0 {
            word & 0xFFFF
        } else {
            word >> 16
        };
        *t = half as i16;
    }
    Ok(TempMeas {
        status: (resp[0] & 0xFFFF) as u16,
        temp_c,
    })
}

/// RUNTIME_CAL temperature bin for a measured temperature.
pub fn temp_bin_for(temp_c: i16) -> u8 {
    if temp_c < 0 {
        TEMP_BIN_LOW
    } else if temp_c < 85 {
        TEMP_BIN_MID
    } else {
        TEMP_BIN_HIGH
    }
}

/// APLL_CLK_CTRL (0x0003): configure the FECSS clocks and bring up the APLL.
/// Three ordered actions like SDK rl_fecssDevClockCtrl: (1) M4-side FECSS
/// core-clock switch + RFS clock-freq hint, (2) un-gate the FRC OSC clock -
/// what delivers frame-start events into the FECSS, (3) the mailbox command.
pub fn apll_clk_ctrl(dev_clk: u8, ft_clk: u8, apll: u8) -> Result<(), MailboxError> {
    if dev_clk == DEV_CLK_FCLK {
        rmw(FEC_SYS_CLKCTL, 0x0FFF_0000, FEC_FCLK_DIVR << 16);
        rmw(FEC_SYS_CLKCTL, 0x0000_FFF0, FEC_FCLK_SRCSEL << 4);
        rmw(RFS_BOOT_INFO, 0x0000_FFFF, RFS_CLK_FREQ_80M);
    }
    wr(
        FRC_OSC_CLK_GATE,
        if ft_clk == FT_CLK_XTAL { 0 } else { 0x7 },
    );
    // T_RL_API_FECSS_DEV_CLK_CTRL_CMD (8 B).
    let payload = [
        dev_clk as u32 | ((ft_clk as u32) << 8) | ((apll as u32) << 16),
        0,
    ];
    mailbox_command(CMD_APLL_CLK_CTRL, &payload, &mut []).map(|_| ())
}

/// APLL up with calibration, SDK cold-boot clock config (FECSS on FCLK,
/// frame timer on XTAL). Must follow [`rf_pwr_onoff`], precede calibration.
pub fn apll_on() -> Result<(), MailboxError> {
    apll_clk_ctrl(DEV_CLK_FCLK, FT_CLK_XTAL, APLL_ON_CAL)
}

/// Result of [`fact_cal`] / [`runtime_cal`].
pub struct CalStatus {
    /// h_CalRunStatus: this trigger's per-cal result bits; equals the
    /// requested mask ([`CAL_MASK_COLDBOOT`]) on success.
    pub run_status: u16,
    /// h_CalResStatus: cumulative cal validity bits.
    pub res_status: u16,
    /// Temperature at calibration, deg C ([`fact_cal`] only).
    pub calib_temp_c: i16,
}

/// FACT_CAL (0x0002) with the SDK factory_cal.c parameters: RxGain 0x28
/// (40 dB), TxBackoff 0, CalRfFreq = band center, TxPwrCal masks {3,1}. Other
/// parameter sets leave res_status without its validity bits, and the RFS then
/// asserts on the first frame-start. Must follow [`apll_on`].
pub fn fact_cal() -> Result<CalStatus, MailboxError> {
    // T_RL_API_FECSS_RF_FACT_CAL_CMD (36 B).
    let payload = [
        0x2800_00CE, // CalCtrl 0xCE | MiscCal 0x00 | RxGainSel 0x28
        0x0000_0000, // CalTxBackOffSel[4]
        0x0000_0000, // reserved
        0x004D_C9D0, // CalRfFreq 0xC9D0 (center) | CalRfSlope 0x004D (2.2 MHz/us)
        0x0000_0103, // TxPwrCalTxEnaMask[4] = {3, 1, 0, 0}
        0x0000_0000, // reserved
        0x0000_0000, // CalTempBinOverrides[3] + reserved
        0x0000_0000,
        0x0000_0000,
    ];
    let mut resp = [0u32; 3]; // T_RL_API_FECSS_RF_FACT_CAL_RSP = 12 B
    mailbox_command(CMD_FACT_CAL, &payload, &mut resp)?;
    Ok(CalStatus {
        run_status: (resp[0] & 0xFFFF) as u16,
        res_status: (resp[0] >> 16) as u16,
        calib_temp_c: ((resp[1] & 0xFF) as i8 as i16) * 2, // 1 LSB = 2 deg C
    })
}

/// RUNTIME_CAL (0x0004) for `temp_bin` ([`temp_bin_for`]). Needs a prior
/// [`fact_cal`]. Mask 0xCA = no PD cal: the SDK enables runtime PD cal only
/// for safety/CLPC apps, and requesting it with no monitor configured asserts
/// the RFS.
pub fn runtime_cal(temp_bin: u8) -> Result<CalStatus, MailboxError> {
    // T_RL_API_FECSS_RF_RUN_CAL_CMD (20 B).
    let payload = [0x00CA_u32, temp_bin as u32, 0, 0, 0];
    let mut resp = [0u32; 3];
    mailbox_command(CMD_RUNTIME_CAL, &payload, &mut resp)?;
    Ok(CalStatus {
        run_status: (resp[0] & 0xFFFF) as u16,
        res_status: (resp[0] >> 16) as u16,
        calib_temp_c: 0,
    })
}
