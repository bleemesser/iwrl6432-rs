use cortex_m::asm;

const TOP_PRCM: usize = 0x5A04_0000; // FEC power domain
const FEC_CTRL: usize = 0x5200_0000; // FECSS control (meminit, IPC, M3 halt)
const FEC_RCM: usize = 0x5202_0000; // FECSS clock/reset
const RFS_IPC: usize = 0x2120_0000; // RFS <-> M4 shared-RAM IPC window
const APP_FEC_CLK_GATE: usize = 0x5606_0394; // APPSS-side FECSS root clock gate

// TOP_PRCM registers.
const FEC_PWR_REQ_PARAM: usize = TOP_PRCM + 0x00C;
const FEC_CORE_SYSRESET_PARAM: usize = TOP_PRCM + 0x018;
const PSCON_FEC_PD_EN: usize = TOP_PRCM + 0x458;
/// Frame-reference-counter OSC clock gate (0 running, 7 gated). Un-gating this
/// is what delivers frame-start events into the FECSS.
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
/// Exposed so [`crate::sensor::start`] can re-assert the DFE un-gate, since the
/// RFS reprograms GATE1 during boot and the [`power_on`] write does not survive.
pub(crate) const FEC_IPCFGCLKGATE1_PUB: usize = FEC_IPCFGCLKGATE1;

/// Un-gate the DFE and chirp-timer IP clocks.
const IPCFGCLKGATE0_ON: u32 = 0x381C_0038;
/// Un-gate `FecDfe`, which resets to gated, and a gated DFE never writes the
/// ADCBUF.
const IPCFGCLKGATE1_ON: u32 = 0x7007;

// RFS IPC shared-RAM offsets.
const RFS_MAILBOX: usize = RFS_IPC + 0x000; // w_IpcMailbox[32], 128 B
const RFS_BOOT_INFO: usize = RFS_IPC + 0x100;
const RFS_BOOT_STS: usize = RFS_IPC + 0x108;
const RFS_CPU_FAULT_STS: usize = RFS_IPC + 0x120;
/// Temp-measurement config is a plain RAM write, not a mailbox command.
const RFS_TEMP_CFG: usize = RFS_IPC + 0x19C;

const IPC_CMD_TRIG: usize = 0x5606_0028; // M4 -> FECSS command doorbell
const FECSS_IPC_RFS: usize = FEC_CTRL + 0x05C; // FECSS -> M4 response status
const FECSS_IPC_BUSY_INT0: usize = FEC_CTRL + 0x060; // busy/handshake flag

const MB_SEND_TRIG: u32 = 0xA1;
const MB_READ_ACK: u32 = 0x50;
const MB_MAX_PAYLOAD_WORDS: usize = 31; // 124 B minus header

const CMD_RF_PWR_ONOFF: u16 = 0x0001;
const CMD_FACT_CAL: u16 = 0x0002;
const CMD_APLL_CLK_CTRL: u16 = 0x0003;
const CMD_RUNTIME_CAL: u16 = 0x0004;
const CMD_TEMP_TRIG: u16 = 0x0007;

/// Run the FECSS off the fast clock.
pub const DEV_CLK_FCLK: u8 = 0x0A;
/// Run the FECSS off the 40 MHz XTAL boot clock.
pub const DEV_CLK_XTAL: u8 = 0x00;
/// Run the frame timer off the XTAL.
pub const FT_CLK_XTAL: u8 = 0x00;
/// Turn the APLL on and calibrate it, for a first boot.
pub const APLL_ON_CAL: u8 = 0xAA;
/// Turn the APLL on without recalibrating, for a warm re-enable.
pub const APLL_ON: u8 = 0x0A;

// divider [27:16] and source [15:4] for DIG_PLL /2 = 80 MHz, divider written first
const FEC_FCLK_DIVR: u32 = 0x111;
const FEC_FCLK_SRCSEL: u32 = 0x333;

/// Cold-boot calibration mask covering VCO, PD, LODIST, RX-gain and TX-power.
/// [`fact_cal`] and [`runtime_cal`] echo it in `run_status` on success.
pub const CAL_MASK_COLDBOOT: u16 = 0x00CE;

/// Temperature bin below 0 deg C, for [`runtime_cal`].
pub const TEMP_BIN_LOW: u8 = 0x00;
/// Temperature bin from 0 to 84 deg C, for [`runtime_cal`].
pub const TEMP_BIN_MID: u8 = 0x08;
/// Temperature bin at or above 85 deg C, for [`runtime_cal`].
pub const TEMP_BIN_HIGH: u8 = 0x10;

/// Cold boot at 80 MHz FCLK off the 40 MHz XTAL, with the RFS debug logger on
/// (its log appears at `RFS_IPC + 0x7C0`).
const BOOT_INFO_W0_FCLK: u32 = 0x0230_5000;
const RFS_CLK_FREQ_80M: u32 = 20480;

const CM3_UNHALT_MAGIC: u32 = 0xF5A3_6A17;
const FEC_CORE_UNHALT: u32 = 0x0002_17FF;

// busy-delay cycle counts at 200 MHz
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

/// Poll `addr` until `(read & mask) == want`, returning false on timeout.
#[inline]
fn wait_until(addr: usize, mask: u32, want: u32, tries: u32) -> bool {
    for _ in 0..tries {
        if rd(addr) & mask == want {
            return true;
        }
    }
    false
}

/// RFS M3 boot status.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum BootStatus {
    /// Still booting, or never released.
    Uninit,
    Pass,
    /// Self-test failed.
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

// domain transitions take hundreds of us, so allow a few ms
const POLL_TRIES: u32 = 1_000_000;

/// Power the FEC domain, un-gate its clocks, switch it to 80 MHz FCLK and
/// initialize the FECSS RAMs. Idempotent, since it power-cycles the domain to a
/// known state. The RFS M3 stays halted, so call [`rfs_boot`] next.
pub fn power_on() -> Result<(), PowerError> {
    // manual power mode, no RAM retention
    rmw(FEC_PWR_REQ_PARAM, PWR_MODE_MANUAL, 0);
    rmw(PSCON_FEC_PD_RAM_STATE, 0x1, 0);
    rmw(PSCON_FEC_PD_GRP4_STATE, 0x3, 0);
    rmw(FEC_PWR_REQ_PARAM, PWR_WAKEUP_OUT, 0);
    if !wait_until(PSCON_FEC_PD_EN, 0x200, 0x200, POLL_TRIES) {
        return Err(PowerError::PowerDown);
    }
    rmw(FEC_PWR_REQ_PARAM, 0, PWR_WAKEUP_OUT);
    if !wait_until(PSCON_FEC_PD_EN, PD_ON_STATUS_MASK, PD_ON_STATUS_UP, POLL_TRIES) {
        return Err(PowerError::PowerUp);
    }
    asm::delay(CYC_300US);

    // root clock first, then the IP clocks
    wr(APP_FEC_CLK_GATE, 0);
    asm::delay(CYC_500US);
    wr(FEC_IPCFGCLKGATE0, IPCFGCLKGATE0_ON);
    wr(FEC_IPCFGCLKGATE1, IPCFGCLKGATE1_ON);
    // switch to FCLK before mem init and the RFS boot, so the RFS runs its
    // frame-service timing at 80 MHz
    rmw(FEC_SYS_CLKCTL, 0x0FFF_0000, FEC_FCLK_DIVR << 16);
    rmw(FEC_SYS_CLKCTL, 0x0000_FFF0, FEC_FCLK_SRCSEL << 4);

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

/// Decoded RFS fault block.
pub struct RfsFault {
    /// 0x8008 idle, 0x4004 booting, 0xFFFF dead, or the id of a running command.
    pub fw_state: u16,
    pub patch: u8,
    /// 0x81 CPU fault, 0x82 FW assert.
    pub fault_type: u8,
    /// For example 0x81 stray MBOX interrupt, 0x83 frame during cal.
    pub error_code: u8,
    pub line: u16,
    pub abort_pc: u32,
    pub abort_lr: u32,
    pub cfsr: u32,
    pub hfsr: u32,
    pub exception_count: u8,
}

impl RfsFault {
    /// Report whether the RFS has faulted, in which case it is dead and will
    /// never service frames again.
    pub fn is_faulted(&self) -> bool {
        self.fault_type != 0 || self.exception_count != 0
    }
}

/// Read and decode the RFS fault block.
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

/// Init the RFS mailbox, write boot-info, un-halt the RFS M3 and poll boot
/// status, returning [`BootStatus::Uninit`] on timeout. Requires the RFS patch
/// RPRC resident at 0x21204000, which the flash appimage bundles. Without it
/// the ROM self-test never reaches [`BootStatus::Pass`].
pub fn rfs_boot() -> BootStatus {
    // the RFS reads its config mirrors out of this RAM
    for i in 0..(0x800 / 4) {
        wr(RFS_IPC + i * 4, 0);
    }

    for i in 0..32 {
        wr(RFS_MAILBOX + i * 4, 0);
    }
    rmw(IPC_CMD_TRIG, 0xFF, 0);
    rmw(FECSS_IPC_RFS, 0xFF, 0);
    wr(FECSS_IPC_BUSY_INT0, 0x1);

    wr(RFS_BOOT_INFO, BOOT_INFO_W0_FCLK);
    wr(RFS_BOOT_INFO + 4, 0);
    // clear stale status so an old 0xA cannot pass the poll
    rmw(RFS_CPU_FAULT_STS, 0x0000_FFFF, 0);
    rmw(RFS_BOOT_STS, 0x0000_00FF, 0);
    wr(CM3_CPU_HALT_HANDSHAKE, CM3_UNHALT_MAGIC);
    wr(FEC_CORE_SYSRESET_PARAM, FEC_CORE_UNHALT);

    // boot takes ~120 us typical, budget ~2 ms
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
    /// Busy flag set before send, so a prior response was never consumed.
    Busy,
    /// Payload exceeds the 124-byte mailbox.
    TooLarge,
    /// No response within the poll budget.
    Timeout,
    /// RFS error response, carrying its error code.
    CmdError(i32),
    /// Response cmd id did not match the command sent.
    Protocol { got: u16, resp_size: u16 },
}

/// Run one polled command over the RFS mailbox, copying up to `resp.len()`
/// response words out and returning the response payload size in bytes.
///
/// Waits for completion rather than accept. Both the busy bit and the status
/// byte are raised when the RFS merely accepts a command, but calibrations
/// finish tens of ms later, so completion also requires that `fw_state` no
/// longer holds this cmd id. Reading at accept returns garbage and leaves the
/// real completion unconsumed, after which the RFS asserts on the next doorbell
/// or frame-start event.
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

    // header word0 is cmd_id | (payload_bytes << 16), then the payload
    let size_bytes = (payload.len() * 4) as u32;
    wr(RFS_MAILBOX, cmd_id as u32 | (size_bytes << 16));
    for (i, &w) in payload.iter().enumerate() {
        wr(RFS_MAILBOX + 4 + i * 4, w);
    }

    // clear stale response status, then ring the doorbell
    rmw(FECSS_IPC_RFS, 0xFF, 0);
    rmw(IPC_CMD_TRIG, 0xFF, MB_SEND_TRIG);

    // ~2 s budget, since calibrations are the slow ones
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
        return Err(MailboxError::Timeout); // response-ready precondition
    }

    // the RFS echoes cmd_id on success and writes 0 on error
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

    // ack and clear busy so the next command can be sent
    rmw(IPC_CMD_TRIG, 0xFF, MB_READ_ACK);
    wr(FECSS_IPC_BUSY_INT0, 0x1);
    result
}

/// Arm the RX chains in `rx_mask` and TX chains in `tx_mask`. In `misc`, bit 0
/// is the RDIF clock, which needs the APLL up first, and bit 1 is the 1V LDO
/// bypass. All-zero powers the RF off.
pub fn rf_pwr_onoff(rx_mask: u16, tx_mask: u16, misc: u8) -> Result<(), MailboxError> {
    let payload = [
        rx_mask as u32 | ((tx_mask as u32) << 16),
        (misc as u32) << 8,
        0,
    ];
    mailbox_command(CMD_RF_PWR_ONOFF, &payload, &mut []).map(|_| ())
}

/// Select the on-die temperature sensors for [`temp_trig`]. 0x311 covers RX,
/// TX, PM and DIG.
pub fn temp_config(ctrl_mask: u16) {
    wr(RFS_TEMP_CFG, ctrl_mask as u32);
    wr(RFS_TEMP_CFG + 4, 0); // remainder of the 8-byte struct is reserved
}

/// Result of a [`temp_trig`] measurement.
pub struct TempMeas {
    /// Two status bits per sensor, where 0b01 is valid.
    pub status: u16,
    /// Per-sensor deg C, indexed RX0..2 at 0..2, TX0..1 at 4..5, PM 8, DIG 9.
    pub temp_c: [i16; 10],
}

/// Measure the sensors selected by [`temp_config`]. Needs the RF and PM analog
/// powered for meaningful numbers.
pub fn temp_trig() -> Result<TempMeas, MailboxError> {
    let mut resp = [0u32; 7];
    mailbox_command(CMD_TEMP_TRIG, &[], &mut resp)?;
    let mut temp_c = [0i16; 10];
    for (i, t) in temp_c.iter_mut().enumerate() {
        let word = resp[1 + i / 2];
        let half = if i % 2 == 0 { word & 0xFFFF } else { word >> 16 };
        *t = half as i16;
    }
    Ok(TempMeas {
        status: (resp[0] & 0xFFFF) as u16,
        temp_c,
    })
}

/// Pick the [`runtime_cal`] temperature bin for a measured temperature.
pub fn temp_bin_for(temp_c: i16) -> u8 {
    if temp_c < 0 {
        TEMP_BIN_LOW
    } else if temp_c < 85 {
        TEMP_BIN_MID
    } else {
        TEMP_BIN_HIGH
    }
}

/// Configure the FECSS clocks and bring up the APLL. Order matters: the M4-side
/// core-clock switch and RFS clock hint, then the FRC OSC un-gate, then the
/// mailbox command.
pub fn apll_clk_ctrl(dev_clk: u8, ft_clk: u8, apll: u8) -> Result<(), MailboxError> {
    if dev_clk == DEV_CLK_FCLK {
        rmw(FEC_SYS_CLKCTL, 0x0FFF_0000, FEC_FCLK_DIVR << 16);
        rmw(FEC_SYS_CLKCTL, 0x0000_FFF0, FEC_FCLK_SRCSEL << 4);
        rmw(RFS_BOOT_INFO, 0x0000_FFFF, RFS_CLK_FREQ_80M);
    }
    wr(FRC_OSC_CLK_GATE, if ft_clk == FT_CLK_XTAL { 0 } else { 0x7 });
    let payload = [
        dev_clk as u32 | ((ft_clk as u32) << 8) | ((apll as u32) << 16),
        0,
    ];
    mailbox_command(CMD_APLL_CLK_CTRL, &payload, &mut []).map(|_| ())
}

/// Bring the APLL up with calibration, FECSS on FCLK and frame timer on XTAL.
/// Must follow [`rf_pwr_onoff`] and precede calibration.
pub fn apll_on() -> Result<(), MailboxError> {
    apll_clk_ctrl(DEV_CLK_FCLK, FT_CLK_XTAL, APLL_ON_CAL)
}

/// Result of [`fact_cal`] / [`runtime_cal`].
pub struct CalStatus {
    /// This trigger's per-cal result bits, equal to the requested mask on success.
    pub run_status: u16,
    /// Cumulative cal validity bits.
    pub res_status: u16,
    /// Temperature at calibration in deg C, set by [`fact_cal`] only.
    pub calib_temp_c: i16,
}

/// Run the factory calibration. Must follow [`apll_on`]. The parameters are
/// fixed: other sets leave `res_status` without its validity bits, and the RFS
/// then asserts on the first frame-start.
pub fn fact_cal() -> Result<CalStatus, MailboxError> {
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
    let mut resp = [0u32; 3];
    mailbox_command(CMD_FACT_CAL, &payload, &mut resp)?;
    Ok(CalStatus {
        run_status: (resp[0] & 0xFFFF) as u16,
        res_status: (resp[0] >> 16) as u16,
        calib_temp_c: ((resp[1] & 0xFF) as i8 as i16) * 2, // 1 LSB = 2 deg C
    })
}

/// Run the runtime calibration for `temp_bin`, which [`temp_bin_for`] picks.
/// Needs a prior [`fact_cal`]. The mask omits the PD cal, since requesting it
/// with no monitor configured asserts the RFS.
pub fn runtime_cal(temp_bin: u8) -> Result<CalStatus, MailboxError> {
    let payload = [0x00CA_u32, temp_bin as u32, 0, 0, 0];
    let mut resp = [0u32; 3];
    mailbox_command(CMD_RUNTIME_CAL, &payload, &mut resp)?;
    Ok(CalStatus {
        run_status: (resp[0] & 0xFFFF) as u16,
        res_status: (resp[0] >> 16) as u16,
        calib_temp_c: 0,
    })
}
