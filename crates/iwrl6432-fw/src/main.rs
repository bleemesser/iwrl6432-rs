#![no_std]
#![no_main]

mod cli;
mod profiles;
mod stream;

use cortex_m_rt::entry;
use embassy_executor::Executor;
use embassy_time::Timer;
use iwrl6432_hal::{
    self as hal, capture, fecss, mcspi, println, radar, sensor,
    uart::Uart,
};
use panic_halt as _;
use static_cell::StaticCell;

const BAUD: u32 = 115_200;

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

/// Once a second: frame/chirp counters, capture stats, RFS health.
#[embassy_executor::task]
async fn radar_status() {
    loop {
        Timer::after_millis(1000).await;
        if !cli::LOG.load(core::sync::atomic::Ordering::Relaxed) {
            continue;
        }
        let (_, rfs_frames, _) = sensor::rfs_frame_status();
        let f = fecss::rfs_fault_status();
        let (seen, dropped) = capture::stats();
        println!(
            "[radar] frames={} rfs_frames={} cap seen={} drop={} oor={} state=0x{:04x}{}",
            sensor::frame_count(),
            rfs_frames,
            seen,
            dropped,
            capture::out_of_range(),
            f.fw_state,
            if f.is_faulted() { " FAULT" } else { "" },
        );
    }
}

/// Bring the radar up into [`profiles::AUTO_START`], if one is configured.
fn radar_bringup() {
    let Some(name) = profiles::AUTO_START else {
        println!("radar: idle (no auto-start profile; 'start <name>' on the CLI)");
        return;
    };
    let Some(p) = profiles::find(name) else {
        println!("radar: BAD AUTO_START profile '{}'", name);
        return;
    };
    let m = profiles::metrics(&p.cfg);
    println!(
        "radar: auto-start '{}' ({} samp, {} chirps/s, {:?})",
        p.name, p.cfg.common.num_adc_samples, m.chirps_per_sec, p.cfg.capture_format,
    );
    match radar::start(&p.cfg) {
        Ok(r) => {
            println!(
                "radar: up  fact_cal=0x{:04x}/0x{:04x} runtime_cal=0x{:04x} temp={} C",
                r.fact_cal.run_status, r.fact_cal.res_status, r.runtime_cal.run_status, r.temp_c,
            );
            cli::note_started(p.name);
        }
        Err(e) => {
            let f = fecss::rfs_fault_status();
            println!(
                "radar: FAILED {:?} (rfs state=0x{:04x} type=0x{:02x} err=0x{:02x} line={})",
                e, f.fw_state, f.fault_type, f.error_code, f.line,
            );
        }
    }
}

#[entry]
fn main() -> ! {
    hal::normalize_after_bootloader();

    let dp = unsafe { hal::pac::Peripherals::steal() };

    hal::clock::init_core(&dp);
    hal::clock::enable_hwass(&dp);
    let _console = Uart::new(&dp, BAUD);
    hal::time::init(&dp);
    Uart::enable_interrupts();
    mcspi::init(&dp);

    println!();
    println!("=== IWRL6432 pure-Rust firmware ===");
    println!("  core   : {} MHz", hal::clock::CORE_CLK_HZ / 1_000_000);
    println!("  console: UARTB @ {} 8N1 (CLI: 'help')", BAUD);
    println!("  stream : MCSPIA slave, poll protocol (S1.6=ON)");

    radar_bringup();

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(cli::cli_task().unwrap());
        spawner.spawn(radar_status().unwrap());
        spawner.spawn(stream::spi_stream().unwrap());
    });
}
