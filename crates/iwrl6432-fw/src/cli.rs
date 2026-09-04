use core::sync::atomic::{AtomicBool, Ordering};

use embassy_time::Timer;
use iwrl6432_hal::{capture, fecss, mcspi, print, println, radar, sensor, uart};

use crate::profiles::{self, Metrics};

/// Set once [`radar::start`] has run, after which starts only reconfigure.
pub static BOOTED: AtomicBool = AtomicBool::new(false);
static RUNNING: AtomicBool = AtomicBool::new(false);
/// Periodic status line, off by default so it does not clobber the prompt.
pub static LOG: AtomicBool = AtomicBool::new(false);

static ACTIVE: critical_section::Mutex<core::cell::Cell<&'static str>> =
    critical_section::Mutex::new(core::cell::Cell::new("-"));
static CUSTOM: critical_section::Mutex<core::cell::RefCell<Option<sensor::RadarConfig>>> =
    critical_section::Mutex::new(core::cell::RefCell::new(None));

/// Record that `name` is now the running profile.
pub fn note_started(name: &'static str) {
    BOOTED.store(true, Ordering::Relaxed);
    RUNNING.store(true, Ordering::Relaxed);
    critical_section::with(|cs| ACTIVE.borrow(cs).set(name));
}

fn custom_cfg() -> sensor::RadarConfig {
    critical_section::with(|cs| {
        *CUSTOM
            .borrow(cs)
            .borrow_mut()
            .get_or_insert_with(sensor::RadarConfig::default)
    })
}

/// Run the line-based console CLI.
#[embassy_executor::task]
pub async fn cli_task() {
    let mut buf = [0u8; 96];
    let mut len = 0usize;
    println!("cli: ready ('help' for commands)");
    loop {
        let b = uart::read_byte().await;
        match b {
            b'\r' | b'\n' => {
                println!();
                if len > 0 {
                    let line = core::str::from_utf8(&buf[..len]).unwrap_or("");
                    dispatch(line).await;
                    len = 0;
                }
                print!("> ");
            }
            0x08 | 0x7F => {
                if len > 0 {
                    len -= 1;
                    print!("\x08 \x08");
                }
            }
            b' '..=b'~' => {
                if len < buf.len() {
                    buf[len] = b;
                    len += 1;
                    uart::write_byte_async(b).await;
                }
            }
            _ => {}
        }
    }
}

async fn dispatch(line: &str) {
    let mut it = line.split_whitespace();
    let cmd = it.next().unwrap_or("");
    let arg1 = it.next();
    let arg2 = it.next();
    match cmd {
        "help" => help(),
        "profiles" => list_profiles(),
        "show" => show(arg1),
        "status" => status(),
        "log" => log_cmd(arg1),
        "start" => start(arg1).await,
        "stop" => stop(),
        "set" => set(arg1, arg2),
        "nvic" => nvic(),
        "ct" => {
            let (s, _) = capture::stats();
            println!(
                "chirp_cnt={} burst_cnt={} frame_cnt={} seen={}",
                sensor::chirp_count(),
                sensor::burst_count(),
                sensor::frame_count(),
                s,
            );
        }
        "" => {}
        _ => println!("unknown command '{}'; try 'help'", cmd),
    }
}

fn help() {
    println!("commands:");
    println!("  profiles            list preset profiles");
    println!("  show [name|custom]  parameters + derived metrics (default: active)");
    println!("  status              frame/capture/stream counters");
    println!("  log [on|off]        toggle the once-a-second status line");
    println!("  start <name|custom> start or live-reconfigure to a profile");
    println!("  stop                stop frames (SPI stream idles)");
    println!("  set <field> <val>   edit the custom config (see 'show custom')");
    println!("set fields (raw register units):");
    println!("  samples   ADC samples per chirp (2..2048; ring limit 680 packed,");
    println!("            more with subsampling, e.g. 1536 at keep 7, 2048 at keep 5)");
    println!("  rate      sample-rate code, f_s = 100/code MSPS (8..100)");
    println!("  slope     FMCW slope, 28.61 kHz/us per LSB (signed)");
    println!("  start     RF start freq (0xBE00=57GHz..0xD555=64GHz)");
    println!("  ramp      ramp end time, 100 ns per LSB");
    println!("  idle      inter-chirp idle, 100 ns per LSB");
    println!("  cpb       chirps per burst (EVEN; ping/pong parity)");
    println!("  bpf       bursts per frame");
    println!("  burst     burst period, 100 ns per LSB");
    println!("  frame     frame period, 25 ns per LSB");
    println!("  txmask    TX enable mask (bit0 TX0, bit1 TX1)");
    println!("  keep      encoding: 1..15 = CS subsample k/16, 16 = exact packed12,");
    println!("            17 = bfp16 (block floating point, ~1.4x smaller, semi-lossless)");
}

fn list_profiles() {
    for p in profiles::profiles() {
        println!("  {:<12} {}", p.name, p.summary);
    }
    println!("  {:<12} {}", "custom", "the staged 'set' config");
}

fn resolve(name: &str) -> Option<(&'static str, sensor::RadarConfig)> {
    if name == "custom" {
        return Some(("custom", custom_cfg()));
    }
    profiles::find(name).map(|p| (p.name, p.cfg))
}

fn show(arg: Option<&str>) {
    let active = critical_section::with(|cs| ACTIVE.borrow(cs).get());
    let name = arg.unwrap_or(active);
    let Some((name, cfg)) = resolve(name) else {
        println!("no profile '{}' (see 'profiles')", name);
        return;
    };
    if let Some(p) = profiles::find(name) {
        println!("{}: {}", p.name, p.detail);
    } else {
        println!("custom (staged via 'set')");
    }
    let c = &cfg;
    println!(
        "  chirp : {} samples @ code {} | slope {} | start 0x{:X} | ramp {} idle {}",
        c.common.num_adc_samples,
        c.common.sample_rate_code,
        c.timing.freq_slope,
        c.timing.freq_start,
        c.common.ramp_end_time,
        c.timing.idle_time,
    );
    println!(
        "  frame : {} chirps/burst x {} bursts | burst {} | frame {} | tx 0x{:X}",
        c.frame.chirps_per_burst,
        c.frame.bursts_per_frame,
        c.frame.burst_periodicity,
        c.frame.frame_periodicity,
        c.timing.tx_en,
    );
    let m: Metrics = profiles::metrics(c);
    println!(
        "  rf    : f_s {:.1} MSPS | slope {:.1} MHz/us | sweep {:.0} MHz",
        m.fs_msps, m.slope_mhz_us, m.sweep_mhz,
    );
    println!(
        "  range : res {:.3} m | max {:.1} m (IF-limited)",
        m.range_res_m, m.max_range_m,
    );
    println!(
        "  vel   : max +-{:.2} m/s | res {:.3} m/s",
        m.v_max_ms, m.v_res_ms
    );
    match c.capture_format {
        capture::Format::Packed12 => println!(
            "  rate  : {} chirps/s | {:.0} fps | ~{} KB/s wire (packed12)",
            m.chirps_per_sec,
            m.fps,
            m.wire_bytes_per_sec / 1000,
        ),
        capture::Format::Sub12(k) => println!(
            "  rate  : {} chirps/s | {:.0} fps | ~{} KB/s wire (sub12 {}/16, CS)",
            m.chirps_per_sec,
            m.fps,
            m.wire_bytes_per_sec / 1000,
            k,
        ),
        capture::Format::Bfp16 => println!(
            "  rate  : {} chirps/s | {:.0} fps | ~{} KB/s wire (bfp16, semi-lossless)",
            m.chirps_per_sec,
            m.fps,
            m.wire_bytes_per_sec / 1000,
        ),
    }
}

fn status() {
    let active = critical_section::with(|cs| ACTIVE.borrow(cs).get());
    let (seen, dropped) = capture::stats();
    let (pack_cyc, queued) = capture::isr_timing();
    let f = fecss::rfs_fault_status();
    println!(
        "{} (profile {}) | frames={} chirps seen={} drop={} torn={} resync={} oor={} pack_us={} queued={} | spi served={} empty={} retry={} starved={} | rfs=0x{:04x}{}",
        if RUNNING.load(Ordering::Relaxed) { "running" } else { "stopped" },
        active,
        sensor::frame_count(),
        seen,
        dropped,
        capture::torn(),
        capture::resyncs(),
        capture::out_of_range(),
        pack_cyc / 40,
        queued,
        mcspi::served(),
        mcspi::empty_polls(),
        mcspi::retries(),
        mcspi::tx_starved(),
        f.fw_state,
        if f.is_faulted() { " FAULT" } else { "" },
    );
}

// irq 14 must outrank the rest or the SPI feed underruns
fn nvic() {
    for irq in [14u8, 34, 43, 62] {
        let pri = unsafe { core::ptr::read_volatile((0xE000_E400 + irq as usize) as *const u8) };
        let iser = unsafe {
            core::ptr::read_volatile((0xE000_E100 + (irq as usize >> 5) * 4) as *const u32)
        };
        println!(
            "irq {:2}: prio=0x{:02x} enabled={}",
            irq,
            pri,
            (iser >> (irq & 31)) & 1
        );
    }
}

fn log_cmd(arg: Option<&str>) {
    match arg {
        Some("on") => LOG.store(true, Ordering::Relaxed),
        Some("off") => LOG.store(false, Ordering::Relaxed),
        None => {
            LOG.fetch_xor(true, Ordering::Relaxed);
        }
        Some(other) => {
            println!("usage: log [on|off]  (got '{}')", other);
            return;
        }
    }
    println!("log {}", if LOG.load(Ordering::Relaxed) { "on" } else { "off" });
}

async fn start(arg: Option<&str>) {
    let Some(name) = arg else {
        println!("usage: start <name|custom>");
        return;
    };
    let Some((name, cfg)) = resolve(name) else {
        println!("no profile '{}' (see 'profiles')", name);
        return;
    };
    // mirror capture::init's asserts, which panic rather than report
    let samples = cfg.common.num_adc_samples as usize;
    let pair_mod = match cfg.capture_format {
        capture::Format::Packed12 => 2,
        capture::Format::Sub12(_) => 32,
        capture::Format::Bfp16 => 64,
    };
    if samples % pair_mod != 0 {
        println!("samples must be a multiple of {} for this format", pair_mod);
        return;
    }
    let payload = capture::payload_bytes(samples, 3, cfg.capture_format);
    if payload > capture::MAX_CHIRP_BYTES || payload % 4 != 0 {
        println!(
            "chirp payload {} B exceeds the {} B ring slot (or is not \
             word-sized); lower samples or 'set keep'",
            payload,
            capture::MAX_CHIRP_BYTES,
        );
        return;
    }
    if cfg.frame.chirps_per_burst % 2 != 0 {
        println!("chirps/burst must be even (ADCBUF ping/pong parity)");
        return;
    }
    let res = if BOOTED.load(Ordering::Relaxed) {
        radar::reconfigure(&cfg).map(|_| None)
    } else {
        radar::start(&cfg).map(Some)
    };
    // reconfigure blocks ~80 ms in a delay loop, so yield once for the ISRs
    Timer::after_millis(1).await;
    match res {
        Ok(report) => {
            if let Some(r) = report {
                println!(
                    "radar: up  fact_cal=0x{:04x}/0x{:04x} runtime_cal=0x{:04x} temp={} C",
                    r.fact_cal.run_status,
                    r.fact_cal.res_status,
                    r.runtime_cal.run_status,
                    r.temp_c,
                );
            }
            note_started(name);
            println!("started '{}'", name);
        }
        Err(e) => println!("start FAILED: {:?}", e),
    }
}

fn stop() {
    radar::stop();
    RUNNING.store(false, Ordering::Relaxed);
    println!("stopping at the next frame boundary");
}

fn set(field: Option<&str>, val: Option<&str>) {
    let (Some(field), Some(val)) = (field, val) else {
        println!("usage: set <field> <value>   (see 'help')");
        return;
    };
    let Some(v) = parse_num(val) else {
        println!("bad value '{}'", val);
        return;
    };
    let mut cfg = custom_cfg();
    let ok = match field {
        "samples" => set_u16(&mut cfg.common.num_adc_samples, v, 2, 2048),
        "rate" => set_u8(&mut cfg.common.sample_rate_code, v, 8, 100),
        "slope" => set_i16(&mut cfg.timing.freq_slope, v),
        "start" => {
            cfg.timing.freq_start = v as u32;
            true
        }
        "ramp" => set_u16(&mut cfg.common.ramp_end_time, v, 1, 65535),
        "idle" => set_u16(&mut cfg.timing.idle_time, v, 1, 65535),
        "cpb" => set_u16(&mut cfg.frame.chirps_per_burst, v, 2, 65535),
        "bpf" => set_u16(&mut cfg.frame.bursts_per_frame, v, 1, 4096),
        "burst" => {
            cfg.frame.burst_periodicity = v as u32;
            true
        }
        "frame" => {
            cfg.frame.frame_periodicity = v as u32;
            true
        }
        "txmask" => {
            cfg.timing.tx_en = (v & 0x3) as u16;
            true
        }
        "keep" => {
            if (1..=17).contains(&v) {
                cfg.capture_format = match v {
                    16 => capture::Format::Packed12,
                    17 => capture::Format::Bfp16,
                    k => capture::Format::Sub12(k as u8),
                };
                true
            } else {
                false
            }
        }
        _ => {
            println!("unknown field '{}' (see 'help')", field);
            return;
        }
    };
    if !ok {
        println!("value out of range");
        return;
    }
    // keep the CRD slope consistent with the profile formula
    let ramp_us = cfg.common.ramp_end_time as f32 / 10.0;
    let slope_mhz = cfg.timing.freq_slope as f32 * 0.02861;
    let idle_us = cfg.timing.idle_time as f32 / 10.0;
    let div = if idle_us - 1.0 < 6.0 { idle_us - 1.0 } else { 6.0 };
    if div > 0.0 {
        cfg.common.crd_nslope_mag = (2.1845 * ramp_us * slope_mhz / div) as u16;
    }
    critical_section::with(|cs| *CUSTOM.borrow(cs).borrow_mut() = Some(cfg));
    println!("custom.{} = {} (start with 'start custom')", field, v);
}

fn parse_num(s: &str) -> Option<i64> {
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        i64::from_str_radix(hex, 16).ok()
    } else {
        s.parse().ok()
    }
}

fn set_u16(dst: &mut u16, v: i64, min: u16, max: u16) -> bool {
    if v >= min as i64 && v <= max as i64 {
        *dst = v as u16;
        true
    } else {
        false
    }
}

fn set_u8(dst: &mut u8, v: i64, min: u8, max: u8) -> bool {
    if v >= min as i64 && v <= max as i64 {
        *dst = v as u8;
        true
    } else {
        false
    }
}

fn set_i16(dst: &mut i16, v: i64) -> bool {
    if v >= i16::MIN as i64 && v <= i16::MAX as i64 {
        *dst = v as i16;
        true
    } else {
        false
    }
}
