// Radar-register probe: load an ELF (TI demo or ours), run it, then take
// halt/dump/resume snapshots of the whole radar register space. ONE debug
// session for the whole run - reconnecting mid-stream would re-run the GEL
// (which halts FECSS) and disturb the running radar.
//
// Mode via env var PROBE_MODE:
//   demo  (default) - load the prebuilt TI motion_and_presence demo ELF, run,
//                     then WAIT for marker file ~/probe_go (created by
//                     tools/demo_cfg.py once sensorStart is accepted over UART),
//                     then snapshot.
//   fw              - load our firmware ELF, sleep a fixed 8 s (radar_bringup
//                     runs automatically), then snapshot.
//
// Usage (redirect stdout to a file for diffing):
//   PROBE_MODE=demo dss/run.sh probe.js > /tmp/dump_demo.txt
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
importPackage(Packages.java.io);

var HOME = java.lang.System.getenv("HOME");
var FW = java.lang.System.getenv("FW_ROOT");
var CCXML = FW + "/ti/jtag/IWRL6432.ccxml";
// SDK demo ELF for the demo-diff method; override with env DEMO_ELF.
var DEMO_ELF = java.lang.System.getenv("DEMO_ELF");
if (DEMO_ELF == null) DEMO_ELF = HOME + "/ti/MMWAVE_L_SDK_05_05_03_00/examples/mmw_demo/motion_and_presence_detection/xwrL64xx-evm/m4fss0-0_freertos/ti-arm-clang/motion_and_presence_detection_demo.release.out";
var FW_ELF = FW + "/target/thumbv7em-none-eabihf/release/iwrl6432-fw";
var MARKER = HOME + "/probe_go";
var VECS = 0x00400000;

var MODE = java.lang.System.getenv("PROBE_MODE");
if (MODE == null) MODE = "demo";
var ELF = (MODE == "demo") ? DEMO_ELF : FW_ELF;

function hex(v) { return "0x" + ("00000000" + java.lang.Long.toHexString(v & 0xFFFFFFFF)).slice(-8); }

var script = ScriptingEnvironment.instance();
script.setScriptTimeout(600000);
script.traceSetConsoleLevel(TraceLevel.SEVERE);
var ds = script.getServer("DebugServer.1");
ds.setConfig(CCXML);
var s = ds.openSession("*", "Cortex_M4_0");

// Named register windows we care about. Format: [label, start, end) - word reads.
// Wide sweep: targeted windows found no arming delta, so cover whole blocks.
var RANGES = [
  ["CT_chirp_timer",   0x51000000, 0x51000200],
  ["FEC_blk_5101",     0x51010000, 0x51010080],
  ["DFE",              0x51020000, 0x51020100],
  ["FEC_blk_5103",     0x51030000, 0x51030080],
  ["FEC_blk_5104",     0x51040000, 0x51040080],
  ["FT_frame_timer",   0x5B000000, 0x5B000100],
  ["FEC_CTRL",         0x52000000, 0x52000300],
  ["FEC_RCM",          0x52020000, 0x52020200],
  ["APPSS_RCM",        0x56040000, 0x56040200],
  ["APP_CTRL",         0x56060000, 0x56060400],
  ["ADCBUF_ctrl",      0x56080000, 0x56080080],
  ["ADCBUF_intr",      0x56081000, 0x56081040],
  ["TOP_PRCM",         0x5A040000, 0x5A040600],
  ["APPSS_mux",        0x5A020000, 0x5A020200],
  ["RFS_IPC",          0x21200000, 0x21200800],
  ["ADCBUF_data",      0x55060000, 0x55060080],
];

function rd(a) { try { return s.memory.readWord(0, a); } catch (e) { return -1; } }

function dumpAll(tag) {
  for (var i = 0; i < RANGES.length; i++) {
    var r = RANGES[i];
    for (var a = r[1]; a < r[2]; a += 4) {
      var v = rd(a);
      print(tag + " " + r[0] + " " + hex(a) + " " + (v < 0 ? "FAULT" : hex(v)));
    }
  }
}

// Small live-counter poll: the numbers that tell us whether frames are being
// serviced. halt/read/run per iteration (reads while running are not reliable
// on this backend).
function pollCounters(tag, iters, gapMs) {
  for (var i = 0; i < iters; i++) {
    try { s.target.halt(); } catch (e) {}
    print(tag + " poll" + i +
      " frame_cnt=" + hex(rd(0x5B000028)) +
      " ft_done="   + hex(rd(0x5B00002C)) +
      " chirp_cnt=" + hex(rd(0x5100005C)) +
      " burst_cnt=" + hex(rd(0x51000064)) +
      " intr_raw="  + hex(rd(0x56081010)) +
      " rfs_burst_start=" + hex(rd(0x212001D4)) +
      " rfs_frame_start=" + hex(rd(0x212001DC)));
    try { s.target.runAsynch(); } catch (e) {}
    java.lang.Thread.sleep(gapMs);
  }
}

// Clear stale marker.
var mf = new java.io.File(MARKER);
if (mf.exists()) mf["delete"]();

print("[probe] mode=" + MODE + " elf=" + ELF);
print("[probe] connecting (runs GEL init)...");
s.target.connect();
print("[probe] connected; stopping any residual frame timer (RFS boots mid-stream and wedges otherwise)...");
try {
  s.target.halt();
  s.memory.writeWord(0, 0x5B00000C, 0x7 << 6); // FT stop key
  java.lang.Thread.sleep(200);
  var ftrst = s.memory.readWord(0, 0x56040088); // APPSS FT reset pulse
  s.memory.writeWord(0, 0x56040088, ftrst | 0x1C0);
  java.lang.Thread.sleep(50);
  s.memory.writeWord(0, 0x56040088, ftrst & ~0x1C0);
  print("[probe] FT stopped: FRAME_CNT=" + hex(s.memory.readWord(0, 0x5B000028)));
} catch (e) { print("[probe] FT stop skipped: " + e); }

print("[probe] resetting + loading (this can take a while)...");
s.target.reset();
s.memory.loadProgram(ELF);

s.memory.writeWord(0, 0xE000ED08, VECS);
var initialSp = s.memory.readWord(0, VECS);
var resetPc = s.memory.readWord(0, VECS + 4);
s.memory.writeRegister("SP", initialSp);
s.memory.writeRegister("PC", resetPc & ~1);
print("[probe] loaded. SP=" + hex(initialSp) + " PC=" + hex(resetPc));

s.target.runAsynch();
print("[probe] running.");

if (MODE == "demo") {
  // The prebuilt demo has QUICK_START=1: it applies its built-in default config
  // and starts streaming on its own a few seconds after boot - no CLI needed.
  // The demo auto-configures itself (CLI is unresponsive under JTAG load); run 1
  // showed it healthy after a long settle. Poll until the RFS tracks frames or 300 s.
  print("[probe] demo mode: waiting for RFS to service frames (up to 300 s)...");
  var waited = 0;
  while (waited < 300000) {
    java.lang.Thread.sleep(15000);
    waited += 15000;
    try { s.target.halt(); } catch (e) {}
    var fs = s.memory.readWord(0, 0x212001DC);
    var fc = s.memory.readWord(0, 0x5B000028);
    try { s.target.runAsynch(); } catch (e) {}
    print("[probe] t=" + (waited / 1000) + "s frame_cnt=" + hex(fc) + " rfs_frame_start=" + hex(fs));
    if (fs > 16) { print("[probe] RFS is servicing frames - healthy."); break; }
  }
} else {
  print("[probe] fw mode: sleeping 8 s for radar_bringup...");
  java.lang.Thread.sleep(8000);
}

// Live counters first (least invasive), then three full snapshots 1 s apart.
pollCounters("CNT", 8, 300);

for (var snap = 0; snap < 3; snap++) {
  try { s.target.halt(); } catch (e) {}
  // While halted the M4 ISR can't clear ADCBUF ping/pong-done - if the DFE is
  // writing, INTR_RAW latches during this pause. Read before and after.
  var irawBefore = rd(0x56081010);
  java.lang.Thread.sleep(300);
  var irawAfter = rd(0x56081010);
  print("SNAP" + snap + " intr_raw_latch before=" + hex(irawBefore) + " after_300ms_halt=" + hex(irawAfter));
  dumpAll("SNAP" + snap);
  try { s.target.runAsynch(); } catch (e) {}
  java.lang.Thread.sleep(1000);
}

s.target.disconnect();
s.terminate();
ds.stop();
print("[probe] done - target left running.");
