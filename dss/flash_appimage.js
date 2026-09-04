// Flash our pure-Rust multicore .appimage to the IWRL6432's QSPI over JTAG.
//
// Stages the appimage into L3 SRAM at 0x60000000, then runs TI's prebuilt flasher
// stub (empty.c): on start it erases 512 KB of QSPI and writes our image (length
// from metaimage header[5]) to flash offset 0x0. The stub flashes IMMEDIATELY on
// run - there is NO spare3 handshake - so we must let it run UNINTERRUPTED (erase
// + write take several seconds); halting it mid-erase leaves flash blank. The stub
// prints progress on UARTB (ttyACM0, 115200): "Erasing SFlash..." ->
// "Flash Erase complete" -> "Image Loading Successful...".
//
// After this, power-cycle in Functional mode (SOP2) to boot our firmware from
// flash. To restore TI's demo, point IMAGE at the demo appimage and re-run.
//
// Run:  dss/run.sh flash_appimage.js
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);

var FW    = java.lang.System.getenv("FW_ROOT");
var CCXML = FW + "/ti/jtag/IWRL6432.ccxml";
var STUB  = FW + "/ti/jtag/jtag_flasher_xwrL64xx-aop_m4fss0-0_freertos_ti-arm-clang.out";
var IMAGE = FW + "/appimage/fw.appimage";

print("=== Flashing " + IMAGE + " to QSPI over JTAG ===");
var s = ScriptingEnvironment.instance();
s.setScriptTimeout(120000);
s.traceSetConsoleLevel(TraceLevel.INFO);
var ds = s.getServer("DebugServer.1");
ds.setConfig(CCXML);
var t = ds.openSession("*", "Cortex_M4_0");

t.target.connect();
try { t.target.reset(); } catch (e) { print("reset (ignored): " + e); }

// Stage the metaimage into L3 SRAM (the stub reads its length from header[5]).
t.memory.loadRaw(0, 0x60000000, IMAGE, 32, false);
print("[flash] metaimage staged to L3 0x60000000");

// Load and RUN the flasher stub; let it erase+write uninterrupted.
t.memory.loadProgram(STUB);
print("[flash] flasher stub loaded; running (erase 512 KB + write, ~20 s)...");
t.target.runAsynch();
java.lang.Thread.sleep(25000); // DO NOT halt/reconnect during this window

// Detach without resetting; the stub has finished flashing by now.
try { t.target.halt(); } catch (e) {}
t.target.disconnect();
t.terminate();
ds.stop();
print("=== done - check stub debug on ttyACM0, then power-cycle in SOP2 ===");
