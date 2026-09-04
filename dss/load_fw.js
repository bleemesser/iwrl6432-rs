// TI Debug Server Scripting (DSS): load and run the pure-Rust firmware over JTAG.
//
// Flow: connect (runs the GEL: FECSS halt, front-end power, clock/PLL init) ->
// core reset -> loadProgram (loads our ELF into M4F RAM and, being Cortex-M
// aware, sets PC to the reset vector and SP from vector[0]) -> run.
//
// The image is RAM-resident: nothing is written to QSPI, so it does not persist
// across a power cycle. Re-running this script re-loads it.
//
// Run:  dss/run.sh load_fw.js

importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);

var FW    = java.lang.System.getenv("FW_ROOT");
var CCXML = FW + "/ti/jtag/IWRL6432.ccxml";
var ELF   = FW + "/target/thumbv7em-none-eabihf/release/iwrl6432-fw";
var VECS  = 0x00400000;   // start of the M4F code region == our vector table

var script = ScriptingEnvironment.instance();
script.setScriptTimeout(60000);
script.traceSetConsoleLevel(TraceLevel.INFO);

var ds = script.getServer("DebugServer.1");
ds.setConfig(CCXML);
var s = ds.openSession("*", "Cortex_M4_0");

print("[dss] connecting (runs GEL init)...");
s.target.connect();
print("[dss] connected");

// Clean core state before loading.
s.target.reset();

print("[dss] loading " + ELF);
s.memory.loadProgram(ELF);
print("[dss] loaded");

// Our vector table is at 0x00400000, not 0x0. Point VTOR there and explicitly
// seed MSP/PC from the table so `run` works regardless of whether the TCM is
// aliased at 0x0 or whether loadProgram set the core registers. (cortex-m-rt's
// set-sp/set-vtor features also do this in software, so this is belt-and-braces.)
s.memory.writeWord(0, 0xE000ED08, VECS);       // SCB->VTOR
var initialSp = s.memory.readWord(0, VECS);
var resetPc   = s.memory.readWord(0, VECS + 4);
s.memory.writeRegister("SP", initialSp);
s.memory.writeRegister("PC", resetPc & ~1);    // even addr; core stays in Thumb
print("[dss] VTOR=0x" + java.lang.Long.toHexString(VECS) +
      " SP=0x" + java.lang.Long.toHexString(initialSp) +
      " PC=0x" + java.lang.Long.toHexString(resetPc));

print("[dss] running - UART console on UARTB (/dev/ttyACM0, 115200 8N1)");
s.target.runAsynch();

// Leave the core running; just detach the debugger.
java.lang.Thread.sleep(1000);
s.target.disconnect();
s.terminate();
ds.stop();
print("[dss] detached; firmware continues");
