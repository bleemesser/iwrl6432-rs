// Validate the FAULTMASK fix WITHOUT a flash cycle: load the firmware over JTAG
// but set FAULTMASK=1 before running (reproducing how the ROM bootloader hands
// off). If main()'s unmask_cpu_interrupts() clears it and the console comes up,
// the fix is proven.
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function hex(v){return "0x"+("00000000"+java.lang.Long.toHexString(v & 0xFFFFFFFF)).slice(-8);}
var FW=java.lang.System.getenv("FW_ROOT");
var CCXML=FW+"/ti/jtag/IWRL6432.ccxml";
var ELF=FW+"/target/thumbv7em-none-eabihf/release/iwrl6432-fw";
var VECS=0x00400000, DHCSR=0xE000EDF0, DCRSR=0xE000EDF4, DCRDR=0xE000EDF8;
var s=ScriptingEnvironment.instance(); s.setScriptTimeout(60000);
s.traceSetConsoleLevel(TraceLevel.SEVERE);
var ds=s.getServer("DebugServer.1"); ds.setConfig(CCXML);
var t=ds.openSession("*","Cortex_M4_0");
t.target.connect(); t.target.reset(); t.memory.loadProgram(ELF);
t.memory.writeWord(0,0xE000ED08,VECS);
t.memory.writeRegister("SP", t.memory.readWord(0,VECS));
t.memory.writeRegister("PC", t.memory.readWord(0,VECS+4) & ~1);
// Simulate RBL leftovers: set the unaligned/div-0 UsageFault traps in CCR...
t.memory.writeWord(0, 0xE000ED14, t.memory.readWord(0,0xE000ED14) | 0x18);
// ...and enable + pend ALL NVIC interrupts so the firmware must scrub them.
t.memory.writeWord(0, 0xE000E100, 0xFFFFFFFF); // ISER0 enable
t.memory.writeWord(0, 0xE000E104, 0xFFFFFFFF); // ISER1 enable
t.memory.writeWord(0, 0xE000E200, 0xFFFFFFFF); // ISPR0 pend
t.memory.writeWord(0, 0xE000E204, 0xFFFFFFFF); // ISPR1 pend
// Set FAULTMASK=1 (mask group reg 0x14, byte2). REGWnR=bit16.
t.memory.writeWord(0, DCRDR, 0x00010000);
t.memory.writeWord(0, DCRSR, 0x00010014);
for (var i=0;i<100;i++){ if (t.memory.readWord(0,DHCSR) & 0x10000) break; }
// Read back to confirm it took.
t.memory.writeWord(0, DCRSR, 0x00000014);
for (var i=0;i<100;i++){ if (t.memory.readWord(0,DHCSR) & 0x10000) break; }
print("FAULTMASK before run = "+hex((t.memory.readWord(0,DCRDR)>>16)&0xFF)+" (want 0x1)");
t.target.runAsynch();
java.lang.Thread.sleep(1000);
t.target.disconnect(); t.terminate(); ds.stop();
print("running with FAULTMASK preset");
