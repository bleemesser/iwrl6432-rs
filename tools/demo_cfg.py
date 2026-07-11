"""Configure the TI mmwave demo over its CLI UART (single-port IWRL6432).

Speaks the same protocol as the host-side ti-mmwave driver (src/driver.rs):
send each cfg line + '\n', wait for the ':/>' prompt, treat 'error'/'not
recognized' in the response as failure. Handles the in-band 'baudRate N'
command by reopening the port at the new rate (termios2/BOTHER, any rate).

After sensorStart is accepted, touches MARKER (handshake with dss/probe.js),
then reads a few seconds of the data stream and reports whether TI frame
magic words are arriving.

Usage: demo_cfg.py <cfg-file> [dev=/dev/ttyACM0] [marker=~/probe_go]
"""
import os, sys, fcntl, array, time, select

TCGETS2 = 0x802C542A
TCSETS2 = 0x402C542B
BOTHER = 0x1000
CBAUD = 0x100F
MAGIC = bytes([0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07])


def open_port(dev, baud):
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    b = array.array("I", [0] * 16)
    fcntl.ioctl(fd, TCGETS2, b, True)
    b[2] &= ~CBAUD
    b[2] |= BOTHER
    b[0] = 0  # raw: no iflags
    b[1] = 0
    b[3] = 0
    b[9] = baud
    b[10] = baud
    fcntl.ioctl(fd, TCSETS2, b, True)
    return fd


def read_for(fd, secs):
    out = bytearray()
    end = time.time() + secs
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.1)
        if r:
            try:
                out += os.read(fd, 4096)
            except OSError:
                pass
    return bytes(out)


def wait_for(fd, token, timeout):
    out = bytearray()
    end = time.time() + timeout
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.1)
        if r:
            try:
                out += os.read(fd, 4096)
            except OSError:
                pass
            if token in out:
                return bytes(out), True
    return bytes(out), False


def main():
    cfg_path = sys.argv[1]
    dev = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyACM0"
    marker = sys.argv[3] if len(sys.argv) > 3 else os.path.expanduser("~/probe_go")

    lines = []
    with open(cfg_path) as f:
        for line in f:
            t = line.strip()
            if t and not t.startswith("%") and not t.startswith("#"):
                lines.append(t)

    fd = open_port(dev, 115200)
    banner = read_for(fd, 2.0)
    if banner:
        print("=== banner ===")
        print(banner.decode("latin1"))

    failures = 0
    for cmd in lines:
        os.write(fd, (cmd + "\n").encode())
        if cmd.split()[0] == "baudRate":
            new_baud = int(cmd.split()[1])
            time.sleep(0.3)
            os.close(fd)
            fd = open_port(dev, new_baud)
            print(f">>> {cmd}  [port reopened at {new_baud}]")
            continue
        resp, ok = wait_for(fd, b":/>", 2.0)
        text = resp.decode("latin1")
        bad = ("error" in text.lower()) or ("recognized" in text.lower())
        status = "OK" if (ok and not bad) else ("ERROR" if bad else "TIMEOUT")
        if status != "OK":
            failures += 1
        print(f">>> {cmd}  [{status}]")
        if status != "OK":
            print(text)
        if cmd.split()[0] == "sensorStart":
            # Touch the marker regardless: the prompt match is unreliable when TLV
            # frames interleave with CLI echo.
            open(marker, "w").close()
            print(f"[marker touched: {marker}] (cmd status {status})")

    print("=== data stream check (4 s) ===")
    data = read_for(fd, 4.0)
    n = data.count(MAGIC)
    print(f"received {len(data)} bytes, {n} frame magic words")
    os.close(fd)
    sys.exit(0 if failures == 0 else 2)


if __name__ == "__main__":
    main()
