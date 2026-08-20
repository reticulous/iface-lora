#!/usr/bin/env python3
"""Live integration test of the RNode endpoint (TCP and serial doors), run
from the build container against the real device, with no user in the loop.

  this script ── spangap cli ─────────────────► device CLI (settings, rnprobe)
  this script ── tcp 7633 (workspace bridge) ─► RNode endpoint, TCP door
  reference RNS (rnprobe / rnsd) ── tcp 7633 ─► RNode endpoint, TCP door
  this script / reference RNS ── pty ── tcp 2325 (monitor --aux) ── spare CDC
                                              ► RNode endpoint, serial door

Stages (each needs the ones before it; --stage N stops after N):

  0  preflight — device CLI answers, the TCP door is open (writes
     s.lora.rnode.tcp=1 when off — persisted, which is the point of a
     standing test door), radio parameters read.
  1  raw KISS detect burst on the TCP door — DETECT_RESP, firmware version,
     platform, MCU: the endpoint answers as RNode hardware.
  2  reference `rnprobe` attaches as a real RNodeInterface client and probes
     the device's rnstransport.probe destination: the whole handshake, the
     configuration echo, path discovery and a proved round trip,
     client → device.
  3  reference `rnsd` holds the endpoint while the DEVICE runs `rnprobe`
     against our probe destination: the device-initiated direction.
  4  raw KISS detect burst on the SERIAL door, through the monitor's aux
     serial relay — this exercises the in-band trigger takeover on the spare
     CDC port, the same attach path a one-port console shares with a client.
  5  reference `rnprobe` again, as a *serial* RNodeInterface client on a pty
     this script splices to the aux relay.

Stages 4-5 need the serial-door setup and skip with a note without it: the
device's console on CDC (`spangap cli "usb cdc"` — per boot, not persisted),
and the host monitor re-pointed at the new console node with the spare one on
the relay (`spangap monitor <cdc0-dev> --aux <cdc1-dev>`). Note the aux relay
carries bytes only — no DTR — which is exactly why the serial door attaches on
the in-band KISS FEND, not on line state.

The client's radio parameters are mirrored from the device's own s.lora.<n>.*
keys, so a run does not move the radio's channel (an RNodeInterface client's
parameters overwrite the device's and persist). The reference stack runs as a
transport node while stage 3 holds the endpoint — it is a third station on the
radio segment for that minute, and its frames do go out over the air.

Needs: the workspace bridge fronting port 7633 (`bridge_ports:` in the
buildable's straddle.yaml; restart `spangap monitor`/`spangap dev` after
changing it), and an authorized `spangap cli` (run it once by hand for the
`sshd add` line if not). Environment overrides: SPANGAP (the spangap command,
shell-split), RNS_VENV (reference-stack venv, default ~/rns-ref-venv),
RNODE_TEST_DIR (client config/identity dir, default ~/.rnode-tcp-test),
RNODE_AUX (aux serial relay endpoint, default host.docker.internal:2325).
"""

import os
import re
import shlex
import socket
import subprocess
import sys
import time
from pathlib import Path

RNODE_TCP_PORT = 7633            # fixed in the client (TCPConnection.TARGET_PORT)

# KISS / RNode command bytes, from the client (RNS/Interfaces/RNodeInterface.py)
FEND, FESC, TFEND, TFESC = 0xC0, 0xDB, 0xDC, 0xDD
CMD_DETECT, DETECT_REQ, DETECT_RESP = 0x08, 0x73, 0x46
CMD_PLATFORM, CMD_MCU, CMD_FW_VERSION = 0x48, 0x49, 0x50

SPANGAP = shlex.split(os.environ.get("SPANGAP", "spangap"))
VENV = Path(os.environ.get("RNS_VENV", Path.home() / "rns-ref-venv"))
TEST_DIR = Path(os.environ.get("RNODE_TEST_DIR", Path.home() / ".rnode-tcp-test"))


def fail(msg: str) -> None:
    print(f"FAIL: {msg}")
    sys.exit(1)


def workspace_root() -> Path | None:
    for base in (Path(__file__).resolve(), Path.cwd().resolve()):
        for d in [base, *base.parents]:
            if (d / "spangap.workspace.yaml").is_file():
                return d
    return None


def rnode_host() -> str:
    """Where a client dials the RNode TCP door: the workspace bridge on Docker
    Desktop, the device itself on a native-Linux host (same rule as the rest of
    the in-container tooling). Returned as an IPv4 literal when one resolves:
    the reference client connects to getaddrinfo()[0] with no family fallback,
    and host.docker.internal also carries an AAAA record the IPv4-only bridge
    refuses — the same trap install-reticulum dodges for TCPClientInterface."""
    try:
        infos = socket.getaddrinfo("host.docker.internal", RNODE_TCP_PORT,
                                   socket.AF_INET, socket.SOCK_STREAM)
        return infos[0][4][0]
    except OSError:
        pass
    ws = workspace_root()
    if ws and (ws / ".spangap-tcp").is_file():
        host = (ws / ".spangap-tcp").read_text().strip()
        if host:
            return host
    fail("no host.docker.internal and no .spangap-tcp — can't locate the device")


def cli(cmd: str, timeout: int = 30) -> str:
    r = subprocess.run([*SPANGAP, "cli", cmd], capture_output=True, text=True,
                       timeout=timeout)
    if r.returncode != 0:
        fail(f"`spangap cli {cmd}` exited {r.returncode}:\n{r.stdout}{r.stderr}")
    return r.stdout


def show(prefix: str) -> dict[str, str]:
    """Parse the device's `show` output ("key = value" lines) into a dict."""
    out = {}
    for line in cli(f"show {prefix}").splitlines():
        m = re.match(r"\s*([A-Za-z0-9._-]+)\s*=\s*(.*?)\s*$", line)
        if m:
            out[m.group(1)] = m.group(2).strip().strip('"')
    return out


# --- stage 0 — preflight ---------------------------------------------------

def preflight() -> dict[str, str]:
    up = show("rnsd.up").get("rnsd.up")
    if up is None:
        fail("device CLI gave no answer for `show rnsd.up` — run `spangap cli` "
             "by hand: if it prints an `sshd add` line, paste that in the "
             "monitor window first")
    if up != "1":
        fail("rnsd is not up on the device (rnsd.up != 1)")

    rnode = show("s.lora.rnode")
    if rnode.get("s.lora.rnode.tcp") != "1":
        print("preflight: opening the TCP door (set s.lora.rnode.tcp=1)")
        cli("set s.lora.rnode.tcp=1")
        cli("save")
        time.sleep(2)

    radio = rnode.get("s.lora.rnode.radio", "0") or "0"
    keys = show(f"s.lora.{radio}")
    params = {}
    for dev_key, client_key in (("frequency", "frequency"),
                                ("bandwidth", "bandwidth"),
                                ("tx_power", "txpower"),
                                ("spreading_factor", "spreadingfactor"),
                                ("coding_rate", "codingrate")):
        v = keys.get(f"s.lora.{radio}.{dev_key}")
        if not v and dev_key == "tx_power":
            # Unset tx_power runs the radio at 0 dBm; mirroring 0 persists the
            # key at the power already in use rather than moving it.
            print(f"preflight: s.lora.{radio}.tx_power unset — mirroring the "
                  "effective 0 dBm")
            v = "0"
        if not v:
            fail(f"s.lora.{radio}.{dev_key} is unset on the device — the radio "
                 "is not configured, nothing to expose")
        params[client_key] = v
    if keys.get(f"s.lora.{radio}.enable") != "1":
        fail(f"radio {radio} is not enabled (s.lora.{radio}.enable != 1)")
    if keys.get(f"s.lora.{radio}.ifac_netname"):
        print("preflight: WARNING — radio has an IFAC network name; mirroring "
              "it into the client config (untested path)")
        params["networkname"] = keys[f"s.lora.{radio}.ifac_netname"]
    print(f"preflight: OK — rnsd up, endpoint on radio {radio}, "
          + ", ".join(f"{k}={v}" for k, v in params.items()))
    return params


# --- stage 1 — raw KISS detect ---------------------------------------------

def kiss_frames(buf: bytearray):
    """Split and unescape complete KISS frames; leave a partial frame in buf."""
    frames = []
    while True:
        try:
            start = buf.index(FEND)
        except ValueError:
            buf.clear()
            return frames
        try:
            end = buf.index(FEND, start + 1)
        except ValueError:
            del buf[:start]
            return frames
        raw = bytes(buf[start + 1:end])
        del buf[:end]          # keep the trailing FEND: it may open the next frame
        if raw:
            frames.append(raw.replace(bytes([FESC, TFEND]), bytes([FEND]))
                             .replace(bytes([FESC, TFESC]), bytes([FESC])))


def detect(host: str, port: int, hint: str, door: str = "tcp") -> None:
    try:
        s = socket.create_connection((host, port), timeout=8)
    except OSError as e:
        fail(f"{host}:{port} did not connect ({e}) — {hint}")
    s.settimeout(2)
    # The client's own opening burst, verbatim.
    s.sendall(bytes([FEND, CMD_DETECT, DETECT_REQ, FEND, CMD_FW_VERSION, 0x00,
                     FEND, CMD_PLATFORM, 0x00, FEND, CMD_MCU, 0x00, FEND]))
    got, buf, deadline = {}, bytearray(), time.time() + 8
    while time.time() < deadline and len(got) < 4:
        try:
            data = s.recv(4096)
        except socket.timeout:
            continue
        if not data:
            break
        buf += data
        for f in kiss_frames(buf):
            got.setdefault(f[0], f[1:])
    s.close()
    if got.get(CMD_DETECT) != bytes([DETECT_RESP]):
        fail(f"no DETECT_RESP on the TCP door (got frames: "
             f"{{{', '.join(f'0x{c:02x}:{v.hex()}' for c, v in got.items())}}}) "
             "— accepted but not speaking RNode: door held by another client, "
             "or the bridge spliced to something else")
    fw = got.get(CMD_FW_VERSION, b"")
    fw_s = f"{fw[0]}.{fw[1]}" if len(fw) >= 2 else fw.hex() or "?"
    plat = got.get(CMD_PLATFORM, b"?").hex()
    mcu = got.get(CMD_MCU, b"?").hex()
    print(f"{door} detect: OK — DETECT_RESP, fw {fw_s}, platform 0x{plat}, mcu 0x{mcu}")


# --- stages 2 + 3 — the reference stack ------------------------------------

def write_client_config(port_value: str, params: dict[str, str],
                        transport: bool = False) -> None:
    """`transport` only for the stage the device probes us in: the reference
    stack hosts its probe responder only as a transport instance — and a
    transport node re-broadcasts announces it hears onto the air, so every
    other stage runs without it."""
    TEST_DIR.mkdir(mode=0o700, exist_ok=True)
    iface = "\n".join(f"    {k} = {v}" for k, v in params.items())
    (TEST_DIR / "config").write_text(f"""# generated by rnode_test.py — edits are overwritten each run
[reticulum]
  enable_transport = {"True" if transport else "False"}
  share_instance = No
  respond_to_probes = Yes
  panic_on_interface_error = No

[logging]
  loglevel = 6

[interfaces]
  [[RNode TCP door]]
    type = RNodeInterface
    interface_enabled = True
    port = {port_value}
{iface}
""")


def probe_dest_hash(identity_hash_hex: str) -> str:
    """RNS destination hash for rnstransport.probe on a known identity hash:
    sha256("rnstransport.probe")[:10] + identity_hash, sha256, [:16]."""
    import hashlib
    ident = bytes.fromhex(identity_hash_hex)
    if len(ident) != 16:
        fail(f"identity hash {identity_hash_hex!r} is not 16 bytes")
    name_hash = hashlib.sha256(b"rnstransport.probe").digest()[:10]
    return hashlib.sha256(name_hash + ident).digest()[:16].hex()


def device_probe_dest() -> str:
    ident = show("rnsd.identity_hash").get("rnsd.identity_hash")
    if not ident:
        fail("device gave no rnsd.identity_hash")
    return probe_dest_hash(ident)


def run_rnprobe(dest: str, label: str, timeout: int = 45) -> None:
    r = subprocess.run([str(VENV / "bin" / "rnprobe"), "--config", str(TEST_DIR),
                        "rnstransport.probe", dest, "-t", str(timeout)],
                       capture_output=True, text=True, timeout=timeout * 4)
    tail = (r.stdout + r.stderr).strip()
    print("\n".join("  | " + l for l in tail.splitlines()[-8:]))
    if r.returncode != 0:
        fail(f"reference rnprobe exited {r.returncode} (1 = no path, 2 = no "
             f"reply) — {label}: client → device leg broken")
    print(f"{label}: OK")


def probe_out() -> None:
    dest = device_probe_dest()
    print(f"probe-out: rnprobe → device rnstransport.probe <{dest}> "
          "(attach + config echo + path request + proved round trip)")
    run_rnprobe(dest, "probe-out")


def probe_in(host: str, params: dict[str, str]) -> None:
    write_client_config(f"tcp://{host}", params, transport=True)
    logfile = open(TEST_DIR / "rnsd.log", "wb")
    daemon = subprocess.Popen([str(VENV / "bin" / "rnsd"), "--config",
                               str(TEST_DIR), "-vvv"],
                              stdout=logfile, stderr=subprocess.STDOUT,
                              env={**os.environ, "PYTHONUNBUFFERED": "1"})
    try:
        deadline = time.time() + 45
        while time.time() < deadline:
            log = (TEST_DIR / "rnsd.log").read_bytes()
            if b"is now open" in log or b"established" in log:
                break
            if daemon.poll() is not None:
                fail(f"reference rnsd exited {daemon.returncode} — see {TEST_DIR}/rnsd.log")
            time.sleep(1)
        else:
            fail(f"reference rnsd never attached to the endpoint — see {TEST_DIR}/rnsd.log")
        time.sleep(5)          # let the detect/config dance and announces settle

        r = subprocess.run([str(VENV / "bin" / "python"), "-c",
                            "import RNS,sys;"
                            f"i=RNS.Identity.from_file('{TEST_DIR}/storage/transport_identity');"
                            "print(RNS.Destination.hash(i,'rnstransport','probe').hex())"],
                           capture_output=True, text=True, timeout=30)
        if r.returncode != 0:
            fail(f"couldn't derive our probe destination: {r.stderr.strip()}")
        dest = r.stdout.strip()
        print(f"probe-in: device rnprobe → our rnstransport.probe <{dest}>")
        out = cli(f"rnprobe {dest} -t 30", timeout=60)
        print("\n".join("  | " + l for l in out.strip().splitlines()[-6:]))
        if "delivered to" not in out:
            fail("device rnprobe got no delivery proof — device → client leg broken")
        print("probe-in: OK")
    finally:
        daemon.terminate()
        try:
            daemon.wait(timeout=10)
        except subprocess.TimeoutExpired:
            daemon.kill()
        logfile.close()


# --- stages 4 + 5 — the serial door, through the monitor's aux relay --------

def aux_endpoint() -> tuple[str, int] | None:
    spec = os.environ.get("RNODE_AUX", "host.docker.internal:2325")
    host, _, p = spec.rpartition(":")
    try:
        infos = socket.getaddrinfo(host, int(p), socket.AF_INET, socket.SOCK_STREAM)
        return infos[0][4][0], int(p)
    except OSError:
        return None


def serial_ready() -> str | None:
    """None when the serial-door setup is in place, else the reason to skip."""
    ports = show("sys.usb.serial_ports").get("sys.usb.serial_ports", "1")
    if ports != "2":
        return (f"console is not on CDC (sys.usb.serial_ports = {ports}) — run "
                "`spangap cli \"usb cdc\"`, then re-point the host monitor: "
                "`spangap monitor <cdc0-dev> --aux <cdc1-dev>`")
    ep = aux_endpoint()
    if ep is None:
        return "aux relay hostname does not resolve"
    try:
        socket.create_connection(ep, timeout=4).close()
    except OSError as e:
        return (f"aux serial relay {ep[0]}:{ep[1]} not reachable ({e}) — "
                "(re)start the monitor with --aux <cdc1-dev>")
    return None


def serial_detect() -> None:
    host, port = aux_endpoint()
    detect(host, port,
           "did the relay open the CDC device? see .spangap-forward-*.log on "
           "the host", door="serial")


def serial_probe_out(params: dict[str, str]) -> None:
    """rnprobe as a real serial RNodeInterface client: this end is a pty whose
    master this script splices to the aux relay — bytes only, exactly what a
    serial cable minus its control lines carries."""
    import pty
    import threading
    host, port = aux_endpoint()
    try:
        sock = socket.create_connection((host, port), timeout=8)
    except OSError as e:
        fail(f"aux relay: {e}")
    sock.settimeout(None)
    mfd, sfd = pty.openpty()
    sname = os.ttyname(sfd)

    def pump_to_relay():
        try:
            while True:
                data = os.read(mfd, 4096)
                if not data:
                    break
                sock.sendall(data)
        except OSError:
            pass

    def pump_from_relay():
        try:
            while True:
                data = sock.recv(4096)
                if not data:
                    break
                os.write(mfd, data)
        except OSError:
            pass

    threading.Thread(target=pump_to_relay, daemon=True).start()
    threading.Thread(target=pump_from_relay, daemon=True).start()
    try:
        dest = device_probe_dest()
        write_client_config(sname, params)
        print(f"serial probe-out: rnprobe over {sname} ↔ aux relay → device "
              f"rnstransport.probe <{dest}>")
        run_rnprobe(dest, "serial probe-out", timeout=60)
    finally:
        sock.close()
        os.close(mfd)
        os.close(sfd)


def main() -> None:
    last = 5
    args = sys.argv[1:]
    if args[:1] == ["--stage"] and len(args) > 1:
        last = int(args[1])
    host = rnode_host()
    print(f"RNode TCP door: {host}:{RNODE_TCP_PORT}")
    params = preflight()
    if last >= 1:
        detect(host, RNODE_TCP_PORT,
               "is the bridge fronting 7633? (restart `spangap monitor` after "
               "the bridge_ports change), and is s.lora.rnode.tcp set on the "
               "device?", door="tcp")
    if last >= 2:
        write_client_config(f"tcp://{host}", params)
        probe_out()
    if last >= 3:
        time.sleep(3)          # let the endpoint release the rnprobe session
        probe_in(host, params)
    if last >= 4:
        skip = serial_ready()
        if skip:
            print(f"serial door: SKIPPED — {skip}")
        else:
            time.sleep(3)
            serial_detect()
            if last >= 5:
                time.sleep(3)
                serial_probe_out(params)
    print("PASS")


if __name__ == "__main__":
    main()
