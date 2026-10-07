#!/usr/bin/env python3
"""
ESP8266 simulator for the leak controller (ATmega8535).

Talks the same serial protocol as the future ESP firmware (docs/protocol.md):
  * prints decoded $ST status frames and '#' debug logs,
  * sends $HB heartbeats automatically (keeps the ESP link "up"),
  * lets you type commands; checksums are added automatically,
  * can keep a relay on with a lease that is refreshed automatically.

Install:  pip install pyserial
Run:      python tools/esp_sim.py COM5
          python tools/esp_sim.py COM5 --no-hb      (test link-loss handling)

Commands at the prompt:
  gs | ver | open | close | hb | help      -> $GS, $VER, ...
  rl <n> <0|1> [lease_s]                   -> $RL,n,v[,lease]
  keep <n> <lease_s>                       -> keep relay n ON, refresh every lease/3 s
  unkeep <n>                               -> stop refreshing (relay goes off by lease)
  raw <text>                               -> send "$<text>*HH" as is
  bad <text>                               -> send with a WRONG checksum
  hb on | hb off                           -> enable/disable automatic heartbeat
  quiet | verbose                          -> hide/show periodic unchanged $ST
  q                                        -> quit
"""
import argparse
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")

HB_PERIOD_S = 3.0

STATE_UA = {
    "INIT": "старт", "OPENING": "відкривається", "OPEN": "норма (відкрито)",
    "CLOSING": "закривається", "CLOSED": "закрито вручну",
    "ALARM": "АВАРІЯ (протічка)", "FAULT": "НЕСПРАВНІСТЬ",
}


def checksum(body: str) -> str:
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"{cs:02X}"


def frame(body: str) -> bytes:
    return f"${body}*{checksum(body)}\r\n".encode("ascii")


def parse_frame(line: str):
    """Returns (body, ok) for a '$...*HH' line."""
    if "*" not in line:
        return line[1:], False
    body, cs = line[1:].rsplit("*", 1)
    return body, cs.strip().upper() == checksum(body)


class Sim:
    def __init__(self, port: str, baud: int, hb: bool):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.lock = threading.Lock()
        self.hb_enabled = hb
        self.verbose = False
        self.keep = {}          # relay -> lease_s
        self.last_st = None
        self.running = True

    # ---------------- TX ----------------
    def send(self, body: str, bad=False):
        data = frame(body)
        if bad:
            data = f"${body}*00\r\n".encode("ascii")
        with self.lock:
            self.ser.write(data)
        if body != "HB" or self.verbose:
            print(f"  >> {data.decode().strip()}")

    # ---------------- RX ----------------
    def rx_loop(self):
        buf = b""
        while self.running:
            try:
                chunk = self.ser.read(256)
            except serial.SerialException as e:
                print(f"!! serial error: {e}")
                self.running = False
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", errors="replace").strip()
                if line:
                    self.handle_line(line)

    def handle_line(self, line: str):
        ts = time.strftime("%H:%M:%S")
        if line.startswith("#"):
            print(f"{ts} {line}")
            return
        if not line.startswith("$"):
            print(f"{ts} ?? {line}")
            return

        body, ok = parse_frame(line)
        if not ok:
            print(f"{ts} !! BAD CHECKSUM: {line}")
            return

        f = body.split(",")
        if f[0] == "ST" and len(f) >= 8:
            key = tuple(f[1:])
            if key == self.last_st and not self.verbose:
                return
            self.last_st = key
            state, pos, leak, fault, alarm, relays, link = f[1:8]
            leak_list = [str(i + 1) for i in range(8) if int(leak, 16) & (1 << i)]
            relay_list = [str(i) for i in range(8) if int(relays, 16) & (1 << i)]
            print(f"{ts} ST  стан={state} ({STATE_UA.get(state, '?')})  кран={pos}  "
                  f"протічка={','.join(leak_list) or '-'}  fault={fault}  "
                  f"alarm={alarm}  реле_on={','.join(relay_list) or '-'}  esp_link={link}")
        else:
            print(f"{ts} << {line}")

    # ---------------- periodic ----------------
    def periodic_loop(self):
        next_hb = 0.0
        next_keep = {}
        while self.running:
            now = time.monotonic()
            if self.hb_enabled and now >= next_hb:
                self.send("HB")
                next_hb = now + HB_PERIOD_S
            for n, lease in list(self.keep.items()):
                if now >= next_keep.get(n, 0):
                    self.send(f"RL,{n},1,{lease}")
                    next_keep[n] = now + max(1.0, lease / 3.0)
            for n in list(next_keep):
                if n not in self.keep:
                    del next_keep[n]
            time.sleep(0.1)

    # ---------------- CLI ----------------
    def command(self, cmd: str):
        p = cmd.split()
        if not p:
            return
        c = p[0].lower()
        if c in ("gs", "ver", "open", "close"):
            self.send(c.upper())
        elif c == "help":
            self.send("?")
        elif c == "hb" and len(p) == 2:
            self.hb_enabled = p[1].lower() == "on"
            print(f"  auto heartbeat: {'ON' if self.hb_enabled else 'OFF'}")
        elif c == "hb":
            self.send("HB")
        elif c == "rl" and len(p) in (3, 4):
            self.send("RL," + ",".join(p[1:]))
        elif c == "keep" and len(p) == 3:
            self.keep[int(p[1])] = int(p[2])
        elif c == "unkeep" and len(p) == 2:
            self.keep.pop(int(p[1]), None)
        elif c == "raw" and len(p) >= 2:
            self.send(cmd.split(None, 1)[1])
        elif c == "bad" and len(p) >= 2:
            self.send(cmd.split(None, 1)[1], bad=True)
        elif c == "quiet":
            self.verbose = False
        elif c == "verbose":
            self.verbose = True
        else:
            print(__doc__.split("Commands at the prompt:")[1])


def main():
    ap = argparse.ArgumentParser(description="ESP8266 simulator for the leak controller")
    ap.add_argument("port", help="COM port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=19200)
    ap.add_argument("--no-hb", action="store_true", help="start without automatic heartbeat")
    args = ap.parse_args()

    sim = Sim(args.port, args.baud, hb=not args.no_hb)
    threading.Thread(target=sim.rx_loop, daemon=True).start()
    threading.Thread(target=sim.periodic_loop, daemon=True).start()
    print(f"Connected to {args.port} @ {args.baud}. Type 'help' for controller commands, "
          f"'?' for simulator commands, 'q' to quit.")
    sim.send("VER")
    sim.send("GS")

    try:
        while sim.running:
            cmd = input()
            if cmd.strip().lower() in ("q", "quit", "exit"):
                break
            if cmd.strip() == "?":
                print(__doc__.split("Commands at the prompt:")[1])
                continue
            sim.command(cmd)
    except (KeyboardInterrupt, EOFError):
        pass
    sim.running = False
    sim.ser.close()


if __name__ == "__main__":
    main()
