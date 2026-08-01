"""Non-interactive ST_Car bench console driver.

Runs a scripted command list against the ST-Link VCP and logs everything.

Script file format (one directive per line):
    <any other text>   sent verbatim to the console, followed by \n
    wait <ms>          just collect output for <ms>

Usage:
    python tools/bench_cli.py --port COM3 --script script.txt [--log out.txt]

Lines received from the board are printed with a elapsed-ms prefix.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError as exc:
    raise SystemExit(
        "pyserial is required. Install it with: python -m pip install pyserial"
    ) from exc


def drain(port: serial.Serial, ms: int, log, t0: float) -> None:
    deadline = time.monotonic() + ms / 1000.0
    buf = b""
    while time.monotonic() < deadline:
        chunk = port.read(port.in_waiting or 1)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode("utf-8", errors="replace").rstrip("\r")
            stamp = int((time.monotonic() - t0) * 1000)
            out = f"[{stamp:>7}] {text}"
            print(out)
            if log:
                log.write(out + "\n")
                log.flush()
    if buf:
        text = buf.decode("utf-8", errors="replace").rstrip("\r")
        stamp = int((time.monotonic() - t0) * 1000)
        out = f"[{stamp:>7}] {text}"
        print(out)
        if log:
            log.write(out + "\n")
            log.flush()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--script", required=True, help="directive file")
    parser.add_argument("--log", help="optional log file")
    parser.add_argument("--settle-ms", type=int, default=600,
                        help="collect time after each command")
    args = parser.parse_args()

    directives = []
    for raw in Path(args.script).read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        directives.append(line)

    log = open(args.log, "a", encoding="utf-8") if args.log else None
    t0 = time.monotonic()
    try:
        with serial.Serial(args.port, args.baud, timeout=0.05) as port:
            port.reset_input_buffer()
            for directive in directives:
                parts = directive.split()
                if parts[0].lower() == "wait" and len(parts) == 2:
                    drain(port, int(parts[1]), log, t0)
                    continue
                stamp = int((time.monotonic() - t0) * 1000)
                out = f"[{stamp:>7}] >>> {directive}"
                print(out)
                if log:
                    log.write(out + "\n")
                    log.flush()
                port.write((directive + "\n").encode("ascii"))
                port.flush()
                drain(port, args.settle_ms, log, t0)
    finally:
        if log:
            log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
