"""Bounded live probe for the independent moving-zero controller."""

from __future__ import annotations

import argparse
import re
import time
from pathlib import Path

import serial


FIELD_RE = re.compile(r"(?:^|\s)([a-z]+)=([^\s]+)")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--kx", type=float, required=True)
    parser.add_argument("--kv", type=float, required=True)
    parser.add_argument("--ki", type=float, required=True)
    parser.add_argument("--kq", type=float, required=True)
    parser.add_argument("--ka", type=float, default=0.0)
    parser.add_argument("--rate", type=float, required=True)
    parser.add_argument("--pose", type=float, required=True)
    parser.add_argument("--max-ms", type=int, default=8000)
    parser.add_argument("--max-speed", type=float, default=100.0)
    parser.add_argument("--loss-ms", type=int, default=100)
    parser.add_argument("--stable-error", type=float, default=8.0)
    parser.add_argument("--stable-speed", type=float, default=4.0)
    parser.add_argument("--stable-ms", type=int, default=800)
    parser.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()

    t0 = time.monotonic()
    run_started = t0
    stable_since = None
    invalid_since = None
    reason = "time_limit"

    with args.log.open("w", encoding="utf-8") as log, serial.Serial(
        args.port, 115200, timeout=0.02
    ) as port:
        def record(message: str) -> None:
            line = f"[{int((time.monotonic() - t0) * 1000):>7}] {message}"
            print(line, flush=True)
            log.write(line + "\n")
            log.flush()

        def send(command: str) -> None:
            record(">>> " + command)
            port.write((command + "\n").encode("ascii"))
            port.flush()

        def collect(seconds: float) -> None:
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                raw = port.readline()
                if raw:
                    record(raw.decode("utf-8", errors="replace").rstrip())

        port.reset_input_buffer()
        for command in (
            "bench on", "clear", "diag reset",
            f"gain zero {args.kx} {args.kv} {args.ki} {args.kq} {args.ka}",
            f"limit zero {args.rate} {args.pose}", "stream on 20",
            "zeroctl start",
        ):
            send(command)
            collect(0.14)
        run_started = time.monotonic()

        try:
            while True:
                now = time.monotonic()
                raw = port.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").rstrip()
                    record(line)
                    if not line.startswith("STATUS "):
                        continue
                    status = dict(FIELD_RE.findall(line))
                    try:
                        ball = float(status["ball"])
                        speed = abs(float(status["vball"]))
                        current = int(status["current"])
                        encoder = int(status["enc"])
                        requested = int(status["sreq"])
                    except (KeyError, ValueError):
                        continue
                    if current:
                        invalid_since = None
                    elif invalid_since is None:
                        invalid_since = now
                    if speed >= args.max_speed:
                        reason = "speed_limit"
                        break
                    if encoder < -3 or encoder > 1203:
                        reason = "encoder_guard"
                        break
                    if abs(requested) > args.rate + 2:
                        reason = "rate_limit_bypass"
                        break
                    if invalid_since is not None and (
                        now - invalid_since
                    ) * 1000 >= args.loss_ms:
                        reason = "vision_loss"
                        break
                    if current and abs(ball) <= args.stable_error and speed <= args.stable_speed:
                        stable_since = stable_since or now
                    else:
                        stable_since = None
                    if stable_since is not None and (
                        now - stable_since
                    ) * 1000 >= args.stable_ms:
                        reason = "stable"
                        break
                if (now - run_started) * 1000 >= args.max_ms:
                    break
        finally:
            send("stop")
            collect(0.30)
            send("stream off")
            collect(0.15)
            record("STOP_REASON " + reason)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
