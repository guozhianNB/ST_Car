"""Run a bounded live ball-control probe and stop on unsafe observations.

The firmware bench console remains responsible for all normal safety checks.
This host-side helper adds a second, independent stop path for tuning: it sends
``stop`` when the run time, measured displacement, ball speed, or consecutive
invalid-vision time reaches a caller supplied limit.
"""

from __future__ import annotations

import argparse
import re
import time
from pathlib import Path

import serial


FIELD_RE = re.compile(r"(?:^|\s)([a-z]+)=([^\s]+)")


def fields(line: str) -> dict[str, str]:
    return dict(FIELD_RE.findall(line)) if line.startswith("STATUS ") else {}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--target", type=float, required=True)
    parser.add_argument("--kp", type=float, required=True)
    parser.add_argument("--ki", type=float, default=0.0)
    parser.add_argument("--kd", type=float, required=True)
    parser.add_argument("--ka", type=float, required=True)
    parser.add_argument("--rate", type=int, required=True)
    parser.add_argument("--max-ms", type=int, default=6000)
    parser.add_argument("--max-displacement-mm", type=float, default=40.0)
    parser.add_argument("--max-speed-mm-s", type=float, default=350.0)
    parser.add_argument("--loss-ms", type=int, default=80)
    parser.add_argument("--stable-error-mm", type=float, default=0.0)
    parser.add_argument("--stable-speed-mm-s", type=float, default=8.0)
    parser.add_argument("--stable-ms", type=int, default=500)
    parser.add_argument("--log", required=True)
    args = parser.parse_args()

    log_path = Path(args.log)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    t0 = time.monotonic()
    run_started = None
    first_ball = None
    invalid_since = None
    stable_since = None
    displacement_violations = 0
    stop_reason = "host_timeout"

    with log_path.open("w", encoding="utf-8") as log, serial.Serial(
        args.port, args.baud, timeout=0.02
    ) as port:
        port.reset_input_buffer()

        def record(text: str) -> None:
            stamp = int((time.monotonic() - t0) * 1000)
            output = f"[{stamp:>7}] {text}"
            print(output, flush=True)
            log.write(output + "\n")
            log.flush()

        def send(command: str) -> None:
            record(">>> " + command)
            port.write((command + "\n").encode("ascii"))
            port.flush()

        def collect(duration_s: float) -> None:
            deadline = time.monotonic() + duration_s
            while time.monotonic() < deadline:
                raw = port.readline()
                if raw:
                    record(raw.decode("utf-8", errors="replace").rstrip())

        for command in (
            "bench on",
            "clear",
            "diag reset",
            f"gain ball {args.kp} {args.ki} {args.kd} -1",
            f"gain accel {args.ka}",
            f"limit rate {args.rate}",
            "stream on 20",
        ):
            send(command)
            collect(0.16)

        send(f"ball {args.target}")
        run_started = time.monotonic()
        try:
            while True:
                now = time.monotonic()
                raw = port.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").rstrip()
                    record(line)
                    status = fields(line)
                    if status:
                        try:
                            ball = float(status["ball"])
                            speed = abs(float(status["vball"]))
                            current = int(status["current"])
                        except (KeyError, ValueError):
                            continue
                        if first_ball is None and current:
                            first_ball = ball
                        if current:
                            invalid_since = None
                            if (args.stable_error_mm > 0.0 and
                                    abs(ball - args.target) <=
                                    args.stable_error_mm and
                                    speed <= args.stable_speed_mm_s):
                                if stable_since is None:
                                    stable_since = now
                            else:
                                stable_since = None
                        elif invalid_since is None:
                            invalid_since = now
                        if (first_ball is not None and
                                abs(ball - first_ball) >= args.max_displacement_mm):
                            # The bench UART is deliberately non-blocking and
                            # may drop a byte when its telemetry ring is full.
                            # Require two consecutive reports so a damaged
                            # decimal field cannot stop an otherwise healthy
                            # probe.  At 20 ms streaming this only adds 20 ms
                            # to the independent displacement stop path.
                            displacement_violations += 1
                        else:
                            displacement_violations = 0
                        if displacement_violations >= 2:
                            stop_reason = "displacement_limit"
                            break
                        if speed >= args.max_speed_mm_s:
                            stop_reason = "speed_limit"
                            break
                        if (invalid_since is not None and
                                (now - invalid_since) * 1000.0 >= args.loss_ms):
                            stop_reason = "vision_loss"
                            break
                        if (stable_since is not None and
                                (now - stable_since) * 1000.0 >= args.stable_ms):
                            stop_reason = "stable"
                            break
                if (now - run_started) * 1000.0 >= args.max_ms:
                    stop_reason = "time_limit"
                    break
        finally:
            send("stop")
            collect(0.35)
            send("stream off")
            collect(0.20)
            record("STOP_REASON " + stop_reason)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
