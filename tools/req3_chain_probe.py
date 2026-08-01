"""Center the ball without integral windup, then run requirement 3 in-place."""

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
    parser.add_argument("--log", required=True)
    parser.add_argument("--center-timeout-ms", type=int, default=12000)
    parser.add_argument("--sequence-timeout-ms", type=int, default=7500)
    args = parser.parse_args()

    path = Path(args.log)
    path.parent.mkdir(parents=True, exist_ok=True)
    t0 = time.monotonic()
    phase = "center"
    phase_started = t0
    stable_since = None
    invalid_since = None
    result = "host_timeout"

    with path.open("w", encoding="utf-8") as log, serial.Serial(
        args.port, 115200, timeout=0.02
    ) as port:
        port.reset_input_buffer()

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

        for command in (
            "bench on", "clear", "diag reset",
            "gain ball 1.2 2.0 4.0 -1", "gain accel 0", "limit rate 400",
            "stream on 20", "ball 0",
        ):
            send(command)
            collect(0.12)
        phase_started = time.monotonic()

        try:
            while True:
                now = time.monotonic()
                raw = port.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").rstrip()
                record(line)
                if not line.startswith("STATUS "):
                    continue
                status = dict(FIELD_RE.findall(line))
                try:
                    ball = float(status["ball"])
                    speed = abs(float(status["vball"]))
                    current = int(status["current"])
                    seq = int(status["seq"])
                    elapsed = int(status["elapsed"])
                except (KeyError, ValueError):
                    continue

                if current:
                    invalid_since = None
                elif invalid_since is None:
                    invalid_since = now
                if invalid_since is not None and (now - invalid_since) >= 0.35:
                    result = "vision_loss"
                    break
                if speed >= 190.0 or abs(ball) >= 139.0:
                    result = "motion_limit"
                    break

                if phase == "center":
                    if abs(ball) <= 8.0 and speed <= 4.0 and current:
                        stable_since = stable_since or now
                    else:
                        stable_since = None
                    # Do not mistake a slow zero crossing for a settled O
                    # start.  Residual tube tilt at that instant costs about
                    # one second at the beginning of the judged sequence.
                    if stable_since is not None and (now - stable_since) >= 0.80:
                        # The board's non-blocking telemetry queue can occupy
                        # the UART continuously at 20 ms.  Pause it while the
                        # mode-changing commands are acknowledged; otherwise
                        # a sequence-start command can be dropped even though
                        # the host has already changed phase.
                        send("stream off")
                        collect(0.10)
                        send("stop")
                        collect(0.08)
                        send("gain ball 1.2 2.0 2.0 -1")
                        collect(0.10)
                        send("ball sequence")
                        collect(0.10)
                        send("stream on 20")
                        collect(0.10)
                        phase = "sequence"
                        phase_started = time.monotonic()
                        record("PHASE sequence")
                    elif (now - phase_started) * 1000 >= args.center_timeout_ms:
                        result = "center_timeout"
                        break
                else:
                    if seq == 3:
                        result = f"sequence_complete elapsed={elapsed}"
                        break
                    if (now - phase_started) * 1000 >= args.sequence_timeout_ms:
                        result = f"sequence_timeout seq={seq} elapsed={elapsed}"
                        break
        finally:
            success = result.startswith("sequence_complete")
            if not success:
                send("stop")
                collect(0.25)
            send("stream off")
            collect(0.15)
            record("RESULT " + result)
    return 0 if result.startswith("sequence_complete") else 1


if __name__ == "__main__":
    raise SystemExit(main())
