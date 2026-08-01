"""Return the actuator to the already-calibrated right software guard.

Each motion request is independently bounded to ten command pulses.  The
currently running firmware enforces the absolute encoder guard as a second
stop path.  This helper never resets the encoder and always stops on exit.
"""

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
    parser.add_argument("--target", type=int, default=3)
    parser.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()
    started = time.monotonic()
    previous_encoder: int | None = None
    wrong_way = 0
    reached = False

    with args.log.open("w", encoding="utf-8") as log, serial.Serial(
        args.port, 115200, timeout=0.02
    ) as port:
        def record(message: str) -> None:
            line = f"[{int((time.monotonic() - started) * 1000):>7}] {message}"
            print(line, flush=True)
            log.write(line + "\n")
            log.flush()

        def send(command: str) -> None:
            record(">>> " + command)
            port.write((command + "\n").encode("ascii"))
            port.flush()

        def status(timeout_s: float = 0.35) -> dict[str, str] | None:
            send("status")
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                raw = port.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                record(line)
                if line.startswith("STATUS "):
                    return dict(FIELD_RE.findall(line))
            return None

        port.reset_input_buffer()
        try:
            send("bench on")
            time.sleep(0.08)
            send("stop")
            time.sleep(0.08)
            for _ in range(120):
                sample = status()
                if sample is None:
                    raise RuntimeError("no status response")
                if sample.get("fault", "none") != "none":
                    raise RuntimeError(f"firmware fault: {sample['fault']}")
                encoder = int(sample["enc"])
                if encoder <= args.target:
                    reached = True
                    break
                if previous_encoder is not None and encoder > previous_encoder + 3:
                    wrong_way += 1
                    if wrong_way >= 2:
                        raise RuntimeError("encoder moved away from right guard")
                else:
                    wrong_way = 0
                previous_encoder = encoder
                send("run -100 100")
                time.sleep(0.16)
            if not reached:
                raise RuntimeError("right guard not reached within pulse budget")
            record(f"RESULT reached calibrated guard enc={encoder}")
            return 0
        finally:
            send("stop")
            time.sleep(0.08)


if __name__ == "__main__":
    raise SystemExit(main())
