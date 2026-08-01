"""Move to a bounded actuator debug count using ten-pulse transactions."""

from __future__ import annotations

import argparse
import re
import time
from pathlib import Path

import serial


FIELDS = re.compile(r"(?:^|\s)([a-z]+)=([^\s]+)")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--target", type=int, required=True)
    parser.add_argument("--tolerance", type=int, default=5)
    parser.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()
    if not 20 <= args.target <= 1180:
        raise SystemExit("target must retain at least 20 counts from each guard")
    t0 = time.monotonic()

    with args.log.open("w", encoding="utf-8") as log, serial.Serial(
        args.port, 115200, timeout=0.02
    ) as port:
        def emit(message: str) -> None:
            line = f"[{int((time.monotonic() - t0) * 1000):>7}] {message}"
            print(line, flush=True)
            log.write(line + "\n")
            log.flush()

        def send(command: str) -> None:
            emit(">>> " + command)
            port.write((command + "\n").encode("ascii"))
            port.flush()

        def get_status() -> dict[str, str]:
            send("status")
            deadline = time.monotonic() + 0.35
            while time.monotonic() < deadline:
                raw = port.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                emit(line)
                if line.startswith("STATUS "):
                    return dict(FIELDS.findall(line))
            raise RuntimeError("no status response")

        port.reset_input_buffer()
        try:
            send("bench on")
            time.sleep(0.08)
            send("stop")
            time.sleep(0.08)
            for _ in range(140):
                sample = get_status()
                encoder = int(sample["enc"])
                if sample.get("fault", "none") != "none":
                    raise RuntimeError(f"firmware fault: {sample['fault']}")
                error = args.target - encoder
                if abs(error) <= args.tolerance:
                    emit(f"RESULT target reached enc={encoder}")
                    return 0
                send(f"run {100 if error > 0 else -100} 100")
                time.sleep(0.16)
            raise RuntimeError("target not reached within pulse budget")
        finally:
            send("stop")
            time.sleep(0.08)


if __name__ == "__main__":
    raise SystemExit(main())
