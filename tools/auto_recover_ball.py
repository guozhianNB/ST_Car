"""Recover an end-hidden ball and hand it to the zero-position loop.

This is a bench-only helper.  It never drives the chassis and always returns
the beam to 0 degrees before stopping outputs, including on Ctrl+C/error.
"""

from __future__ import annotations

import argparse
import re
import time

import serial


FIELD_RE = re.compile(r"\b(ball|vel|vage|ang|fault)=([^ ]+)")


def write_command(port: serial.Serial, command: str) -> None:
    print(f"> {command}", flush=True)
    port.write((command + "\n").encode("ascii"))
    port.flush()


def fields(line: str) -> dict[str, str]:
    return dict(FIELD_RE.findall(line))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM3")
    parser.add_argument(
        "--reveal-angle",
        type=float,
        required=True,
        help="signed tube angle in degrees that moves the hidden ball inward",
    )
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    handed_to_ball_loop = False
    settled_since: float | None = None
    deadline = time.monotonic() + args.timeout

    with serial.Serial(args.port, 115200, timeout=0.05) as port:
        port.reset_input_buffer()
        write_command(port, "stop")
        time.sleep(0.15)
        write_command(port, "bench on")
        time.sleep(0.20)
        write_command(port, "stream on 50")
        time.sleep(0.20)
        write_command(port, f"angle {args.reveal_angle:.3f}")
        try:
            while time.monotonic() < deadline:
                raw = port.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                print(line, flush=True)
                if "fault=" not in line:
                    continue
                values = fields(line)
                if values.get("fault", "none") != "none":
                    raise RuntimeError(f"bench fault: {values['fault']}")
                try:
                    ball_mm = float(values["ball"])
                    speed_mm_s = float(values["vel"])
                    vision_age_ms = int(values["vage"])
                except (KeyError, ValueError):
                    continue

                if not handed_to_ball_loop and vision_age_ms <= 120:
                    write_command(port, "ball 0")
                    handed_to_ball_loop = True

                if handed_to_ball_loop and vision_age_ms <= 200:
                    if abs(ball_mm) <= 10.0 and abs(speed_mm_s) <= 25.0:
                        if settled_since is None:
                            settled_since = time.monotonic()
                        elif time.monotonic() - settled_since >= 0.5:
                            print("CENTERED: within +/-10 mm and +/-25 mm/s", flush=True)
                            return 0
                    else:
                        settled_since = None
            raise TimeoutError("ball did not settle at 0 mm before timeout")
        finally:
            # First stop covers faults.  Re-entering bench then commanding zero
            # follows the repository's recovery order and cannot move chassis.
            write_command(port, "stop")
            time.sleep(0.1)
            write_command(port, "bench on")
            time.sleep(0.1)
            write_command(port, "angle 0")
            # Keep telemetry enabled so the level check below has angle data.
            level_deadline = time.monotonic() + 5.0
            while time.monotonic() < level_deadline:
                raw = port.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                print(line, flush=True)
                values = fields(line)
                try:
                    if abs(float(values["ang"])) <= 0.15:
                        break
                except (KeyError, ValueError):
                    pass
            write_command(port, "stop")
            write_command(port, "stream off")


if __name__ == "__main__":
    raise SystemExit(main())
