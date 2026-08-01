"""Bench-only endpoint recovery using visual release detection.

The ball loop remains disabled.  The helper applies short inward P60 pulses,
then immediately cancels the relative actuator travel accumulated during this
run as soon as vision confirms that the ball left the end pocket.  Encoder
counts are used only to make this characterization/recovery repeatable; the
production controller must not depend on the absolute P60 position.
"""

from __future__ import annotations

import argparse
import re
import time

import serial


FIELD_RE = re.compile(r"\b([a-zA-Z0-9]+)=(-?\d+(?:\.\d+)?|none|[a-z_]+)")


def send(port: serial.Serial, command: str) -> None:
    print(f">>> {command}", flush=True)
    port.write((command + "\n").encode("ascii"))
    port.flush()


def brake(port: serial.Serial) -> None:
    """Hold TB6612 short-brake without dropping beam STBY."""
    send(port, "pulse 0 500")


def read_status(port: serial.Serial, deadline: float) -> dict[str, str] | None:
    while time.monotonic() < deadline:
        line = port.readline().decode("utf-8", errors="replace").strip()
        if not line:
            continue
        print(line, flush=True)
        if line.startswith("STATUS"):
            return dict(FIELD_RE.findall(line))
    return None


def number(fields: dict[str, str], key: str) -> float:
    return float(fields.get(key, "0"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--target", type=float, default=0.0)
    parser.add_argument("--max-pulses", type=int, default=45)
    parser.add_argument("--pulse-ms", type=int, default=25)
    parser.add_argument("--release-mm", type=float, default=3.0)
    parser.add_argument("--release-speed", type=float, default=8.0)
    args = parser.parse_args()

    with serial.Serial(args.port, 115200, timeout=0.02) as port:
        port.reset_input_buffer()
        send(port, "stop")
        time.sleep(0.08)
        send(port, "zero")
        time.sleep(0.08)
        send(port, "stream on 20")
        first = read_status(port, time.monotonic() + 1.0)
        if first is None:
            raise RuntimeError("no telemetry")
        start_pos = number(first, "ball")
        direction = 1 if args.target < start_pos else -1
        # Positive motor command moves the ball toward negative x.
        inward_pwm = 130 * direction
        inward_speed_sign = -direction
        released = False
        last = first

        try:
            for index in range(args.max_pulses):
                send(port, f"pulse {inward_pwm} {args.pulse_ms}")
                pulse_deadline = time.monotonic() + 0.10
                while time.monotonic() < pulse_deadline:
                    sample = read_status(port, pulse_deadline)
                    if sample is None:
                        break
                    last = sample
                    pos = number(sample, "ball")
                    speed = number(sample, "vball")
                    moved_inward = ((pos - start_pos) * inward_speed_sign >=
                                    args.release_mm)
                    speed_inward = speed * inward_speed_sign
                    if moved_inward or speed_inward >= args.release_speed:
                        released = True
                        print(
                            f"RELEASE pulse={index + 1} p60={number(sample, 'p60'):.0f} "
                            f"pos={pos:.1f} speed={speed:.1f}",
                            flush=True,
                        )
                        break
                if released:
                    break

            if not released:
                raise RuntimeError("endpoint did not release before pulse limit")

            brake(port)
            time.sleep(0.01)
            # Cancel this run's relative actuator displacement quickly.  Stop
            # slightly beyond zero so the tube initially brakes the ball.
            return_target = -35.0 * direction
            send(port, f"pulse {-140 * direction} 500")
            deadline = time.monotonic() + 0.70
            while time.monotonic() < deadline:
                sample = read_status(port, deadline)
                if sample is None:
                    break
                last = sample
                p60 = number(sample, "p60")
                speed_inward = number(sample, "vball") * inward_speed_sign
                reached = (p60 <= return_target if direction > 0
                           else p60 >= return_target)
                reversed_ball = speed_inward < -10.0
                if reached or reversed_ball:
                    brake(port)
                    print(
                        f"RETURN p60={p60:.0f} pos={number(sample, 'ball'):.1f} "
                        f"speed={number(sample, 'vball'):.1f}",
                        flush=True,
                    )
                    break

            # Observe the result with every motor output off.
            brake(port)
            observe_end = time.monotonic() + 1.2
            while time.monotonic() < observe_end:
                sample = read_status(port, observe_end)
                if sample is not None:
                    last = sample
            print(
                f"FINAL p60={number(last, 'p60'):.0f} "
                f"pos={number(last, 'ball'):.1f} speed={number(last, 'vball'):.1f}",
                flush=True,
            )
        finally:
            send(port, "stop")
            send(port, "stream off")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
