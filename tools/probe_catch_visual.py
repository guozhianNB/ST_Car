"""Bench-only visual probe/debt-catch experiment for the P60 ball actuator.

All state-machine decisions use only ball position history derived from genuine
status=1 vision frames plus this run's command history.  P60 encoder values are
logged for post-test characterization and the permitted stall safety abort,
but are never used to start, reverse or normally stop a pulse.  The formal
firmware controller must preserve that boundary.

The first 2026-08-01 version used a train of separated catch pulses and stopped
when speed merely fell from its peak.  That is unsafe: an endpoint collision
also reduces speed.  This version instead repays the accumulated PWM*time debt
in one prompt opposite pulse.  It never uses endpoint slowdown as success.
"""

from __future__ import annotations

import argparse
import re
import statistics
import time
from collections import deque
from pathlib import Path

import serial


FIELD_RE = re.compile(r"\b([a-zA-Z0-9]+)=(-?\d+(?:\.\d+)?|none|[a-z_]+)")


class Experiment:
    def __init__(self, port: serial.Serial, log_path: Path) -> None:
        self.port = port
        self.log = log_path.open("w", encoding="utf-8")
        self.t0 = time.monotonic()
        self.samples: deque[tuple[float, float, int]] = deque(maxlen=24)
        self.last_frame = -1
        self.last_fields: dict[str, str] = {}

    def close(self) -> None:
        self.log.close()

    def emit(self, text: str) -> None:
        elapsed_ms = int((time.monotonic() - self.t0) * 1000.0)
        line = f"[{elapsed_ms:7d}] {text}"
        print(line, flush=True)
        self.log.write(line + "\n")
        self.log.flush()

    def send(self, command: str) -> None:
        self.emit(f">>> {command}")
        self.port.write((command + "\n").encode("ascii"))
        self.port.flush()

    def read_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            raw = self.port.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            self.emit(line)
            if not line.startswith("STATUS"):
                continue
            fields = dict(FIELD_RE.findall(line))
            self.last_fields = fields
            try:
                frame = int(fields["vframe"])
                status = int(fields["vs"])
                position = float(fields["ball"])
                age_s = float(fields["vage"]) * 0.001
            except (KeyError, ValueError):
                continue
            if status != 1 or frame == self.last_frame:
                continue
            self.last_frame = frame
            measurement_time = time.monotonic() - age_s
            self.samples.append((measurement_time, position, frame))

    def position(self) -> float:
        if not self.samples:
            raise RuntimeError("no genuine visual samples")
        return statistics.median(sample[1] for sample in list(self.samples)[-5:])

    def robust_speed(self, window_s: float = 0.40) -> float:
        if len(self.samples) < 3:
            return 0.0
        newest = self.samples[-1][0]
        selected = [sample for sample in self.samples
                    if newest - sample[0] <= window_s]
        slopes: list[float] = []
        for index, first in enumerate(selected):
            for second in selected[index + 1:]:
                dt = second[0] - first[0]
                if dt >= 0.045:
                    slopes.append((second[1] - first[1]) / dt)
        return statistics.median(slopes) if slopes else 0.0

    def pulse_and_brake(self, pwm: int, pulse_ms: int,
                        brake_ms: int, settle_ms: int) -> None:
        self.send(f"pulse {pwm} {pulse_ms}")
        self.read_for((pulse_ms + 5) * 0.001)
        self.send(f"pulse 0 {brake_ms}")
        self.read_for((brake_ms + settle_ms) * 0.001)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--target", type=float, default=0.0)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--max-probes", type=int, default=25)
    parser.add_argument("--pulse-ms", type=int, default=25)
    parser.add_argument("--brake-ms", type=int, default=120)
    parser.add_argument("--release-mm", type=float, default=1.0)
    parser.add_argument("--release-speed", type=float, default=5.0)
    parser.add_argument("--return-ratio-positive", type=float, default=1.20)
    parser.add_argument("--return-ratio-negative", type=float, default=0.75)
    parser.add_argument("--max-catch-ms", type=int, default=750)
    args = parser.parse_args()

    with serial.Serial(args.port, 115200, timeout=0.015) as port:
        port.reset_input_buffer()
        experiment = Experiment(port, args.log)
        try:
            for command in ("bench on", "stop", "zero", "stream on 20"):
                experiment.send(command)
                experiment.read_for(0.12)
            experiment.read_for(0.60)
            start = experiment.position()
            direction = 1 if args.target > start else -1
            motor_pwm = -130 * direction
            experiment.emit(
                f"BASE start={start:.2f} target={args.target:.2f} "
                f"direction={direction:+d} motor={motor_pwm:+d}"
            )

            drive_pulses = 0
            maximum_encoder_motion = 0
            for drive_pulses in range(1, args.max_probes + 1):
                experiment.pulse_and_brake(
                    motor_pwm, args.pulse_ms, args.brake_ms, 35
                )
                position = experiment.position()
                inward_displacement = (position - start) * direction
                inward_speed = experiment.robust_speed() * direction
                try:
                    maximum_encoder_motion = max(
                        maximum_encoder_motion,
                        abs(int(experiment.last_fields.get("p60", "0"))),
                    )
                except ValueError:
                    pass
                if drive_pulses >= 4 and maximum_encoder_motion < 2:
                    raise RuntimeError(
                        "P60 stall safety: four drive pulses produced less "
                        "than two encoder counts"
                    )
                moving = (inward_displacement >= args.release_mm and
                          inward_speed >= args.release_speed)
                experiment.emit(
                    f"PROBE n={drive_pulses} pos={position:.2f} "
                    f"din={inward_displacement:.2f} vin={inward_speed:.2f} "
                    f"moving={int(moving)}"
                )
                if moving:
                    break
            else:
                raise RuntimeError("no visually confirmed motion before probe limit")

            # Command-history compensation is part of the input shaper, not
            # feedback from the P60 encoder.  Directional ratios are temporary
            # bench calibrations and must be validated before becoming defaults.
            catch_pwm = -140 if motor_pwm > 0 else 140
            catch_ratio = (args.return_ratio_positive if catch_pwm > 0 else
                           args.return_ratio_negative)
            drive_debt = abs(motor_pwm) * args.pulse_ms * drive_pulses
            catch_ms = round(drive_debt * catch_ratio / abs(catch_pwm))
            if catch_ms > args.max_catch_ms:
                raise RuntimeError(
                    f"required catch {catch_ms} ms exceeds safety limit "
                    f"{args.max_catch_ms} ms"
                )
            catch_ms = max(1, catch_ms)
            experiment.emit(
                f"RELEASE drive={drive_pulses} debt={drive_debt} "
                f"catch_pwm={catch_pwm:+d} catch_ms={catch_ms} "
                f"ratio={catch_ratio:.3f}"
            )
            remaining_catch_ms = catch_ms
            while remaining_catch_ms > 0:
                catch_chunk_ms = min(500, remaining_catch_ms)
                experiment.send(f"pulse {catch_pwm} {catch_chunk_ms}")
                experiment.read_for((catch_chunk_ms + 5) * 0.001)
                remaining_catch_ms -= catch_chunk_ms
            experiment.send(f"pulse 0 {args.brake_ms}")
            experiment.read_for((args.brake_ms + 80) * 0.001)
            experiment.emit(
                f"CATCH pos={experiment.position():.2f} "
                f"speed={experiment.robust_speed():.2f}"
            )

            experiment.send("pulse 0 500")
            experiment.read_for(0.80)
            experiment.send("status")
            experiment.read_for(0.15)
            experiment.emit(
                f"RESULT drive={drive_pulses} catch_ms={catch_ms} "
                f"pos={experiment.position():.2f} "
                f"speed={experiment.robust_speed():.2f}"
            )
            return 0
        finally:
            experiment.send("stop")
            experiment.read_for(0.10)
            experiment.send("stream off")
            experiment.read_for(0.10)
            experiment.close()


if __name__ == "__main__":
    raise SystemExit(main())
