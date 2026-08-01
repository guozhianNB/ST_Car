"""One bounded vision-only drive/catch identification step.

The script applies exactly one start pulse.  It then waits for a robust visual
motion estimate and starts one opposite catch pulse from a stopping-distance
guard.  P60 is telemetry/stall evidence only and never participates in the
motion decision.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import serial

from probe_catch_visual import Experiment


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--target", type=float, required=True)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--drive-ms", type=int, default=50)
    parser.add_argument("--lookahead", type=float, default=0.75)
    parser.add_argument("--max-wait", type=float, default=3.0)
    args = parser.parse_args()

    with serial.Serial(args.port, 115200, timeout=0.015) as port:
        port.reset_input_buffer()
        experiment = Experiment(port, args.log)
        try:
            for command in ("bench on", "stop", "diag reset", "stream on 20"):
                experiment.send(command)
                experiment.read_for(0.12)
            experiment.read_for(0.60)
            start = experiment.position()
            direction = 1 if args.target > start else -1
            drive_pwm = -130 * direction
            catch_pwm = -140 if drive_pwm > 0 else 140
            catch_ratio = 1.30 if catch_pwm > 0 else 0.75
            catch_ms = round(
                abs(drive_pwm) * args.drive_ms * catch_ratio / abs(catch_pwm)
            )
            experiment.emit(
                f"BASE start={start:.2f} target={args.target:.2f} "
                f"direction={direction:+d} drive={drive_pwm:+d}/{args.drive_ms} "
                f"catch={catch_pwm:+d}/{catch_ms}"
            )

            experiment.send(f"pulse {drive_pwm} {args.drive_ms}")
            experiment.read_for((args.drive_ms + 5) * 0.001)
            experiment.send("pulse 0 120")
            experiment.read_for(0.14)

            catch_reason = "timeout"
            deadline = time.monotonic() + args.max_wait
            while time.monotonic() < deadline:
                experiment.read_for(0.020)
                position = experiment.position()
                inward_speed = experiment.robust_speed() * direction
                remaining = (args.target - position) * direction
                displacement = (position - start) * direction
                projected_travel = max(0.0, inward_speed) * args.lookahead
                if abs(position) >= 118.0:
                    catch_reason = "endpoint_guard"
                    break
                if inward_speed >= 80.0:
                    catch_reason = "speed_guard"
                    break
                if (displacement >= 2.0 and inward_speed >= 5.0 and
                        remaining <= max(12.0, projected_travel)):
                    catch_reason = "stopping_guard"
                    break
            else:
                raise RuntimeError("no bounded catch condition before timeout")

            experiment.emit(
                f"CATCH_TRIGGER reason={catch_reason} "
                f"pos={experiment.position():.2f} "
                f"vin={experiment.robust_speed() * direction:.2f}"
            )
            experiment.send(f"pulse {catch_pwm} {catch_ms}")
            experiment.read_for((catch_ms + 5) * 0.001)
            experiment.send("pulse 0 500")
            experiment.read_for(2.50)
            experiment.send("status")
            experiment.read_for(0.15)
            experiment.emit(
                f"RESULT pos={experiment.position():.2f} "
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
