"""Feedback-driven bench experiment runner for requirement-3 tuning.

Talks to the STM32 bench console (ST-Link VCP). The MCU runs its SA100
angle inner loop; this script plays the role of the ball-position outer
loop by sending `angle <deg>` commands based on streamed telemetry.

Usage:
    python tools/pyseq.py --port COM3 --script <spec> --csv out.csv

Spec is a semicolon-separated list of phases:
    drive:<deg>:<until_expr>:<max_ms>
    brake:<deg>:<until_expr>:<max_ms>
    level:<max_ms>
    wait:<ms>
    settle:<target_mm>:<err_mm>:<vel_mms>:<hold_ms>:<max_ms>

until_expr examples:  pos>=40  pos<=-30  absvel<=20  vel>=50  pos+vel*0.3>=50
Expressions may use pos, vel, ang, t (phase elapsed ms), absvel, and
python operators.  The phase ends when the expression is true or max_ms
elapses (whichever first).  `level` commands 0 deg for max_ms.
`settle` waits until |pos-target|<=err and |vel|<=vel for hold_ms.

Every received BENCH line is logged to CSV with host-relative ms time.
The runner always finishes with: angle 0 -> wait for |ang|<=0.15 -> stop.
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
import threading
import time

import serial

FIELD_RE = re.compile(r"\b([a-zA-Z]+)=(-?\d+(?:\.\d+)?|none|[a-z_]+)")


class BenchLink:
    def __init__(self, port_name: str, baud: int = 115200):
        self.port = serial.Serial(port_name, baud, timeout=0.02)
        self.port.reset_input_buffer()
        self.latest: dict[str, str] = {}
        self.latest_host_ms = 0.0
        self.rows: list[tuple[float, dict[str, str]]] = []
        self.t0 = time.monotonic()
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self) -> None:
        buf = b""
        while not self.stopped.is_set():
            try:
                chunk = self.port.read(self.port.in_waiting or 1)
            except serial.SerialException:
                self.stopped.set()
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                host_ms = (time.monotonic() - self.t0) * 1000.0
                if line.startswith("BENCH"):
                    fields = dict(FIELD_RE.findall(line))
                    with self.lock:
                        self.latest = fields
                        self.latest_host_ms = host_ms
                        self.rows.append((host_ms, fields))
                else:
                    print(f"    << {line}", flush=True)

    def cmd(self, text: str, pause: float = 0.05) -> None:
        print(f"    >> {text}", flush=True)
        self.port.write((text + "\n").encode("ascii"))
        self.port.flush()
        time.sleep(pause)

    def snapshot(self) -> dict[str, str]:
        with self.lock:
            return dict(self.latest)

    def close(self) -> None:
        self.stopped.set()
        self.thread.join(timeout=0.5)
        self.port.close()


def fnum(fields: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(fields.get(key, default))
    except ValueError:
        return default


def make_env(fields: dict[str, str], phase_t_ms: float) -> dict[str, float]:
    pos = fnum(fields, "ball")
    vel = fnum(fields, "vel")
    return {
        "pos": pos,
        "vel": vel,
        "absvel": abs(vel),
        "ang": fnum(fields, "ang"),
        "vis": fnum(fields, "vision", 0.0),
        "t": phase_t_ms,
    }


def run(link: BenchLink, spec: str, angle_limit: float) -> None:
    last_sent_angle: float | None = None

    def set_angle(deg: float) -> None:
        nonlocal last_sent_angle
        deg = max(-angle_limit, min(angle_limit, deg))
        if last_sent_angle is None or abs(deg - last_sent_angle) >= 0.03:
            link.cmd(f"angle {deg:.3f}", pause=0.02)
            last_sent_angle = deg

    for phase in [p.strip() for p in spec.split(";") if p.strip()]:
        parts = phase.split(":")
        kind = parts[0]
        phase_start = time.monotonic()
        print(f"== phase {phase}", flush=True)
        if kind in ("drive", "brake"):
            deg = float(parts[1])
            expr = parts[2]
            max_ms = int(parts[3])
            set_angle(deg)
            while (time.monotonic() - phase_start) * 1000.0 < max_ms:
                fields = link.snapshot()
                if fields.get("fault", "none") != "none":
                    raise RuntimeError(f"bench fault: {fields.get('fault')}")
                env = make_env(fields, (time.monotonic() - phase_start) * 1000.0)
                if eval(expr, {"__builtins__": {}}, env):  # noqa: S307 - local tool
                    break
                time.sleep(0.01)
            print(f"   end pos={env['pos']:.1f} vel={env['vel']:.1f} "
                  f"t={env['t']:.0f}", flush=True)
        elif kind == "level":
            max_ms = int(parts[1])
            set_angle(0.0)
            time.sleep(max_ms / 1000.0)
        elif kind == "wait":
            time.sleep(int(parts[1]) / 1000.0)
        elif kind == "settle":
            target = float(parts[1])
            err = float(parts[2])
            vel_lim = float(parts[3])
            hold_ms = int(parts[4])
            max_ms = int(parts[5])
            ok_since: float | None = None
            while (time.monotonic() - phase_start) * 1000.0 < max_ms:
                fields = link.snapshot()
                env = make_env(fields, (time.monotonic() - phase_start) * 1000.0)
                if abs(env["pos"] - target) <= err and abs(env["vel"]) <= vel_lim:
                    if ok_since is None:
                        ok_since = time.monotonic()
                    elif (time.monotonic() - ok_since) * 1000.0 >= hold_ms:
                        print(f"   settled pos={env['pos']:.1f}", flush=True)
                        break
                else:
                    ok_since = None
                time.sleep(0.01)
        else:
            raise ValueError(f"unknown phase kind: {kind}")

    # Always finish level and stopped.
    set_angle(0.0)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if abs(fnum(link.snapshot(), "ang")) <= 0.15:
            break
        time.sleep(0.02)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--script", required=True)
    parser.add_argument("--csv", required=True)
    parser.add_argument("--angle-limit", type=float, default=2.3)
    args = parser.parse_args()

    link = BenchLink(args.port)
    try:
        link.cmd("stop", pause=0.15)
        link.cmd("bench on", pause=0.2)
        link.cmd(f"limit angle {args.angle_limit:.2f}", pause=0.1)
        link.cmd("stream on 50", pause=0.3)
        run(link, args.script, args.angle_limit)
        link.cmd("stop", pause=0.1)
        link.cmd("stream off", pause=0.05)
    finally:
        rows = list(link.rows)
        link.close()
    if rows:
        keys: list[str] = []
        for _, fields in rows:
            for key in fields:
                if key not in keys:
                    keys.append(key)
        with open(args.csv, "w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(["host_ms"] + keys)
            for host_ms, fields in rows:
                writer.writerow([f"{host_ms:.0f}"] +
                                [fields.get(k, "") for k in keys])
        print(f"wrote {len(rows)} rows to {args.csv}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
