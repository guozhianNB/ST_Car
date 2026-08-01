"""Requirement-3 sequence controller (Python prototype over the bench console).

The STM32 `vel` field is too spiky for fine logic, so this controller keeps
its own estimator fed by the raw BENCH stream: fresh (vision==1) position
frames update an EMA velocity; stale frames decay the velocity toward 0.

Per leg (target e.g. +50 then -50 mm):
    SHAKE   alternate the tube to break static friction until the ball moves
    ACCEL   full drive angle in the travel direction
    COAST   level the tube when v >= v_cap; re-accelerate below v_reaccel
    BRAKE   full reverse angle once predicted stop distance >= remaining dist
    SETTLE  level; micro-shake correction if stopped outside the window

Usage:
    python tools/req3.py --port COM3 --csv out.csv [--targets 50,-50]
"""

from __future__ import annotations

import argparse
import csv
import sys
import time

from pyseq import BenchLink, fnum


class Params:
    drive_deg = 2.2          # full drive/brake angle
    shake_deg = 2.2          # shake amplitude
    shake_ms = 200           # half-cycle duration
    shake_max_cycles = 6
    shake_move_mm = 4.0      # ball displacement proving release
    v_cap = 100.0            # accelerate until this speed (band top)
    v_cap_frames = 2         # require sustained speed before coasting
    v_reaccel = 85.0         # leave coast and re-accelerate below this
    v_sample_max = 500.0     # discard non-physical velocity samples
    brake_offset_mm = 45.0   # distance from target where brake starts
    brake_margin_mm = 0.0    # aim short(-)/long(+) of target
    still_mm = 1.0           # position no-change threshold
    still_ms = 250.0         # ...for this long -> ball considered stopped
    still_vel = 20.0         # and |vel_ema| below this
    settle_err = 10.0
    settle_vel = 25.0
    settle_ms = 250
    correct_deg = 2.2
    correct_ms = 200         # per half-cycle of the correction mini-shake
    correct_max = 4
    brake_min_speed = 25.0   # only brake when actually moving toward target
    shake_cycles_escalate = True
    vel_alpha = 0.35         # EMA weight of each fresh-frame velocity sample


class Estimator:
    """Tracks position and a windowed-difference velocity from BENCH rows."""

    def __init__(self) -> None:
        self.pos = 0.0
        self.vel = 0.0
        self.vision = 0
        self.last_update_host_ms = -1.0
        self.last_move_host_ms = -1.0
        self._last_row_index = -1
        self.history: list[tuple[float, float]] = []  # (host_ms, pos) fresh

    def feed(self, rows: list[tuple[float, dict[str, str]]], now_ms: float) -> None:
        fresh = False
        for i in range(self._last_row_index + 1, len(rows)):
            host_ms, fields = rows[i]
            self._last_row_index = i
            vision = int(fnum(fields, "vision", 0.0))
            self.vision = vision
            if vision != 1:
                continue
            pos = fnum(fields, "ball")
            if abs(pos - self.pos) > Params.still_mm:
                self.last_move_host_ms = host_ms
            self.pos = pos
            self.last_update_host_ms = host_ms
            self.history.append((host_ms, pos))
            if len(self.history) > 12:
                self.history.pop(0)
            fresh = True
        # Windowed velocity over the newest span >= 90 ms.
        if len(self.history) >= 2:
            t_new, p_new = self.history[-1]
            for t_old, p_old in self.history:
                dt = (t_new - t_old) / 1000.0
                if dt >= 0.09:
                    sample = (p_new - p_old) / dt
                    if abs(sample) <= Params.v_sample_max:
                        self.vel = sample
                    break
        if not fresh and self.last_update_host_ms >= 0.0:
            age = (now_ms - self.last_update_host_ms) / 1000.0
            if age > 0.25:
                self.vel *= 0.9

    def stopped(self, now_ms: float) -> bool:
        if self.last_move_host_ms < 0.0:
            return False
        return ((now_ms - self.last_move_host_ms) >= Params.still_ms and
                abs(self.vel) <= Params.still_vel)


def run_leg(link: BenchLink, p: Params, est: Estimator, target: float,
            budget_ms: float, log_prefix: str) -> tuple[bool, float]:
    t0 = time.monotonic()

    def now_ms() -> float:
        return (time.monotonic() - t0) * 1000.0

    def host_ms() -> float:
        return (time.monotonic() - link.t0) * 1000.0

    def sample() -> None:
        est.feed(link.rows, host_ms())
        if est.vision not in (0, 1) and link.snapshot().get("fault", "none") != "none":
            raise RuntimeError(f"fault: {link.snapshot().get('fault')}")

    def over_budget() -> bool:
        return now_ms() > budget_ms

    # --- SHAKE ---
    sample()
    if est.last_update_host_ms < 0.0:
        # No fresh frame yet (e.g. ball hidden in an end zone): seed with the
        # last-known (possibly stale) position from the telemetry snapshot.
        est.pos = fnum(link.snapshot(), "ball")
    start_pos = est.pos
    direction = 1.0 if target > start_pos else -1.0
    released = (abs(start_pos - target) <= p.settle_err and
                est.last_update_host_ms >= 0.0)
    for cycle in range(p.shake_max_cycles):
        if released:
            break
        amp = min(p.shake_deg + 0.1 * (cycle // 2), 2.4)
        for deg in (-amp * direction, amp * direction):
            link.cmd(f"angle {deg:.2f}", pause=0.01)
            t_end = time.monotonic() + p.shake_ms / 1000.0
            while time.monotonic() < t_end:
                sample()
                if (est.pos - start_pos) * direction >= p.shake_move_mm:
                    released = True
                if released:
                    break
            if released:
                break
    link.cmd(f"angle {-direction * p.drive_deg:.2f}", pause=0.01)
    print(f"{log_prefix} released={released} t={now_ms():.0f} pos={est.pos:.0f}",
          flush=True)

    # --- ACCEL / COAST / BRAKE ---
    state = "ACCEL"
    cap_frames = 0
    rev_frames = 0
    while not over_budget():
        sample()
        v_fwd = est.vel * direction
        dist = (target - est.pos) * direction
        stop_dist = p.brake_offset_mm + p.brake_margin_mm
        no_fresh_ms = (host_ms() - est.last_update_host_ms
                       if est.last_update_host_ms >= 0.0 else 1e9)
        if state == "BRAKE" and no_fresh_ms > 800.0:
            break  # hidden ball: assume it stopped or left view
        if state == "BRAKE":
            if v_fwd <= 5.0:
                rev_frames += 1
            else:
                rev_frames = 0
            if rev_frames >= 2 or est.stopped(host_ms()):
                break  # ball stopped or reversed: level off immediately
            time.sleep(0.008)
            continue
        if est.stopped(host_ms()):
            break
        if state != "BRAKE" and no_fresh_ms <= 400.0:
            if v_fwd >= p.v_cap:
                cap_frames += 1
            else:
                cap_frames = 0
        if state == "ACCEL":
            if cap_frames >= p.v_cap_frames:
                state = "COAST"
                link.cmd("angle 0.000", pause=0.01)
                print(f"{log_prefix} cap t={now_ms():.0f} pos={est.pos:.0f} "
                      f"v={est.vel:.0f}", flush=True)
            elif (dist <= stop_dist and no_fresh_ms <= 400.0 and
                  v_fwd >= p.brake_min_speed):
                state = "BRAKE"
                link.cmd(f"angle {direction * p.drive_deg:.2f}", pause=0.01)
                print(f"{log_prefix} brake-on t={now_ms():.0f} pos={est.pos:.0f} "
                      f"v={est.vel:.0f}", flush=True)
        elif state == "COAST":
            if (dist <= stop_dist and no_fresh_ms <= 400.0 and
                    v_fwd >= p.brake_min_speed):
                state = "BRAKE"
                link.cmd(f"angle {direction * p.drive_deg:.2f}", pause=0.01)
                print(f"{log_prefix} brake-on t={now_ms():.0f} pos={est.pos:.0f} "
                      f"v={est.vel:.0f}", flush=True)
            elif v_fwd < p.v_reaccel and no_fresh_ms <= 400.0:
                state = "ACCEL"
                link.cmd(f"angle {-direction * p.drive_deg:.2f}", pause=0.01)
        time.sleep(0.008)
    link.cmd("angle 0.000", pause=0.02)
    print(f"{log_prefix} travel done t={now_ms():.0f} pos={est.pos:.0f} "
          f"state={state}", flush=True)

    # --- SETTLE / CORRECT ---
    ok_since: float | None = None
    corrections = 0
    while not over_budget():
        sample()
        if abs(est.pos - target) <= p.settle_err and abs(est.vel) <= p.settle_vel:
            if ok_since is None:
                ok_since = time.monotonic()
            elif (time.monotonic() - ok_since) * 1000.0 >= p.settle_ms:
                return True, now_ms()
        else:
            ok_since = None
            if (est.stopped(host_ms()) and corrections < p.correct_max and
                    abs(est.pos - target) > p.settle_err):
                corrections += 1
                corr_dir = 1.0 if target > est.pos else -1.0
                corr_start = est.pos
                moved = False
                for deg in (corr_dir * p.correct_deg, -corr_dir * p.correct_deg,
                            -corr_dir * p.correct_deg):
                    link.cmd(f"angle {deg:.2f}", pause=0.01)
                    t_end = time.monotonic() + p.correct_ms / 1000.0
                    while time.monotonic() < t_end:
                        sample()
                        if (est.pos - corr_start) * corr_dir >= p.shake_move_mm:
                            moved = True
                        time.sleep(0.008)
                link.cmd("angle 0.000", pause=0.01)
                print(f"{log_prefix} correct#{corrections} pos={est.pos:.0f} "
                      f"moved={moved}", flush=True)
                if moved:
                    # Ball is loose again: run a fresh short travel to target.
                    return run_leg_travel_only(link, p, est, target, budget_ms,
                                               now_ms(), log_prefix, t0)
        time.sleep(0.01)
    return False, now_ms()


def run_leg_travel_only(link: BenchLink, p: Params, est: Estimator,
                        target: float, budget_ms: float, used_ms: float,
                        log_prefix: str, t0: float) -> tuple[bool, float]:
    """After a correction released the ball, drive/brake to target and settle."""

    def now_ms() -> float:
        return (time.monotonic() - t0) * 1000.0

    def host_ms() -> float:
        return (time.monotonic() - link.t0) * 1000.0

    def sample() -> None:
        est.feed(link.rows, host_ms())

    def over_budget() -> bool:
        return now_ms() > budget_ms

    direction = 1.0 if target > est.pos else -1.0
    link.cmd(f"angle {-direction * p.drive_deg:.2f}", pause=0.01)
    state = "ACCEL"
    while not over_budget():
        sample()
        v_fwd = est.vel * direction
        dist = (target - est.pos) * direction
        stop_dist = p.brake_offset_mm + p.brake_margin_mm
        if est.stopped(host_ms()):
            break
        if state == "ACCEL" and ((dist <= stop_dist and v_fwd >= p.brake_min_speed) or
                                 v_fwd >= p.v_cap):
            state = "BRAKE" if dist <= stop_dist else "COAST"
            link.cmd(f"angle {direction * p.drive_deg:.2f}"
                     if state == "BRAKE" else "angle 0.000", pause=0.01)
        elif state == "COAST":
            if dist <= stop_dist:
                state = "BRAKE"
                link.cmd(f"angle {direction * p.drive_deg:.2f}", pause=0.01)
            elif v_fwd < p.v_reaccel:
                state = "ACCEL"
                link.cmd(f"angle {-direction * p.drive_deg:.2f}", pause=0.01)
        time.sleep(0.008)
    link.cmd("angle 0.000", pause=0.02)
    ok_since: float | None = None
    while not over_budget():
        sample()
        if abs(est.pos - target) <= p.settle_err and abs(est.vel) <= p.settle_vel:
            if ok_since is None:
                ok_since = time.monotonic()
            elif (time.monotonic() - ok_since) * 1000.0 >= p.settle_ms:
                return True, now_ms()
        else:
            ok_since = None
        time.sleep(0.01)
    return False, now_ms()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--csv", required=True)
    parser.add_argument("--targets", default="50,-50")
    parser.add_argument("--total-ms", type=float, default=5000.0)
    parser.add_argument("--angle-limit", type=float, default=2.4)
    args = parser.parse_args()
    targets = [float(x) for x in args.targets.split(",")]

    p = Params()
    link = BenchLink(args.port)
    est = Estimator()
    t_start = time.monotonic()
    ok_all = False
    try:
        link.cmd("stop", pause=0.15)
        link.cmd("bench on", pause=0.2)
        link.cmd(f"limit angle {args.angle_limit:.2f}", pause=0.1)
        link.cmd("stream on 50", pause=0.3)
        ok_all = True
        for i, target in enumerate(targets):
            remaining = args.total_ms - (time.monotonic() - t_start) * 1000.0
            ok, used = run_leg(link, p, est, target, remaining,
                               f"[leg{i}->{target:+.0f}]")
            print(f"leg{i} target={target:+.0f} settled={ok} used={used:.0f} ms",
                  flush=True)
            ok_all = ok_all and ok
        total = (time.monotonic() - t_start) * 1000.0
        print(f"TOTAL {total:.0f} ms  all_settled={ok_all}", flush=True)
        link.cmd("angle 0.000", pause=0.05)
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
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
