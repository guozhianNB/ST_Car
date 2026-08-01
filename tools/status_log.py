"""Parse and summarize ST_Car bench STATUS logs.

Usage:
    python tools/status_log.py <logfile> [--slice t0_ms t1_ms] [--step N]
        [--target MM] [--cols ball,vball,pid,ppos]

Prints a downsampled table plus key event stats: first-in-window time,
overshoot, settle time, vision gap stats.
"""

from __future__ import annotations

import argparse
import re
import sys

STATUS_RE = re.compile(r"^\[\s*(\d+)\]\s*(>>>.*|S?TATUS.*|OK.*|ERR.*|FAULT.*|STOP_REASON.*)$")


def parse(path: str):
    rows = []
    events = []
    pending_prefix = ""
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.rstrip("\n")
            m = re.match(r"^\[\s*(\d+)\]\s?(.*)$", line)
            if not m:
                continue
            t = int(m.group(1))
            text = m.group(2)
            if text.startswith(">>>"):
                events.append((t, text))
                continue
            text = pending_prefix + text
            if text.startswith("S") and not text.startswith("STATUS"):
                # split line: "S" fragment; wait for rest
                pending_prefix = text
                continue
            pending_prefix = ""
            if text.startswith("TATUS"):
                text = "S" + text
            if text.startswith(("OK", "ERR", "FAULT", "STOP_REASON")):
                events.append((t, text))
                continue
            if not text.startswith("STATUS"):
                continue
            fields = {}
            for part in text.split():
                if "=" in part:
                    k, v = part.split("=", 1)
                    fields[k] = v
            rows.append((t, fields))
    return rows, events


def f(fields, key, default=float("nan")):
    try:
        return float(fields.get(key, default))
    except ValueError:
        return default


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--slice", nargs=2, type=float, default=None,
                    metavar=("T0", "T1"))
    ap.add_argument("--step", type=int, default=5, help="print every Nth row")
    ap.add_argument("--target", type=float, default=None)
    ap.add_argument("--window", type=float, default=10.0)
    ap.add_argument("--cols", default="ball,vball,pid,ppos,pref")
    args = ap.parse_args()

    rows, events = parse(args.log)
    if args.slice:
        t0, t1 = args.slice
        rows = [(t, f_) for t, f_ in rows if t0 <= t <= t1]
        events = [(t, e) for t, e in events if t0 <= t <= t1]
    if not rows:
        print("no STATUS rows")
        return 1

    cols = args.cols.split(",")
    print("   t_ms " + " ".join(f"{c:>9}" for c in cols) + "   enc  sreq crate vs vage  vdt")
    merged = []
    ei = 0
    for i, (t, fields) in enumerate(rows):
        merged.append((t, fields))
    for i, (t, fields) in enumerate(merged):
        while ei < len(events) and events[ei][0] <= t:
            print(f"[{events[ei][0]:>7}] {events[ei][1]}")
            ei += 1
        if i % args.step == 0 or i == len(merged) - 1:
            vals = " ".join(f"{f(fields, c):>9.1f}" for c in cols)
            print(f"{t:>7} {vals} {f(fields,'enc'):>6.0f} {f(fields,'sreq'):>5.0f} "
                  f"{f(fields,'crate'):>5.0f} {f(fields,'vs'):>2.0f} {f(fields,'vage'):>4.0f} "
                  f"{f(fields,'vdt'):>4.0f}")
    while ei < len(events):
        print(f"[{events[ei][0]:>7}] {events[ei][1]}")
        ei += 1

    # stats
    balls = [f(f_, "ball") for _, f_ in rows]
    balls = [b for b in balls if b == b]
    speeds = [abs(f(f_, "vball")) for _, f_ in rows]
    pids = [abs(f(f_, "pid")) for _, f_ in rows]
    vdts = [f(f_, "vdt") for _, f_ in rows]
    vs = [f(f_, "vs") for _, f_ in rows]
    print(f"\nrows={len(rows)} span={rows[-1][0]-rows[0][0]} ms")
    print(f"ball min/max={min(balls):.1f}/{max(balls):.1f}  final={balls[-1]:.1f}")
    print(f"|vball| max={max(speeds):.0f}  |pid| max={max(pids):.0f}")
    real = [d for d, s in zip(vdts, vs) if s == 1 and d == d]
    if real:
        print(f"vdt(status=1) mean={sum(real)/len(real):.0f} max={max(real):.0f} "
              f"n>60ms={sum(1 for d in real if d > 60)} n>150ms={sum(1 for d in real if d > 150)}")
    if args.target is not None:
        tgt = args.target
        w = args.window
        first_in = None
        settled_since = None
        settle_t = None
        overshoot = 0.0
        for t, fields in rows:
            b = f(fields, "ball")
            v = f(fields, "vball")
            if b != b:
                continue
            err = b - tgt
            if first_in is None and abs(err) <= w:
                first_in = t
            if first_in is not None:
                if (b - tgt) * (1 if balls[0] < tgt else -1) < 0:
                    overshoot = max(overshoot, abs(b - tgt))
            good = abs(err) <= w and abs(v) <= 25.0
            if good:
                if settled_since is None:
                    settled_since = t
                elif t - settled_since >= 250 and settle_t is None:
                    settle_t = settled_since
            else:
                settled_since = None
        print(f"target={tgt} first|err|<={w}: "
              f"{(str(first_in - rows[0][0]) + ' ms') if first_in else 'never'}"
              f"  settled250ms@: "
              f"{(str(settle_t - rows[0][0]) + ' ms') if settle_t else 'never'}"
              f"  overshoot_past_target={overshoot:.1f} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
