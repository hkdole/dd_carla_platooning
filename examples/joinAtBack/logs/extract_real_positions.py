#!/usr/bin/env python3

import argparse
import csv
import math
import re
from collections import defaultdict


REAL_POSITION_MARKER = "[CarlanetManager][updateNodesPosition] event=REAL_POSITION"


KV_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([^ \t\r\n]+)")


def parse_kv_line(line: str):
    """
    Parse log lines like:

    [INFO] [CarlanetManager][updateNodesPosition] event=REAL_POSITION source=CARLA
    simulation_time=0 actor=veh0 x=280.993 y=16.6058 ...
    """
    if REAL_POSITION_MARKER not in line:
        return None

    fields = {}
    for match in KV_RE.finditer(line):
        key = match.group(1)
        value = match.group(2)
        fields[key] = value

    required = ("simulation_time", "actor", "x", "y", "z", "vx", "vy", "vz")
    for key in required:
        if key not in fields:
            return None

    def f(key, default=0.0):
        try:
            return float(fields.get(key, default))
        except Exception:
            return float(default)

    actor = fields["actor"]

    return {
        "time": f("simulation_time"),
        "actor": actor,
        "x": f("x"),
        "y": f("y"),
        "z": f("z"),
        "vx": f("vx"),
        "vy": f("vy"),
        "vz": f("vz"),
        "ax": f("ax"),
        "ay": f("ay"),
        "az": f("az"),
        "speed": f("speed", math.sqrt(f("vx") ** 2 + f("vy") ** 2 + f("vz") ** 2)),
        "yaw": f("yaw"),
        "pitch": f("pitch"),
        "roll": f("roll"),
    }


def vehicle_sort_key(actor_id: str):
    if actor_id.startswith("veh") and actor_id[3:].isdigit():
        return int(actor_id[3:])
    return 10**9


def distance_xy(a, b):
    dx = b["x"] - a["x"]
    dy = b["y"] - a["y"]
    return math.sqrt(dx * dx + dy * dy)


def should_sample(t, next_sample_t, eps=1e-9):
    return t + eps >= next_sample_t


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logfile")
    parser.add_argument("--sample-period", type=float, default=1.0)
    parser.add_argument("--target-gap", type=float, default=5.0)
    parser.add_argument("--gap-tolerance", type=float, default=1.0)
    parser.add_argument("--vehicle-length", type=float, default=4.5)
    parser.add_argument("--out", default="real_positions_summary.csv")
    args = parser.parse_args()

    if args.sample_period <= 0:
        raise SystemExit("ERROR: --sample-period must be > 0")

    rows = []

    with open(args.logfile, "r", errors="replace") as f:
        for line in f:
            parsed = parse_kv_line(line)
            if parsed is not None:
                rows.append(parsed)

    if not rows:
        raise SystemExit(
            "ERROR: no real position lines found. Expected lines containing:\n"
            f"  {REAL_POSITION_MARKER}\n\n"
            "Your log may be different, or the script is reading the wrong file."
        )

    rows.sort(key=lambda r: (r["time"], vehicle_sort_key(r["actor"])))

    by_time = defaultdict(dict)
    for row in rows:
        by_time[row["time"]][row["actor"]] = row

    times = sorted(by_time.keys())
    actor_ids = sorted(
        {row["actor"] for row in rows},
        key=vehicle_sort_key,
    )

    print(f"Found {len(rows)} REAL_POSITION rows")
    print(f"Actors: {', '.join(actor_ids)}")
    print(f"Time range: {times[0]:.3f}s to {times[-1]:.3f}s")

    out_rows = []
    next_sample_t = times[0]

    for t in times:
        if not should_sample(t, next_sample_t):
            continue

        snapshot = by_time[t]
        present = [a for a in actor_ids if a in snapshot]

        if not present:
            continue

        # Emit one row per vehicle.
        for actor in present:
            r = snapshot[actor]

            predecessor = None
            gap_center = None
            gap_bumper = None
            relative_speed = None
            gap_error = None
            gap_ok = None

            idx = present.index(actor)
            if idx > 0:
                predecessor = present[idx - 1]
                pred = snapshot[predecessor]

                gap_center = distance_xy(r, pred)
                gap_bumper = gap_center - args.vehicle_length
                relative_speed = r["speed"] - pred["speed"]

                gap_error = gap_bumper - args.target_gap
                gap_ok = abs(gap_error) <= args.gap_tolerance

            out_rows.append({
                "time": t,
                "actor": actor,
                "x": r["x"],
                "y": r["y"],
                "z": r["z"],
                "vx": r["vx"],
                "vy": r["vy"],
                "vz": r["vz"],
                "ax": r["ax"],
                "ay": r["ay"],
                "az": r["az"],
                "speed": r["speed"],
                "yaw": r["yaw"],
                "predecessor": predecessor or "",
                "center_gap_to_predecessor": "" if gap_center is None else gap_center,
                "bumper_gap_to_predecessor": "" if gap_bumper is None else gap_bumper,
                "relative_speed_to_predecessor": "" if relative_speed is None else relative_speed,
                "gap_error": "" if gap_error is None else gap_error,
                "gap_ok": "" if gap_ok is None else int(gap_ok),
            })

        while next_sample_t <= t + 1e-9:
            next_sample_t += args.sample_period

    fieldnames = [
        "time",
        "actor",
        "x",
        "y",
        "z",
        "vx",
        "vy",
        "vz",
        "ax",
        "ay",
        "az",
        "speed",
        "yaw",
        "predecessor",
        "center_gap_to_predecessor",
        "bumper_gap_to_predecessor",
        "relative_speed_to_predecessor",
        "gap_error",
        "gap_ok",
    ]

    with open(args.out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(out_rows)

    print(f"Wrote {len(out_rows)} sampled rows to {args.out}")

    # Print compact final sampled snapshot.
    print("\nLast sampled rows:")
    for row in out_rows[-len(actor_ids):]:
        print(
            f"t={row['time']:.1f} "
            f"{row['actor']} "
            f"speed={float(row['speed']):.3f} "
            f"pred={row['predecessor']} "
            f"bumper_gap={row['bumper_gap_to_predecessor']}"
        )


if __name__ == "__main__":
    main()