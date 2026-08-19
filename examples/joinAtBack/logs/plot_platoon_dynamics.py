#!/usr/bin/env python3
"""
plot_platoon_dynamics.py
=========================
Generates speed and gap plots from the real_positions_summary.csv
produced by extract_real_positions.py.
 
Reproduces the style of the reference paper plots:
  - Left:  Speed over time per vehicle + desired speed reference line
  - Right: Bumper gap over time per vehicle + desired gap reference line
 
Usage:
  python3 plot_platoon_dynamics.py <csv_file> [options]
 
Options:
  --desired-speed FLOAT     Reference speed line (m/s, default: 10.0)
  --desired-gap FLOAT       Reference gap line (m, default: 5.0)
  --headway FLOAT           Headway for CTS gap = gap_min + headway*speed (default: 0.5)
  --t-start FLOAT           Start time for plot window (default: auto)
  --t-end FLOAT             End time for plot window (default: auto)
  --exclude ACTORS          Comma-separated actor IDs to exclude (e.g. veh0)
  --output PATH             Save plot to file instead of showing
  --title STR               Plot title (default: derived from CSV filename)
  --style {paper,dark}      Plot style (default: paper)
 
Examples:
  python3 plot_platoon_dynamics.py real_positions_summary.csv
  python3 plot_platoon_dynamics.py real_positions_summary.csv \\
      --desired-speed 10 --desired-gap 5 --headway 0.5 \\
      --t-start 120 --t-end 240 --output dynamics.png
"""
 
import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path
 
 
# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------
 
def load_csv(path: str, exclude: list):
    """Load position CSV into per-actor time series."""
    actors = defaultdict(lambda: {
        "time": [], "speed": [], "gap": [], "rel_speed": [], "gap_error": []
    })
 
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            actor = row["actor"]
            if actor in exclude:
                continue
            try:
                t     = float(row["time"])
                speed = float(row["speed"])
                gap   = row.get("bumper_gap_to_predecessor", "")
                rel_v = row.get("relative_speed_to_predecessor", "")
                gerr  = row.get("gap_error", "")
 
                actors[actor]["time"].append(t)
                actors[actor]["speed"].append(speed)
                actors[actor]["gap"].append(float(gap) if gap else None)
                actors[actor]["rel_speed"].append(float(rel_v) if rel_v else None)
                actors[actor]["gap_error"].append(float(gerr) if gerr else None)
            except (ValueError, KeyError):
                continue
 
    return dict(actors)
 
 
# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
 
# Color palette matching the reference image style
COLORS = [
    "#4C72B0",  # blue
    "#55A868",  # green
    "#C44E52",  # red
    "#8172B2",  # purple
    "#CCB974",  # yellow
    "#64B5CD",  # light blue
]
 
LINESTYLES = ["-", "--", "-.", ":", "-", "--"]
 
 
def make_plots(actors: dict,
               desired_speed: float,
               desired_gap: float,
               headway: float,
               t_start: float,
               t_end: float,
               title: str,
               output: str,
               style: str,
               show_reference_lines: bool = True):
 
    try:
        import matplotlib.pyplot as plt
        import matplotlib.ticker as ticker
        import numpy as np
    except ImportError:
        print("matplotlib is required. Install with: pip3 install matplotlib --break-system-packages")
        sys.exit(1)
 
    # Apply style
    if style == "dark":
        plt.style.use("dark_background")
    else:
        plt.rcParams.update({
            "axes.facecolor":  "white",
            "figure.facecolor": "white",
            "axes.grid": True,
            "grid.alpha": 0.4,
            "grid.linestyle": "--",
            "font.size": 11,
            "axes.labelsize": 12,
            "legend.fontsize": 9,
            "axes.spines.top": False,
            "axes.spines.right": False,
        })
 
    sorted_actors = sorted(actors.keys())
 
    fig, (ax_speed, ax_gap) = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle(title, fontsize=13, fontweight="bold", y=1.01)
 
    # -----------------------------------------------------------------------
    # Left: Speed
    # -----------------------------------------------------------------------
    for i, actor in enumerate(sorted_actors):
        d = actors[actor]
        times  = d["time"]
        speeds = d["speed"]
 
        # Apply time window
        if t_start is not None or t_end is not None:
            ts = t_start if t_start is not None else times[0]
            te = t_end   if t_end   is not None else times[-1]
            pairs = [(t, s) for t, s in zip(times, speeds) if ts <= t <= te]
            if not pairs:
                continue
            times_w, speeds_w = zip(*pairs)
        else:
            times_w, speeds_w = times, speeds
 
        label = actor.replace("veh", "Vehicle ")
        ax_speed.plot(times_w, speeds_w,
                      color=COLORS[i % len(COLORS)],
                      linestyle=LINESTYLES[i % len(LINESTYLES)],
                      linewidth=1.5,
                      label=label)
 
    # Desired speed reference line
    if show_reference_lines:
        ax_speed.axhline(desired_speed, color="#C44E52", linewidth=1.2,
                         linestyle="-", alpha=0.7, label=f"Desired Speed ({desired_speed} m/s)")
 
    ax_speed.set_xlabel("Time (sec)")
    ax_speed.set_ylabel("Speed (m/s)")
    ax_speed.set_title("Speed")
    ax_speed.legend(loc="lower right")
 
    if t_start is not None:
        ax_speed.set_xlim(left=t_start)
    if t_end is not None:
        ax_speed.set_xlim(right=t_end)
 
    # -----------------------------------------------------------------------
    # Right: Gap
    # -----------------------------------------------------------------------
    followers = [a for a in sorted_actors
                 if any(g is not None for g in actors[a]["gap"])]
 
    for i, actor in enumerate(followers):
        d = actors[actor]
        times = d["time"]
        gaps  = d["gap"]
 
        # Apply time window + filter None
        if t_start is not None or t_end is not None:
            ts = t_start if t_start is not None else times[0]
            te = t_end   if t_end   is not None else times[-1]
            pairs = [(t, g) for t, g in zip(times, gaps)
                     if ts <= t <= te and g is not None]
        else:
            pairs = [(t, g) for t, g in zip(times, gaps) if g is not None]
 
        if not pairs:
            continue
 
        times_w, gaps_w = zip(*pairs)
        label = actor.replace("veh", "Vehicle ")
        ax_gap.plot(times_w, gaps_w,
                    color=COLORS[i % len(COLORS)],
                    linestyle=LINESTYLES[i % len(LINESTYLES)],
                    linewidth=1.5,
                    label=label)
 
    # Desired gap reference line — CTS: gap_min + headway * desired_speed
    if show_reference_lines:
        cts_gap = desired_gap + headway * desired_speed
        ax_gap.axhline(cts_gap, color="#C44E52", linewidth=1.2,
                       linestyle="-", alpha=0.7,
                       label=f"Desired Gap ({cts_gap:.1f} m)")
 
    ax_gap.set_xlabel("Time (sec)")
    ax_gap.set_ylabel("Gap (meters)")
    ax_gap.set_title("Bumper-to-Bumper Gap")
    ax_gap.legend(loc="upper right")
 
    if t_start is not None:
        ax_gap.set_xlim(left=t_start)
    if t_end is not None:
        ax_gap.set_xlim(right=t_end)
 
    # -----------------------------------------------------------------------
    # Optional: Acceleration subplot
    # -----------------------------------------------------------------------
    plt.tight_layout()
 
    if output:
        plt.savefig(output, dpi=150, bbox_inches="tight")
        print(f"Plot saved to: {output}")
    else:
        plt.show()
 
 
# ---------------------------------------------------------------------------
# Acceleration / gap error summary plots
# ---------------------------------------------------------------------------
 
def make_acceleration_plot(actors: dict,
                            t_start: float,
                            t_end: float,
                            title: str,
                            output: str):
    """
    Separate plot showing gap error over time — useful for string stability analysis.
    Gap error > 0 means vehicle is farther than desired (opening).
    Gap error < 0 means vehicle is closer than desired (closing / collision risk).
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return
 
    sorted_actors = sorted(actors.keys())
    followers = [a for a in sorted_actors
                 if any(g is not None for g in actors[a]["gap_error"])]
 
    if not followers:
        print("No gap error data found — skipping gap error plot.")
        return
 
    fig, ax = plt.subplots(figsize=(10, 4))
    fig.suptitle(f"{title} — Gap Error", fontsize=12)
 
    for i, actor in enumerate(followers):
        d = actors[actor]
        times  = d["time"]
        errors = d["gap_error"]
 
        if t_start is not None or t_end is not None:
            ts = t_start if t_start is not None else times[0]
            te = t_end   if t_end   is not None else times[-1]
            pairs = [(t, e) for t, e in zip(times, errors)
                     if ts <= t <= te and e is not None]
        else:
            pairs = [(t, e) for t, e in zip(times, errors) if e is not None]
 
        if not pairs:
            continue
 
        times_w, errors_w = zip(*pairs)
        label = actor.replace("veh", "Vehicle ")
        ax.plot(times_w, errors_w,
                color=COLORS[i % len(COLORS)],
                linestyle=LINESTYLES[i % len(LINESTYLES)],
                linewidth=1.4,
                label=label)
 
    ax.axhline(0, color="black", linewidth=1.0, linestyle="--", alpha=0.5)
    ax.fill_between(ax.get_xlim(), -100, 0, alpha=0.04, color="red")
    ax.set_xlabel("Time (sec)")
    ax.set_ylabel("Gap Error (m)")
    ax.set_title("Gap Error over Time  (negative = too close)")
    ax.legend()
    ax.grid(True, alpha=0.35)
    plt.tight_layout()
 
    if output:
        err_path = Path(output).stem + "_gap_error" + Path(output).suffix
        plt.savefig(err_path, dpi=150, bbox_inches="tight")
        print(f"Gap error plot saved to: {err_path}")
    else:
        plt.show()
 
 
# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
 
def main():
    parser = argparse.ArgumentParser(
        description="Plot speed and gap dynamics from platoon position CSV."
    )
    parser.add_argument("csv", help="Path to real_positions_summary.csv")
    parser.add_argument("--desired-speed", type=float, default=10.0,
                        help="Reference desired speed in m/s (default: 10.0)")
    parser.add_argument("--desired-gap", type=float, default=5.0,
                        help="Minimum gap (gap_min) in meters (default: 5.0)")
    parser.add_argument("--headway", type=float, default=0.5,
                        help="Headway in seconds for CTS desired gap (default: 0.5)")
    parser.add_argument("--t-start", type=float, default=None,
                        help="Start of plot time window in seconds")
    parser.add_argument("--t-end", type=float, default=None,
                        help="End of plot time window in seconds")
    parser.add_argument("--exclude", type=str, default="",
                        help="Comma-separated actor IDs to exclude (e.g. veh0)")
    parser.add_argument("--output", type=str, default=None,
                        help="Save plot to this file (PNG/PDF)")
    parser.add_argument("--title", type=str, default=None,
                        help="Plot title")
    parser.add_argument("--style", choices=["paper", "dark"], default="paper",
                        help="Visual style (default: paper)")
    parser.add_argument("--no-reference-lines", action="store_true",
                        help="Hide desired speed and gap reference lines")
    args = parser.parse_args()
 
    exclude = [a.strip() for a in args.exclude.split(",") if a.strip()]
    title   = args.title or Path(args.csv).stem.replace("_", " ")
 
    print(f"Loading: {args.csv}")
    actors = load_csv(args.csv, exclude)
 
    if not actors:
        print("No data loaded. Check CSV path and format.")
        sys.exit(1)
 
    print(f"Actors found: {sorted(actors.keys())}")
    for a, d in sorted(actors.items()):
        n = len(d["time"])
        t0 = d["time"][0] if n else 0
        t1 = d["time"][-1] if n else 0
        has_gap = sum(1 for g in d["gap"] if g is not None)
        print(f"  {a}: {n} samples, t={t0:.1f}–{t1:.1f}s, gap_samples={has_gap}")
 
    make_plots(
        actors,
        desired_speed=args.desired_speed,
        desired_gap=args.desired_gap,
        headway=args.headway,
        t_start=args.t_start,
        t_end=args.t_end,
        title=title,
        output=args.output,
        style=args.style,
        show_reference_lines=not args.no_reference_lines,
    )
 
    # if args.gap_error:
    #     make_acceleration_plot(
    #         actors,
    #         t_start=args.t_start,
    #         t_end=args.t_end,
    #         title=title,
    #         output=args.output,
    #     )
 
 
if __name__ == "__main__":
    main()