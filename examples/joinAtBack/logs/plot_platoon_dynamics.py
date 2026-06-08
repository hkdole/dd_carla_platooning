"""
plot_platoon_dynamics_ieee.py
==============================
IEEE-journal-ready figures from real_positions_summary.csv.
 
Differences from the original script:
  - Each plot (Speed, Gap) is saved as a SEPARATE file.
  - Figure dimensions match IEEE single-column width (88 mm = 3.46 in).
  - Font sizes, line widths, and marker sizes are tuned for print legibility.
  - Output defaults to PDF (vector) for lossless scaling; use --fmt png for raster.
  - DPI defaults to 300 for raster output.
 
Usage:
  python3 plot_platoon_dynamics_ieee.py <csv_file> [options]
 
Options:
  --desired-speed FLOAT     Reference speed line (m/s, default: 10.0)
  --desired-gap FLOAT       Reference gap line (m, default: 5.0)
  --headway FLOAT           Headway for CTS gap = gap_min + headway*speed (default: 0.5)
  --t-start FLOAT           Start time for plot window (default: auto)
  --t-end FLOAT             End time for plot window (default: auto)
  --exclude ACTORS          Comma-separated actor IDs to exclude (e.g. veh0)
  --output-prefix PATH      Prefix for output files (default: "platoon")
                              Produces <prefix>_speed.<fmt> and <prefix>_gap.<fmt>
  --fmt {pdf,eps,png}       Output format (default: pdf)
  --dpi INT                 DPI for raster output (default: 300)
  --col {single,double}     IEEE column width: single=88mm, double=180mm (default: single)
  --no-reference-lines      Hide desired speed/gap reference lines
  --gap-error               Also generate a gap-error figure
 
Examples:
  python3 plot_platoon_dynamics_ieee.py real_positions_summary.csv
  python3 plot_platoon_dynamics_ieee.py real_positions_summary.csv \\
      --desired-speed 10 --desired-gap 5 --headway 0.5 \\
      --t-start 120 --t-end 240 --output-prefix fig3 --fmt pdf
"""
 
import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path
 
 
# ---------------------------------------------------------------------------
# IEEE figure dimensions
# ---------------------------------------------------------------------------
 
IEEE_WIDTHS = {
    "single": 88 / 25.4,   # 3.46 inches
    "double": 180 / 25.4,  # 7.09 inches
}
# Height chosen to give a ~4:3 aspect ratio, readable at column width
IEEE_HEIGHT = 65 / 25.4    # 2.56 inches
 
 
# ---------------------------------------------------------------------------
# Matplotlib style for IEEE
# ---------------------------------------------------------------------------
 
def apply_ieee_style():
    try:
        import matplotlib.pyplot as plt
        import matplotlib as mpl
    except ImportError:
        print("matplotlib is required: pip3 install matplotlib --break-system-packages")
        sys.exit(1)
 
    mpl.rcParams.update({
        # Figure
        "figure.facecolor":     "white",
        "figure.dpi":           300,
        # Axes
        "axes.facecolor":       "white",
        "axes.linewidth":       0.8,
        "axes.labelsize":       8,
        "axes.titlesize":       9,
        "axes.grid":            True,
        "axes.spines.top":      False,
        "axes.spines.right":    False,
        # Grid
        "grid.alpha":           0.35,
        "grid.linestyle":       "--",
        "grid.linewidth":       0.5,
        # Ticks
        "xtick.labelsize":      7,
        "ytick.labelsize":      7,
        "xtick.major.width":    0.8,
        "ytick.major.width":    0.8,
        "xtick.direction":      "in",
        "ytick.direction":      "in",
        # Legend
        "legend.fontsize":      7,
        "legend.framealpha":    0.8,
        "legend.edgecolor":     "0.8",
        "legend.handlelength":  1.5,
        # Lines
        "lines.linewidth":      1.2,
        # Font  — Times New Roman matches IEEE body text; fallback to serif
        "font.family":          "serif",
        "font.serif":           ["Times New Roman", "Times", "DejaVu Serif"],
        "mathtext.fontset":     "stix",
        # PDF/EPS output
        "pdf.fonttype":         42,   # embed TrueType (avoids Type-3 issues)
        "ps.fonttype":          42,
    })
 
 
# ---------------------------------------------------------------------------
# Colour / linestyle palette (IEEE-friendly, prints well in greyscale too)
# ---------------------------------------------------------------------------
 
COLORS = [
    "#1f77b4",  # blue
    "#2ca02c",  # green
    "#d62728",  # red
    "#9467bd",  # purple
    "#8c564b",  # brown
    "#e377c2",  # pink
]
 
LINESTYLES = ["-", "--", "-.", ":", (0, (3, 1, 1, 1)), (0, (5, 2))]
 
 
# ---------------------------------------------------------------------------
# Data loading (unchanged logic from original)
# ---------------------------------------------------------------------------
 
def load_csv(path: str, exclude: list):
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
# Helpers
# ---------------------------------------------------------------------------
 
def _window(times, values, t_start, t_end):
    """Filter a (time, value) series to [t_start, t_end], dropping None values."""
    ts = t_start if t_start is not None else times[0]
    te = t_end   if t_end   is not None else times[-1]
    pairs = [(t, v) for t, v in zip(times, values) if ts <= t <= te and v is not None]
    if not pairs:
        return [], []
    return zip(*pairs)
 
 
def _save(fig, path: str, fmt: str, dpi: int):
    out = f"{path}.{fmt}"
    fig.savefig(out, dpi=dpi, bbox_inches="tight", format=fmt)
    print(f"Saved: {out}")
 
 
# ---------------------------------------------------------------------------
# Speed figure
# ---------------------------------------------------------------------------
 
def plot_speed(actors: dict,
               desired_speed: float,
               t_start, t_end,
               title: str,
               output_prefix: str,
               fmt: str,
               dpi: int,
               col_width: float,
               show_ref: bool):
 
    import matplotlib.pyplot as plt
 
    sorted_actors = sorted(actors.keys())
 
    fig, ax = plt.subplots(figsize=(col_width, IEEE_HEIGHT))
 
    for i, actor in enumerate(sorted_actors):
        d = actors[actor]
        tw, sw = _window(d["time"], d["speed"], t_start, t_end)
        if not tw:
            continue
        label = actor.replace("veh", "Veh.\\ ") if actor.startswith("veh") else actor
        ax.plot(list(tw), list(sw),
                color=COLORS[i % len(COLORS)],
                linestyle=LINESTYLES[i % len(LINESTYLES)],
                linewidth=1.2,
                label=label)
 
    if show_ref:
        ax.axhline(desired_speed, color="black", linewidth=0.9,
                   linestyle=":", alpha=0.85,
                   label=f"$v_{{\\mathrm{{des}}}}$ = {desired_speed} m/s")
 
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (m/s)")
    # ax.set_title(title, pad=4)
    ax.legend(loc="lower right", ncol=1)
 
    if t_start is not None:
        ax.set_xlim(left=t_start)
    if t_end is not None:
        ax.set_xlim(right=t_end)
 
    plt.tight_layout(pad=0.4)
    _save(fig, f"{output_prefix}_speed", fmt, dpi)
    plt.close(fig)
 
 
# ---------------------------------------------------------------------------
# Gap figure
# ---------------------------------------------------------------------------
 
def plot_gap(actors: dict,
             desired_gap: float,
             headway: float,
             desired_speed: float,
             t_start, t_end,
             title: str,
             output_prefix: str,
             fmt: str,
             dpi: int,
             col_width: float,
             show_ref: bool):
 
    import matplotlib.pyplot as plt
 
    sorted_actors = sorted(actors.keys())
    followers = [a for a in sorted_actors
                 if any(g is not None for g in actors[a]["gap"])]
 
    fig, ax = plt.subplots(figsize=(col_width, IEEE_HEIGHT))
 
    for i, actor in enumerate(followers):
        d = actors[actor]
        tw, gw = _window(d["time"], d["gap"], t_start, t_end)
        if not tw:
            continue
        label = actor.replace("veh", "Veh.\\ ") if actor.startswith("veh") else actor
        ax.plot(list(tw), list(gw),
                color=COLORS[i % len(COLORS)],
                linestyle=LINESTYLES[i % len(LINESTYLES)],
                linewidth=1.2,
                label=label)
 
    if show_ref:
        cts_gap = desired_gap + headway * desired_speed
        ax.axhline(cts_gap, color="black", linewidth=0.9,
                   linestyle=":", alpha=0.85,
                   label=f"$d_{{\\mathrm{{des}}}}$ = {cts_gap:.1f} m")
 
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Gap (m)")
    # ax.set_title(title, pad=4)
    ax.legend(loc="upper right", ncol=1)
 
    if t_start is not None:
        ax.set_xlim(left=t_start)
    if t_end is not None:
        ax.set_xlim(right=t_end)
 
    plt.tight_layout(pad=0.4)
    _save(fig, f"{output_prefix}_gap", fmt, dpi)
    plt.close(fig)
 
 
# ---------------------------------------------------------------------------
# Gap error figure (optional)
# ---------------------------------------------------------------------------
 
def plot_gap_error(actors: dict,
                   t_start, t_end,
                   title: str,
                   output_prefix: str,
                   fmt: str,
                   dpi: int,
                   col_width: float):
 
    import matplotlib.pyplot as plt
 
    sorted_actors = sorted(actors.keys())
    followers = [a for a in sorted_actors
                 if any(g is not None for g in actors[a]["gap_error"])]
 
    if not followers:
        print("No gap_error data found — skipping gap error figure.")
        return
 
    fig, ax = plt.subplots(figsize=(col_width, IEEE_HEIGHT))
 
    for i, actor in enumerate(followers):
        d = actors[actor]
        tw, ew = _window(d["time"], d["gap_error"], t_start, t_end)
        if not tw:
            continue
        label = actor.replace("veh", "Veh.\\ ") if actor.startswith("veh") else actor
        ax.plot(list(tw), list(ew),
                color=COLORS[i % len(COLORS)],
                linestyle=LINESTYLES[i % len(LINESTYLES)],
                linewidth=1.2,
                label=label)
 
    ax.axhline(0, color="black", linewidth=0.8, linestyle="--", alpha=0.6)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Gap Error (m)")
    # ax.set_title(title, pad=4)
    ax.legend(ncol=1)
 
    if t_start is not None:
        ax.set_xlim(left=t_start)
    if t_end is not None:
        ax.set_xlim(right=t_end)
 
    plt.tight_layout(pad=0.4)
    _save(fig, f"{output_prefix}_gap_error", fmt, dpi)
    plt.close(fig)
 
 
# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
 
def main():
    parser = argparse.ArgumentParser(
        description="IEEE-ready separate speed and gap figures from platoon CSV."
    )
    parser.add_argument("csv", help="Path to real_positions_summary.csv")
    parser.add_argument("--desired-speed", type=float, default=10.0)
    parser.add_argument("--desired-gap",   type=float, default=5.0)
    parser.add_argument("--headway",       type=float, default=0.5)
    parser.add_argument("--t-start",       type=float, default=None)
    parser.add_argument("--t-end",         type=float, default=None)
    parser.add_argument("--exclude",       type=str,   default="")
    parser.add_argument("--output-prefix", type=str,   default="platoon",
                        help="Prefix for output filenames (default: platoon)")
    parser.add_argument("--fmt",           choices=["pdf", "eps", "png"], default="pdf",
                        help="Output format (default: pdf)")
    parser.add_argument("--dpi",           type=int,   default=300,
                        help="DPI for raster output (default: 300)")
    parser.add_argument("--col",           choices=["single", "double"], default="single",
                        help="IEEE column width (default: single = 88 mm)")
    parser.add_argument("--no-reference-lines", action="store_true")
    parser.add_argument("--gap-error",     action="store_true")
    args = parser.parse_args()
 
    apply_ieee_style()
 
    exclude    = [a.strip() for a in args.exclude.split(",") if a.strip()]
    col_width  = IEEE_WIDTHS[args.col]
    show_ref   = not args.no_reference_lines
    title      = Path(args.csv).stem.replace("_", " ")
 
    print(f"Loading: {args.csv}")
    actors = load_csv(args.csv, exclude)
 
    if not actors:
        print("No data loaded. Check CSV path and format.")
        sys.exit(1)
 
    print(f"Actors: {sorted(actors.keys())}")
    for a, d in sorted(actors.items()):
        n = len(d["time"])
        t0, t1 = (d["time"][0], d["time"][-1]) if n else (0, 0)
        has_gap = sum(1 for g in d["gap"] if g is not None)
        print(f"  {a}: {n} samples, t={t0:.1f}–{t1:.1f}s, gap_samples={has_gap}")
 
    print(f"\nOutput format : {args.fmt.upper()}  |  Column width: {args.col} ({col_width*25.4:.0f} mm)  |  DPI: {args.dpi}")
 
    plot_speed(
        actors,
        desired_speed=args.desired_speed,
        t_start=args.t_start,
        t_end=args.t_end,
        title=title,
        output_prefix=args.output_prefix,
        fmt=args.fmt,
        dpi=args.dpi,
        col_width=col_width,
        show_ref=show_ref,
    )
 
    plot_gap(
        actors,
        desired_gap=args.desired_gap,
        headway=args.headway,
        desired_speed=args.desired_speed,
        t_start=args.t_start,
        t_end=args.t_end,
        title=title,
        output_prefix=args.output_prefix,
        fmt=args.fmt,
        dpi=args.dpi,
        col_width=col_width,
        show_ref=show_ref,
    )
 
    if args.gap_error:
        plot_gap_error(
            actors,
            t_start=args.t_start,
            t_end=args.t_end,
            title=title,
            output_prefix=args.output_prefix,
            fmt=args.fmt,
            dpi=args.dpi,
            col_width=col_width,
        )
 
    print("\nDone.")
 
 
if __name__ == "__main__":
    main()