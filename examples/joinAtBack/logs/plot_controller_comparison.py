"""
plot_controller_comparison.py
==============================
Generates a PLEXE-style controller comparison plot.
One row per controller, showing speed and gap over time.
Not-yet-implemented controllers show a placeholder row.
 
Usage:
  python3 plot_controller_comparison.py [options]
 
Options:
  --configs         List of implemented config names (e.g. CC ACC_03 ACC_12 CACC)
  --not-implemented List of unimplemented config names (e.g. PLOEG FLATBED CONSENSUS)
  --csv-dir         Directory containing positions_<CONFIG>.csv files
  --output          Output PNG path (default: controller_comparison.png)
  --desired-speed   Reference speed line (default: 10.0)
  --desired-gap     Reference gap_min (default: 5.0)
  --headway         Headway for CTS gap line (default: 0.5)
  --t-start         Start time (default: auto)
  --t-end           End time (default: auto)
  --metric          'speed' or 'gap' or 'both' (default: both)
  --exclude         Comma-separated actors to exclude (default: veh0)
 
Example:
  python3 plot_controller_comparison.py \\
      --configs CC ACC_03 ACC_12 CACC \\
      --not-implemented PLOEG FLATBED CONSENSUS \\
      --csv-dir logs/ \\
      --output logs/controller_comparison.png
"""
 
import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path
 
# ---------------------------------------------------------------------------
# Config display names (matches PLEXE labels)
# ---------------------------------------------------------------------------
DISPLAY_NAMES = {
    "CC":        "CC",
    "ACC_03":    "ACC (0.3 s)",
    "ACC_12":    "ACC (1.2 s)",
    "CACC":      "CACC",
    "PLOEG":     "PLOEG",
    "FLATBED":   "FLATBED",
    "CONSENSUS": "CONSENSUS",
}
 
# Headway values per config for CTS desired gap line
HEADWAY_MAP = {
    "CC":        1.2,
    "ACC_03":    0.3,
    "ACC_12":    1.2,
    "CACC":      0.5,
    "PLOEG":     0.5,
    "FLATBED":   0.5,
    "CONSENSUS": 0.5,
}
 
COLORS = [
    "#E41A1C",  # red      — veh0
    "#377EB8",  # blue     — veh1
    "#4DAF4A",  # green    — veh2
    "#984EA3",  # purple   — veh3
    "#FF7F00",  # orange   — veh4
    "#A65628",  # brown    — veh5
    "#F781BF",  # pink     — veh6
    "#999999",  # grey     — veh7
]
LINESTYLES = ["-", "--", "-.", ":", "-", "--", "-.", ":"]
 
# ---------------------------------------------------------------------------
# CSV loading
# ---------------------------------------------------------------------------

def discover_configs(csv_dir: str, all_configs: list):
    import glob
    found = set()
    for f in glob.glob(str(Path(csv_dir) / "real_positions_*.csv")):
        stem = Path(f).stem
        for config in all_configs:
            if stem.endswith(f"_{config}"):
                found.add(config)
                break
    implemented     = [c for c in all_configs if c in found]
    not_implemented = [c for c in all_configs if c not in found]
    return implemented, not_implemented


def find_csv(csv_dir: str, config: str, all_configs: list):
    import glob
    sorted_configs = sorted(all_configs, key=len, reverse=True)
    for f in glob.glob(str(Path(csv_dir) / "real_positions_*.csv")):
        stem = Path(f).stem
        for c in sorted_configs:
            if stem.endswith(f"_{c}"):
                if c == config:
                    return f
                break
    return None

def load_csv(path: str, exclude: list):
    actors = defaultdict(lambda: {"time": [], "speed": [], "gap": []})
    try:
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                actor = row.get("actor", "")
                if actor in exclude:
                    continue
                try:
                    t     = float(row["time"])
                    speed = float(row["speed"])
                    gap   = row.get("bumper_gap_to_predecessor", "")
                    actors[actor]["time"].append(t)
                    actors[actor]["speed"].append(speed)
                    actors[actor]["gap"].append(float(gap) if gap else None)
                except (ValueError, KeyError):
                    continue
    except FileNotFoundError:
        return None
    return dict(actors)

 
# ---------------------------------------------------------------------------
# Main plot
# ---------------------------------------------------------------------------
 
def make_comparison_plot(
        configs, not_implemented, csv_dir,
        output, desired_speed, desired_gap,
        t_start, t_end, metric, exclude):
 
    try:
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
    except ImportError:
        print("matplotlib required: pip3 install matplotlib --break-system-packages")
        sys.exit(1)
 
    all_configs = configs + not_implemented
    n_rows = len(all_configs)
 
    show_speed = metric in ("speed", "both")
    show_gap   = metric in ("gap", "both")
    n_cols = (1 if show_speed else 0) + (1 if show_gap else 0)
 
    fig_height = 2.8 * n_rows
    fig, axes = plt.subplots(n_rows, n_cols,
                              figsize=(7 * n_cols, fig_height),
                              squeeze=False)
 
    fig.suptitle("Controller Comparison — Decentralized Platoon (Sinusoidal Leader)",
                 fontsize=13, fontweight="bold", y=1.01)
 
    plt.rcParams.update({
        "axes.facecolor":    "white",
        "figure.facecolor":  "white",
        "axes.grid":         True,
        "grid.alpha":        0.35,
        "grid.linestyle":    "--",
        "font.size":         9,
        "axes.labelsize":    9,
        "axes.spines.top":   False,
        "axes.spines.right": False,
    })
 
    for row_idx, config in enumerate(all_configs):
        label     = DISPLAY_NAMES.get(config, config)
        headway   = HEADWAY_MAP.get(config, 0.5)
        cts_gap   = desired_gap + headway * desired_speed
        col_idx   = 0
 
        is_implemented = config in configs
        csv_path = find_csv(csv_dir, config, all_configs)
        actors = load_csv(csv_path, exclude) if (is_implemented and csv_path) else None
 
        # ---- Speed column ----
        if show_speed:
            ax = axes[row_idx][col_idx]
 
            # Row label on the right (matches PLEXE style)
            ax.set_ylabel(label, rotation=0, labelpad=60,
                          ha="right", va="center", fontsize=9, fontweight="bold")
 
            if row_idx == 0:
                ax.set_title("Speed (m/s)", fontsize=10)
 
            if not is_implemented or actors is None:
                _placeholder(ax, "Not implemented yet")
            else:
                _plot_speed(ax, actors, desired_speed, t_start, t_end)
 
            col_idx += 1
 
        # ---- Gap column ----
        if show_gap:
            ax = axes[row_idx][col_idx]
 
            if not show_speed:
                ax.set_ylabel(label, rotation=0, labelpad=60,
                              ha="right", va="center", fontsize=9, fontweight="bold")
 
            if row_idx == 0:
                ax.set_title("Bumper-to-Bumper Gap (m)", fontsize=10)
 
            if not is_implemented or actors is None:
                _placeholder(ax, "Not implemented yet")
            else:
                _plot_gap(ax, actors, cts_gap, t_start, t_end)
 
            col_idx += 1
 
        # Only show x label on bottom row
        for c in range(n_cols):
            if row_idx == n_rows - 1:
                axes[row_idx][c].set_xlabel("Time (s)")
            else:
                axes[row_idx][c].set_xticklabels([])
 
    # Shared legend
    handles = []
    try:
        import matplotlib.lines as mlines
        # Use last successfully loaded actors for legend
        for config in configs:
            csv_path = find_csv(csv_dir, config, all_configs)
            actors = load_csv(str(csv_path), exclude)
            if actors:
                for i, actor in enumerate(sorted(actors.keys())):
                    h = mlines.Line2D([], [],
                                      color=COLORS[i % len(COLORS)],
                                      linestyle=LINESTYLES[i % len(LINESTYLES)],
                                      label=actor.replace("veh", "Vehicle "))
                    handles.append(h)
                break
        ref_speed = mlines.Line2D([], [], color="black", linestyle="--",
                                  linewidth=0.8, label=f"Reference")
        handles.append(ref_speed)
    except Exception:
        pass
 
    if handles:
        fig.legend(handles=handles, loc="upper right",
                   bbox_to_anchor=(1.0, 1.0), fontsize=8, framealpha=0.9)
 
    plt.tight_layout()
 
    plt.savefig(output, dpi=150, bbox_inches="tight")
    print(f"Plot saved to: {output}")
    plt.show()
 
 
def _plot_speed(ax, actors, desired_speed, t_start, t_end):
    for i, actor in enumerate(sorted(actors.keys())):
        d = actors[actor]
        tw, sw = _window(d["time"], d["speed"], t_start, t_end)
        if not tw:
            continue
        ax.plot(tw, sw,
                color=COLORS[i % len(COLORS)],
                linestyle=LINESTYLES[i % len(LINESTYLES)],
                linewidth=1.2)
    ax.axhline(desired_speed, color="black", linestyle="--",
               linewidth=0.8, alpha=0.7)
    if t_start: ax.set_xlim(left=t_start)
    if t_end:   ax.set_xlim(right=t_end)
 
 
def _plot_gap(ax, actors, cts_gap, t_start, t_end):
    for i, actor in enumerate(sorted(actors.keys())):
        d = actors[actor]
        gaps = d["gap"]
        pairs = [(t, g) for t, g in zip(d["time"], gaps)
                 if g is not None]
        if not pairs:
            continue
        if t_start or t_end:
            ts = t_start or pairs[0][0]
            te = t_end   or pairs[-1][0]
            pairs = [(t, g) for t, g in pairs if ts <= t <= te]
        if not pairs:
            continue
        tw, gw = zip(*pairs)
        ax.plot(tw, gw,
                color=COLORS[i % len(COLORS)],
                linestyle=LINESTYLES[i % len(LINESTYLES)],
                linewidth=1.2)
    ax.axhline(cts_gap, color="black", linestyle="--",
               linewidth=0.8, alpha=0.7)
    if t_start: ax.set_xlim(left=t_start)
    if t_end:   ax.set_xlim(right=t_end)
 
 
def _placeholder(ax, text):
    ax.text(0.5, 0.5, text,
            ha="center", va="center",
            transform=ax.transAxes,
            fontsize=10, color="gray",
            style="italic")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_edgecolor("#cccccc")
 
 
def _window(times, values, t_start, t_end):
    if not t_start and not t_end:
        return times, values
    ts = t_start or times[0]
    te = t_end   or times[-1]
    pairs = [(t, v) for t, v in zip(times, values) if ts <= t <= te]
    if not pairs:
        return [], []
    return zip(*pairs)
 
 
# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
 
def main():
    parser = argparse.ArgumentParser(
        description="PLEXE-style controller comparison plot."
    )
    parser.add_argument("--configs", nargs="+",
                        default=["CC", "ACC_03", "ACC_12", "CACC"],
                        help="Implemented config names")
    parser.add_argument("--csv-dir",       default="logs/",
                        help="Directory with positions_<CONFIG>.csv files")
    parser.add_argument("--output",        default="logs/controller_comparison.png")
    parser.add_argument("--desired-speed", type=float, default=10.0)
    parser.add_argument("--desired-gap",   type=float, default=5.0)
    parser.add_argument("--headway",       type=float, default=0.5)
    parser.add_argument("--t-start",       type=float, default=None)
    parser.add_argument("--t-end",         type=float, default=None)
    parser.add_argument("--metric",        choices=["speed", "gap", "both"],
                        default="both")
    parser.add_argument("--exclude",       type=str, default="veh0",
                        help="Comma-separated actors to exclude (default: veh0)")
    args = parser.parse_args()

    implemented, not_implemented = discover_configs(args.csv_dir, args.configs)
    print(f"Found: {implemented}")
    print(f"Not implemented: {not_implemented}")

    exclude = [a.strip() for a in args.exclude.split(",") if a.strip()]

    make_comparison_plot(
        configs=implemented,
        not_implemented=not_implemented,
        csv_dir=args.csv_dir,
        output=args.output,
        desired_speed=args.desired_speed,
        desired_gap=args.desired_gap,
        t_start=args.t_start,
        t_end=args.t_end,
        metric=args.metric,
        exclude=exclude,
    )
 
 
if __name__ == "__main__":
    main()
