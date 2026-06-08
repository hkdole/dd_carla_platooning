#!/usr/bin/env python3
"""
compare_protocols.py
=====================
Compares centralized vs decentralized platooning on:
  1. Bandwidth over time (KB/s)
  2. Message count over time
  3. Maneuver message breakdown
  4. Join timing (discovery → request → in-position → complete)
  5. Summary statistics table
 
Usage:
  python3 compare_protocols.py \\
      --centralized ../omnetpp_JoinAtBackFull.log \\
      --decentralized ../omnetpp_DeliberatelyDecentralizedJoinAtBack.log \\
      [--bin-size 1.0] [--output comparison.png] [--joiner veh4]
 
"""
 
import re
import sys
import csv
import argparse
import collections
from pathlib import Path
 
 
# ---------------------------------------------------------------------------
# Message size estimates (bytes) — same as analyze_network_overhead.py
# ---------------------------------------------------------------------------
BEACON_SIZE_BYTES = 4 + 8*11 + 4 + 40       # 136 bytes
MANEUVER_BASE_BYTES = 4 + 8 + 4 + 4 + 40    # 60 bytes
FORMATION_ENTRY_BYTES = 4
DEFAULT_FORMATION_SIZE = 2
 
MESSAGE_EXTRA_BYTES = {
    "JoinPlatoonRequest":     4 + 8 + 8,
    "JoinPlatoonResponse":    4 + 1,
    "MoveToPosition":         8 + 4,
    "MoveToPositionAck":      8 + 4,
    "JoinFormation":          8 + 4,
    "JoinFormationAck":       8 + 4,
    "UpdatePlatoonFormation": 8 + 4,
    "UpdatePlatoonData":      8 + 4 + 4,
}
 
FORMATION_MSG_TYPES = {
    "MoveToPosition", "MoveToPositionAck",
    "JoinFormation", "JoinFormationAck",
    "UpdatePlatoonFormation", "UpdatePlatoonData",
}
 
 
def estimate_size(msg_type: str, formation_size: int = DEFAULT_FORMATION_SIZE) -> int:
    base  = MANEUVER_BASE_BYTES
    extra = MESSAGE_EXTRA_BYTES.get(msg_type, 0)
    fsize = formation_size * FORMATION_ENTRY_BYTES if msg_type in FORMATION_MSG_TYPES else 0
    return base + extra + fsize
 
 
# ---------------------------------------------------------------------------
# Log parsing
# ---------------------------------------------------------------------------
 
RE_EVENT_TIME  = re.compile(r'\*\* Event #\d+ +t=([\d.eE+\-]+)')
RE_BEACON      = re.compile(r'\[CarlaGeneralPlatooningApp\]\[sendPlatooningBeacon\] actor=(\S+)')
RE_UNICAST     = re.compile(r'\[CarlaGeneralPlatooningApp\]\[sendUnicast\] actor=(\S+) logical_dst=(\d+) packet=(\S+)')
 
# Join timing patterns
RE_HEURISTIC   = re.compile(r'\[CarlaGeneralPlatooningApp\]\[heuristicEvaluation\] actor=(\S+) candidateId=')
RE_INIT_MANEUVER = re.compile(r'\[CarlaJoinAtBack\]\[initializeJoinManeuver\] actor=(\S+) .*state=J_WAIT_REPLY')
RE_IN_POSITION = re.compile(r'\[CarlaJoinAtBack\]\[onPlatoonBeacon\] actor=(\S+) .*action=send_MoveToPositionAck')
RE_JOIN_COMPLETE = re.compile(r'\[CarlaJoinAtBack\]\[handleJoinFormation\] actor=(\S+) action=join_complete')
RE_SPAWN       = re.compile(r'\[CarlanetManager\]\[handleLateSpawn\] simulation_time=([\d.]+) actor=(\S+)')
 
 
def parse_log(log_path: str, joiner: str):
    """Parse log into transmission events and join timing milestones."""
    events   = []
    timings  = {}
    current_time = 0.0
 
    with open(log_path, "r", errors="replace") as f:
        for line in f:
            m = RE_EVENT_TIME.search(line)
            if m:
                try:
                    current_time = float(m.group(1))
                except ValueError:
                    pass
                continue
 
            # Spawn
            m = RE_SPAWN.search(line)
            if m and m.group(2) == joiner:
                timings.setdefault("spawn", float(m.group(1)))
                continue
 
            # Heuristic evaluation (decentralized only)
            m = RE_HEURISTIC.search(line)
            if m and m.group(1) == joiner:
                timings.setdefault("heuristic", current_time)
                continue
 
            # Join request sent
            m = RE_INIT_MANEUVER.search(line)
            if m and m.group(1) == joiner:
                timings.setdefault("join_request", current_time)
                continue
 
            # In position ACK sent
            m = RE_IN_POSITION.search(line)
            if m and m.group(1) == joiner:
                timings.setdefault("in_position", current_time)
                continue
 
            # Join complete
            m = RE_JOIN_COMPLETE.search(line)
            if m and m.group(1) == joiner:
                timings.setdefault("join_complete", current_time)
                continue
 
            # Beacon
            m = RE_BEACON.search(line)
            if m:
                events.append({
                    "time":     current_time,
                    "actor":    m.group(1),
                    "msg_type": "PlatooningBeacon",
                    "bytes":    BEACON_SIZE_BYTES,
                })
                continue
 
            # Unicast
            m = RE_UNICAST.search(line)
            if m:
                msg_type = m.group(3)
                events.append({
                    "time":     current_time,
                    "actor":    m.group(1),
                    "msg_type": msg_type,
                    "bytes":    estimate_size(msg_type),
                })
 
    return events, timings
 
 
def bin_events(events, bin_size):
    bins = collections.defaultdict(lambda: collections.defaultdict(lambda: {"count": 0, "bytes": 0}))
    for ev in events:
        b = int(ev["time"] / bin_size) * bin_size
        bins[b][ev["msg_type"]]["count"] += 1
        bins[b][ev["msg_type"]]["bytes"] += ev["bytes"]
    return bins
 
 
# ---------------------------------------------------------------------------
# Print summary
# ---------------------------------------------------------------------------
 
def print_summary(label, events, timings, joiner):
    print(f"\n{'='*60}")
    print(f"  {label}")
    print(f"{'='*60}")
 
    total_counts = collections.Counter()
    total_bytes  = collections.Counter()
    for ev in events:
        total_counts[ev["msg_type"]] += 1
        total_bytes[ev["msg_type"]]  += ev["bytes"]
 
    grand_count = sum(total_counts.values())
    grand_bytes = sum(total_bytes.values())
    beacon_bytes   = total_bytes.get("PlatooningBeacon", 0)
    maneuver_bytes = grand_bytes - beacon_bytes
    maneuver_count = grand_count - total_counts.get("PlatooningBeacon", 0)
 
    print(f"  {'Message Type':<28} {'Count':>8} {'KB':>8}")
    print(f"  {'-'*46}")
    for mt in sorted(total_counts, key=lambda x: total_bytes[x], reverse=True):
        print(f"  {mt:<28} {total_counts[mt]:>8,} {total_bytes[mt]/1024:>8.2f}")
    print(f"  {'-'*46}")
    print(f"  {'TOTAL':<28} {grand_count:>8,} {grand_bytes/1024:>8.2f}")
    print(f"\n  Beacon traffic:   {beacon_bytes/1024:.2f} KB ({100*beacon_bytes/grand_bytes:.1f}%)")
    print(f"  Maneuver traffic: {maneuver_bytes/1024:.2f} KB ({100*maneuver_bytes/grand_bytes:.1f}%)")
    print(f"  Maneuver msgs:    {maneuver_count}")
 
    if events:
        t0 = min(e["time"] for e in events)
        t1 = max(e["time"] for e in events)
        dur = t1 - t0
        print(f"\n  Duration: {dur:.1f}s  |  Avg BW: {grand_bytes/dur/1024:.2f} KB/s")
 
    print(f"\n  --- Join Timings for {joiner} ---")
    spawn   = timings.get("spawn",        None)
    heur    = timings.get("heuristic",    None)
    req     = timings.get("join_request", None)
    inpos   = timings.get("in_position",  None)
    done    = timings.get("join_complete",None)
 
    def fmt(t): return f"{t:.2f}s" if t is not None else "N/A"
    def delta(a, b):
        if a is not None and b is not None: return f"(+{b-a:.2f}s)"
        return ""
 
    print(f"  Spawn:            {fmt(spawn)}")
    if heur:
        print(f"  Heuristic fired:  {fmt(heur)}  {delta(spawn, heur)}")
    print(f"  Join request:     {fmt(req)}   {delta(spawn, req)}")
    print(f"  In position:      {fmt(inpos)}  {delta(req, inpos)}")
    print(f"  Join complete:    {fmt(done)}  {delta(req, done)}")
    if req and done:
        print(f"  Total maneuver:   {done-req:.2f}s  (request → complete)")
    if spawn and done:
        print(f"  Total from spawn: {done-spawn:.2f}s")
 
 
# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
 
def make_comparison_plots(
        c_events, c_timings,
        d_events, d_timings,
        bin_size, joiner, output):
 
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
        import numpy as np
    except ImportError:
        print("matplotlib not available — skipping plots.")
        return
 
    c_bins = bin_events(c_events, bin_size)
    d_bins = bin_events(d_events, bin_size)
 
    all_times = sorted(set(list(c_bins.keys()) + list(d_bins.keys())))
 
    def series(bins, key, metric):
        return [bins[t][key][metric] for t in all_times]
 
    # Total KB/s
    def total_kbps(bins):
        return [(sum(bins[t][k]["bytes"] for k in bins[t]) / 1024 / bin_size) for t in all_times]
 
    def beacon_kbps(bins):
        return [(bins[t]["PlatooningBeacon"]["bytes"] / 1024 / bin_size) for t in all_times]
 
    def maneuver_kbps(bins):
        return [((sum(bins[t][k]["bytes"] for k in bins[t]) -
                  bins[t]["PlatooningBeacon"]["bytes"]) / 1024 / bin_size)
                for t in all_times]
 
    def total_count(bins):
        return [sum(bins[t][k]["count"] for k in bins[t]) for t in all_times]
 
    def maneuver_count(bins):
        return [sum(bins[t][k]["count"] for k in bins[t]
                    if k != "PlatooningBeacon")
                for t in all_times]
 
    # Maneuver types
    all_maneuver_types = sorted(set(
        k for bins in [c_bins, d_bins]
        for t in bins for k in bins[t]
        if k != "PlatooningBeacon"
    ))
 
    fig, axes = plt.subplots(2, 3, figsize=(18, 9))
    fig.suptitle("Centralized vs Decentralized Protocol Comparison", fontsize=14, fontweight="bold")
 
    C_COLOR = "#4C72B0"
    D_COLOR = "#DD8452"
 
    # ---- 1. Total bandwidth ----
    ax = axes[0, 0]
    ax.plot(all_times, total_kbps(c_bins), color=C_COLOR, linewidth=1.8, label="Centralized")
    ax.plot(all_times, total_kbps(d_bins), color=D_COLOR, linewidth=1.8, linestyle="--", label="Decentralized")
    ax.set_title("Total Bandwidth (KB/s)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("KB/s")
    ax.legend()
    ax.grid(True, alpha=0.3)
    _add_join_vlines(ax, c_timings, d_timings, C_COLOR, D_COLOR)
 
    # ---- 2. Maneuver bandwidth ----
    ax = axes[0, 1]
    ax.plot(all_times, maneuver_kbps(c_bins), color=C_COLOR, linewidth=1.8, label="Centralized")
    ax.plot(all_times, maneuver_kbps(d_bins), color=D_COLOR, linewidth=1.8, linestyle="--", label="Decentralized")
    ax.set_title("Maneuver Bandwidth (KB/s)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("KB/s")
    ax.legend()
    ax.grid(True, alpha=0.3)
    _add_join_vlines(ax, c_timings, d_timings, C_COLOR, D_COLOR)
 
    # ---- 3. Message count ----
    ax = axes[0, 2]
    ax.plot(all_times, total_count(c_bins), color=C_COLOR, linewidth=1.8, label="Centralized")
    ax.plot(all_times, total_count(d_bins), color=D_COLOR, linewidth=1.8, linestyle="--", label="Decentralized")
    ax.set_title(f"Total Message Count per {bin_size:.0f}s Bin")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Messages")
    ax.legend()
    ax.grid(True, alpha=0.3)
    _add_join_vlines(ax, c_timings, d_timings, C_COLOR, D_COLOR)
 
    # ---- 4. Maneuver message count ----
    ax = axes[1, 0]
    ax.plot(all_times, maneuver_count(c_bins), color=C_COLOR, linewidth=1.8, label="Centralized")
    ax.plot(all_times, maneuver_count(d_bins), color=D_COLOR, linewidth=1.8, linestyle="--", label="Decentralized")
    ax.set_title("Maneuver Messages per Bin")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Messages")
    ax.legend()
    ax.grid(True, alpha=0.3)
    _add_join_vlines(ax, c_timings, d_timings, C_COLOR, D_COLOR)
 
    # ---- 5. Maneuver type breakdown (grouped bar) ----
    ax = axes[1, 1]
    _plot_maneuver_breakdown(ax, c_bins, d_bins, all_maneuver_types, C_COLOR, D_COLOR)
 
    # ---- 6. Join timing comparison ----
    ax = axes[1, 2]
    _plot_join_timing(ax, c_timings, d_timings, joiner, C_COLOR, D_COLOR)
 
    plt.tight_layout()
 
    if output:
        plt.savefig(output, dpi=150, bbox_inches="tight")
        print(f"\nPlot saved to: {output}")
    else:
        plt.show()
 
 
def _add_join_vlines(ax, c_timings, d_timings, c_color, d_color):
    """Add vertical lines for join request and join complete events."""
    for timings, color, prefix in [
            (c_timings, c_color, "C"),
            (d_timings, d_color, "D")]:
        if "join_request" in timings:
            ax.axvline(timings["join_request"], color=color,
                       linestyle=":", alpha=0.6, linewidth=1.2)
        if "join_complete" in timings:
            ax.axvline(timings["join_complete"], color=color,
                       linestyle="-.", alpha=0.6, linewidth=1.2)
 
 
def _plot_maneuver_breakdown(ax, c_bins, d_bins, msg_types, c_color, d_color):
    """Side-by-side bar chart of total maneuver message counts by type."""
    import numpy as np
 
    c_totals = {mt: sum(c_bins[t][mt]["count"] for t in c_bins) for mt in msg_types}
    d_totals = {mt: sum(d_bins[t][mt]["count"] for t in d_bins) for mt in msg_types}
 
    # Only show types with at least 1 message in either scenario
    active = [mt for mt in msg_types if c_totals[mt] > 0 or d_totals[mt] > 0]
    if not active:
        ax.text(0.5, 0.5, "No maneuver messages", ha="center", va="center",
                transform=ax.transAxes)
        return
 
    x      = np.arange(len(active))
    width  = 0.35
    c_vals = [c_totals[mt] for mt in active]
    d_vals = [d_totals[mt] for mt in active]
 
    ax.bar(x - width/2, c_vals, width, label="Centralized",   color=c_color, alpha=0.85)
    ax.bar(x + width/2, d_vals, width, label="Decentralized", color=d_color, alpha=0.85)
 
    short = [mt.replace("Platoon", "").replace("Position", "Pos")
                .replace("Formation", "Form") for mt in active]
    ax.set_xticks(x)
    ax.set_xticklabels(short, rotation=35, ha="right", fontsize=8)
    ax.set_title("Total Maneuver Message Counts")
    ax.set_ylabel("Count")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, axis="y")
 
 
def _plot_join_timing(ax, c_timings, d_timings, joiner, c_color, d_color):
    """Horizontal Gantt-style join timeline comparison."""
    phases = [
        ("Spawn → Request",  "spawn",        "join_request"),
        ("Request → InPos",  "join_request", "in_position"),
        ("InPos → Complete", "in_position",  "join_complete"),
    ]
 
    y_positions = [2, 1]  # centralized on top, decentralized below
    labels      = ["Centralized", "Decentralized"]
    colors      = [c_color, d_color]
    timings_list = [c_timings, d_timings]
 
    phase_colors = ["#7fbfff", "#ffb347", "#90ee90"]
 
    for row, (timings, y, label, color) in enumerate(
            zip(timings_list, y_positions, labels, colors)):
 
        for (phase_label, start_key, end_key), pc in zip(phases, phase_colors):
            t0 = timings.get(start_key)
            t1 = timings.get(end_key)
            if t0 is None or t1 is None:
                continue
            ax.barh(y, t1 - t0, left=t0, height=0.4,
                    color=pc, edgecolor="gray", linewidth=0.5, alpha=0.85)
            mid = t0 + (t1 - t0) / 2
            if t1 - t0 > 1.0:
                ax.text(mid, y, f"{t1-t0:.1f}s", ha="center", va="center",
                        fontsize=7, fontweight="bold")
 
    # Legend for phases
    import matplotlib.patches as mpatches
    patches = [mpatches.Patch(color=c, label=p[0])
               for c, p in zip(phase_colors, phases)]
    ax.legend(handles=patches, loc="lower right", fontsize=7)
 
    ax.set_yticks(y_positions)
    ax.set_yticklabels(labels)
    ax.set_xlabel("Simulation Time (s)")
    ax.set_title(f"Join Timeline ({joiner})")
    ax.grid(True, alpha=0.3, axis="x")
    ax.set_ylim(0.5, 2.7)
 
 
# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
 
def main():
    parser = argparse.ArgumentParser(
        description="Compare centralized vs decentralized platooning protocols."
    )
    parser.add_argument("--centralized",   required=True, help="Centralized log file")
    parser.add_argument("--decentralized", required=True, help="Decentralized log file")
    parser.add_argument("--bin-size",  type=float, default=1.0, help="Time bin in seconds")
    parser.add_argument("--joiner",    default="veh4", help="Joiner actor ID (default: veh4)")
    parser.add_argument("--output",    default=None,   help="Save plot to file")
    parser.add_argument("--no-plot",   action="store_true", help="Skip plotting")
    args = parser.parse_args()
 
    print(f"Parsing centralized:   {args.centralized}")
    c_events, c_timings = parse_log(args.centralized,   args.joiner)
    print(f"  {len(c_events):,} events found")
 
    print(f"Parsing decentralized: {args.decentralized}")
    d_events, d_timings = parse_log(args.decentralized, args.joiner)
    print(f"  {len(d_events):,} events found")
 
    print_summary("CENTRALIZED",   c_events, c_timings, args.joiner)
    print_summary("DECENTRALIZED", d_events, d_timings, args.joiner)
 
    # Delta summary
    print(f"\n{'='*60}")
    print("  DELTA (Decentralized − Centralized)")
    print(f"{'='*60}")
 
    c_maneuver = sum(ev["bytes"] for ev in c_events if ev["msg_type"] != "PlatooningBeacon")
    d_maneuver = sum(ev["bytes"] for ev in d_events if ev["msg_type"] != "PlatooningBeacon")
    c_man_count = sum(1 for ev in c_events if ev["msg_type"] != "PlatooningBeacon")
    d_man_count = sum(1 for ev in d_events if ev["msg_type"] != "PlatooningBeacon")
 
    print(f"  Maneuver bytes:  {d_maneuver - c_maneuver:+.0f} bytes  "
          f"({'more' if d_maneuver > c_maneuver else 'less'} in decentralized)")
    print(f"  Maneuver msgs:   {d_man_count - c_man_count:+d} messages")
 
    for key, label in [
            ("join_request", "Time to join request"),
            ("join_complete", "Time to join complete"),
    ]:
        ct = c_timings.get(key)
        dt = d_timings.get(key)
        if ct and dt:
            print(f"  {label}: {dt - ct:+.2f}s  "
                  f"({'slower' if dt > ct else 'faster'} in decentralized)")
 
    if not args.no_plot:
        output = args.output or "protocol_comparison.png"
        make_comparison_plots(
            c_events, c_timings,
            d_events, d_timings,
            args.bin_size, args.joiner, output
        )
 
 
if __name__ == "__main__":
    main()