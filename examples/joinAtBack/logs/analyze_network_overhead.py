#!/usr/bin/env python3
"""
analyze_network_overhead.py
============================
Measures network overhead of the decentralized platooning protocol.
 
Tracks:
  - Message counts per type over time
  - Estimated bytes over time
  - Steady-state vs maneuver-phase breakdown
  - Per-vehicle breakdown
 
Usage:
  python3 analyze_network_overhead.py <omnetpp_log_file> [options]
 
  Options:
    --bin-size FLOAT      Time bin size in seconds (default: 1.0)
    --output-csv PATH     Write per-bin CSV to this path
    --plot                Show matplotlib plots (requires matplotlib)
    --scenario NAME       Label for plots/output (default: log filename)
 
Example:
  python3 analyze_network_overhead.py ../omnetpp_DeliberatelyDecentralizedJoinAtBack.log \\
      --bin-size 1.0 --output-csv overhead.csv --plot
"""
 
import re
import sys
import csv
import argparse
import collections
from pathlib import Path
 
 
# ---------------------------------------------------------------------------
# Message size estimates (bytes)
# Each field is estimated at its wire size. Formation arrays are variable.
# ---------------------------------------------------------------------------
 
# PlatooningBeacon fields (from sendPlatooningBeacon in CarlaGeneralPlatooningApp.cc):
#   vehicleId(4), acceleration(8), controllerAcceleration(8), speed(8),
#   positionX(8), positionY(8), time(8), sequenceNumber(4), length(8),
#   speedX(8), speedY(8), angle(8), desiredSpeed(8)
#   + 802.11p MAC/PHY header ~40 bytes
BEACON_SIZE_BYTES = 4 + 8*11 + 4 + 40  # = 136 bytes
 
# ManeuverMessage base fields:
#   vehicleId(4), externalId(~8), platoonId(4), destinationId(4) + header(40)
MANEUVER_BASE_BYTES = 4 + 8 + 4 + 4 + 40  # = 60 bytes
 
# Per-formation-entry overhead (int = 4 bytes)
FORMATION_ENTRY_BYTES = 4
 
# Message-specific additional fields:
MESSAGE_EXTRA_BYTES = {
    "JoinPlatoonRequest":    4 + 8 + 8,          # laneIndex(4), posX(8), posY(8)
    "JoinPlatoonResponse":   4 + 1,               # platoonId(4), permitted(1)
    "MoveToPosition":        8 + 4,               # platoonSpeed(8), platoonLane(4)  + formation (variable)
    "MoveToPositionAck":     8 + 4,               # platoonSpeed(8), platoonLane(4)  + formation (variable)
    "JoinFormation":         8 + 4,               # platoonSpeed(8), platoonLane(4)  + formation (variable)
    "JoinFormationAck":      8 + 4,               # platoonSpeed(8), platoonLane(4)  + formation (variable)
    "UpdatePlatoonFormation":8 + 4,               # platoonSpeed(8), platoonLane(4)  + formation (variable)
    "UpdatePlatoonData":     8 + 4 + 4,           # platoonSpeed(8), platoonLane(4), newPlatoonId(4) + formation
}
 
# Formation size assumptions (number of vehicles) for variable-size messages
DEFAULT_FORMATION_SIZE = 2  # conservative default for join-at-back
 
 
def estimate_message_size(msg_type: str, formation_size: int = DEFAULT_FORMATION_SIZE) -> int:
    """Estimate wire size in bytes for a given message type."""
    base = MANEUVER_BASE_BYTES
    extra = MESSAGE_EXTRA_BYTES.get(msg_type, 0)
    formation_bytes = 0
    if msg_type in ("MoveToPosition", "MoveToPositionAck",
                    "JoinFormation", "JoinFormationAck",
                    "UpdatePlatoonFormation", "UpdatePlatoonData"):
        formation_bytes = formation_size * FORMATION_ENTRY_BYTES
    return base + extra + formation_bytes
 
 
# ---------------------------------------------------------------------------
# Log parsing
# ---------------------------------------------------------------------------
 
# Matches beacon send lines:
#   [CarlaGeneralPlatooningApp][sendPlatooningBeacon] actor=veh0 ...
RE_BEACON = re.compile(
    r'\*\* Event.*?t=([\d.]+).*?\n.*?'
    r'\[CarlaGeneralPlatooningApp\]\[sendPlatooningBeacon\] actor=(\S+)',
    re.MULTILINE
)
 
# Matches unicast send lines:
#   [CarlaGeneralPlatooningApp][sendUnicast] actor=veh3 logical_dst=4 packet=MoveToPosition ...
RE_UNICAST = re.compile(
    r'\*\* Event.*?t=([\d.]+).*?\n.*?'
    r'\[CarlaGeneralPlatooningApp\]\[sendUnicast\] actor=(\S+) logical_dst=(\d+) packet=(\S+)',
    re.MULTILINE
)
 
# Simpler single-line versions (fallback if event header is on same line)
RE_BEACON_SIMPLE = re.compile(
    r'\[CarlaGeneralPlatooningApp\]\[sendPlatooningBeacon\] actor=(\S+)'
)
RE_UNICAST_SIMPLE = re.compile(
    r'\[CarlaGeneralPlatooningApp\]\[sendUnicast\] actor=(\S+) logical_dst=(\d+) packet=(\S+)'
)
RE_EVENT_TIME = re.compile(r'\*\* Event #\d+ +t=([\d.eE+\-]+)')
 
 
def parse_log(log_path: str):
    """
    Parse the OMNeT++ log and return a list of transmission events.
    Each event: {'time': float, 'actor': str, 'msg_type': str, 'bytes': int}
    """
    events = []
    current_time = 0.0
 
    with open(log_path, "r", errors="replace") as f:
        for line in f:
            # Update current simulation time from event headers
            m = RE_EVENT_TIME.search(line)
            if m:
                try:
                    current_time = float(m.group(1))
                except ValueError:
                    pass
                continue
 
            # Beacon
            m = RE_BEACON_SIMPLE.search(line)
            if m:
                events.append({
                    "time":     current_time,
                    "actor":    m.group(1),
                    "msg_type": "PlatooningBeacon",
                    "bytes":    BEACON_SIZE_BYTES,
                })
                continue
 
            # Unicast maneuver message
            m = RE_UNICAST_SIMPLE.search(line)
            if m:
                msg_type = m.group(3)
                events.append({
                    "time":     current_time,
                    "actor":    m.group(1),
                    "msg_type": msg_type,
                    "bytes":    estimate_message_size(msg_type),
                })
                continue
 
    return events
 
 
# ---------------------------------------------------------------------------
# Aggregation
# ---------------------------------------------------------------------------
 
def bin_events(events, bin_size: float):
    """
    Aggregate events into time bins.
    Returns a dict: bin_start -> { msg_type -> {'count': int, 'bytes': int} }
    """
    bins = collections.defaultdict(lambda: collections.defaultdict(lambda: {"count": 0, "bytes": 0}))
 
    for ev in events:
        bin_start = int(ev["time"] / bin_size) * bin_size
        bins[bin_start][ev["msg_type"]]["count"] += 1
        bins[bin_start][ev["msg_type"]]["bytes"] += ev["bytes"]
 
    return bins
 
 
def summarize(events, bin_size: float):
    """Print a summary of network overhead."""
 
    if not events:
        print("No events found. Check log path and format.")
        return {}, []
 
    all_msg_types = sorted(set(e["msg_type"] for e in events))
    bins = bin_events(events, bin_size)
    sorted_bins = sorted(bins.keys())
 
    # --- Overall summary ---
    total_counts = collections.Counter()
    total_bytes = collections.Counter()
    for ev in events:
        total_counts[ev["msg_type"]] += 1
        total_bytes[ev["msg_type"]] += ev["bytes"]
 
    print("\n" + "="*60)
    print("NETWORK OVERHEAD SUMMARY")
    print("="*60)
    print(f"{'Message Type':<28} {'Count':>8} {'Total KB':>10} {'Avg B/msg':>10}")
    print("-"*60)
 
    grand_count = 0
    grand_bytes = 0
    for mt in sorted(total_counts.keys(), key=lambda x: total_bytes[x], reverse=True):
        c = total_counts[mt]
        b = total_bytes[mt]
        grand_count += c
        grand_bytes += b
        print(f"  {mt:<26} {c:>8,} {b/1024:>10.2f} {b/c:>10.1f}")
 
    print("-"*60)
    print(f"  {'TOTAL':<26} {grand_count:>8,} {grand_bytes/1024:>10.2f}")
 
    # --- Beacon vs maneuver breakdown ---
    beacon_bytes = total_bytes.get("PlatooningBeacon", 0)
    maneuver_bytes = grand_bytes - beacon_bytes
    print(f"\n  Beacon traffic:   {beacon_bytes/1024:>8.2f} KB "
          f"({100*beacon_bytes/grand_bytes:.1f}%)")
    print(f"  Maneuver traffic: {maneuver_bytes/1024:>8.2f} KB "
          f"({100*maneuver_bytes/grand_bytes:.1f}%)")
 
    # --- Per-vehicle breakdown ---
    per_vehicle = collections.defaultdict(lambda: collections.Counter())
    per_vehicle_bytes = collections.defaultdict(lambda: collections.Counter())
    for ev in events:
        per_vehicle[ev["actor"]][ev["msg_type"]] += 1
        per_vehicle_bytes[ev["actor"]][ev["msg_type"]] += ev["bytes"]
 
    print(f"\n{'Vehicle':<10} {'Beacons':>8} {'Maneuver msgs':>14} {'Total KB':>10}")
    print("-"*46)
    for actor in sorted(per_vehicle.keys()):
        b_count = per_vehicle[actor].get("PlatooningBeacon", 0)
        m_count = sum(v for k, v in per_vehicle[actor].items() if k != "PlatooningBeacon")
        total_kb = sum(per_vehicle_bytes[actor].values()) / 1024
        print(f"  {actor:<8} {b_count:>8,} {m_count:>14,} {total_kb:>10.2f}")
 
    # --- Time range ---
    t_min = min(e["time"] for e in events)
    t_max = max(e["time"] for e in events)
    duration = t_max - t_min
    print(f"\n  Simulation time: {t_min:.1f}s – {t_max:.1f}s  ({duration:.1f}s)")
    print(f"  Avg bandwidth:   {grand_bytes/duration/1024:.2f} KB/s total")
    print(f"  Avg beacon rate: {total_counts['PlatooningBeacon']/duration:.1f} beacons/s")
 
    return bins, sorted_bins, all_msg_types
 
 
# ---------------------------------------------------------------------------
# CSV output
# ---------------------------------------------------------------------------
 
def write_csv(bins, sorted_bins, all_msg_types, output_path: str, bin_size: float):
    """Write per-bin metrics to CSV."""
    maneuver_types = [mt for mt in all_msg_types if mt != "PlatooningBeacon"]
 
    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
 
        # Header
        header = ["time_bin"]
        header += ["beacon_count", "beacon_bytes"]
        for mt in maneuver_types:
            header += [f"{mt}_count", f"{mt}_bytes"]
        header += ["total_count", "total_bytes", "total_kb_per_sec"]
        writer.writerow(header)
 
        for t in sorted_bins:
            row = [f"{t:.2f}"]
            b = bins[t]
 
            beacon_count = b["PlatooningBeacon"]["count"]
            beacon_bytes = b["PlatooningBeacon"]["bytes"]
            row += [beacon_count, beacon_bytes]
 
            total_count = beacon_count
            total_bytes = beacon_bytes
 
            for mt in maneuver_types:
                row += [b[mt]["count"], b[mt]["bytes"]]
                total_count += b[mt]["count"]
                total_bytes += b[mt]["bytes"]
 
            kb_per_sec = (total_bytes / 1024) / bin_size
            row += [total_count, total_bytes, f"{kb_per_sec:.4f}"]
            writer.writerow(row)
 
    print(f"\n  CSV written to: {output_path}")
 
 
# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
 
def plot_overhead(bins, sorted_bins, all_msg_types, scenario: str, bin_size: float):
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
        import numpy as np
    except ImportError:
        print("\n  matplotlib not available — skipping plots.")
        return
 
    times = sorted_bins
    maneuver_types = [mt for mt in all_msg_types if mt != "PlatooningBeacon"]
 
    beacon_kb   = [bins[t]["PlatooningBeacon"]["bytes"] / 1024 / bin_size for t in times]
    maneuver_kb = [
        sum(bins[t][mt]["bytes"] for mt in maneuver_types) / 1024 / bin_size
        for t in times
    ]
    beacon_count   = [bins[t]["PlatooningBeacon"]["count"] for t in times]
    maneuver_count = [sum(bins[t][mt]["count"] for mt in maneuver_types) for t in times]
 
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    fig.suptitle(f"Network Overhead — {scenario}", fontsize=13)
 
    # 1. KB/s stacked area
    ax = axes[0, 0]
    ax.stackplot(times, beacon_kb, maneuver_kb,
                 labels=["Beacon (KB/s)", "Maneuver (KB/s)"],
                 colors=["#4C72B0", "#DD8452"], alpha=0.85)
    ax.set_title("Bandwidth (KB/s)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("KB/s")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)
 
    # 2. Message count stacked
    ax = axes[0, 1]
    ax.stackplot(times, beacon_count, maneuver_count,
                 labels=["Beacons/bin", "Maneuver msgs/bin"],
                 colors=["#4C72B0", "#DD8452"], alpha=0.85)
    ax.set_title("Message Count per Bin")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(f"Messages per {bin_size:.1f}s bin")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)
 
    # 3. Maneuver message breakdown (bar)
    ax = axes[1, 0]
    colors = plt.cm.Set2.colors
    bottom = [0.0] * len(times)
    for i, mt in enumerate(maneuver_types):
        counts = [bins[t][mt]["count"] for t in times]
        if any(c > 0 for c in counts):
            ax.bar(times, counts, bottom=bottom, width=bin_size * 0.9,
                   label=mt, color=colors[i % len(colors)], alpha=0.85)
            bottom = [b + c for b, c in zip(bottom, counts)]
    ax.set_title("Maneuver Message Types per Bin")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Count")
    ax.legend(loc="upper right", fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3, axis="y")
 
    # 4. Cumulative bytes
    ax = axes[1, 1]
    cum_beacon   = []
    cum_maneuver = []
    running_b = 0.0
    running_m = 0.0
    for t in times:
        running_b += bins[t]["PlatooningBeacon"]["bytes"] / 1024
        running_m += sum(bins[t][mt]["bytes"] for mt in maneuver_types) / 1024
        cum_beacon.append(running_b)
        cum_maneuver.append(running_m)
    ax.plot(times, cum_beacon,   label="Cumulative Beacon KB",   color="#4C72B0")
    ax.plot(times, cum_maneuver, label="Cumulative Maneuver KB", color="#DD8452")
    ax.set_title("Cumulative Bytes Transferred")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("KB")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
 
    plt.tight_layout()
    out_png = Path(scenario).stem + "_overhead.png"
    plt.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"  Plot saved to: {out_png}")
    plt.show()
 
 
# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
 
def main():
    parser = argparse.ArgumentParser(description="Analyze platooning network overhead from OMNeT++ logs.")
    parser.add_argument("log", help="Path to omnetpp log file")
    parser.add_argument("--bin-size", type=float, default=1.0,
                        help="Time bin size in seconds (default: 1.0)")
    parser.add_argument("--output-csv", default=None,
                        help="Write per-bin CSV to this path")
    parser.add_argument("--plot", action="store_true",
                        help="Show matplotlib plots")
    parser.add_argument("--scenario", default=None,
                        help="Scenario label for output/plots")
    args = parser.parse_args()
 
    log_path = args.log
    scenario = args.scenario or Path(log_path).stem
 
    print(f"Parsing: {log_path}")
    events = parse_log(log_path)
    print(f"Found {len(events):,} transmission events")
 
    result = summarize(events, args.bin_size)
    if len(result) != 3:
        return
 
    bins, sorted_bins, all_msg_types = result
 
    if args.output_csv:
        write_csv(bins, sorted_bins, all_msg_types, args.output_csv, args.bin_size)
 
    if args.plot:
        plot_overhead(bins, sorted_bins, all_msg_types, scenario, args.bin_size)
 
 
if __name__ == "__main__":
    main()