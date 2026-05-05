#!/usr/bin/env python3
"""
Keep only:
1) compact control snapshots at EXACTLY N.00 for a configurable integer-second interval
2) explicit JoinAtBack spawn / maneuver lines

Snapshot lines kept only at selected exact points:
- [CarlaGeneralPlatooningApp][computeFollowerPlatoonControl]
- [CarlaGeneralPlatooningApp][computeJoinerMoveInPositionControl]
- [CarlaJoinAtBack][onPlatoonBeacon]
- [CarlaGeneralPlatooningApp][onControlTick]

Examples:
- SNAPSHOT_SECOND_INTERVAL = 1, SNAPSHOT_SECOND_PHASE = 0
    keeps 10.000000, 11.000000, 12.000000, 13.000000, ...

- SNAPSHOT_SECOND_INTERVAL = 2, SNAPSHOT_SECOND_PHASE = 0
    keeps 10.000000, 12.000000, 14.000000, ...

- SNAPSHOT_SECOND_INTERVAL = 2, SNAPSHOT_SECOND_PHASE = 1
    keeps 11.000000, 13.000000, 15.000000, ...

- SNAPSHOT_SECOND_INTERVAL = 5, SNAPSHOT_SECOND_PHASE = 0
    keeps 0.000000, 5.000000, 10.000000, 15.000000, ...

- SNAPSHOT_SECOND_INTERVAL = 5, SNAPSHOT_SECOND_PHASE = 2
    keeps 2.000000, 7.000000, 12.000000, 17.000000, ...

Examples always rejected:
- 10.050000
- 10.052367
- 10.100000
- 10.250000
- 24.050000

Always-kept join lines:
- [CarlanetManager][handleLateSpawn]
- [CarlanetManager][createAndInitializeActor] actor=veh4
- [CarlanetManager][registerMobilityModule] actor=veh4
- [CarlaGeneralPlatooningApp][initialize] actor=veh4
- [BridgeApp][initialize] actor=veh4
- stage-0 initialization lines for actors[4]
- ConnectionManager registerNic line for actors[4]
- actual JoinAtBack maneuver logs, including:
  [CarlaJoinAtBack][initializeJoinManeuver]
  [CarlaJoinAtBack][sendJoinRequest]
  [CarlaJoinAtBack][processJoinRequest]
  [CarlaJoinAtBack][handleJoinPlatoonRequest]
  [CarlaJoinAtBack][handleJoinPlatoonResponse]
  [CarlaJoinAtBack][handleMoveToPosition]
  [CarlaJoinAtBack][handleMoveToPositionAck]
  [CarlaJoinAtBack][handleJoinFormation]
  [CarlaJoinAtBack][handleJoinFormationAck]
  [CarlaJoinAtBack][onPlatoonBeacon]
  [CarlaJoinAtBack][abortManeuver]
  [CarlaJoinAtBack][handleSelfMsg]
- maneuver packet send lines such as:
  packet=JoinPlatoonRequest
  packet=JoinPlatoonResponse
  packet=MoveToPosition
  packet=MoveToPositionAck
  packet=JoinFormation
  packet=JoinFormationAck
  packet=UpdatePlatoonFormation

Usage:
    python3 extract_omnet_log.py omnetpp_<scenario>.log -o omnet.log
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from dataclasses import dataclass
from typing import List, Optional


# Snapshot capture cadence:
# Keep exact integer-second times N.000000 where:
#     (N % SNAPSHOT_SECOND_INTERVAL) == SNAPSHOT_SECOND_PHASE
#
# Examples:
# interval=1, phase=0 -> every second
# interval=2, phase=0 -> 0,2,4,6,...
# interval=2, phase=1 -> 1,3,5,7,...
# interval=5, phase=0 -> 0,5,10,15,...
# interval=5, phase=2 -> 2,7,12,17,...
SNAPSHOT_SECOND_INTERVAL = 2
SNAPSHOT_SECOND_PHASE = 0

# Keep only exact N.000000 points
SNAPSHOT_OFFSET_MS = 0
EPS_MS = 1  # tolerance for float noise in milliseconds

ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]")
FLOAT_RE = r"-?\d+(?:\.\d+)?"

EVENT_TIME_RE = re.compile(rf"^\*\*\s+Event\s+#\d+\s+t=({FLOAT_RE})\b")
TIME_PATTERNS = [
    re.compile(rf"\bsimulation_time=({FLOAT_RE})\b"),
    re.compile(rf"\blast=({FLOAT_RE})\b"),
    re.compile(rf"\btimestamp=({FLOAT_RE})\b"),
    re.compile(rf"\btime=({FLOAT_RE})\b"),
    re.compile(rf"\bt=({FLOAT_RE})\b"),
]


@dataclass
class CapturedLine:
    category: str          # "join" or "snapshot"
    time: Optional[float]
    snapshot_second: Optional[int]
    source_file: str
    line_no: int
    text: str


def strip_ansi(text: str) -> str:
    return ANSI_RE.sub("", text).rstrip("\n")


def extract_event_time(text: str) -> Optional[float]:
    m = EVENT_TIME_RE.search(text)
    if not m:
        return None
    try:
        return float(m.group(1))
    except ValueError:
        return None


def extract_time(text: str) -> Optional[float]:
    for pat in TIME_PATTERNS:
        m = pat.search(text)
        if m:
            try:
                return float(m.group(1))
            except ValueError:
                return None
    return None


def get_snapshot_second(time_s: float) -> Optional[int]:
    """
    Return the integer second N if time_s is effectively at N.000000 only.
    Otherwise return None.
    """
    t_ms = int(round(time_s * 1000.0))
    second = t_ms // 1000
    offset_ms = t_ms % 1000

    if abs(offset_ms - SNAPSHOT_OFFSET_MS) <= EPS_MS:
        return second

    return None


def should_keep_snapshot_time(
    time_s: float,
    interval: int = SNAPSHOT_SECOND_INTERVAL,
    phase: int = SNAPSHOT_SECOND_PHASE,
) -> bool:
    """
    Keep only exact N.000000 points, and only when:
        N % interval == phase

    Examples:
      interval=1, phase=0 -> every second
      interval=2, phase=0 -> even seconds
      interval=2, phase=1 -> odd seconds
      interval=3, phase=0 -> 0,3,6,9,...
      interval=5, phase=2 -> 2,7,12,17,...
    """
    if interval <= 0:
        raise ValueError("SNAPSHOT_SECOND_INTERVAL must be >= 1")
    if phase < 0 or phase >= interval:
        raise ValueError("SNAPSHOT_SECOND_PHASE must satisfy 0 <= phase < interval")

    second = get_snapshot_second(time_s)
    return second is not None and (second % interval) == phase


def snapshot_schedule_description(interval: int, phase: int) -> str:
    if interval == 1:
        return "every integer second (0.00, 1.00, 2.00, 3.00, ...)"
    if interval == 2 and phase == 0:
        return "every 2 seconds starting at 0 (0.00, 2.00, 4.00, 6.00, ...)"
    if interval == 2 and phase == 1:
        return "every 2 seconds starting at 1 (1.00, 3.00, 5.00, 7.00, ...)"
    return f"every {interval} seconds with phase {phase} ({phase}.00, {phase + interval}.00, {phase + 2*interval}.00, ...)"


def snapshot_section_title(interval: int) -> str:
    if interval == 1:
        return "=== EVERY-SECOND CONTROL SNAPSHOTS ==="
    if interval == 2:
        return "=== EVERY-OTHER-SECOND CONTROL SNAPSHOTS ==="
    return f"=== EVERY-{interval}-SECONDS CONTROL SNAPSHOTS ==="


def is_snapshot_line(text: str) -> bool:
    u = text.upper()
    return (
        "[CARLAGENERALPLATOONINGAPP][COMPUTEFOLLOWERPLATOONCONTROL]" in u
        or "[CARLAGENERALPLATOONINGAPP][COMPUTEJOINERMOVEINPOSITIONCONTROL]" in u
        or "[CARLAJOINATBACK][ONPLATOONBEACON]" in u
        or "[CARLAGENERALPLATOONINGAPP][ONCONTROLTICK]" in u
    )


def is_spawn_or_init_line(text: str) -> bool:
    u = text.upper()

    if "[CARLANETMANAGER][HANDLELATESPAWN]" in u:
        return True
    if "[CARLANETMANAGER][CREATEANDINITIALIZEACTOR]" in u and "ACTOR=VEH4" in u:
        return True
    if "[CARLANETMANAGER][REGISTERMOBILITYMODULE]" in u and "ACTOR=VEH4" in u:
        return True
    if "[CARLAGENERALPLATOONINGAPP][INITIALIZE]" in u:
        return True
    if "[BRIDGEAPP][INITIALIZE]" in u and "ACTOR=VEH4" in u:
        return True

    if "INITIALIZING MODULE CARLAPLATOONINGNETWORK.ACTORS[4]" in u and "STAGE 0" in u:
        return True

    if "CONNECTIONMANAGER::REGISTERNIC" in u and "ACTORS[4]" in u:
        return True

    return False


def is_explicit_maneuver_line(text: str) -> bool:
    u = text.upper()

    # High-value JoinAtBack state-transition lines only.
    important_joinatback_tags = [
        "[CARLAJOINATBACK][INITIALIZEJOINMANEUVER]",
        "[CARLAJOINATBACK][SENDJOINREQUEST]",
        "[CARLAJOINATBACK][PROCESSJOINREQUEST]",
        "[CARLAJOINATBACK][HANDLEJOINPLATOONREQUEST]",
        "[CARLAJOINATBACK][HANDLEJOINPLATOONRESPONSE]",
        "[CARLAJOINATBACK][HANDLEMOVETOPOSITION]",
        "[CARLAJOINATBACK][HANDLEMOVETOPOSITIONACK]",
        "[CARLAJOINATBACK][HANDLEJOINFORMATION]",
        "[CARLAJOINATBACK][HANDLEJOINFORMATIONACK]",
        "[CARLAJOINATBACK][ABORTMANEUVER]",
        "[CARLAJOINATBACK][HANDLESELFMSG]",
    ]
    if any(tag in u for tag in important_joinatback_tags):
        return True

    # onPlatoonBeacon is high-rate telemetry. Keep only meaningful convergence points
    # plus explicit maneuver-trigger actions emitted from beacon processing.
    if "[CARLAJOINATBACK][ONPLATOONBEACON]" in u:
        if "READY=1" in u:
            return True

        samples_match = re.search(r"\bsamples=(\d+)\b", u)
        if samples_match and int(samples_match.group(1)) > 0:
            return True

        beacon_action_markers = [
            "ACTION="
        ]
        if any(marker in u for marker in beacon_action_markers):
            return True

        return False

    # Explicit maneuver packet names appearing in sendUnicast/send logs.
    maneuver_packets = [
        "JOINPLATOONREQUEST",
        "JOINPLATOONRESPONSE",
        "MOVETOPOSITION",
        "MOVETOPOSITIONACK",
        "JOINFORMATION",
        "JOINFORMATIONACK",
        "UPDATEPLATOONFORMATION",
    ]
    if any(f"PACKET={pkt}" in u for pkt in maneuver_packets):
        return True

    # Explicit veh4 state/action markers.
    veh4_maneuver_markers = [
        "STATE=J_WAIT_REPLY",
        "STATE=J_WAIT_INFORMATION",
        "STATE=J_MOVE_IN_POSITION",
        "STATE=J_WAIT_JOIN",
        "ACTION=SEND_MOVETOPOSITION",
        "ACTION=SEND_MOVETOPOSITIONACK",
        "ACTION=SEND_JOINFORMATION",
        "ACTION=JOIN_COMPLETE",
        "ACTION=BROADCAST_UPDATEPLATOONFORMATION",
        "WARNING=JOIN_REQ_TIMEOUT",
    ]
    if "ACTOR=VEH4" in u and any(marker in u for marker in veh4_maneuver_markers):
        return True

    # Fallback generic keywords.
    generic_keywords = [
        "JOIN_REQ",
        "JOIN_RSP",
        "JOIN_ACK",
        "CATCH_UP",
        "MOVE_TO_POSITION",
        "MOVE_TO_POS",
        "OPEN_GAP",
        "IN_POSITION",
        "MANEUVER_COMPLETE",
        "JOIN_COMPLETE",
        "MERGE_COMPLETE",
    ]
    if any(k in u for k in generic_keywords):
        return True

    return False

def is_startup_param_line(text: str) -> bool:
    u = text.upper()

    # Preferred: explicit stable tag
    if "[CARLAGENERALPLATOONINGAPP][STARTUPPARAMS]" in u:
        return True

    # Fallback: match the actual parameter keys if you log them on one line
    if (
        "[CARLAGENERALPLATOONINGAPP]" in u
        and "HEADWAY=" in u
        and "GAP_MIN=" in u
        and "APPROACH_DELTA_V=" in u
        and "IN_POS_SLACK=" in u
    ):
        return True

    return False


def is_always_keep_join_line(text: str) -> bool:
    return (
        is_spawn_or_init_line(text)
        or is_explicit_maneuver_line(text)
        or is_startup_param_line(text)
    )


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Extract exact-point OMNeT control snapshots at a configurable interval plus explicit JoinAtBack spawn/maneuver lines."
    )
    ap.add_argument("input_files", nargs="+", help="Input OMNeT log files")
    ap.add_argument("-o", "--output", required=True, help="Output file")
    return ap.parse_args()


def main() -> int:
    args = parse_args()

    if SNAPSHOT_SECOND_INTERVAL <= 0:
        print("ERROR: SNAPSHOT_SECOND_INTERVAL must be >= 1", file=sys.stderr)
        return 2

    if SNAPSHOT_SECOND_PHASE < 0 or SNAPSHOT_SECOND_PHASE >= SNAPSHOT_SECOND_INTERVAL:
        print(
            f"ERROR: SNAPSHOT_SECOND_PHASE must satisfy 0 <= phase < interval "
            f"(got phase={SNAPSHOT_SECOND_PHASE}, interval={SNAPSHOT_SECOND_INTERVAL})",
            file=sys.stderr,
        )
        return 2

    total_lines = 0
    kept_join = 0
    kept_snapshot = 0
    captured: List[CapturedLine] = []

    for file_path in args.input_files:
        base = os.path.basename(file_path)
        current_event_time: Optional[float] = None

        try:
            with open(file_path, "r", encoding="utf-8", errors="replace") as f:
                for line_no, raw in enumerate(f, start=1):
                    total_lines += 1
                    text = strip_ansi(raw)
                    if not text.strip():
                        continue

                    evt_time = extract_event_time(text)
                    if evt_time is not None:
                        current_event_time = evt_time
                        continue

                    time_s = extract_time(text)
                    if time_s is None:
                        time_s = current_event_time

                    # Treat untimestamped startup/init lines as t=0.0 so they
                    # do not show up as t=? and can be grouped at startup.
                    if time_s is None and (
                        is_spawn_or_init_line(text) or is_startup_param_line(text)
                    ):
                        time_s = 0.0

                    # Keep explicit join/maneuver lines exactly as before.
                    if is_always_keep_join_line(text):
                        captured.append(
                            CapturedLine(
                                category="join",
                                time=time_s,
                                snapshot_second=(0 if time_s == 0.0 else None),
                                source_file=base,
                                line_no=line_no,
                                text=text,
                            )
                        )
                        kept_join += 1
                        continue

                    if not is_snapshot_line(text):
                        continue
                    if time_s is None:
                        continue
                    if not should_keep_snapshot_time(
                        time_s,
                        interval=SNAPSHOT_SECOND_INTERVAL,
                        phase=SNAPSHOT_SECOND_PHASE,
                    ):
                        continue

                    second = get_snapshot_second(time_s)
                    if second is None:
                        continue

                    captured.append(
                        CapturedLine(
                            category="snapshot",
                            time=time_s,
                            snapshot_second=second,
                            source_file=base,
                            line_no=line_no,
                            text=text,
                        )
                    )
                    kept_snapshot += 1

        except FileNotFoundError:
            print(f"ERROR: file not found: {file_path}", file=sys.stderr)
            return 2
        except OSError as exc:
            print(f"ERROR: could not read {file_path}: {exc}", file=sys.stderr)
            return 2

    def sort_key(item: CapturedLine):
        cat_rank = 0 if item.category == "join" else 1
        join_time = item.time if item.time is not None else -1.0
        snap_second = item.snapshot_second if item.snapshot_second is not None else -1
        snap_time = item.time if item.time is not None else -1.0
        return (cat_rank, join_time, snap_second, snap_time, item.source_file, item.line_no)

    captured.sort(key=sort_key)

    try:
        with open(args.output, "w", encoding="utf-8") as out:
            out.write("=== OMNET JOIN-AT-BACK EXTRACT ===\n")
            out.write(f"input_files: {', '.join(args.input_files)}\n")
            out.write(f"output_file: {args.output}\n")
            out.write("snapshot_points_seconds: N.00 only\n")
            out.write(f"snapshot_second_interval: {SNAPSHOT_SECOND_INTERVAL}\n")
            out.write(f"snapshot_second_phase: {SNAPSHOT_SECOND_PHASE}\n")
            out.write(
                f"snapshot_seconds_kept: "
                f"{snapshot_schedule_description(SNAPSHOT_SECOND_INTERVAL, SNAPSHOT_SECOND_PHASE)}\n"
            )
            out.write("snapshot_line_types:\n")
            out.write("  - [CarlaGeneralPlatooningApp][computeFollowerPlatoonControl]\n")
            out.write("  - [CarlaGeneralPlatooningApp][computeJoinerMoveInPositionControl]\n")
            out.write("  - [CarlaJoinAtBack][onPlatoonBeacon]\n")
            out.write("  - [CarlaGeneralPlatooningApp][onControlTick]\n")
            out.write("always_kept_join_lines:\n")
            out.write("  - late spawn / veh4 creation / veh4 init / explicit JoinAtBack maneuver logs\n")
            out.write("  - startup parameter lines with headway, gap_min, approach_delta_v, in_pos_slack\n")
            out.write("  - maneuver packet send lines (JoinPlatoonRequest/Response, MoveToPosition, MoveToPositionAck, JoinFormation, JoinFormationAck, UpdatePlatoonFormation)\n")
            out.write(f"lines_scanned: {total_lines}\n")
            out.write(f"join_lines_kept: {kept_join}\n")
            out.write(f"snapshot_lines_kept: {kept_snapshot}\n")
            out.write(f"lines_kept_total: {len(captured)}\n\n")
            
            join_lines = [x for x in captured if x.category == "join"]
            snap_lines = [x for x in captured if x.category == "snapshot"]

            startup_lines = [
                x for x in join_lines
                if x.time is not None and abs(x.time - 0.0) < 1e-9
            ]
            other_join_lines = [
                x for x in join_lines
                if not (x.time is not None and abs(x.time - 0.0) < 1e-9)
            ]

            out.write("=== EXPLICIT JOIN / MANEUVER LINES ===\n")
            if not other_join_lines:
                out.write("No explicit join/maneuver lines found.\n")
            else:
                for item in other_join_lines:
                    t = f"{item.time:.6f}" if item.time is not None else "?"
                    out.write(f"t={t} [{item.source_file}:{item.line_no}] {item.text}\n")

            out.write("\n=== STARTUP / TIME 0.00 ===\n")
            if not startup_lines:
                out.write("No startup lines found.\n")
            else:
                for item in startup_lines:
                    out.write(f"[{item.source_file}:{item.line_no}] {item.text}\n")

            out.write(f"\n{snapshot_section_title(SNAPSHOT_SECOND_INTERVAL)}\n")
            if not snap_lines:
                out.write("No snapshot lines found.\n")
            else:
                current_second: Optional[int] = None
                for item in snap_lines:
                    if item.snapshot_second != current_second:
                        if current_second is not None:
                            out.write("\n")
                        sec = item.snapshot_second if item.snapshot_second is not None else 0
                        out.write(f"simulation_time={sec:.2f}\n")
                        current_second = item.snapshot_second

                    out.write(f"[{item.source_file}:{item.line_no}] {item.text}\n")

        print(f"Wrote {len(captured)} lines to {args.output}")
        return 0

    except OSError as exc:
        print(f"ERROR: could not write {args.output}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())