#!/usr/bin/env python3
"""
Extract JoinAtBack maneuver semantics from OMNeT++ logs.

What this keeps in RAW mode:
1) strict veh4 late-spawn / initialization lines
   - exact stage-0 init subtree only
   - no stage 1..N drift
2) CarlaJoinAtBack maneuver lifecycle lines
3) CarlaGeneralPlatooningApp maneuver send/receive/apply lines
4) CarlaPlexeApp maneuver/state/send/receive lines
5) optional free-form [JAB] maneuver lines
6) important onPlatoonBeacon convergence lines only by default
   - ready=1
   - action=send_MoveToPositionAck
   - samples=1/2/3/4
   - join timeout warnings

What this also adds:
- grouped step-by-step PASS / FAIL checks for the full join maneuver
- keeps the raw extracted lines unchanged and separate

Usage:
    python3 extract_join_semantics.py omnetpp.log -o join_semantics.log

Include every CarlaJoinAtBack beacon line:
    python3 extract_join_semantics.py omnetpp.log -o join_semantics.log --all-beacons
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple


ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]")


GENERAL_APP_MANEUVER_PACKETS = {
    "JOINPLATOONREQUEST",
    "JOINPLATOONRESPONSE",
    "MOVETOPOSITION",
    "MOVETOPOSITIONACK",
    "JOINFORMATION",
    "JOINFORMATIONACK",
    "UPDATEPLATOONFORMATION",
    "UPDATEPLATOONDATA",
}

PLEXE_MANEUVER_TYPES = {
    "JOIN_REQ",
    "JOIN_RSP",
    "MOVE_TO_POS",
    "MOVE_TO_POS_ACK",
    "JOIN_FORMATION",
    "JOIN_FORMATION_ACK",
    "UPDATE_FORMATION",
}

CARLA_JOIN_METHODS = {
    "INITIALIZEJOINMANEUVER",
    "SENDJOINREQUEST",
    "PROCESSJOINREQUEST",
    "HANDLEJOINPLATOONREQUEST",
    "HANDLEJOINPLATOONRESPONSE",
    "HANDLEMOVETOPOSITION",
    "HANDLEMOVETOPOSITIONACK",
    "HANDLEJOINFORMATION",
    "HANDLEJOINFORMATIONACK",
    "HANDLESELFMSG",
    "ABORTMANEUVER",
    "ONPLATOONBEACON",
}

GENERAL_APP_METHODS = {
    "INITIALIZE",
    "STARTJOINMANEUVERIFCONFIGURED",
    "SENDUNICAST",
    "HANDLELOWERMSG",
    "ONMANEUVERMESSAGE",
    "HANDLEUPDATEPLATOONFORMATION",
    "HANDLEUPDATEPLATOONDATA",
}

PLEXE_APP_METHODS = {
    "INITIALIZE",
    "SETSTATE",
    "STARTJOINMANEUVER",
    "SENDMANEUVERMSG",
    "ONBSM",
    "HANDLEJOINREQ",
    "HANDLEJOINRSP",
    "HANDLEMOVETOPOS",
    "HANDLEMOVETOPOSACK",
    "HANDLEJOINFORMATION",
    "HANDLEJOINFORMATIONACK",
    "HANDLEUPDATEFORMATION",
}


@dataclass(frozen=True)
class CapturedLine:
    category: str
    source_file: str
    line_no: int
    text: str


@dataclass
class StepRequirement:
    label: str
    predicate: Callable[[CapturedLine], bool]
    mode: str = "first"   # "first" or "all"
    min_count: int = 1


@dataclass
class StepResult:
    name: str
    passed: bool
    requirements: List[Tuple[StepRequirement, List[CapturedLine]]]


def strip_ansi(text: str) -> str:
    return ANSI_RE.sub("", text).rstrip("\n")


def upper(text: str) -> str:
    return text.upper()


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Extract JoinAtBack maneuver semantics from OMNeT++ logs.")
    ap.add_argument("input_files", nargs="+", help="Input OMNeT++ log files")
    ap.add_argument("-o", "--output", required=True, help="Output file")
    ap.add_argument("--actor", default="veh4", help="Joiner actor id to track for spawn/init lines (default: veh4)")
    ap.add_argument(
        "--all-beacons",
        action="store_true",
        help="Keep every [CarlaJoinAtBack][onPlatoonBeacon] line instead of only decisive ones",
    )
    return ap.parse_args()


def actor_index_from_actor(actor: str) -> Optional[str]:
    m = re.search(r"(\d+)$", actor)
    return m.group(1) if m else None


def strict_stage0_actor_lines(actor_index: Optional[str]) -> List[str]:
    if actor_index is None:
        return []
    base = f"INITIALIZING MODULE CARLAPLATOONINGNETWORK.ACTORS[{actor_index}]"
    return [
        f"{base}, STAGE 0",
        f"{base}.MOBILITY, STAGE 0",
        f"{base}.NIC, STAGE 0",
        f"{base}.NIC.PHY80211P, STAGE 0",
        f"{base}.NIC.MAC1609_4, STAGE 0",
        f"{base}.APPL, STAGE 0",
        f"{base}.BRIDGE, STAGE 0",
    ]


def is_spawn_or_init_line(text: str, actor: str) -> bool:
    """
    STRICT raw spawn/init matcher.

    Keeps:
    - exact vehX late-spawn lines
    - exact vehX stage-0 init subtree only
    - no stage 1..N mobility init drift
    """
    u = upper(text)
    actor_u = actor.upper()
    actor_index = actor_index_from_actor(actor)

    if "[CARLANETMANAGER][HANDLELATESPAWN]" in u:
        return True

    if "[CARLANETMANAGER][CREATEANDINITIALIZEACTOR]" in u and f"ACTOR={actor_u}" in u:
        return True

    if "[CARLANETMANAGER][REGISTERMOBILITYMODULE]" in u and f"ACTOR={actor_u}" in u:
        return True

    if "[CARLAGENERALPLATOONINGAPP][INITIALIZE]" in u and f"ACTOR={actor_u}" in u:
        return True

    if "[CARLAPLEXEAPP][INITIALIZE]" in u and f"ACTOR={actor_u}" in u:
        return True

    if "[BRIDGEAPP][INITIALIZE]" in u and f"ACTOR={actor_u}" in u:
        return True

    if u in strict_stage0_actor_lines(actor_index):
        return True

    if actor_index is not None and "CONNECTIONMANAGER::REGISTERNIC" in u and f"ACTORS[{actor_index}]" in u:
        return True

    return False


def extract_bracket_method(text: str, class_name: str) -> str:
    """
    From:
      [INFO] [CarlaJoinAtBack][handleMoveToPosition] ...
    return:
      HANDLEMOVETOPOSITION
    """
    m = re.search(rf"\[{re.escape(class_name)}\]\[([^\]]+)\]", text, flags=re.IGNORECASE)
    if not m:
        return ""
    return m.group(1).upper()


def keep_carla_join_at_back_line(text: str, all_beacons: bool) -> bool:
    if "[CarlaJoinAtBack][" not in text:
        return False

    method = extract_bracket_method(text, "CarlaJoinAtBack")
    if method not in CARLA_JOIN_METHODS:
        return False

    if method != "ONPLATOONBEACON":
        return True

    if all_beacons:
        return True

    u = upper(text)
    important_tokens = [
        "READY=1",
        "ACTION=SEND_MOVETOPOSITIONACK",
        "SAMPLES=1",
        "SAMPLES=2",
        "SAMPLES=3",
        "SAMPLES=4",
        "WARNING=JOIN_REQ_TIMEOUT",
    ]
    return any(tok in u for tok in important_tokens)


def keep_general_app_line(text: str) -> bool:
    if "[CarlaGeneralPlatooningApp][" not in text:
        return False

    method = extract_bracket_method(text, "CarlaGeneralPlatooningApp")
    if method not in GENERAL_APP_METHODS:
        return False

    u = upper(text)

    if method == "INITIALIZE":
        return True

    if method == "STARTJOINMANEUVERIFCONFIGURED":
        return True

    if method == "SENDUNICAST":
        return any(f"PACKET={pkt}" in u for pkt in GENERAL_APP_MANEUVER_PACKETS)

    if method == "HANDLELOWERMSG":
        return "RECEIVED_MANEUVER=" in u

    if method == "ONMANEUVERMESSAGE":
        return (
            "MANEUVER=MOVETOPOSITION" in u
            or "MANEUVER=JOINFORMATION" in u
        )

    if method in {"HANDLEUPDATEPLATOONFORMATION", "HANDLEUPDATEPLATOONDATA"}:
        return True

    return False


def keep_plexe_app_line(text: str) -> bool:
    if "[CarlaPlexeApp][" not in text:
        return False

    method = extract_bracket_method(text, "CarlaPlexeApp")
    if method not in PLEXE_APP_METHODS:
        return False

    u = upper(text)

    if method == "INITIALIZE":
        return True

    if method == "SETSTATE":
        return True

    if method == "STARTJOINMANEUVER":
        return True

    if method == "SENDMANEUVERMSG":
        return any(f"MSGTYPE={msg}" in u for msg in PLEXE_MANEUVER_TYPES)

    if method == "ONBSM":
        return ("MSGTYPE=" in u and "PLATOONINGBEACON" not in u) or ("REASON=" in u)

    if method in {
        "HANDLEJOINREQ",
        "HANDLEJOINRSP",
        "HANDLEMOVETOPOS",
        "HANDLEMOVETOPOSACK",
        "HANDLEJOINFORMATION",
        "HANDLEJOINFORMATIONACK",
        "HANDLEUPDATEFORMATION",
    }:
        return True

    return False


def keep_freeform_jab_line(text: str) -> bool:
    u = upper(text)

    if "[JAB]" not in u:
        return False

    freeform_tokens = [
        "JOIN_REQ",
        "JOIN_RSP",
        "MOVE_TO_POS",
        "MOVE_TO_POS_ACK",
        "JOIN_FORMATION",
        "JOIN_FORMATION_ACK",
        "UPDATE_FORMATION",
        "IN POSITION",
        "DENIED",
        "ABORT",
    ]
    return any(tok in u for tok in freeform_tokens)


def categorize(text: str, actor: str, all_beacons: bool) -> Optional[str]:
    if is_spawn_or_init_line(text, actor):
        return "spawn_init"
    if keep_carla_join_at_back_line(text, all_beacons):
        return "carla_join_at_back"
    if keep_general_app_line(text):
        return "general_app"
    if keep_plexe_app_line(text):
        return "plexe_app"
    if keep_freeform_jab_line(text):
        return "jab_freeform"
    return None


def line_text_has_all(item: CapturedLine, *parts: str) -> bool:
    u = upper(item.text)
    return all(p.upper() in u for p in parts)


def line_text_has_any(item: CapturedLine, *parts: str) -> bool:
    u = upper(item.text)
    return any(p.upper() in u for p in parts)


def make_all_pred(*parts: str) -> Callable[[CapturedLine], bool]:
    return lambda item: line_text_has_all(item, *parts)


def make_any_pred(*parts: str) -> Callable[[CapturedLine], bool]:
    return lambda item: line_text_has_any(item, *parts)


def extract_joiner_node_id(captured: Sequence[CapturedLine], actor: str) -> Optional[str]:
    actor_u = actor.upper()

    for item in captured:
        u = upper(item.text)
        if f"ACTOR={actor_u}" not in u:
            continue

        m = re.search(r"\bNODEID=(\d+)\b", u)
        if m:
            return m.group(1)

        m = re.search(r"\bNODE_ID=(\d+)\b", u)
        if m:
            return m.group(1)

    return actor_index_from_actor(actor)


def extract_leader_id(captured: Sequence[CapturedLine], actor: str) -> Optional[str]:
    actor_u = actor.upper()

    patterns = [
        r"\bLEADERID=(\d+)\b",
        r"\bLEADER_ID=(\d+)\b",
        r"\bDSTLEADER=(\d+)\b",
        r"\bLOGICAL_DST=(\d+)\b",
    ]

    for item in captured:
        u = upper(item.text)
        if f"ACTOR={actor_u}" not in u:
            continue

        for pat in patterns:
            m = re.search(pat, u)
            if m:
                return m.group(1)

    return None


def dedupe_preserve_order(lines: Sequence[CapturedLine]) -> List[CapturedLine]:
    seen = set()
    out: List[CapturedLine] = []
    for item in lines:
        key = (item.source_file, item.line_no)
        if key in seen:
            continue
        seen.add(key)
        out.append(item)
    return out


def evaluate_step(captured: Sequence[CapturedLine], name: str, requirements: Sequence[StepRequirement]) -> StepResult:
    req_results: List[Tuple[StepRequirement, List[CapturedLine]]] = []
    passed = True

    for req in requirements:
        matches = [item for item in captured if req.predicate(item)]
        if req.mode == "first":
            matches = matches[:1]
        elif req.mode == "all":
            pass
        else:
            raise ValueError(f"Unknown requirement mode: {req.mode}")

        if len(matches) < req.min_count:
            passed = False

        req_results.append((req, dedupe_preserve_order(matches)))

    return StepResult(name=name, passed=passed, requirements=req_results)


def build_step_checks(captured: Sequence[CapturedLine], actor: str) -> List[StepResult]:
    actor_u = actor.upper()
    joiner_id = extract_joiner_node_id(captured, actor)
    leader_id = extract_leader_id(captured, actor)
    leader_actor_u = f"VEH{leader_id}" if leader_id is not None else None

    def pred_actor(*parts: str) -> Callable[[CapturedLine], bool]:
        return make_all_pred(f"ACTOR={actor_u}", *parts)

    def pred_leader(*parts: str) -> Callable[[CapturedLine], bool]:
        if leader_actor_u is not None:
            return make_all_pred(f"ACTOR={leader_actor_u}", *parts)
        return make_all_pred(*parts)

    def pred_any_of(*preds: Callable[[CapturedLine], bool]) -> Callable[[CapturedLine], bool]:
        return lambda item: any(p(item) for p in preds)

    steps: List[StepResult] = []

    # 1) Spawn / init
    steps.append(
        evaluate_step(
            captured,
            "Joiner spawned and initialized",
            [
                StepRequirement(
                    "late spawn triggered",
                    pred_any_of(
                        make_all_pred("[CARLANETMANAGER][HANDLELATESPAWN]"),
                    ),
                ),
                StepRequirement(
                    "joiner app initialized",
                    pred_any_of(
                        pred_actor("[CARLAGENERALPLATOONINGAPP][INITIALIZE]"),
                        pred_actor("[CARLAPLEXEAPP][INITIALIZE]"),
                    ),
                ),
                StepRequirement(
                    "joiner bridge initialized",
                    pred_actor("[BRIDGEAPP][INITIALIZE]"),
                ),
                StepRequirement(
                    "joiner actor created",
                    pred_actor("[CARLANETMANAGER][CREATEANDINITIALIZEACTOR]"),
                ),
            ],
        )
    )

    # 2) Join request sent
    steps.append(
        evaluate_step(
            captured,
            "Join request started and transmitted",
            [
                StepRequirement(
                    "start trigger",
                    pred_any_of(
                        pred_actor("[CARLAGENERALPLATOONINGAPP][STARTJOINMANEUVERIFCONFIGURED]"),
                        pred_actor("[CARLAPLEXEAPP][STARTJOINMANEUVER]"),
                    ),
                ),
                StepRequirement(
                    "join wait-reply state entered",
                    pred_any_of(
                        pred_actor("[CARLAJOINATBACK][INITIALIZEJOINMANEUVER]", "STATE=J_WAIT_REPLY"),
                        pred_actor("[CARLAPLEXEAPP][SETSTATE]", "JOIN_STATE_NEXT=J_WAIT_REPLY"),
                    ),
                ),
                StepRequirement(
                    "join request emitted",
                    pred_any_of(
                        pred_actor("[CARLAJOINATBACK][SENDJOINREQUEST]"),
                        pred_actor("[CARLAPLEXEAPP][SENDMANEUVERMSG]", "MSGTYPE=JOIN_REQ"),
                    ),
                ),
                StepRequirement(
                    "join request packet sent",
                    pred_any_of(
                        pred_actor("[CARLAGENERALPLATOONINGAPP][SENDUNICAST]", "PACKET=JOINPLATOONREQUEST"),
                        pred_actor("[CARLAPLEXEAPP][SENDMANEUVERMSG]", "MSGTYPE=JOIN_REQ"),
                    ),
                ),
            ],
        )
    )

    # 3) Leader accepted
    leader_received_req_parts = ["RECEIVED_MANEUVER=JOINPLATOONREQUEST"]
    if joiner_id is not None:
        leader_received_req_parts.append(f"SRC={joiner_id}")
    if leader_id is not None:
        leader_received_req_parts.append(f"DST={leader_id}")

    steps.append(
        evaluate_step(
            captured,
            "Leader received and accepted the join request",
            [
                StepRequirement(
                    "leader received join request",
                    pred_any_of(
                        pred_leader("[CARLAGENERALPLATOONINGAPP][HANDLELOWERMSG]", *leader_received_req_parts),
                        pred_leader("[CARLAPLEXEAPP][ONBSM]", "MSGTYPE=JOIN_REQ"),
                    ),
                ),
                StepRequirement(
                    "leader permitted join",
                    pred_any_of(
                        pred_leader("[CARLAJOINATBACK][PROCESSJOINREQUEST]", "PERMITTED=1"),
                        pred_leader("[CARLAPLEXEAPP][HANDLEJOINREQ]", "PERMITTED=1"),
                    ),
                ),
                StepRequirement(
                    "leader sent response",
                    pred_any_of(
                        pred_leader("[CARLAGENERALPLATOONINGAPP][SENDUNICAST]", "PACKET=JOINPLATOONRESPONSE"),
                        pred_leader("[CARLAPLEXEAPP][SENDMANEUVERMSG]", "MSGTYPE=JOIN_RSP"),
                    ),
                ),
                StepRequirement(
                    "leader sent move-to-position command",
                    pred_any_of(
                        pred_leader("[CARLAJOINATBACK][HANDLEJOINPLATOONREQUEST]", "ACTION=SEND_MOVETOPOSITION"),
                        pred_leader("[CARLAGENERALPLATOONINGAPP][SENDUNICAST]", "PACKET=MOVETOPOSITION"),
                        pred_leader("[CARLAPLEXEAPP][SENDMANEUVERMSG]", "MSGTYPE=MOVE_TO_POS"),
                    ),
                ),
            ],
        )
    )

    # 4) Joiner received permission and move command
    joiner_rsp_parts = ["RECEIVED_MANEUVER=JOINPLATOONRESPONSE"]
    if leader_id is not None:
        joiner_rsp_parts.append(f"SRC={leader_id}")
    if joiner_id is not None:
        joiner_rsp_parts.append(f"DST={joiner_id}")

    joiner_mtp_parts = ["RECEIVED_MANEUVER=MOVETOPOSITION"]
    if leader_id is not None:
        joiner_mtp_parts.append(f"SRC={leader_id}")
    if joiner_id is not None:
        joiner_mtp_parts.append(f"DST={joiner_id}")

    steps.append(
        evaluate_step(
            captured,
            "Joiner received permission and moved into join positioning state",
            [
                StepRequirement(
                    "joiner received response",
                    pred_any_of(
                        pred_actor("[CARLAGENERALPLATOONINGAPP][HANDLELOWERMSG]", *joiner_rsp_parts),
                        pred_actor("[CARLAPLEXEAPP][ONBSM]", "MSGTYPE=JOIN_RSP"),
                    ),
                ),
                StepRequirement(
                    "joiner entered wait-information state",
                    pred_any_of(
                        pred_actor("[CARLAJOINATBACK][HANDLEJOINPLATOONRESPONSE]", "STATE=J_WAIT_INFORMATION"),
                        pred_actor("[CARLAPLEXEAPP][SETSTATE]", "JOIN_STATE_NEXT=J_WAIT_INFORMATION"),
                    ),
                ),
                StepRequirement(
                    "joiner received move-to-position",
                    pred_any_of(
                        pred_actor("[CARLAGENERALPLATOONINGAPP][HANDLELOWERMSG]", *joiner_mtp_parts),
                        pred_actor("[CARLAPLEXEAPP][ONBSM]", "MSGTYPE=MOVE_TO_POS"),
                    ),
                ),
                StepRequirement(
                    "joiner applied move-to-position semantics",
                    pred_any_of(
                        pred_actor("[CARLAGENERALPLATOONINGAPP][ONMANEUVERMESSAGE]", "MANEUVER=MOVETOPOSITION"),
                        pred_actor("[CARLAJOINATBACK][HANDLEMOVETOPOSITION]", "STATE=J_MOVE_IN_POSITION"),
                        pred_actor("[CARLAPLEXEAPP][HANDLEMOVETOPOS]"),
                    ),
                ),
            ],
        )
    )

    # 5) In-position convergence and ack
    steps.append(
        evaluate_step(
            captured,
            "Joiner reached in-position condition and acknowledged",
            [
                StepRequirement(
                    "decisive in-position detection",
                    pred_any_of(
                        pred_actor("[CARLAJOINATBACK][ONPLATOONBEACON]", "READY=1"),
                        pred_actor("[JAB]", "IN POSITION"),
                    ),
                ),
                StepRequirement(
                    "move-to-position ack decision",
                    pred_any_of(
                        pred_actor("[CARLAJOINATBACK][ONPLATOONBEACON]", "ACTION=SEND_MOVETOPOSITIONACK"),
                        pred_actor("[CARLAPLEXEAPP][SENDMANEUVERMSG]", "MSGTYPE=MOVE_TO_POS_ACK"),
                    ),
                ),
                StepRequirement(
                    "move-to-position ack transmitted",
                    pred_any_of(
                        pred_actor("[CARLAGENERALPLATOONINGAPP][SENDUNICAST]", "PACKET=MOVETOPOSITIONACK"),
                        pred_actor("[CARLAPLEXEAPP][SENDMANEUVERMSG]", "MSGTYPE=MOVE_TO_POS_ACK"),
                    ),
                ),
            ],
        )
    )

    # 6) Leader sent join formation
    leader_mtp_ack_parts = ["RECEIVED_MANEUVER=MOVETOPOSITIONACK"]
    if joiner_id is not None:
        leader_mtp_ack_parts.append(f"SRC={joiner_id}")
    if leader_id is not None:
        leader_mtp_ack_parts.append(f"DST={leader_id}")

    steps.append(
        evaluate_step(
            captured,
            "Leader received joiner ACK and sent JoinFormation",
            [
                StepRequirement(
                    "leader received move-to-position ack",
                    pred_any_of(
                        pred_leader("[CARLAGENERALPLATOONINGAPP][HANDLELOWERMSG]", *leader_mtp_ack_parts),
                        pred_leader("[CARLAPLEXEAPP][ONBSM]", "MSGTYPE=MOVE_TO_POS_ACK"),
                    ),
                ),
                StepRequirement(
                    "leader triggered join formation",
                    pred_any_of(
                        pred_leader("[CARLAJOINATBACK][HANDLEMOVETOPOSITIONACK]", "ACTION=SEND_JOINFORMATION"),
                        pred_leader("[CARLAPLEXEAPP][HANDLEMOVETOPOSACK]"),
                    ),
                ),
                StepRequirement(
                    "leader transmitted join formation",
                    pred_any_of(
                        pred_leader("[CARLAGENERALPLATOONINGAPP][SENDUNICAST]", "PACKET=JOINFORMATION"),
                        pred_leader("[CARLAPLEXEAPP][SENDMANEUVERMSG]", "MSGTYPE=JOIN_FORMATION"),
                    ),
                ),
            ],
        )
    )

    # 7) Joiner completed join
    joiner_jf_parts = ["RECEIVED_MANEUVER=JOINFORMATION"]
    if leader_id is not None:
        joiner_jf_parts.append(f"SRC={leader_id}")
    if joiner_id is not None:
        joiner_jf_parts.append(f"DST={joiner_id}")

    steps.append(
        evaluate_step(
            captured,
            "Joiner received JoinFormation and completed the join as follower",
            [
                StepRequirement(
                    "joiner received join formation",
                    pred_any_of(
                        pred_actor("[CARLAGENERALPLATOONINGAPP][HANDLELOWERMSG]", *joiner_jf_parts),
                        pred_actor("[CARLAPLEXEAPP][ONBSM]", "MSGTYPE=JOIN_FORMATION"),
                    ),
                ),
                StepRequirement(
                    "join formation semantics applied",
                    pred_any_of(
                        pred_actor("[CARLAGENERALPLATOONINGAPP][ONMANEUVERMESSAGE]", "MANEUVER=JOINFORMATION"),
                        pred_actor("[CARLAJOINATBACK][HANDLEJOINFORMATION]", "ACTION=JOIN_COMPLETE", "ROLE=FOLLOWER"),
                        pred_actor("[CARLAPLEXEAPP][HANDLEJOINFORMATION]"),
                    ),
                ),
                StepRequirement(
                    "join formation ack transmitted",
                    pred_any_of(
                        pred_actor("[CARLAGENERALPLATOONINGAPP][SENDUNICAST]", "PACKET=JOINFORMATIONACK"),
                        pred_actor("[CARLAPLEXEAPP][SENDMANEUVERMSG]", "MSGTYPE=JOIN_FORMATION_ACK"),
                    ),
                ),
            ],
        )
    )

    # 8) Leader finalized and broadcasted update
    leader_jf_ack_parts = ["RECEIVED_MANEUVER=JOINFORMATIONACK"]
    if joiner_id is not None:
        leader_jf_ack_parts.append(f"SRC={joiner_id}")
    if leader_id is not None:
        leader_jf_ack_parts.append(f"DST={leader_id}")

    steps.append(
        evaluate_step(
            captured,
            "Leader received final ACK and broadcasted updated formation",
            [
                StepRequirement(
                    "leader received join formation ack",
                    pred_any_of(
                        pred_leader("[CARLAGENERALPLATOONINGAPP][HANDLELOWERMSG]", *leader_jf_ack_parts),
                        pred_leader("[CARLAPLEXEAPP][ONBSM]", "MSGTYPE=JOIN_FORMATION_ACK"),
                    ),
                ),
                StepRequirement(
                    "leader finalized formation broadcast step",
                    pred_any_of(
                        pred_leader("[CARLAJOINATBACK][HANDLEJOINFORMATIONACK]", "ACTION=BROADCAST_UPDATEPLATOONFORMATION"),
                        pred_leader("[CARLAPLEXEAPP][HANDLEJOINFORMATIONACK]"),
                    ),
                ),
                StepRequirement(
                    "updated formation packets transmitted",
                    pred_any_of(
                        pred_leader("[CARLAGENERALPLATOONINGAPP][SENDUNICAST]", "PACKET=UPDATEPLATOONFORMATION"),
                        pred_leader("[CARLAPLEXEAPP][SENDMANEUVERMSG]", "MSGTYPE=UPDATE_FORMATION"),
                    ),
                    mode="all",
                    min_count=1,
                ),
            ],
        )
    )

    # 9) Formation update applied
    steps.append(
        evaluate_step(
            captured,
            "Formation update was applied by platoon members",
            [
                StepRequirement(
                    "update application observed",
                    pred_any_of(
                        make_all_pred("[CARLAGENERALPLATOONINGAPP][HANDLEUPDATEPLATOONFORMATION]"),
                        make_all_pred("[CARLAGENERALPLATOONINGAPP][HANDLEUPDATEPLATOONDATA]"),
                        make_all_pred("[CARLAPLEXEAPP][HANDLEUPDATEFORMATION]"),
                    ),
                    mode="all",
                    min_count=1,
                ),
            ],
        )
    )

    return steps


def sort_captured(captured: List[CapturedLine]) -> None:
    captured.sort(key=lambda x: (x.source_file, x.line_no))


def write_step_checks(out, steps: Sequence[StepResult]) -> Tuple[int, int, bool]:
    passed_steps = sum(1 for s in steps if s.passed)
    total_steps = len(steps)
    overall_pass = passed_steps == total_steps

    out.write("=== JOIN MANEUVER STEP CHECKS ===\n")
    out.write(f"steps_passed: {passed_steps}/{total_steps}\n")
    out.write(f"overall_result: {'PASS' if overall_pass else 'FAIL'}\n\n")

    for idx, step in enumerate(steps, start=1):
        out.write(f"[{'PASS' if step.passed else 'FAIL'}] Step {idx}: {step.name}\n")
        for req, matches in step.requirements:
            req_pass = len(matches) >= req.min_count
            out.write(f"  [{'PASS' if req_pass else 'FAIL'}] {req.label}\n")
            if matches:
                for item in matches:
                    out.write(f"    [{item.source_file}:{item.line_no}] {item.text}\n")
            else:
                out.write("    <missing>\n")
        out.write("\n")

    return passed_steps, total_steps, overall_pass


def main() -> int:
    args = parse_args()

    captured: List[CapturedLine] = []
    total_lines = 0

    counts: Dict[str, int] = {
        "spawn_init": 0,
        "carla_join_at_back": 0,
        "general_app": 0,
        "plexe_app": 0,
        "jab_freeform": 0,
    }

    for file_path in args.input_files:
        base = os.path.basename(file_path)
        try:
            with open(file_path, "r", encoding="utf-8", errors="replace") as f:
                for line_no, raw in enumerate(f, start=1):
                    total_lines += 1
                    text = strip_ansi(raw)
                    if not text.strip():
                        continue

                    category = categorize(text, args.actor, args.all_beacons)
                    if category is None:
                        continue

                    counts[category] += 1
                    captured.append(
                        CapturedLine(
                            category=category,
                            source_file=base,
                            line_no=line_no,
                            text=text,
                        )
                    )

        except FileNotFoundError:
            print(f"ERROR: file not found: {file_path}", file=sys.stderr)
            return 2
        except OSError as exc:
            print(f"ERROR: could not read {file_path}: {exc}", file=sys.stderr)
            return 2

    sort_captured(captured)
    steps = build_step_checks(captured, args.actor)

    try:
        with open(args.output, "w", encoding="utf-8") as out:
            out.write("=== JOIN-AT-BACK SEMANTICS EXTRACT ===\n")
            out.write(f"input_files: {', '.join(args.input_files)}\n")
            out.write(f"output_file: {args.output}\n")
            out.write(f"join_actor: {args.actor}\n")
            out.write(f"all_beacons: {1 if args.all_beacons else 0}\n")
            out.write(f"lines_scanned: {total_lines}\n")
            out.write(f"lines_kept_total: {len(captured)}\n")
            out.write(f"spawn_init_lines: {counts['spawn_init']}\n")
            out.write(f"carla_join_at_back_lines: {counts['carla_join_at_back']}\n")
            out.write(f"general_app_lines: {counts['general_app']}\n")
            out.write(f"plexe_app_lines: {counts['plexe_app']}\n")
            out.write(f"jab_freeform_lines: {counts['jab_freeform']}\n")
            out.write("\n")
            out.write("Kept categories:\n")
            out.write("  - strict joiner spawn/init raw lines\n")
            out.write("  - CarlaJoinAtBack lifecycle\n")
            out.write("  - CarlaGeneralPlatooningApp maneuver send/receive/apply\n")
            out.write("  - CarlaPlexeApp maneuver/state/send/receive\n")
            out.write("  - free-form [JAB] maneuver lines\n")
            if args.all_beacons:
                out.write("  - all CarlaJoinAtBack onPlatoonBeacon lines\n")
            else:
                out.write("  - decisive CarlaJoinAtBack onPlatoonBeacon lines only\n")
            out.write("\n")

            write_step_checks(out, steps)

            out.write("=== RAW EXTRACTED LINES ===\n")
            current_file = None
            for item in captured:
                if item.source_file != current_file:
                    if current_file is not None:
                        out.write("\n")
                    out.write(f"--- {item.source_file} ---\n")
                    current_file = item.source_file

                out.write(f"[{item.source_file}:{item.line_no}] {item.text}\n")

        print(f"Wrote {len(captured)} lines to {args.output}")
        return 0

    except OSError as exc:
        print(f"ERROR: could not write {args.output}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())