#!/usr/bin/env python3
"""
pyCARLANeT main.py — CARLA↔OMNeT++ bridge for joinAtBack-style V2V simulations.

STRICT protocol mode:
- INIT must contain: user_defined.map and moving_actors (NOT map_name / actors)
- GENERIC_MESSAGE must use: user_defined.msg_type, and for CONTROL: user_defined.actor_id + user_defined.ctrl
- CONTROL_BATCH entries must use: actor_id + ctrl
- seq/src/hop are read from user_defined (as OMNeT sends)
"""

import json
import math
import os
import signal
import time
import zmq

from pycarlanet_log import log, banner, log_once, reset_once

try:
    import carla
except Exception as import_error:
    carla = None
    _CARLA_IMPORT_ERROR = import_error


# -------------------- Status codes (must match C++) --------------------
SIM_STATUS_RUNNING = 0
SIM_STATUS_FINISHED_OK = 1
SIM_STATUS_FINISHED_ACCIDENT = 2
SIM_STATUS_FINISHED_TIME_LIMIT = 3
SIM_STATUS_ERROR = -1


# -------------------- Configuration --------------------
ZMQ_BIND = "tcp://*:5555"

CARLA_HOST = "127.0.0.1"
CARLA_PORT = 2000
CARLA_RPC_TIMEOUT_SECONDS = 10.0

DEFAULT_MAP_NAME = "Town04"

RELOAD_WORLD_ON_INIT = True
DESTROY_OLD_ACTORS_ON_INIT = True
DESTROY_ROLE_PREFIXES = ("veh",)
DESTROY_OLD_ACTORS_ON_SHUTDOWN = False


def _env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name, "1" if default else "0").strip().lower()
    return value in ("1", "true", "yes", "y", "on")


RELOAD_WORLD_ON_INIT = _env_bool("PYCARLANET_RELOAD_ON_INIT", RELOAD_WORLD_ON_INIT)
DESTROY_OLD_ACTORS_ON_INIT = _env_bool("PYCARLANET_DESTROY_ON_INIT", DESTROY_OLD_ACTORS_ON_INIT)
DESTROY_OLD_ACTORS_ON_SHUTDOWN = _env_bool("PYCARLANET_DESTROY_ON_SHUTDOWN", DESTROY_OLD_ACTORS_ON_SHUTDOWN)

if os.getenv("PYCARLANET_REFRESH_ON_START") is not None:
    RELOAD_WORLD_ON_INIT = _env_bool("PYCARLANET_REFRESH_ON_START", RELOAD_WORLD_ON_INIT)

_env_prefixes = os.getenv("PYCARLANET_DESTROY_ROLE_PREFIXES", "").strip()
if _env_prefixes:
    DESTROY_ROLE_PREFIXES = tuple(prefix.strip() for prefix in _env_prefixes.split(",") if prefix.strip()) or DESTROY_ROLE_PREFIXES

DEFAULT_VEHICLE_BLUEPRINT = "vehicle.tesla.model3"
TRY_SPAWN_RETRIES = 12
SPAWN_Z_OFFSET_METERS = 0.75

# Plexe-parity initial platoon:
# fixed 5.0 m clearance gap + 4.5 m vehicle length = 9.5 m center spacing
PLATOON_SPACING_METERS = 9.5
PLATOON_LATERAL_JITTER_METERS = 0.25
PLATOON_MAX_LANESIDE_TRIES = 7

LATE_SPAWN_BACK_METERS = float(os.getenv("PYCARLANET_LATE_SPAWN_BACK_M", "0.0"))

HOLD_BRAKE_UNTIL_CONTROL = True
THROTTLE_GAIN_PER_MPS2 = 0.15
BRAKE_GAIN_PER_MPS2 = 0.20

SPEED_ERROR_TO_ACCELERATION_KP = 1.2
MAX_ACCELERATION_COMMAND_MPS2 = 2.5
MAX_BRAKE_COMMAND_MPS2 = -4.0

EXIT_ON_FINISH = True
FREEZE_VEHICLES_ON_FINISH = True
FORCE_ASYNC_ON_RESTORE = True

ENABLE_LANE_KEEPING = True
LANE_KEEP_LOOKAHEAD_METERS = 12.0
LANE_KEEP_KP = 0.9

LATE_SPAWN_USE_INIT_LEADER_ANCHOR = True
LATE_SPAWN_INIT_LEADER_BACK_METERS = float(os.getenv("PYCARLANET_LATE_SPAWN_INIT_LEADER_BACK_M", "0.0"))
LATE_SPAWN_INIT_LEADER_EXTRA_BACKOFFS = (0.0, 20.0, 40.0, 60.0)
LATE_SPAWN_ONLY_FOR_MAP = "Town04"

ENABLE_LEADER_PROFILE = False
LEADER_ACTOR_ID = "veh0"
LEADER_PULSE_PERIOD_SECONDS = 1.0
LEADER_PULSE_BRAKE_SECONDS = 0.0
LEADER_BRAKE_ACCELERATION_MPS2 = 0.0
LEADER_CRUISE_ACCELERATION_MPS2 = 2.0
LEADER_START_IN_CRUISE = True

MINIMUM_LAUNCH_SPEED_MPS = 0.5
MINIMUM_LAUNCH_THROTTLE = 0.25

LATE_SPAWN_USE_HARDCODED_TOWN04_ANCHOR = True
LATE_SPAWN_HARDCODED_BACK_METERS = 100.0
LATE_SPAWN_HARDCODED_EXTRA_BACKOFFS = (0.0, 20.0, 40.0, 60.0)

TOWN04_INIT_LEADER_ANCHOR = {
    "x": 280.9085,
    "y": 16.6043,
    "z": 3.6058,
    "pitch": 2.6797,
    "yaw": -179.0221,
    "roll": 0.0,
}

# telemetry
TELEMETRY_ENABLED = _env_bool("PYCARLANET_TELEM_ENABLE", True)
TELEMETRY_PERIOD_SECONDS = float(os.getenv("PYCARLANET_TELEM_PERIOD_S", "1.0"))
if TELEMETRY_PERIOD_SECONDS <= 0.0:
    TELEMETRY_PERIOD_SECONDS = 1.0

# ZMQ step log throttle (0 = log all steps)
ZMQ_STEP_LOG_PERIOD_SECONDS = float(os.getenv("PYCARLANET_ZMQ_STEP_LOG_PERIOD_S", "0"))
if ZMQ_STEP_LOG_PERIOD_SECONDS < 0:
    ZMQ_STEP_LOG_PERIOD_SECONDS = 0.0

# Raw JSON printing (useful defaults)
LOG_RAW_JSON = _env_bool("PYCARLANET_LOG_RAW", True)
LOG_ALL = _env_bool("PYCARLANET_LOG_ALL", False)
LOG_RAW_ALL = _env_bool("PYCARLANET_LOG_RAW_ALL", True)
LOG_RAW_MAX_CHARS = int(os.getenv("PYCARLANET_LOG_RAW_MAX_CHARS", "600"))

# Periodic raw snapshot (0 = only on meaningful change)
LOG_RAW_PERIOD_SECONDS = float(os.getenv("PYCARLANET_LOG_RAW_PERIOD_S", "0"))
if LOG_RAW_PERIOD_SECONDS < 0:
    LOG_RAW_PERIOD_SECONDS = 0.0

# By default do NOT dump step/position raw (too noisy)
LOG_RAW_INCLUDE_STEPS = _env_bool("PYCARLANET_LOG_RAW_INCLUDE_STEPS", False)
LOG_RAW_INCLUDE_POSITIONS = _env_bool("PYCARLANET_LOG_RAW_INCLUDE_POSITIONS", False)

# If you do include steps, throttle raw step dumps by sim time
LOG_RAW_STEP_PERIOD_SECONDS = float(os.getenv("PYCARLANET_LOG_RAW_STEP_PERIOD_S", "2.0"))
if LOG_RAW_STEP_PERIOD_SECONDS < 0:
    LOG_RAW_STEP_PERIOD_SECONDS = 0.0

# spectator camera
SPECTATOR_ENABLED = _env_bool("PYCARLANET_SPECTATOR_ENABLE", True)
SPECTATOR_MIN_UP_METERS = 45.0
SPECTATOR_MIN_BACK_METERS = 25.0
SPECTATOR_PITCH_DEGREES = -45.0
SPECTATOR_HEIGHT_GAIN = 0.8
SPECTATOR_BACK_GAIN = 0.6

# -------------------- INIT settling gate (pre INIT_COMPLETED) --------------------
INIT_SETTLE_GATE_ENABLED = _env_bool("PYCARLANET_INIT_SETTLE_GATE", True)

# How long we allow CARLA to settle after spawning (in sync ticks)
INIT_SETTLE_MAX_TICKS = int(os.getenv("PYCARLANET_INIT_SETTLE_MAX_TICKS", "25"))

# Require this many consecutive "stable" frames before we proceed
INIT_SETTLE_STABLE_FRAMES = int(os.getenv("PYCARLANET_INIT_SETTLE_STABLE_FRAMES", "4"))

# Stability thresholds
INIT_SETTLE_VZ_EPS = float(os.getenv("PYCARLANET_INIT_SETTLE_VZ_EPS", "0.08"))   # m/s
INIT_SETTLE_DZ_EPS = float(os.getenv("PYCARLANET_INIT_SETTLE_DZ_EPS", "0.01"))   # m per tick

# CURVATURE_DS_METERS = float(os.getenv("PYCARLANET_CURVATURE_DS_M", "3.0"))
# CURVATURE_RADIUS_STRAIGHT_M = float(os.getenv("PYCARLANET_CURVATURE_STRAIGHT_M", "1000000.0"))
# CURVATURE_RADIUS_MIN_M = float(os.getenv("PYCARLANET_CURVATURE_MIN_M", "1.0"))

def clamp(value: float, minimum: float = 0.0, maximum: float = 1.0) -> float:
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def yaw_forward_and_right_vectors(yaw_degrees: float):
    yaw_radians = math.radians(float(yaw_degrees))
    forward = (math.cos(yaw_radians), math.sin(yaw_radians))
    right = (-math.sin(yaw_radians), math.cos(yaw_radians))
    return forward, right


def wrap_to_pi(radians_value: float) -> float:
    while radians_value > math.pi:
        radians_value -= 2 * math.pi
    while radians_value < -math.pi:
        radians_value += 2 * math.pi
    return radians_value


def safe_float(value, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def safe_int(value, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return int(default)


# -------------------- Raw JSON monitor --------------------
class RawJsonMonitor:
    """
    Prints raw JSON when it changes meaningfully, and optionally periodically.
    Avoids spam by default: steps/positions are excluded unless enabled.
    """
    def __init__(self):
        self._last_sig_by_key = {}         # (direction,key)->sig
        self._last_time_by_key = {}        # (direction,key)->t
        self._last_step_raw_time = None    # sim time gating
        self.initial_spacing_m = float(PLATOON_SPACING_METERS)

    @staticmethod
    def _raw_event(direction: str, *, actor: bool = False) -> str:
        d = str(direction or "").strip().upper()
        if d in ("RX", "RECEIVED"):
            base = "RAW_RECEIVED"
        elif d in ("TX", "REPLIED", "REPLY"):
            base = "RAW_REPLIED"
        else:
            base = f"RAW_{d}" if d else "RAW"
        if actor:
            base += "_ACTOR"
        return base

    @staticmethod
    def _compact_json(text: str) -> str:
        try:
            obj = json.loads(text)
            text = json.dumps(obj, separators=(",", ":"), ensure_ascii=False, allow_nan=True)
            text = RawJsonMonitor._string_trim_floats(text, decimals=3)
        except Exception:
            pass

        one_line = " ".join(str(text).split())
        if len(one_line) > LOG_RAW_MAX_CHARS:
            return one_line[:LOG_RAW_MAX_CHARS] + "…"
        return one_line

    @staticmethod
    def _string_trim_floats(s: str, decimals: int = 3) -> str:
        import re
        num_re = re.compile(r"(-?\d+\.\d+(?:[eE][+-]?\d+)?)")

        def repl(m):
            token = m.group(1)
            try:
                v = float(token)
            except Exception:
                return token
            fmt = f"{{:.{decimals}f}}"
            out = fmt.format(v).rstrip("0").rstrip(".")
            return out

        return num_re.sub(repl, s)

    @staticmethod
    def _msg_key(parsed: dict) -> str:
        mt = str(parsed.get("message_type", "UNKNOWN")).upper()
        if mt == "GENERIC_MESSAGE":
            ud = parsed.get("user_defined") or {}
            if isinstance(ud, dict):
                # STRICT: OMNeT uses msg_type (not message_type)
                sub = str(ud.get("msg_type", "")).upper()
                return f"{mt}:{sub}" if sub else mt
        return mt

    @staticmethod
    def _sig(obj, depth=0):
        if depth > 5:
            return "<deep>"

        volatile_keys = {
            "timestamp", "initial_timestamp",
            "position", "velocity", "rotation",
        }

        if isinstance(obj, dict):
            items = []
            for k in sorted(obj.keys()):
                v = obj.get(k)

                if k in volatile_keys:
                    items.append((k, "<volatile>"))
                    continue

                if k == "actor_positions":
                    ids = []
                    if isinstance(v, list):
                        for e in v:
                            if isinstance(e, dict):
                                ids.append(str(e.get("actor_id", "?")))
                    items.append((k, ("actor_ids", tuple(ids))))
                    continue

                if k in ("actors", "moving_actors"):
                    ids = []
                    if isinstance(v, list):
                        for e in v:
                            if isinstance(e, dict):
                                ids.append(str(e.get("actor_id", "?")))
                    items.append((k, ("actor_ids", tuple(ids))))
                    continue

                if k == "user_defined":
                    if isinstance(v, dict):
                        # STRICT: msg_type key
                        message_type = str(v.get("msg_type", "")).upper()
                        keys = tuple(sorted(v.keys()))
                        items.append((k, (message_type, keys)))
                    else:
                        items.append((k, type(v).__name__))
                    continue

                if isinstance(v, (int, float)):
                    items.append((k, "<n>"))
                elif isinstance(v, (dict, list)):
                    items.append((k, RawJsonMonitor._sig(v, depth + 1)))
                else:
                    items.append((k, str(v)[:80]))
            return tuple(items)

        if isinstance(obj, list):
            if not obj:
                return ("list", 0)
            if isinstance(obj[0], dict) and ("actor_id" in obj[0]):
                ids = []
                for e in obj:
                    if isinstance(e, dict):
                        ids.append(str(e.get("actor_id", "?")))
                return ("list_actor_ids", tuple(ids))
            return ("list_len", len(obj))

        return type(obj).__name__

    def maybe_log(self, direction: str, parsed: dict, raw_text: str) -> None:
        if not LOG_RAW_JSON:
            return
        if not isinstance(parsed, dict):
            return

        message_type = str(parsed.get("message_type", "UNKNOWN")).upper()
        key = self._msg_key(parsed)

        if LOG_ALL:
            preview = self._compact_json(raw_text)
            log("ZMQ", self._raw_event(direction), preview, message_type=message_type, msg_key=key)

            actor_positions = parsed.get("actor_positions", None)
            if isinstance(actor_positions, list) and actor_positions:
                for entry in actor_positions:
                    if not isinstance(entry, dict):
                        continue
                    actor_id = str(entry.get("actor_id", "")).strip() or "?"
                    try:
                        entry_text = json.dumps(entry, separators=(",", ":"), ensure_ascii=False, allow_nan=True)
                    except Exception:
                        entry_text = str(entry)
                    entry_preview = self._compact_json(entry_text)
                    log(
                        "ZMQ",
                        self._raw_event(direction, actor=True),
                        entry_preview,
                        message_type=message_type,
                        msg_key=f"{key}:{actor_id}",
                        actor=actor_id,
                    )
            return

        if message_type == "SIMULATION_STEP" and not LOG_RAW_INCLUDE_STEPS:
            return
        if message_type == "UPDATED_POSITIONS" and not LOG_RAW_INCLUDE_POSITIONS:
            return

        if LOG_RAW_ALL:
            preview = self._compact_json(raw_text)
            log("ZMQ", self._raw_event(direction), preview, message_type=message_type, msg_key=key)
            return

        sig = self._sig(parsed)

        timestamp = parsed.get("timestamp", None)
        sim_time = safe_float(timestamp, None) if timestamp is not None else None

        if (
            message_type == "SIMULATION_STEP"
            and LOG_RAW_INCLUDE_STEPS
            and LOG_RAW_STEP_PERIOD_SECONDS > 0
            and sim_time is not None
        ):
            if self._last_step_raw_time is not None and (sim_time - float(self._last_step_raw_time)) < LOG_RAW_STEP_PERIOD_SECONDS:
                return
            self._last_step_raw_time = sim_time

        state_key = (str(direction).upper(), key)
        last_sig = self._last_sig_by_key.get(state_key)
        changed = (last_sig != sig)

        now_time = sim_time if sim_time is not None else time.monotonic()
        last_time = self._last_time_by_key.get(state_key)

        periodic = False
        if LOG_RAW_PERIOD_SECONDS > 0 and last_time is not None:
            periodic = (float(now_time) - float(last_time)) >= float(LOG_RAW_PERIOD_SECONDS)

        if not changed and not periodic:
            return

        self._last_sig_by_key[state_key] = sig
        self._last_time_by_key[state_key] = now_time

        preview = self._compact_json(raw_text)
        log("ZMQ", self._raw_event(direction), preview, message_type=message_type, msg_key=key)


# -------------------- CARLA helpers --------------------
def choose_spawn_point_near_spectator(world, seed: int):
    """
    Pick a CARLA spawn point nearest to the spectator camera (XY distance).
    If spectator is unavailable, fall back to seed-based spawn.
    """
    spawn_points = list(world.get_map().get_spawn_points())
    if not spawn_points:
        return None

    # Fallback: stable seed-based choice
    fallback = spawn_points[int(seed) % len(spawn_points)]

    try:
        spec = world.get_spectator()
        anchor = spec.get_transform().location
        ax = float(anchor.x)
        ay = float(anchor.y)
    except Exception:
        return fallback

    best = None
    best_d2 = None
    for t in spawn_points:
        dx = float(t.location.x) - ax
        dy = float(t.location.y) - ay
        d2 = dx * dx + dy * dy
        if best_d2 is None or d2 < best_d2:
            best_d2 = d2
            best = t

    return best or fallback

def compute_lane_keep_steering(world, vehicle_actor) -> float:
    map_object = world.get_map()

    try:
        vehicle_transform = vehicle_actor.get_transform()
        vehicle_location = vehicle_transform.location
    except Exception:
        return 0.0

    try:
        v = vehicle_actor.get_velocity()
        speed = math.sqrt(float(v.x * v.x + v.y * v.y))
    except Exception:
        speed = 0.0

    if speed < 0.5:
        return 0.0

    try:
        current_waypoint = map_object.get_waypoint(
            vehicle_location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
    except Exception:
        current_waypoint = None

    if current_waypoint is None:
        return 0.0

    try:
        next_waypoints = current_waypoint.next(float(LANE_KEEP_LOOKAHEAD_METERS))
    except Exception:
        next_waypoints = None

    if not next_waypoints:
        return 0.0

    target_location = next_waypoints[0].transform.location

    dx = float(target_location.x - vehicle_location.x)
    dy = float(target_location.y - vehicle_location.y)

    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return 0.0

    desired_heading = math.atan2(dy, dx)
    current_heading = math.radians(float(vehicle_transform.rotation.yaw))

    fx = math.cos(current_heading)
    fy = math.sin(current_heading)
    if (dx * fx + dy * fy) < 0.0:
        return 0.0

    heading_error = wrap_to_pi(desired_heading - current_heading)
    steering = float(LANE_KEEP_KP) * float(heading_error)
    return max(-1.0, min(1.0, steering))


def pick_spawn_transforms(world, actor_count: int, seed: int, spacing_meters: float, z_offset_meters: float):
    spawn_points = list(world.get_map().get_spawn_points())
    if not spawn_points:
        base_transform = carla.Transform(
            carla.Location(x=0.0, y=0.0, z=2.0 + float(z_offset_meters)),
            carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0),
        )
        return [base_transform for _ in range(actor_count)]

    base_spawn = spawn_points[int(seed) % len(spawn_points)]

    # HARD-CODED forward push: no env var, no knob
    base_spawn = advance_transform_forward_on_same_lane(world, base_spawn, 625.0)

    base_location = base_spawn.location
    base_rotation = base_spawn.rotation
    yaw_degrees = float(base_rotation.yaw)
    forward, _ = yaw_forward_and_right_vectors(yaw_degrees)

    base_x = float(base_location.x)
    base_y = float(base_location.y)
    base_z = float(base_location.z) + float(z_offset_meters)

    transforms = []
    for index in range(actor_count):
        back_distance = float(index) * float(spacing_meters)
        location_x = base_x - back_distance * forward[0]
        location_y = base_y - back_distance * forward[1]
        location_z = base_z

        transforms.append(
            carla.Transform(
                carla.Location(x=location_x, y=location_y, z=location_z),
                carla.Rotation(
                    pitch=float(base_rotation.pitch),
                    yaw=yaw_degrees,
                    roll=float(base_rotation.roll),
                ),
            )
        )

    log(
        "CARLA",
        "PLATOON_SPAWN_LAYOUT",
        "",
        actor_count=int(actor_count),
        spacing_m=float(spacing_meters),
        hardcoded_forward_shift_m=625.0,
        base_x=float(base_x),
        base_y=float(base_y),
        base_yaw=float(yaw_degrees),
    )

    return transforms

def advance_transform_forward_on_same_lane(world, base_transform, forward_distance_m: float):
    """
    Move a transform forward along the same driving lane by forward_distance_m.
    Falls back to the original transform if lane walking fails.
    """
    if forward_distance_m <= 0.0:
        return base_transform

    try:
        map_object = world.get_map()
        start_wp = map_object.get_waypoint(
            base_transform.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        if start_wp is None:
            return base_transform

        current_wp = start_wp
        remaining = float(forward_distance_m)
        step_m = 4.0

        while remaining > 1e-6:
            step = min(step_m, remaining)
            candidates = current_wp.next(step)
            if not candidates:
                break

            next_wp = None
            for wp in candidates:
                try:
                    if wp.road_id == current_wp.road_id and wp.lane_id == current_wp.lane_id:
                        next_wp = wp
                        break
                except Exception:
                    pass

            if next_wp is None:
                next_wp = candidates[0]

            current_wp = next_wp
            remaining -= step

        out = current_wp.transform

        # preserve caller's intended Z offset relative to the road waypoint
        out.location.z = float(out.location.z) + (
            float(base_transform.location.z) - float(start_wp.transform.location.z)
        )
        return out

    except Exception:
        return base_transform
    
def move_transform_along_same_lane(world, base_transform, signed_distance_m: float):
    """
    Move a transform along its current driving lane.
    Positive distance = forward on lane
    Negative distance = backward on lane

    Keeps the same travel direction semantics as the current lane.
    """
    if abs(float(signed_distance_m)) <= 1e-6:
        return base_transform

    try:
        map_object = world.get_map()
        start_wp = map_object.get_waypoint(
            base_transform.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        if start_wp is None:
            return base_transform

        current_wp = start_wp
        remaining = abs(float(signed_distance_m))
        step_m = 4.0

        while remaining > 1e-6:
            step = min(step_m, remaining)

            if signed_distance_m >= 0.0:
                candidates = current_wp.next(step)
            else:
                candidates = current_wp.previous(step)

            if not candidates:
                break

            next_wp = None
            for wp in candidates:
                try:
                    if wp.road_id == current_wp.road_id and wp.lane_id == current_wp.lane_id:
                        next_wp = wp
                        break
                except Exception:
                    pass

            if next_wp is None:
                next_wp = candidates[0]

            current_wp = next_wp
            remaining -= step

        out = current_wp.transform

        # Preserve the caller's relative Z offset above the lane centerline.
        out.location.z = float(out.location.z) + (
            float(base_transform.location.z) - float(start_wp.transform.location.z)
        )

        # Preserve the original yaw/pitch/roll so we do not accidentally flip direction.
        out.rotation = carla.Rotation(
            pitch=float(base_transform.rotation.pitch),
            yaw=float(base_transform.rotation.yaw),
            roll=float(base_transform.rotation.roll),
        )
        return out

    except Exception:
        return base_transform

# def _pick_same_lane_waypoint(candidates, ref_wp):
#     if not candidates:
#         return None
#     for w in candidates:
#         try:
#             if w.road_id == ref_wp.road_id and w.lane_id == ref_wp.lane_id:
#                 return w
#         except Exception:
#             continue
#     return candidates[0]


# def _circumradius_from_3_xy(p1, p2, p3, straight_radius=CURVATURE_RADIUS_STRAIGHT_M):
#     # p1/p2/p3 are (x,y) tuples
#     x1, y1 = p1
#     x2, y2 = p2
#     x3, y3 = p3

#     a = math.hypot(x2 - x3, y2 - y3)
#     b = math.hypot(x1 - x3, y1 - y3)
#     c = math.hypot(x1 - x2, y1 - y2)

#     s = 0.5 * (a + b + c)
#     A2 = s * (s - a) * (s - b) * (s - c)  # Heron inside sqrt

#     if A2 <= 1e-12:
#         return float(straight_radius)

#     A = math.sqrt(A2)
#     R = (a * b * c) / (4.0 * A)
#     if not math.isfinite(R) or R <= 0.0:
#         return float(straight_radius)
#     return float(R)


# def curvature_radius_from_carla_map(world, vehicle_actor, ds_m=CURVATURE_DS_METERS):
#     """
#     Estimate local lane-centerline curvature radius (meters) at vehicle location.
#     Uses 3 CARLA waypoints: previous(ds), current, next(ds) on same lane if possible.
#     Returns a finite number; uses CURVATURE_RADIUS_STRAIGHT_M when degenerate.
#     """
#     if world is None or vehicle_actor is None:
#         return float(CURVATURE_RADIUS_STRAIGHT_M)

#     try:
#         m = world.get_map()
#         loc = vehicle_actor.get_location()
#         wp0 = m.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
#     except Exception:
#         return float(CURVATURE_RADIUS_STRAIGHT_M)

#     if wp0 is None:
#         return float(CURVATURE_RADIUS_STRAIGHT_M)

#     try:
#         wpF = _pick_same_lane_waypoint(wp0.next(float(ds_m)), wp0)
#         wpB = _pick_same_lane_waypoint(wp0.previous(float(ds_m)), wp0)
#     except Exception:
#         return float(CURVATURE_RADIUS_STRAIGHT_M)

#     if wpF is None or wpB is None:
#         return float(CURVATURE_RADIUS_STRAIGHT_M)

#     pB = (float(wpB.transform.location.x), float(wpB.transform.location.y))
#     p0 = (float(wp0.transform.location.x), float(wp0.transform.location.y))
#     pF = (float(wpF.transform.location.x), float(wpF.transform.location.y))

#     R = _circumradius_from_3_xy(pB, p0, pF, straight_radius=CURVATURE_RADIUS_STRAIGHT_M)
#     if R < float(CURVATURE_RADIUS_MIN_M):
#         R = float(CURVATURE_RADIUS_MIN_M)
#     if R > float(CURVATURE_RADIUS_STRAIGHT_M):
#         R = float(CURVATURE_RADIUS_STRAIGHT_M)

#     return float(R)

class Bridge:
    def __init__(self):
        self.client = None
        self.world = None
        self.blueprint_library = None
        self.spectator = None

        self.actors_by_id = {}
        self.control_targets_by_actor_id = {}

        self.initialized = False
        self.fixed_delta_seconds = 0.05
        self.carla_seed = 0
        self.simulation_time_seconds = 0.0
        self.last_map_name = DEFAULT_MAP_NAME

        self.initial_base_transform = None
        self.original_world_settings = None

        self.step_index = 0
        self._forced_apply_log_step = -1

        self.control_update_pending_for_actor_ids = set()
        self.first_apply_logged_for_actor_ids = set()

        self.last_telemetry_print_time_seconds = -1e9
        self.last_step_logged_timestamp = None

        self.enable_leader_profile = bool(ENABLE_LEADER_PROFILE)
        self.enable_follower_fallback = False
        self.max_brake_command_mps2 = float(MAX_BRAKE_COMMAND_MPS2)
        self.follow_max_brake_mps2 = float(MAX_BRAKE_COMMAND_MPS2)
        self.hold_brake_until_control = bool(HOLD_BRAKE_UNTIL_CONTROL)

    # -------------------- INIT settling gate helpers --------------------
    @staticmethod
    def _park_vehicle_actor(actor):
        """
        Force a vehicle into a 'parked' state: no throttle, full brake, handbrake on, autopilot off.
        Safe to call repeatedly.
        """
        try:
            actor.set_autopilot(False)
        except Exception:
            pass

        try:
            actor.apply_control(
                carla.VehicleControl(
                    throttle=0.0,
                    brake=1.0,
                    steer=0.0,
                    hand_brake=True,
                    reverse=False,
                    manual_gear_shift=False,
                )
            )
        except Exception:
            pass

    def spawn_vehicle_from_hardcoded_town04_anchor(self, actor_id: str, actor_type: str, actor_configuration: dict):
        """
        Hard-coded late spawn for Town04 using the leader's INIT anchor captured from logs.
        """
        anchor = TOWN04_INIT_LEADER_ANCHOR

        yaw_degrees = float(anchor["yaw"])
        forward, _ = yaw_forward_and_right_vectors(yaw_degrees)

        base_x = float(anchor["x"])
        base_y = float(anchor["y"])
        base_z = float(anchor["z"])

        last_exception = None

        for extra_back in LATE_SPAWN_HARDCODED_EXTRA_BACKOFFS:
            back_m = float(LATE_SPAWN_HARDCODED_BACK_METERS) + float(extra_back)

            spawn_transform = carla.Transform(
                carla.Location(
                    x=base_x - back_m * forward[0],
                    y=base_y - back_m * forward[1],
                    z=base_z,
                ),
                carla.Rotation(
                    pitch=float(anchor["pitch"]),
                    yaw=float(anchor["yaw"]),
                    roll=float(anchor["roll"]),
                ),
            )

            log(
                "SPAWN",
                "LATE_SPAWN_LAYOUT",
                "",
                actor=actor_id,
                map_name="Town04",
                anchor="hardcoded_init_leader",
                base_x=base_x,
                base_y=base_y,
                base_z=base_z,
                base_yaw=float(anchor["yaw"]),
                back_m=float(back_m),
                spawn_x=float(spawn_transform.location.x),
                spawn_y=float(spawn_transform.location.y),
                spawn_z=float(spawn_transform.location.z),
                spawn_yaw=float(spawn_transform.rotation.yaw),
            )

            try:
                actor = self.spawn_vehicle_plexe_style(
                    actor_id,
                    actor_type,
                    actor_configuration,
                    spawn_transform,
                )
                return actor
            except Exception as exception:
                last_exception = exception

        raise RuntimeError(f"Failed to spawn actor '{actor_id}' from hardcoded Town04 anchor: {last_exception!r}")
    
    def spawn_vehicle_from_init_leader_anchor(self, actor_id: str, actor_type: str, actor_configuration: dict):
        """
        Spawn late-joining vehicles from the leader's ORIGINAL INIT anchor,
        not from the current rear vehicle transform.
        """
        if self.initial_base_transform is None:
            raise RuntimeError("initial leader spawn anchor is not available")

        yaw_degrees = float(self.initial_base_transform.rotation.yaw)
        forward, _ = yaw_forward_and_right_vectors(yaw_degrees)

        base_x = float(self.initial_base_transform.location.x)
        base_y = float(self.initial_base_transform.location.y)
        base_z = float(self.initial_base_transform.location.z)

        last_exception = None

        for extra_back in LATE_SPAWN_INIT_LEADER_EXTRA_BACKOFFS:
            back_m = float(LATE_SPAWN_INIT_LEADER_BACK_METERS) + float(extra_back)

            spawn_transform = carla.Transform(
                carla.Location(
                    x=base_x - back_m * forward[0],
                    y=base_y - back_m * forward[1],
                    z=base_z,
                ),
                carla.Rotation(
                    pitch=float(self.initial_base_transform.rotation.pitch),
                    yaw=float(self.initial_base_transform.rotation.yaw),
                    roll=float(self.initial_base_transform.rotation.roll),
                ),
            )

            log(
                "SPAWN",
                "LATE_SPAWN_LAYOUT",
                "",
                actor=actor_id,
                map_name=self.last_map_name,
                anchor="init_leader",
                base_x=base_x,
                base_y=base_y,
                base_z=base_z,
                base_yaw=float(yaw_degrees),
                back_m=float(back_m),
                spawn_x=float(spawn_transform.location.x),
                spawn_y=float(spawn_transform.location.y),
                spawn_z=float(spawn_transform.location.z),
                spawn_yaw=float(spawn_transform.rotation.yaw),
            )

            try:
                actor = self.spawn_vehicle_plexe_style(
                    actor_id,
                    actor_type,
                    actor_configuration,
                    spawn_transform,
                )
                return actor
            except Exception as exception:
                last_exception = exception

        raise RuntimeError(f"Failed to spawn actor '{actor_id}' from init leader anchor: {last_exception!r}")
    
    def _settle_spawned_actors_before_init_completed(
        self,
        *,
        max_ticks: int = None,
        stable_frames: int = None,
        vz_eps: float = None,
        dz_eps: float = None,
    ):
        """
        Tick CARLA a few times after spawn until vertical motion is effectively zero and Z is stable.
        This prevents INIT_COMPLETED from capturing the 'free-fall / drop' transient.
        """
        if self.world is None or not self.actors_by_id:
            return

        max_ticks = int(INIT_SETTLE_MAX_TICKS if max_ticks is None else max_ticks)
        stable_frames = int(INIT_SETTLE_STABLE_FRAMES if stable_frames is None else stable_frames)
        vz_eps = float(INIT_SETTLE_VZ_EPS if vz_eps is None else vz_eps)
        dz_eps = float(INIT_SETTLE_DZ_EPS if dz_eps is None else dz_eps)

        # Only vehicles (ignore sensors etc.)
        vehicles = []
        for a in self.actors_by_id.values():
            try:
                if getattr(a, "type_id", "").startswith("vehicle."):
                    vehicles.append(a)
            except Exception:
                continue

        if not vehicles:
            return

        log(
            "CARLA",
            "INIT_SETTLE_BEGIN",
            "",
            max_ticks=max_ticks,
            stable_frames=stable_frames,
            vz_eps=vz_eps,
            dz_eps=dz_eps,
            actor_count=len(vehicles),
        )

        last_z = {v.id: None for v in vehicles}
        stable_count = 0
        ticks_used = 0

        # Keep them hard-parked while settling
        for v in vehicles:
            self._park_vehicle_actor(v)

        for _ in range(max(1, max_ticks)):
            ticks_used += 1

            for v in vehicles:
                self._park_vehicle_actor(v)

            try:
                self.world.tick()
            except Exception:
                break

            all_stable = True
            for v in vehicles:
                try:
                    loc = v.get_location()
                    vel = v.get_velocity()
                except Exception:
                    all_stable = False
                    continue

                prev = last_z.get(v.id, None)
                if prev is None:
                    all_stable = False
                else:
                    dz = abs(float(loc.z) - float(prev))
                    if dz > dz_eps or abs(float(vel.z)) > vz_eps:
                        all_stable = False

                last_z[v.id] = float(loc.z)

            if all_stable:
                stable_count += 1
                if stable_count >= stable_frames:
                    break
            else:
                stable_count = 0

        # Optional but recommended: hard-zero residual motion before we snapshot INIT_COMPLETED
        for v in vehicles:
            try:
                if hasattr(v, "set_target_velocity"):
                    v.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
            except Exception:
                pass
            try:
                if hasattr(v, "set_target_angular_velocity"):
                    v.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
            except Exception:
                pass

            self._park_vehicle_actor(v)

        log(
            "CARLA",
            "INIT_SETTLE_END",
            "",
            ticks_used=ticks_used,
            stable_frames_reached=stable_count,
        )

    def compute_late_spawn_transform(self, actor_id: str):
        """
        Spawn a late joiner behind the CURRENT rear platoon vehicle,
        not behind the original t=0 spawn layout.
        """
        ordered_ids = self.sorted_vehicle_ids(list(self.actors_by_id.keys()))
        if not ordered_ids:
            if self.initial_base_transform is None:
                raise RuntimeError("No existing actors and no initial base transform available")
            return self.initial_base_transform

        rear_id = ordered_ids[-1]
        rear_actor = self.actors_by_id[rear_id]
        rear_tf = rear_actor.get_transform()

        back_m = float(LATE_SPAWN_BACK_METERS)
        if back_m <= 0.0:
            back_m = 10.0

        spawn_tf = move_transform_along_same_lane(self.world, rear_tf, -back_m)

        # Force exact heading match with the rear vehicle.
        spawn_tf = carla.Transform(
            carla.Location(
                x=float(spawn_tf.location.x),
                y=float(spawn_tf.location.y),
                z=float(spawn_tf.location.z),
            ),
            carla.Rotation(
                pitch=float(rear_tf.rotation.pitch),
                yaw=float(rear_tf.rotation.yaw),
                roll=float(rear_tf.rotation.roll),
            ),
        )

        log(
            "SPAWN",
            "LATE_SPAWN_LAYOUT",
            "",
            actor=actor_id,
            rear_actor=rear_id,
            back_m=float(back_m),
            rear_x=float(rear_tf.location.x),
            rear_y=float(rear_tf.location.y),
            rear_yaw=float(rear_tf.rotation.yaw),
            spawn_x=float(spawn_tf.location.x),
            spawn_y=float(spawn_tf.location.y),
            spawn_yaw=float(spawn_tf.rotation.yaw),
        )

        return spawn_tf
    
    def _pick_same_lane_waypoint(self, candidates, ref_wp):
        if not candidates:
            return None

        for wp in candidates:
            try:
                if wp.road_id == ref_wp.road_id and wp.lane_id == ref_wp.lane_id:
                    return wp
            except Exception:
                continue

        return candidates[0]

    def _walk_waypoint_backward_same_lane(self, start_wp, back_distance_m: float, step_m: float = 4.0):
        """
        Walk backward along the lane centerline starting from start_wp.
        Prefer staying on the same road_id/lane_id, but fall back to CARLA's first candidate if needed.
        """
        if start_wp is None:
            return None

        remaining = max(0.0, float(back_distance_m))
        current_wp = start_wp

        while remaining > 1e-6:
            step = min(float(step_m), remaining)
            try:
                candidates = current_wp.previous(step)
            except Exception:
                candidates = None

            if not candidates:
                break

            next_wp = self._pick_same_lane_waypoint(candidates, current_wp)
            if next_wp is None:
                break

            current_wp = next_wp
            remaining -= step

        return current_wp

    def _build_spawn_transform_from_waypoint(self, wp, z_offset_meters: float):
        if wp is None:
            return None

        wt = wp.transform
        return carla.Transform(
            carla.Location(
                x=float(wt.location.x),
                y=float(wt.location.y),
                z=float(wt.location.z) + float(z_offset_meters),
            ),
            carla.Rotation(
                pitch=float(wt.rotation.pitch),
                yaw=float(wt.rotation.yaw),
                roll=float(wt.rotation.roll),
            ),
        )

    def _compute_late_spawn_transform_on_lane(self, rear_actor, back_distance_m: float):
        """
        Compute a late-spawn transform by taking the rear actor's CURRENT lane waypoint,
        then walking backward along the lane centerline.
        """
        if self.world is None or rear_actor is None:
            return None

        try:
            rear_transform = rear_actor.get_transform()
            rear_location = rear_transform.location
            rear_yaw = float(rear_transform.rotation.yaw)
        except Exception:
            return None

        try:
            map_object = self.world.get_map()
            rear_wp = map_object.get_waypoint(
                rear_location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving,
            )
        except Exception:
            rear_wp = None

        if rear_wp is None:
            return None

        target_wp = self._walk_waypoint_backward_same_lane(rear_wp, float(back_distance_m), step_m=4.0)
        if target_wp is None:
            return None

        spawn_transform = self._build_spawn_transform_from_waypoint(target_wp, SPAWN_Z_OFFSET_METERS)
        if spawn_transform is None:
            return None

        log(
            "SPAWN",
            "LATE_SPAWN_LAYOUT",
            "",
            rear_actor=getattr(rear_actor, "attributes", {}).get("role_name", "unknown"),
            rear_x=float(rear_location.x),
            rear_y=float(rear_location.y),
            rear_yaw=float(rear_yaw),
            back_m=float(back_distance_m),
            spawn_x=float(spawn_transform.location.x),
            spawn_y=float(spawn_transform.location.y),
            spawn_yaw=float(spawn_transform.rotation.yaw),
        )

        return spawn_transform

    def _spawn_vehicle_late_on_lane(self, actor_id: str, actor_type: str, actor_configuration: dict,
                                    rear_actor, initial_back_m: float):
        """
        Try several same-lane backward offsets behind the current rear actor.
        This avoids collisions/occupancy failures while keeping placement lane-valid.
        """
        extra_backoffs = [0.0, 8.0, 16.0, 24.0, 32.0, 48.0]
        last_exception = None

        for extra in extra_backoffs:
            back_m = float(initial_back_m) + float(extra)
            spawn_transform = self._compute_late_spawn_transform_on_lane(rear_actor, back_m)
            if spawn_transform is None:
                continue

            try:
                actor = self.spawn_vehicle_plexe_style(
                    actor_id,
                    actor_type,
                    actor_configuration,
                    spawn_transform,
                )
                return actor
            except Exception as exception:
                last_exception = exception

        raise RuntimeError(f"Failed to spawn actor '{actor_id}' on lane behind rear actor: {last_exception!r}")

    # -------------------- protocol response helpers --------------------
    @staticmethod
    def resp_init_completed(initial_timestamp_seconds, actor_positions, simulation_status, status: int = 0):
        return {
            "message_type": "INIT_COMPLETED",
            "status": int(status),
            "initial_timestamp": float(initial_timestamp_seconds),
            "actor_positions": actor_positions,
            "simulation_status": int(simulation_status),
        }

    @staticmethod
    def resp_updated_positions(actor_positions, simulation_status, status: int = 0):
        return {
            "message_type": "UPDATED_POSITIONS",
            "status": int(status),
            "actor_positions": actor_positions,
            "simulation_status": int(simulation_status),
        }

    @staticmethod
    def resp_generic(user_defined, simulation_status, status: int = 0):
        return {
            "message_type": "GENERIC_RESPONSE",
            "status": int(status),
            "user_defined": user_defined,
            "simulation_status": int(simulation_status),
        }

    # -------------------- CARLA connectivity --------------------
    @staticmethod
    def ensure_carla_imported():
        if carla is None:
            raise RuntimeError(f"CARLA Python API import failed: {_CARLA_IMPORT_ERROR!r}")

    def connect_to_carla_with_retry(self, retry_seconds: float = 6.0):
        self.ensure_carla_imported()

        deadline = time.time() + max(0.0, float(retry_seconds))
        last_exception = None
        while time.time() <= deadline:
            try:
                client = carla.Client(CARLA_HOST, CARLA_PORT)
                client.set_timeout(float(CARLA_RPC_TIMEOUT_SECONDS))
                _ = client.get_server_version()
                self.client = client
                log("CARLA", "CONNECTED", "", host=CARLA_HOST, port=CARLA_PORT)
                return
            except Exception as exception:
                last_exception = exception
                time.sleep(0.25)

        raise RuntimeError(f"Failed to connect to CARLA at {CARLA_HOST}:{CARLA_PORT}: {last_exception!r}")

    def load_world(self, desired_map_name: str):
        assert self.client is not None
        world = self.client.get_world()

        if desired_map_name:
            if RELOAD_WORLD_ON_INIT:
                log("CARLA", "WORLD_RELOAD", "", map_name=desired_map_name)
                world = self.client.load_world(desired_map_name)
            else:
                try:
                    current_map = world.get_map().name or ""
                except Exception:
                    current_map = ""
                if (not current_map) or (desired_map_name not in current_map):
                    log("CARLA", "WORLD_LOAD", "", map_name=desired_map_name)
                    world = self.client.load_world(desired_map_name)

        self.world = world
        self.blueprint_library = world.get_blueprint_library()
        self.spectator = world.get_spectator()
        self.last_map_name = desired_map_name

        try:
            self.original_world_settings = self.world.get_settings()
        except Exception:
            self.original_world_settings = None

    def place_spectator_behind_platoon(self):
        if not SPECTATOR_ENABLED:
            return
        if self.spectator is None or not self.actors_by_id:
            return

        leader = self.actors_by_id.get("veh0") or next(iter(self.actors_by_id.values()), None)
        if leader is None:
            return

        try:
            leader_transform = leader.get_transform()
        except Exception:
            return

        yaw_degrees = float(leader_transform.rotation.yaw)
        yaw_radians = math.radians(yaw_degrees)
        forward_x = math.cos(yaw_radians)
        forward_y = math.sin(yaw_radians)

        xs, ys, zs = [], [], []
        rear_projection = None
        rear_xy = None
        max_z = None

        for actor in self.actors_by_id.values():
            try:
                t = actor.get_transform()
                x = float(t.location.x)
                y = float(t.location.y)
                z = float(t.location.z)

                xs.append(x); ys.append(y); zs.append(z)

                proj = x * forward_x + y * forward_y
                if rear_projection is None or proj < rear_projection:
                    rear_projection = proj
                    rear_xy = (x, y)

                max_z = z if (max_z is None or z > max_z) else max_z
            except Exception:
                continue

        if not xs or rear_xy is None or max_z is None:
            return

        span = max((max(xs) - min(xs)), (max(ys) - min(ys)))
        height = max(float(SPECTATOR_MIN_UP_METERS), float(SPECTATOR_MIN_UP_METERS) + float(SPECTATOR_HEIGHT_GAIN) * span)
        back = max(float(SPECTATOR_MIN_BACK_METERS), float(SPECTATOR_MIN_BACK_METERS) + float(SPECTATOR_BACK_GAIN) * span)

        rear_x, rear_y = rear_xy
        cam_x = rear_x - forward_x * back
        cam_y = rear_y - forward_y * back
        cam_z = float(max_z) + height

        try:
            self.spectator.set_transform(
                carla.Transform(
                    carla.Location(x=cam_x, y=cam_y, z=cam_z),
                    carla.Rotation(pitch=float(SPECTATOR_PITCH_DEGREES), yaw=yaw_degrees, roll=0.0),
                )
            )
            log("CARLA", "SPECTATOR_PLACED", "", actor="platoon")
        except Exception:
            pass

    def handle_spawn_actor(self, message: dict):
        def _resp_spawn_completed(ts: float, actor_positions, simulation_status: int, status: int = 0, error: str = ""):
            return {
                "message_type": "SPAWN_COMPLETED",
                "timestamp": float(ts),
                "status": int(status),
                "error": str(error) if error is not None else "",
                "actor_positions": actor_positions if isinstance(actor_positions, list) else [],
                "simulation_status": int(simulation_status),
            }

        if not self.initialized or self.world is None:
            ts = safe_float(message.get("timestamp", self.simulation_time_seconds), self.simulation_time_seconds)
            return _resp_spawn_completed(ts, [], SIM_STATUS_ERROR, status=-1, error="SPAWN_ACTOR before INIT")

        self.simulation_time_seconds = safe_float(
            message.get("timestamp", self.simulation_time_seconds),
            self.simulation_time_seconds,
        )
        ts = float(self.simulation_time_seconds)

        actor = message.get("actor", {}) or {}
        if not isinstance(actor, dict):
            return _resp_spawn_completed(ts, [], SIM_STATUS_RUNNING, status=-1, error="actor must be an object")

        actor_id = str(actor.get("actor_id", "")).strip()
        actor_type = str(actor.get("actor_type", "")).strip() or "car"
        actor_cfg = actor.get("actor_configuration", {}) or {}
        if not isinstance(actor_cfg, dict):
            actor_cfg = {}

        if not actor_id:
            return _resp_spawn_completed(ts, [], SIM_STATUS_RUNNING, status=-1, error="missing actor.actor_id")

        if actor_id in self.actors_by_id:
            snap = self.snapshot_actor(actor_id, self.actors_by_id[actor_id])
            return _resp_spawn_completed(ts, [snap], SIM_STATUS_RUNNING, status=0, error="")

        try:
            new_actor = None
            spawn_errors = []

            ordered_ids = self.sorted_vehicle_ids(list(self.actors_by_id.keys()))

            # Primary path: spawn behind the CURRENT rear platoon vehicle, lane-valid.
            if ordered_ids:
                rear_id = ordered_ids[-1]
                rear_actor = self.actors_by_id.get(rear_id)

                if rear_actor is not None:
                    initial_back_m = float(LATE_SPAWN_BACK_METERS)
                    if initial_back_m <= 0.0:
                        base_spacing = float(getattr(self, "initial_spacing_m", PLATOON_SPACING_METERS))
                        initial_back_m = max(40.0, 4.0 * base_spacing)

                    try:
                        new_actor = self._spawn_vehicle_late_on_lane(
                            actor_id,
                            actor_type,
                            actor_cfg,
                            rear_actor,
                            initial_back_m,
                        )
                    except Exception as exception:
                        spawn_errors.append(f"rear_lane_spawn={type(exception).__name__}: {exception}")

            # Fallback 1: behind the original INIT leader anchor.
            if new_actor is None and LATE_SPAWN_USE_INIT_LEADER_ANCHOR and self.initial_base_transform is not None:
                try:
                    new_actor = self.spawn_vehicle_from_init_leader_anchor(
                        actor_id,
                        actor_type,
                        actor_cfg,
                    )
                except Exception as exception:
                    spawn_errors.append(f"init_leader_anchor={type(exception).__name__}: {exception}")

            # Fallback 2: hardcoded Town04 anchor.
            if (
                new_actor is None
                and self.last_map_name == LATE_SPAWN_ONLY_FOR_MAP
                and LATE_SPAWN_USE_HARDCODED_TOWN04_ANCHOR
            ):
                try:
                    new_actor = self.spawn_vehicle_from_hardcoded_town04_anchor(
                        actor_id,
                        actor_type,
                        actor_cfg,
                    )
                except Exception as exception:
                    spawn_errors.append(f"hardcoded_town04_anchor={type(exception).__name__}: {exception}")

            if new_actor is None:
                detail = " | ".join(spawn_errors) if spawn_errors else "no spawn strategy succeeded"
                raise RuntimeError(detail)

            try:
                if hasattr(new_actor, "set_target_velocity"):
                    new_actor.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
            except Exception:
                pass

            try:
                if hasattr(new_actor, "set_target_angular_velocity"):
                    new_actor.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
            except Exception:
                pass

            self._park_vehicle_actor(new_actor)

            self.actors_by_id[actor_id] = new_actor

            self.control_targets_by_actor_id[actor_id] = {
                "has_control": (not self.hold_brake_until_control),
                "desired_acceleration": 0.0,
                "desired_speed": 0.0,
            }

            self.control_update_pending_for_actor_ids.add(actor_id)
            self.first_apply_logged_for_actor_ids.discard(actor_id)

            snap = self.snapshot_actor(actor_id, new_actor)
            log("SPAWN", "LATE_SPAWNED", "", actor=actor_id, actor_type=actor_type, simulation_time=ts)
            return _resp_spawn_completed(ts, [snap], SIM_STATUS_RUNNING, status=0, error="")

        except Exception as e:
            return _resp_spawn_completed(ts, [], SIM_STATUS_RUNNING, status=-1, error=f"{type(e).__name__}: {e}")
        
    def apply_synchronous_settings(self):
        assert self.world is not None
        current_settings = self.world.get_settings()
        new_settings = carla.WorldSettings(
            synchronous_mode=True,
            fixed_delta_seconds=float(self.fixed_delta_seconds),
            no_rendering_mode=current_settings.no_rendering_mode,
            max_substep_delta_time=current_settings.max_substep_delta_time,
            max_substeps=current_settings.max_substeps,
        )
        self.world.apply_settings(new_settings)
        log("CARLA", "SYNC_MODE_ENABLED", "", carla_fixed_delta=self.fixed_delta_seconds)

        try:
            self.world.tick()
        except Exception:
            pass

    def restore_original_settings(self):
        if self.world is None:
            return
        try:
            current_settings = self.world.get_settings()
            if self.original_world_settings is not None:
                self.world.apply_settings(self.original_world_settings)
            else:
                fallback = carla.WorldSettings(
                    synchronous_mode=False,
                    fixed_delta_seconds=None,
                    no_rendering_mode=current_settings.no_rendering_mode,
                    max_substep_delta_time=current_settings.max_substep_delta_time,
                    max_substeps=current_settings.max_substeps,
                )
                self.world.apply_settings(fallback)

            if FORCE_ASYNC_ON_RESTORE:
                refreshed = self.world.get_settings()
                if getattr(refreshed, "synchronous_mode", False):
                    forced = carla.WorldSettings(
                        synchronous_mode=False,
                        fixed_delta_seconds=None,
                        no_rendering_mode=refreshed.no_rendering_mode,
                        max_substep_delta_time=refreshed.max_substep_delta_time,
                        max_substeps=refreshed.max_substeps,
                    )
                    self.world.apply_settings(forced)

            log("CARLA", "SETTINGS_RESTORED", "")
        except Exception as exception:
            log("CARLA", "SETTINGS_RESTORE_FAILED", "", error=str(exception))

    def tick_or_wait(self, timeout_seconds: float = 1.0):
        if self.world is None:
            return
        try:
            settings = self.world.get_settings()
            if getattr(settings, "synchronous_mode", False):
                self.world.tick()
            else:
                self.world.wait_for_tick(float(timeout_seconds))
        except Exception:
            pass

    def destroy_world_owned_actors(self, role_prefixes):
        if self.client is None or self.world is None:
            return

        prefixes = tuple(str(prefix) for prefix in (role_prefixes or ()))
        if not prefixes:
            return

        try:
            actors = self.world.get_actors()
        except Exception:
            return

        owned_vehicle_ids = set()
        destroy_ids = set()

        for actor in actors:
            try:
                role_name = (actor.attributes.get("role_name") or "")
            except Exception:
                role_name = ""
            if role_name and any(role_name.startswith(prefix) for prefix in prefixes):
                destroy_ids.add(actor.id)
                if actor.type_id.startswith("vehicle."):
                    owned_vehicle_ids.add(actor.id)

        for actor in actors:
            if not actor.type_id.startswith("sensor."):
                continue
            parent = None
            try:
                if hasattr(actor, "get_parent"):
                    parent = actor.get_parent()
            except Exception:
                parent = None
            if parent is not None and getattr(parent, "actor_id", None) in owned_vehicle_ids:
                destroy_ids.add(actor.id)

        if not destroy_ids:
            return

        log("CARLA", "DESTROY_OLD_ACTORS", "", prefixes=",".join(prefixes), count=len(destroy_ids))

        commands = [carla.command.DestroyActor(actor_id) for actor_id in destroy_ids]
        try:
            self.client.apply_batch_sync(commands, True)
        except Exception:
            for actor_id in destroy_ids:
                try:
                    actor = self.world.get_actor(actor_id)
                    if actor is not None:
                        actor.destroy()
                except Exception:
                    pass

        self.tick_or_wait(timeout_seconds=1.0)

    def destroy_tracked_actors(self):
        for _, actor in list(self.actors_by_id.items()):
            try:
                actor.destroy()
            except Exception:
                pass
        self.actors_by_id.clear()
        self.control_targets_by_actor_id.clear()
        self.initialized = False

    # -------------------- spawn helpers --------------------
    def select_vehicle_blueprint(self, actor_type: str, actor_configuration: dict, actor_id: str):
        assert self.blueprint_library is not None

        candidate_blueprint = ""
        if isinstance(actor_configuration, dict):
            blueprint_value = actor_configuration.get("blueprint")
            if isinstance(blueprint_value, str) and blueprint_value.strip():
                candidate_blueprint = blueprint_value.strip()
        if not candidate_blueprint and isinstance(actor_type, str) and actor_type.strip():
            candidate_blueprint = actor_type.strip()

        blueprint = None
        if candidate_blueprint:
            if "*" in candidate_blueprint:
                filtered = self.blueprint_library.filter(candidate_blueprint)
                if filtered:
                    blueprint = filtered[0]
            else:
                try:
                    blueprint = self.blueprint_library.find(candidate_blueprint)
                except Exception:
                    filtered = self.blueprint_library.filter(candidate_blueprint)
                    if filtered:
                        blueprint = filtered[0]

        if blueprint is None:
            try:
                blueprint = self.blueprint_library.find(DEFAULT_VEHICLE_BLUEPRINT)
            except Exception:
                filtered = self.blueprint_library.filter("vehicle.*")
                if not filtered:
                    raise RuntimeError("No vehicle blueprints available in CARLA")
                blueprint = filtered[0]

        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", actor_id)

        return blueprint

    def spawn_vehicle_plexe_style(self, actor_id: str, actor_type: str, actor_configuration: dict, base_transform):
        assert self.world is not None
        blueprint = self.select_vehicle_blueprint(actor_type, actor_configuration, actor_id)

        yaw_degrees = float(base_transform.rotation.yaw)
        _, right = yaw_forward_and_right_vectors(yaw_degrees)

        last_exception = None
        for attempt in range(max(1, TRY_SPAWN_RETRIES)):
            lateral_try_index = attempt % int(PLATOON_MAX_LANESIDE_TRIES)

            if lateral_try_index == 0:
                lateral_offset = 0.0
            else:
                step_index = (lateral_try_index + 1) // 2
                lateral_offset = float(step_index) * float(PLATOON_LATERAL_JITTER_METERS)
                lateral_offset *= (1.0 if (lateral_try_index % 2 == 1) else -1.0)

            offset_x = lateral_offset * right[0]
            offset_y = lateral_offset * right[1]

            candidate_transform = carla.Transform(
                carla.Location(
                    x=float(base_transform.location.x) + offset_x,
                    y=float(base_transform.location.y) + offset_y,
                    z=float(base_transform.location.z),
                ),
                carla.Rotation(
                    pitch=float(base_transform.rotation.pitch),
                    yaw=float(base_transform.rotation.yaw),
                    roll=float(base_transform.rotation.roll),
                ),
            )

            try:
                actor = self.world.try_spawn_actor(blueprint, candidate_transform)
                if actor is not None:
                    try:
                        actor.set_autopilot(False)
                    except Exception:
                        pass
                    try:
                        if hasattr(actor, "set_simulate_physics"):
                            actor.set_simulate_physics(True)
                    except Exception:
                        pass
                    return actor
            except Exception as exception:
                last_exception = exception

        raise RuntimeError(f"Failed to spawn actor '{actor_id}': {last_exception!r}")

    @staticmethod
    def snapshot_actor(actor_id: str, actor):
        transform = actor.get_transform()
        velocity = actor.get_velocity()
        return {
            "actor_id": actor_id,
            "position": [float(transform.location.x), float(transform.location.y), float(transform.location.z)],
            "velocity": [float(velocity.x), float(velocity.y), float(velocity.z)],
            "rotation": [float(transform.rotation.pitch), float(transform.rotation.yaw), float(transform.rotation.roll)],
            "is_net_active": True,
        }

    @staticmethod
    def speed_mps(actor) -> float:
        try:
            velocity = actor.get_velocity()
            return math.sqrt(float(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z))
        except Exception:
            return 0.0

    @staticmethod
    def sorted_vehicle_ids(actor_ids):
        def sort_key(actor_id: str):
            if actor_id.startswith("veh") and actor_id[3:].isdigit():
                return (0, int(actor_id[3:]))
            return (1, actor_id)
        return sorted(actor_ids, key=sort_key)

    def gap_relative_speed_headway_to_leader(self, ordered_actor_ids: list, index: int):
        if index <= 0:
            return None, None, None

        follower_id = ordered_actor_ids[index]
        leader_id = ordered_actor_ids[index - 1]
        follower = self.actors_by_id.get(follower_id)
        leader = self.actors_by_id.get(leader_id)
        if follower is None or leader is None:
            return None, None, None

        try:
            follower_location = follower.get_transform().location
            leader_location = leader.get_transform().location
            dx = float(leader_location.x - follower_location.x)
            dy = float(leader_location.y - follower_location.y)
            gap = math.sqrt(dx * dx + dy * dy)

            follower_speed = self.speed_mps(follower)
            leader_speed = self.speed_mps(leader)
            relative_speed = follower_speed - leader_speed

            headway = (gap / follower_speed) if follower_speed > 0.1 else None
            return gap, relative_speed, headway
        except Exception:
            return None, None, None

    # def snapshot_actor_with_radius(self, actor_id: str, actor):
    #     snap = Bridge.snapshot_actor(actor_id, actor)  # keep existing fields
    #     try:
    #         snap["radius"] = float(curvature_radius_from_carla_map(self.world, actor, CURVATURE_DS_METERS))
    #     except Exception:
    #         snap["radius"] = float(CURVATURE_RADIUS_STRAIGHT_M)
    #     return snap

    # -------------------- control --------------------
    def acceleration_command_from_target(self, actor, target_control: dict) -> float:
        if not isinstance(target_control, dict):
            return 0.0

        if "desired_acceleration" in target_control:
            return safe_float(target_control.get("desired_acceleration", 0.0), 0.0)

        if "desired_speed" in target_control:
            desired_speed = safe_float(target_control.get("desired_speed", 0.0), 0.0)
            current_speed = self.speed_mps(actor)
            acceleration_command = float(SPEED_ERROR_TO_ACCELERATION_KP) * (desired_speed - current_speed)

            if acceleration_command > float(MAX_ACCELERATION_COMMAND_MPS2):
                acceleration_command = float(MAX_ACCELERATION_COMMAND_MPS2)
            if acceleration_command < float(self.max_brake_command_mps2):
                acceleration_command = float(self.max_brake_command_mps2)

            return acceleration_command

        return 0.0

    def apply_controls(self):
        ordered_actor_ids = self.sorted_vehicle_ids(list(self.actors_by_id.keys()))

        for index, actor_id in enumerate(ordered_actor_ids):
            actor = self.actors_by_id[actor_id]

            target_control = self.control_targets_by_actor_id.get(actor_id)
            if target_control is None:
                target_control = {
                    "has_control": (not self.hold_brake_until_control),
                    "desired_acceleration": 0.0,
                    "desired_speed": 0.0,
                }
                self.control_targets_by_actor_id[actor_id] = target_control

            steering = 0.0
            if ENABLE_LANE_KEEPING and self.world is not None:
                try:
                    steering = compute_lane_keep_steering(self.world, actor)
                except Exception:
                    steering = 0.0

            desired_acceleration = safe_float(target_control.get("desired_acceleration", 0.0), 0.0)
            desired_speed = safe_float(target_control.get("desired_speed", 0.0), 0.0)
            acceleration_command = self.acceleration_command_from_target(actor, target_control)

            hold_brake_active = False
            if self.hold_brake_until_control and (not bool(target_control.get("has_control", False))):
                hold_brake_active = True

            if hold_brake_active:
                throttle = 0.0
                brake = 1.0
            else:
                if acceleration_command >= 0.0:
                    throttle = clamp(acceleration_command * THROTTLE_GAIN_PER_MPS2)
                    brake = 0.0
                else:
                    throttle = 0.0
                    brake = clamp((-acceleration_command) * BRAKE_GAIN_PER_MPS2)

                current_speed = self.speed_mps(actor)
                if current_speed < float(MINIMUM_LAUNCH_SPEED_MPS) and throttle > 0.0:
                    throttle = max(float(throttle), float(MINIMUM_LAUNCH_THROTTLE))
                    brake = 0.0

            speed = self.speed_mps(actor)
            gap, relv, headway = self.gap_relative_speed_headway_to_leader(ordered_actor_ids, index)

            # If controller target is STOP, actively hold the vehicle stopped.
            # Do not allow zero-speed/zero-accel targets to become a "coast" command.
            if (not hold_brake_active):
                if abs(desired_speed) < 1e-3 and abs(desired_acceleration) < 1e-3:
                    throttle = 0.0
                    brake = 1.0
                    hold_brake_active = True
                    
            if LOG_ALL:
                should_log_apply = True
            else:
                force_this_step = False
                if ordered_actor_ids:
                    leader_id = ordered_actor_ids[0]
                    if self._forced_apply_log_step != self.step_index and actor_id == leader_id:
                        force_this_step = True
                        self._forced_apply_log_step = self.step_index

                should_log_apply = (
                    force_this_step
                    or (actor_id in self.control_update_pending_for_actor_ids)
                    or (actor_id not in self.first_apply_logged_for_actor_ids)
                )

            if should_log_apply:
                self.control_update_pending_for_actor_ids.discard(actor_id)
                self.first_apply_logged_for_actor_ids.add(actor_id)

                log(
                    "CARLA",
                    "CONTROL_APPLIED",
                    "",
                    step=int(self.step_index),
                    actor=actor_id,
                    simulation_time=float(self.simulation_time_seconds),
                    desired_speed=float(desired_speed),
                    desired_acceleration=float(desired_acceleration),
                    acceleration_command=float(acceleration_command),
                    throttle=float(throttle),
                    brake=float(brake),
                    steering=float(steering),
                    hold_brake_active=bool(hold_brake_active),
                    speed=float(speed),
                    gap_to_leader=gap,
                    relative_speed_to_leader=relv,
                    headway=headway,
                    hop="PY_APPLY_TO_CARLA",
                )

            try:
                actor.apply_control(
                    carla.VehicleControl(
                        throttle=float(throttle),
                        brake=float(brake),
                        steer=float(steering),
                        hand_brake=bool(hold_brake_active),
                        reverse=False,
                        manual_gear_shift=False,
                    )
                )
            except Exception:
                pass

        if TELEMETRY_ENABLED and (
            float(self.simulation_time_seconds) - float(self.last_telemetry_print_time_seconds) >= float(TELEMETRY_PERIOD_SECONDS)
        ):
            self.last_telemetry_print_time_seconds = float(self.simulation_time_seconds)
            for index, actor_id in enumerate(ordered_actor_ids):
                actor = self.actors_by_id.get(actor_id)
                if actor is None:
                    continue
                speed = self.speed_mps(actor)
                gap, relv, headway = self.gap_relative_speed_headway_to_leader(ordered_actor_ids, index)
                log(
                    "TELEMETRY",
                    "STATE",
                    "",
                    actor=actor_id,
                    simulation_time=float(self.simulation_time_seconds),
                    speed=float(speed),
                    gap_to_leader=gap,
                    relative_speed_to_leader=relv,
                    headway=headway,
                )

    def freeze_all_vehicles(self):
        for actor_id, actor in self.actors_by_id.items():
            try:
                actor.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0,
                        hand_brake=True,
                        reverse=False,
                        manual_gear_shift=False,
                    )
                )
                log("CONTROL", "VEHICLE_FROZEN", "", actor=actor_id)
            except Exception:
                pass

    def shutdown(self):
        if self.world is None:
            return
        banner("SYSTEM", "SHUTDOWN")
        if FREEZE_VEHICLES_ON_FINISH:
            self.freeze_all_vehicles()
        if DESTROY_OLD_ACTORS_ON_SHUTDOWN:
            self.destroy_tracked_actors()
            self.destroy_world_owned_actors(DESTROY_ROLE_PREFIXES)
        self.restore_original_settings()
        self.initialized = False

    # -------------------- protocol handlers --------------------
    def handle_init(self, message: dict):
        self.simulation_time_seconds = safe_float(message.get("timestamp", 0.0), 0.0)

        self.step_index = 0
        self._forced_apply_log_step = -1

        carla_configuration = message.get("carla_configuration", {}) or {}
        if not isinstance(carla_configuration, dict):
            carla_configuration = {}

        self.carla_seed = safe_int(carla_configuration.get("seed", 0), 0)
        self.fixed_delta_seconds = safe_float(carla_configuration.get("carla_timestep", 0.05), 0.05)
        if self.fixed_delta_seconds <= 0.0:
            self.fixed_delta_seconds = 0.05

        user_defined = message.get("user_defined", {}) or {}
        if not isinstance(user_defined, dict):
            raise ValueError("INIT.user_defined must be an object")

        if "map" not in user_defined:
            raise ValueError("INIT.user_defined.map is required")
        desired_map_name = str(user_defined.get("map", "")).strip()
        if not desired_map_name:
            raise ValueError("INIT.user_defined.map must be non-empty")

        banner(
            "PROTOCOL",
            "INIT_RECEIVED",
            simulation_time=float(self.simulation_time_seconds),
            map_name=desired_map_name,
            carla_fixed_delta=float(self.fixed_delta_seconds),
            carla_seed=int(self.carla_seed),
        )

        if "moving_actors" not in message:
            raise ValueError("INIT.moving_actors is required")
        actors = message.get("moving_actors")
        if not isinstance(actors, list):
            raise ValueError("INIT.moving_actors must be a list")

        # Stable ordering: veh0, veh1, veh2, ... (prevents slot drift)
        def _actor_sort_key(e):
            aid = str(e.get("actor_id", ""))
            if aid.startswith("veh") and aid[3:].isdigit():
                return (0, int(aid[3:]))
            return (1, aid)
        actors = sorted(actors, key=_actor_sort_key)

        self.connect_to_carla_with_retry(retry_seconds=6.0)
        self.load_world(desired_map_name)

        self.destroy_tracked_actors()
        if DESTROY_OLD_ACTORS_ON_INIT:
            self.destroy_world_owned_actors(DESTROY_ROLE_PREFIXES)

        self.apply_synchronous_settings()

        spawn_transforms = pick_spawn_transforms(
            self.world,
            actor_count=len(actors),
            seed=self.carla_seed,
            spacing_meters=PLATOON_SPACING_METERS,
            z_offset_meters=SPAWN_Z_OFFSET_METERS,
        )
        self.initial_base_transform = spawn_transforms[0] if spawn_transforms else None

        if self.initial_base_transform is not None:
            log(
                "SPAWN",
                "INIT_LEADER_ANCHOR",
                "",
                map_name=desired_map_name,
                x=float(self.initial_base_transform.location.x),
                y=float(self.initial_base_transform.location.y),
                z=float(self.initial_base_transform.location.z),
                yaw=float(self.initial_base_transform.rotation.yaw),
                pitch=float(self.initial_base_transform.rotation.pitch),
                roll=float(self.initial_base_transform.rotation.roll),
            )
        
        self.initial_spacing_m = float(PLATOON_SPACING_METERS)

        self.control_update_pending_for_actor_ids.clear()
        self.first_apply_logged_for_actor_ids.clear()

        spawned_actor_ids = []
        for index, actor_entry in enumerate(actors):
            if not isinstance(actor_entry, dict):
                raise ValueError(f"INIT.moving_actors[{index}] must be an object")

            actor_id = str(actor_entry.get("actor_id", "")).strip()
            actor_type = str(actor_entry.get("actor_type", "")).strip()
            actor_configuration = actor_entry.get("actor_configuration", {}) or {}
            if not isinstance(actor_configuration, dict):
                actor_configuration = {}

            if not actor_id:
                raise ValueError(f"Missing actor_id at moving_actors[{index}]")

            actor = self.spawn_vehicle_plexe_style(actor_id, actor_type, actor_configuration, spawn_transforms[index])
            self.actors_by_id[actor_id] = actor
            self.control_targets_by_actor_id[actor_id] = {
                "has_control": (not self.hold_brake_until_control),
                "desired_acceleration": 0.0,
                "desired_speed": 0.0,
            }
            spawned_actor_ids.append(actor_id)

            log(
                "SPAWN",
                "SPAWNED",
                "",
                actor=actor_id,
                actor_type=(actor_type or "car"),
                simulation_time=float(self.simulation_time_seconds),
            )

        if INIT_SETTLE_GATE_ENABLED:
            self._settle_spawned_actors_before_init_completed()

        self.apply_controls()
        try:
            self.world.tick()
            self.apply_controls()
            self.world.tick()
        except Exception:
            pass

        self.place_spectator_behind_platoon()

        self.initialized = True
        banner(
            "PROTOCOL",
            "INIT_COMPLETED",
            actors=",".join(self.sorted_vehicle_ids(spawned_actor_ids)),
        )

        log(
            "CARLA",
            "STATE_READ",
            "",
            hop="CARLA_TO_PY",
            simulation_time=float(self.simulation_time_seconds),
            message_type="INIT_COMPLETED",
            actors=",".join(self.sorted_vehicle_ids(list(self.actors_by_id.keys()))),
        )

        ordered_ids = self.sorted_vehicle_ids(list(self.actors_by_id.keys()))
        actor_positions = [self.snapshot_actor(actor_id, self.actors_by_id[actor_id]) for actor_id in ordered_ids]
        return self.resp_init_completed(float(self.simulation_time_seconds), actor_positions, SIM_STATUS_RUNNING)

    def handle_simulation_step(self, message: dict):
        if not self.initialized or self.world is None:
            log("PROTOCOL", "SIMULATION_STEP_IGNORED", "")
            return self.resp_updated_positions([], SIM_STATUS_ERROR)

        self.simulation_time_seconds = safe_float(
            message.get("timestamp", self.simulation_time_seconds + self.fixed_delta_seconds),
            self.simulation_time_seconds + self.fixed_delta_seconds,
        )

        self.step_index += 1

        self.apply_controls()
        self.world.tick()

        log(
            "CARLA",
            "STATE_READ",
            "",
            hop="CARLA_TO_PY",
            simulation_time=float(self.simulation_time_seconds),
            message_type="UPDATED_POSITIONS",
            actors=",".join(self.sorted_vehicle_ids(list(self.actors_by_id.keys()))),
        )

        ordered_ids = self.sorted_vehicle_ids(list(self.actors_by_id.keys()))
        actor_positions = [self.snapshot_actor(actor_id, self.actors_by_id[actor_id]) for actor_id in ordered_ids]
        # actor_positions = [self.snapshot_actor_with_radius(actor_id, self.actors_by_id[actor_id]) for actor_id in ordered_ids]
        return self.resp_updated_positions(actor_positions, SIM_STATUS_RUNNING)

    def ingest_control_for_actor(
        self,
        actor_id: str,
        control_payload: dict,
        *,
        source_module=None,
        sequence=None,
        hop=None
    ) -> bool:
        if actor_id not in self.actors_by_id:
            return False

        if not isinstance(control_payload, dict):
            control_payload = {}

        has_accel_key = "desired_acceleration" in control_payload
        has_speed_key = "desired_speed" in control_payload

        desired_acceleration_value = control_payload.get("desired_acceleration", None)
        desired_speed_value = control_payload.get("desired_speed", None)
        explicit_has_control = control_payload.get("has_control", None)

        previous = self.control_targets_by_actor_id.get(actor_id)
        if not isinstance(previous, dict):
            previous = {}

        new_target = dict(previous)

        if explicit_has_control is not None:
            new_target["has_control"] = bool(explicit_has_control)
        else:
            new_target["has_control"] = bool(has_accel_key or has_speed_key)

        if has_accel_key and desired_acceleration_value is not None:
            new_target["desired_acceleration"] = safe_float(desired_acceleration_value, 0.0)

        if has_speed_key and desired_speed_value is not None:
            new_target["desired_speed"] = safe_float(desired_speed_value, 0.0)

        if "desired_acceleration" not in new_target:
            new_target["desired_acceleration"] = 0.0
        if "desired_speed" not in new_target:
            new_target["desired_speed"] = 0.0
        if "has_control" not in new_target:
            new_target["has_control"] = False

        changed = (new_target != previous)

        self.control_targets_by_actor_id[actor_id] = new_target
        if changed:
            self.control_update_pending_for_actor_ids.add(actor_id)

            log(
                "OMNET",
                "CONTROL_INTENT",
                "",
                actor=actor_id,
                simulation_time=float(self.simulation_time_seconds),
                desired_speed=float(new_target.get("desired_speed", 0.0)),
                desired_acceleration=float(new_target.get("desired_acceleration", 0.0)),
                source_module=source_module,
                sequence=sequence,
                hop=hop or "OMNET_TO_PY",
            )

        return True

    def handle_generic_message(self, message: dict):
        user_defined = message.get("user_defined", {}) or {}
        if not isinstance(user_defined, dict):
            return self.resp_generic({"ok": False, "error": "user_defined must be an object"}, SIM_STATUS_ERROR)

        # STRICT: OMNeT uses msg_type and keeps seq/src/hop in user_defined
        if "msg_type" not in user_defined:
            return self.resp_generic({"ok": False, "error": "user_defined.msg_type is required"}, SIM_STATUS_ERROR)

        msg_type = str(user_defined.get("msg_type", "")).upper()
        source_module = user_defined.get("src", None)
        hop = user_defined.get("hop", None)
        sequence = user_defined.get("seq", None)

        if msg_type == "CONTROL":
            if "actor_id" not in user_defined:
                return self.resp_generic({"ok": False, "error": "CONTROL requires user_defined.id"}, SIM_STATUS_ERROR, status=-1)
            if "ctrl" not in user_defined:
                return self.resp_generic({"ok": False, "error": "CONTROL requires user_defined.ctrl"}, SIM_STATUS_ERROR, status=-1)

            actor_id = str(user_defined.get("actor_id", "") or "").strip()
            control_payload = user_defined.get("ctrl", None)

            if not actor_id:
                return self.resp_generic({"ok": False, "error": "CONTROL requires non-empty user_defined.id"}, SIM_STATUS_ERROR, status=-1)
            if not isinstance(control_payload, dict):
                return self.resp_generic({"ok": False, "error": "CONTROL user_defined.ctrl must be an object"}, SIM_STATUS_ERROR, status=-1)

            ok = self.ingest_control_for_actor(
                actor_id,
                control_payload,
                source_module=source_module,
                sequence=sequence,
                hop=hop,
            )
            return self.resp_generic({"ok": True, "applied": bool(ok)}, SIM_STATUS_RUNNING)

        if msg_type == "CONTROL_BATCH":
            if "controls" not in user_defined:
                return self.resp_generic({"ok": False, "error": "CONTROL_BATCH requires user_defined.controls"}, SIM_STATUS_ERROR, status=-1)
            controls = user_defined.get("controls", None)
            if not isinstance(controls, list):
                return self.resp_generic({"ok": False, "error": "controls must be a list"}, SIM_STATUS_ERROR, status=-1)

            applied = 0
            ignored = 0
            for entry in controls:
                if not isinstance(entry, dict):
                    continue
                if "actor_id" not in entry or "ctrl" not in entry:
                    ignored += 1
                    continue

                actor_id = str(entry.get("actor_id", "") or "").strip()
                payload = entry.get("ctrl", None)
                if not actor_id or not isinstance(payload, dict):
                    ignored += 1
                    continue

                if self.ingest_control_for_actor(
                    actor_id,
                    payload,
                    source_module=source_module,
                    sequence=sequence,
                    hop=hop,
                ):
                    applied += 1
                else:
                    ignored += 1

            log(
                "PROTOCOL",
                "CONTROL_BATCH",
                "",
                simulation_time=float(self.simulation_time_seconds),
                applied=applied,
                ignored=ignored,
                sequence=sequence,
            )
            return self.resp_generic({"ok": True, "applied": applied, "ignored": ignored}, SIM_STATUS_RUNNING)

        if msg_type == "PING":
            log("PROTOCOL", "PING", "", sequence=sequence)
            return self.resp_generic({"ok": True, "pong": True}, SIM_STATUS_RUNNING)

        return self.resp_generic({"ok": True}, SIM_STATUS_RUNNING)

    def handle_finished(self, message: dict):
        banner("PROTOCOL", "SIMULATION_FINISHED")
        return self.resp_generic({"ok": True, "finished": True}, SIM_STATUS_FINISHED_OK)

    def dispatch(self, message: dict):
        message_type = str(message.get("message_type", "")).upper()
        if message_type == "INIT":
            return self.handle_init(message), False
        if message_type == "SIMULATION_STEP":
            return self.handle_simulation_step(message), False
        if message_type == "GENERIC_MESSAGE":
            return self.handle_generic_message(message), False
        if message_type == "SPAWN_ACTOR":
            return self.handle_spawn_actor(message), False
        if message_type in ("SIMULATION_FINISHED", "FINISHED", "STOP"):
            return self.handle_finished(message), True

        log("PROTOCOL", "UNKNOWN_MESSAGE_TYPE", "", message_type=message_type)
        return (self.resp_generic({"ok": False, "message_type": message_type}, SIM_STATUS_RUNNING, status=-1), False)


def main():
    context = zmq.Context.instance()
    socket = context.socket(zmq.REP)
    socket.linger = 0
    socket.bind(ZMQ_BIND)

    banner("ZMQ", f"REP bound on {ZMQ_BIND}")

    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    bridge = Bridge()
    raw_monitor = RawJsonMonitor()
    stop_requested = {"flag": False}

    def handle_signal(signum, frame):
        stop_requested["flag"] = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        while True:
            if stop_requested["flag"]:
                log("SYSTEM", "SIGNAL_RECEIVED", "")
                bridge.shutdown()
                break

            events = dict(poller.poll(2000))
            if socket not in events:
                log_once("zmq_waiting", "ZMQ", "WAITING", "Waiting for OMNeT++ request…")
                continue

            reset_once("zmq_waiting")

            incoming_message = None
            incoming_message_type = "UNKNOWN"
            should_exit_after_reply = False
            should_log_step = True
            peek_timestamp = None

            try:
                raw_bytes = socket.recv()
                json_text = raw_bytes.decode("utf-8", errors="replace")

                # Peek quickly (cheap) for message_type/timestamp/seq (OMNeT keeps seq in user_defined)
                peek_type = "UNKNOWN"
                peek_sequence = None
                try:
                    peek = json.loads(json_text)
                    if isinstance(peek, dict):
                        peek_type = str(peek.get("message_type", "UNKNOWN")).upper()
                        peek_timestamp = peek.get("timestamp", None)
                        ud = peek.get("user_defined", {}) or {}
                        if isinstance(ud, dict):
                            peek_sequence = ud.get("seq", None)
                        else:
                            peek_sequence = None
                except Exception:
                    pass

                if peek_type == "SIMULATION_STEP" and ZMQ_STEP_LOG_PERIOD_SECONDS > 0.0:
                    if bridge.last_step_logged_timestamp is None:
                        should_log_step = True
                    else:
                        now_ts = safe_float(peek_timestamp, None) if peek_timestamp is not None else None
                        if now_ts is None:
                            should_log_step = True
                        else:
                            should_log_step = (now_ts - float(bridge.last_step_logged_timestamp)) >= float(ZMQ_STEP_LOG_PERIOD_SECONDS)

                if peek_type != "SIMULATION_STEP" or should_log_step:
                    log(
                        "ZMQ",
                        "RECEIVED",
                        "",
                        hop="OMNET_TO_PY",
                        message_type=peek_type,
                        timestamp=peek_timestamp,
                        sequence=peek_sequence,
                        bytes=len(raw_bytes),
                    )

                incoming_message = json.loads(json_text)
                if not isinstance(incoming_message, dict):
                    raise ValueError("Top-level JSON must be an object")

                incoming_message_type = str(incoming_message.get("message_type", "")).upper()

                raw_monitor.maybe_log("RX", incoming_message, json_text)
                response, should_exit_after_reply = bridge.dispatch(incoming_message)

            except Exception as exception:
                error_text = f"{type(exception).__name__}: {exception}"
                log("SYSTEM", "ERROR", error_text)

                if LOG_RAW_JSON:
                    preview = " ".join(str(json_text).split()) if "json_text" in locals() else ""
                    if len(preview) > LOG_RAW_MAX_CHARS:
                        preview = preview[:LOG_RAW_MAX_CHARS] + "…"
                    if preview:
                        log("ZMQ", RawJsonMonitor._raw_event("RX"), preview, message_type=incoming_message_type)

                if incoming_message_type == "INIT":
                    timestamp = 0.0
                    if isinstance(incoming_message, dict):
                        timestamp = safe_float(incoming_message.get("timestamp", 0.0), 0.0)
                    response = {
                        "message_type": "INIT_COMPLETED",
                        "status": -1,
                        "initial_timestamp": float(timestamp),
                        "actor_positions": [],
                        "simulation_status": SIM_STATUS_ERROR,
                    }
                elif incoming_message_type == "SIMULATION_STEP":
                    response = {
                        "message_type": "UPDATED_POSITIONS",
                        "status": -1,
                        "actor_positions": [],
                        "simulation_status": SIM_STATUS_ERROR,
                    }
                else:
                    response = {
                        "message_type": "GENERIC_RESPONSE",
                        "status": -1,
                        "user_defined": {"ok": False, "error": error_text},
                        "simulation_status": SIM_STATUS_ERROR,
                    }

            response_text = json.dumps(response, separators=(",", ":"))

            if isinstance(response, dict):
                raw_monitor.maybe_log("TX", response, response_text)

            resp_type = ""
            if isinstance(response, dict):
                resp_type = str(response.get("message_type", "")).upper()

            if resp_type in ("INIT_COMPLETED", "UPDATED_POSITIONS"):
                actors_csv = ""
                if bridge.actors_by_id:
                    actors_csv = ",".join(bridge.sorted_vehicle_ids(list(bridge.actors_by_id.keys())))

                log(
                    "BRIDGE",
                    "STATE_SENT",
                    "",
                    hop="PY_TO_OMNET",
                    simulation_time=float(bridge.simulation_time_seconds),
                    message_type=resp_type,
                    actors=actors_csv if actors_csv else None,
                )

            socket.send_string(response_text, encoding="utf-8")

            if incoming_message_type != "SIMULATION_STEP" or (ZMQ_STEP_LOG_PERIOD_SECONDS <= 0.0) or should_log_step:
                if incoming_message_type == "SIMULATION_STEP" and peek_timestamp is not None:
                    bridge.last_step_logged_timestamp = peek_timestamp

                log(
                    "ZMQ",
                    "REPLIED",
                    "",
                    hop="PY_TO_OMNET",
                    message_type=response.get("message_type") if isinstance(response, dict) else None,
                    status=response.get("status") if isinstance(response, dict) else None,
                    simulation_status=response.get("simulation_status") if isinstance(response, dict) else None,
                )

            if should_exit_after_reply and EXIT_ON_FINISH:
                banner("PROTOCOL", "EXIT_AFTER_FINISH")
                bridge.shutdown()
                break

    finally:
        try:
            socket.close()
        except Exception:
            pass
        try:
            context.term()
        except Exception:
            pass


if __name__ == "__main__":
    main()
