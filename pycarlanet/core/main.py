#!/usr/bin/env python3  # Shebang to run with Python 3

"""
Bridge-only CARLA↔OMNeT++ (Plexe):
- Python only spawns cars and applies CONTROL from OMNeT++.
- Robust GENERIC_MESSAGE handler: unwraps nested {user_defined:{...{user_defined:{...}}}}
  and also parses JSON strings if that's how your manager forwards it.

ENV (optional):
  PYCARLANET_REFRESH_ON_START=1   # load Town01 (default 1)
  P_BACK_1_M=15   P_BACK_2_M=55   # spawn offsets behind leader (m)
  P_KTHR=0.60      P_KBRK=0.60    # accel→throttle / (-accel)→brake gains
  PYCARLANET_DEBUG=1              # print incoming controls + applied outputs

Sync toggle (default async):
  --sync              # CLI flag to enable synchronous mode
  CARLA_SYNC=1        # or set env var
  P_FIXED_DT=0.025    # optional: fixed delta seconds in sync mode (default 0.025)
"""  # Module docstring describing behavior and environment variables

from __future__ import annotations  # Enable future-style annotations for type hints
import os, sys, time, math, json, random, atexit, signal, contextlib  # Standard library imports used throughout
from typing import Dict, Optional, Any  # Type aliases for readability

import carla  # CARLA Python API
from pycarlanet import CarlanetManager, CarlanetEventListener, CarlanetActor, SimulatorStatus  # Bridge helpers

# --- Tunables -----------------------------------------------------------------
REFRESH_ON_START = os.environ.get("PYCARLANET_REFRESH_ON_START", "1") == "1"  # Reload Town01 on start if env says so
BACK_1 = float(os.environ.get("P_BACK_1_M", "15.0"))  # Offset for member behind leader (meters)
BACK_2 = float(os.environ.get("P_BACK_2_M", "55.0"))  # Offset for joiner behind leader (meters)
DEBUG  = os.environ.get("PYCARLANET_DEBUG", "0") == "1"  # Verbose debug printing if set

# --- Sync toggle (default async) ---------------------------------------------
SYNC_ENABLED = ("--sync" in sys.argv) or os.environ.get("CARLA_SYNC", "").strip().lower() in ("1","true","yes","on")  # Determine sync mode
FIXED_DT = float(os.environ.get("P_FIXED_DT", "0.025"))  # Sync step size (must match **.manager.simulationTimeStep)

def _fwd_vec(yaw_deg: float) -> carla.Vector3D:  # Compute forward unit vector from yaw angle in degrees
    r = math.radians(yaw_deg)  # Convert yaw degrees to radians
    return carla.Vector3D(math.cos(r), math.sin(r), 0.0)  # Return (cos,sin,0) for yaw

def _offset_tf(base: carla.Transform, back_m: float) -> carla.Transform:  # Build a transform offset backward along yaw
    f = _fwd_vec(base.rotation.yaw)  # Forward direction from base yaw
    loc = carla.Location(base.location.x - f.x*back_m, base.location.y - f.y*back_m, base.location.z)  # Back off by meters
    return carla.Transform(loc, base.rotation)  # Keep same rotation, moved location

# --- App ----------------------------------------------------------------------
class PlatoonWorld(CarlanetEventListener):  # Event listener that owns the CARLA world and bridge
    def __init__(self, host="127.0.0.1", port=2000, zmq_port=5555):  # Configure endpoints for CARLA and ZMQ
        self.host, self.port, self.zmq_port = host, int(port), int(zmq_port)  # Store connection parameters
        self.client: Optional[carla.Client] = None  # Will hold CARLA client
        self.world:  Optional[carla.World]  = None  # Will hold CARLA world
        self.map:    Optional[carla.Map]    = None  # Will hold CARLA map

        self.actors: Dict[str, carla.Vehicle] = {}  # Spawned vehicles by role id
        self._ctrl: Dict[str, Dict[str, float]] = {}  # Last CONTROL (accel/speed) per actor
        self._ctrl_seen: Dict[str, float] = {}  # Wall-clock time when CONTROL was last seen per actor
        self._last_debug = 0.0  # Throttle debug prints

        self.sync_enabled = SYNC_ENABLED  # Desired sync/async mode
        self.fixed_dt = FIXED_DT  # Desired fixed delta in sync mode
        self.prev_settings: Optional[carla.WorldSettings] = None  # To restore on shutdown

        self.manager = CarlanetManager(self.zmq_port, self, log_messages=True)  # Start ZMQ manager with callbacks
        self._is_stopping = False  # Flag to avoid double shutdown

    # === Bridge lifecycle ======================================================
    def omnet_init_completed(self, run_id: str | None = None, **_):
        print(f"[py] omnet_init_completed run_id={run_id}")
        self._connect()
        self._prepare_world()

        spawns = self.map.get_spawn_points()
        if not spawns:
            raise RuntimeError("No CARLA spawn points available.")
        leader_tf  = spawns[0]
        member_tf  = _offset_tf(leader_tf, BACK_1)
        joiner_tf  = _offset_tf(leader_tf, BACK_2)

        self.actors["leader"] = self._spawn_vehicle(leader_tf,  "leader")
        self.actors["member"] = self._spawn_vehicle(member_tf,  "member")
        self.actors["joiner"] = self._spawn_vehicle(joiner_tf,  "joiner")

        for aid in ("leader","member","joiner"):
            self.manager.add_dynamic_actor(aid, CarlanetActor(self.actors[aid], True))

        self._tick_world()

        # <<< ADD THIS >>>
        self._place_spectator_behind("joiner")

        print(f"[py] bridge ready (Town01={REFRESH_ON_START})  offsets: member -{BACK_1} m, joiner -{BACK_2} m")
        print(f"[py] mode: {'SYNC' if self.sync_enabled else 'ASYNC'}  fixed_dt={self.fixed_dt:.3f}s")
        return SimulatorStatus.RUNNING, self.world

    # --- Payload extraction helpers -------------------------------------------
    @staticmethod
    def _extract_msg(args: Any, kwargs: Any) -> dict:  # Normalize incoming callback arguments into a dict
        """Return a dict-like message (handles various dispatch shapes)."""  # Docstring for clarity
        if args and isinstance(args[0], dict):  # Case: first positional is a dict
            return args[0]  # Use it directly
        if "message" in kwargs and isinstance(kwargs["message"], dict):  # Case: keyword 'message' provided
            return kwargs["message"]  # Use that dict
        if len(args) >= 2 and isinstance(args[1], dict):  # Case: (timestamp, dict)
            return {"timestamp": args[0], "user_defined": args[1]}  # Wrap to consistent shape
        return {}  # Fallback empty dict

    @staticmethod
    def _coerce_user_defined(msg: dict) -> dict:  # Extract innermost user_defined dict, parsing JSON if needed
        """
        Return the innermost user_defined dict. Handles:
          - user_defined as dict with msg_type
          - user_defined as dict that itself contains {user_defined:{...}}
          - user_defined as JSON string
        """  # Explain supported forms
        ud = msg.get("user_defined", {})  # Start with top-level user_defined
        if isinstance(ud, str):  # If passed as a JSON string
            try:
                ud = json.loads(ud)  # Parse JSON to dict
            except Exception:
                return {}  # On parse failure, return empty
        while isinstance(ud, dict) and "msg_type" not in ud and "user_defined" in ud:  # Unwrap nested shapes
            inner = ud.get("user_defined")  # Dive into inner layer
            if isinstance(inner, str):  # If inner is JSON string
                try:
                    ud = json.loads(inner)  # Parse to dict
                except Exception:
                    break  # Stop on error
            else:
                ud = inner  # Replace with inner dict
        return ud if isinstance(ud, dict) else {}  # Ensure dict output

    # --- Messaging -------------------------------------------------------------
    def generic_message(self, *args: Any, **kwargs: Any):  # Handle generic messages from OMNeT++ (REQ/REP)
        """Strict REQ/REP handler. Records CONTROL messages and ACKs."""  # Purpose: accept CONTROL and store
        try:
            msg = self._extract_msg(args, kwargs)  # Normalize callback args to a message dict
            ud  = self._coerce_user_defined(msg)  # Extract innermost user_defined payload
            if ud.get("msg_type") == "CONTROL":  # Only process CONTROL
                aid  = str(ud.get("id", ""))  # Actor id ('leader','member','joiner')
                ctrl = ud.get("ctrl", {})  # Control payload dict
                if aid not in self.actors:  # Unknown actor guard
                    print(f"[py] CONTROL for unknown id '{aid}' (known: {list(self.actors.keys())})")  # Warn unknown actor
                if isinstance(ctrl, dict) and aid in self.actors:  # Valid control for known actor
                    a = float(ctrl.get("desired_accel", 0.0))  # Desired acceleration
                    v = float(ctrl.get("desired_speed", 0.0))  # Desired speed (advisory)
                    self._ctrl[aid] = {"desired_accel": a, "desired_speed": v}  # Store last control
                    self._ctrl_seen[aid] = time.time()  # Record last-seen timestamp
                    if DEBUG:  # Optional verbose log
                        print(f"[py] CTRL {aid}: a={a:.2f} m/s^2  v_des={v:.2f} m/s")  # Show control
        except Exception as e:  # Robust parsing guard
            print(f"[py] generic_message parse error: {e}")  # Log error
        return SimulatorStatus.RUNNING, {}  # Always reply RUNNING with empty payload

    def before_world_tick(self, *_args, **_kwargs):  # Called before each CARLA world tick
        """Apply last commanded CONTROL to each actor (handbrake off; forward gear)."""  # Brief description
        if self._is_stopping or not self.actors:  # If shutting down or not ready
            return  # Skip work
        now = time.time()  # Current time for rate-limiting logs
        K_THR = float(os.environ.get("P_KTHR", "0.60"))  # Gain mapping +accel→throttle
        K_BRK = float(os.environ.get("P_KBRK", "0.60"))  # Gain mapping -accel→brake

        for aid, veh in self.actors.items():  # Iterate all actors
            if not veh.is_alive:  # Skip destroyed actors
                continue  # Next actor
            ctrl = self._ctrl.get(aid)  # Last CONTROL for this actor
            if not ctrl:  # No control yet → hold brake
                veh.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0,  # Full brake, no throttle
                                                       hand_brake=False, reverse=False, manual_gear_shift=False))  # Drive flags
                if DEBUG and now - self._last_debug > 1.0:  # Rate-limit logs
                    print(f"[py] WAIT {aid}: no CONTROL yet")  # Announce waiting
                continue  # Next actor

            a = float(ctrl.get("desired_accel", 0.0))  # Desired acceleration
            if a >= 0.0:  # Positive accel → throttle
                thr = max(0.0, min(1.0, a * K_THR))  # Clamp throttle [0,1]
                veh.apply_control(carla.VehicleControl(throttle=thr, brake=0.0, steer=0.0,  # Apply throttle
                                                       hand_brake=False, reverse=False, manual_gear_shift=False))  # Drive flags
                if DEBUG and now - self._last_debug > 0.5:  # Rate-limit logs
                    print(f"[py] APPLY {aid}: a={a:.2f} → throttle={thr:.2f}")  # Show effect
            else:  # Negative accel → braking
                brk = max(0.0, min(1.0, (-a) * K_BRK))  # Clamp brake [0,1]
                veh.apply_control(carla.VehicleControl(throttle=0.0, brake=brk, steer=0.0,  # Apply brake
                                                       hand_brake=False, reverse=False, manual_gear_shift=False))  # Drive flags
                if DEBUG and now - self._last_debug > 0.5:  # Rate-limit logs
                    print(f"[py] APPLY {aid}: a={a:.2f} → brake={brk:.2f}")  # Show effect

        if DEBUG and now - self._last_debug > 0.5:  # Periodic debug update
            self._last_debug = now  # Store last debug timestamp

    def carla_simulation_step(self, *_args, **_kwargs):  # Called by the manager each OMNeT++ step
        """Called by the manager on each OMNeT++ SIMULATION_STEP."""  # Docstring for clarity
        if self.sync_enabled:  # Only tick in synchronous mode
            with contextlib.suppress(Exception):  # Ignore tick exceptions
                self.world.tick()  # Advance world exactly one fixed_dt
        return SimulatorStatus.RUNNING  # Keep the simulator running

    def on_manager_stop(self, *_args, **_kwargs):  # Called when the manager stops
        self._shutdown(keep_actors=True)  # Safe stop and keep actors spawned

    # === Internals =============================================================
    def _connect(self):  # Create CARLA client and load/get a world
        self.client = carla.Client(self.host, self.port)  # Connect to CARLA server
        self.client.set_timeout(10.0)  # Set RPC timeout seconds
        if REFRESH_ON_START:  # Optionally reload a known map
            try:
                self.world = self.client.load_world("Town01")  # Load Town01 map
            except Exception as e:  # Fallback if loading fails
                print(f"[py] load_world('Town01') failed ({e}); falling back to get_world()")  # Warn and fallback
                self.world = self.client.get_world()  # Use current world
        else:
            self.world = self.client.get_world()  # Use existing world without reloading
        self.map = self.world.get_map()  # Cache the map for spawns

    def _prepare_world(self):  # Apply desired sync/async mode and remember old settings
        """Apply desired sync/async mode. Store previous settings for clean restore."""  # Docstring for clarity
        s = self.world.get_settings()  # Read current settings
        self.prev_settings = s  # Remember original settings
        if self.sync_enabled:  # If synchronous mode requested
            new_s = carla.WorldSettings(  # Build new settings
                no_rendering_mode=s.no_rendering_mode,  # Preserve rendering choice
                synchronous_mode=True,  # Enable synchronous stepping
                fixed_delta_seconds=self.fixed_dt,  # Use fixed delta (matches OMNeT++)
                max_substeps=s.max_substeps  # Keep existing substep cap
            )  # End settings build
            self.world.apply_settings(new_s)  # Apply synchronous settings
        else:  # Ensure asynchronous mode
            if s.synchronous_mode or s.fixed_delta_seconds is not None:  # If previously in sync
                new_s = carla.WorldSettings(  # Build async settings
                    no_rendering_mode=s.no_rendering_mode,  # Preserve rendering choice
                    synchronous_mode=False,  # Disable synchronous stepping
                    fixed_delta_seconds=None,  # Clear fixed delta
                    max_substeps=s.max_substeps  # Keep substep cap
                )  # End settings build
                self.world.apply_settings(new_s)  # Apply async settings

    def _spawn_vehicle(self, tf: carla.Transform, role_name: str) -> carla.Vehicle:  # Spawn a vehicle with role name
        bp_lib = self.world.get_blueprint_library()  # Get blueprint library
        preferred = ["vehicle.tesla.model3", "vehicle.audi.tt", "vehicle.lincoln.mkz_2017", "vehicle.nissan.patrol"]  # Preferred models
        bp = None  # Selected blueprint placeholder
        for m in preferred:  # Try each preferred model
            with contextlib.suppress(Exception):  # Ignore lookup errors
                b = bp_lib.find(m)  # Lookup model blueprint
                if b: bp = b; break  # Use first that exists
        if bp is None:  # If none of the preferred exist
            bp = random.choice(bp_lib.filter("vehicle.*"))  # Pick any vehicle blueprint
        if bp.has_attribute("role_name"): bp.set_attribute("role_name", role_name)  # Set role_name if supported
        if bp.has_attribute("color"):  # Colorize by role if possible
            bp.set_attribute("color",  # Assign an RGB color
                "255,0,0" if role_name == "leader" else ("0,128,255" if role_name == "member" else "0,200,100"))  # Red/blue/green-ish
        v = self.world.spawn_actor(bp, tf)  # Spawn the vehicle actor
        v.set_autopilot(False)  # Disable autopilot (we control via CONTROL)
        v.apply_control(carla.VehicleControl(hand_brake=False, reverse=False, manual_gear_shift=False))  # Release handbrake
        print(f"[py] spawned {role_name} id={v.id}")  # Log spawned actor id
        return v  # Return the spawned vehicle

    def _place_spectator_behind(self, role_name: str = "joiner"):
        """
        Place the spectator behind the given role (default: rear car 'joiner'),
        higher and looking down a bit.
        """
        if not self.world:
            return

        veh = self.actors.get(role_name)
        if not veh:
            veh = self.actors.get("member") or self.actors.get("leader")
        if not veh:
            return

        tf = veh.get_transform()

        # Higher and slightly further back
        back_m  = float(os.environ.get("P_SPEC_BACK_M", "20.0"))   # was 15.0
        up_m    = float(os.environ.get("P_SPEC_UP_M",  "25.0"))    # was 5.0
        pitch_d = float(os.environ.get("P_SPEC_PITCH_DEG", "-30")) # was -10

        f = _fwd_vec(tf.rotation.yaw)
        cam_loc = carla.Location(
            tf.location.x - f.x * back_m,
            tf.location.y - f.y * back_m,
            tf.location.z + up_m,
        )

        cam_rot = carla.Rotation(
            pitch=pitch_d,
            yaw=tf.rotation.yaw,
            roll=0.0,
        )

        spectator = self.world.get_spectator()
        spectator.set_transform(carla.Transform(cam_loc, cam_rot))
        print(f"[py] spectator placed behind {role_name} at {cam_loc} pitch={pitch_d}")

    def _tick_world(self):  # Step the world one frame
        with contextlib.suppress(Exception):  # Ignore exceptions if ticking not allowed
            self.world.tick()  # Advance the simulation by one tick

    def _shutdown(self, keep_actors: bool = True):  # Safely stop vehicles and optionally destroy them
        if self._is_stopping: return  # Prevent double execution
        self._is_stopping = True  # Mark as stopping
        for v in list(self.actors.values()):  # Iterate current actors
            with contextlib.suppress(Exception):  # Ignore control errors
                v.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0,  # Full brake on shutdown
                                                     hand_brake=False, reverse=False, manual_gear_shift=False))  # Drive flags
        if not keep_actors:  # If we should destroy actors
            for k, v in list(self.actors.items()):  # Iterate by key/value
                with contextlib.suppress(Exception): v.destroy()  # Destroy actor and ignore errors
                self.actors.pop(k, None)  # Remove from dict
        with contextlib.suppress(Exception):  # Restore world settings safely
            if self.prev_settings:  # If we saved original settings
                self.world.apply_settings(self.prev_settings)  # Restore to original mode
        with contextlib.suppress(Exception):  # Stop manager safely
            self.manager.stop_simulation()  # Tell manager to stop

    def start(self):  # Begin the bridge simulation loop
        self.manager.start_simulation()  # Start ZMQ server and process events

# --- Entrypoint ---------------------------------------------------------------
def _install_signals(world: PlatoonWorld):  # Register SIGINT/SIGTERM handlers
    def _h(_sig, _frm):  # Handler closure to stop cleanly
        world._shutdown(keep_actors=True); sys.exit(0)  # Shutdown and exit
    signal.signal(signal.SIGINT, _h)  # Trap Ctrl+C
    signal.signal(signal.SIGTERM, _h)  # Trap termination signal

def main():  # CLI entrypoint
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"  # Read CARLA host from argv or default
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 2000  # Read CARLA port from argv or default
    zmq  = int(sys.argv[3]) if len(sys.argv) > 3 else 5555  # Read ZMQ port for bridge or default
    world = PlatoonWorld(host, port, zmq)  # Create world/bridge controller
    _install_signals(world)  # Set up signal handlers
    atexit.register(world._shutdown, keep_actors=True)  # Ensure safe shutdown at process exit
    try:  # Protect main loop
        world.start()  # Start the bridge manager loop
    except KeyboardInterrupt:  # Handle Ctrl+C
        world._shutdown(keep_actors=True)  # Graceful shutdown on interrupt
    finally:  # Always run on exit
        world._shutdown(keep_actors=True)  # Final shutdown to be safe
    sys.exit(0)  # Exit with success

if __name__ == "__main__":  # Run only when executed as a script
    main()  # Call entrypoint
