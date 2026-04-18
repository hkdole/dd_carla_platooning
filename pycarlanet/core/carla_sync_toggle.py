# carla_sync_toggle.py
import os
import contextlib

try:
    import carla  # type: ignore
except Exception as e:  # pragma: no cover
    carla = None

def parse_sync_flag(argv=None, env=os.environ, default=False):
    """Return True if sync is requested via CLI ('--sync') or env ('CARLA_SYNC=1/true/yes')."""
    argv = argv or []
    flag = any(a == "--sync" for a in argv)
    if not flag:
        v = str(env.get("CARLA_SYNC", "")).strip().lower()
        flag = v in ("1", "true", "yes", "on")
    return bool(flag) if carla else False  # if CARLA missing, force async

class SyncToggle:
    """Helper to switch CARLA world (and TM) between async/sync and tick in sync mode."""
    def __init__(self, world, traffic_manager=None, enabled=False, fixed_dt=0.025):
        self.world = world
        self.tm = traffic_manager
        self.enabled = bool(enabled)
        self.fixed_dt = float(fixed_dt)
        self.prev_settings = None

    def apply(self):
        """Apply desired mode to world (and TM). Store previous settings for restore()."""
        if not self.world:
            return
        self.prev_settings = self.world.get_settings()
        if self.enabled:
            s = self.prev_settings
            new_s = carla.WorldSettings(
                no_rendering_mode=s.no_rendering_mode,
                synchronous_mode=True,
                fixed_delta_seconds=self.fixed_dt,
                max_substeps=s.max_substeps
            )
            self.world.apply_settings(new_s)
            if self.tm:
                with contextlib.suppress(Exception):
                    self.tm.set_synchronous_mode(True)
        else:
            # Async: make sure world is not in sync mode; keep current fixed_dt unchanged
            if self.prev_settings.synchronous_mode:
                new_s = carla.WorldSettings(
                    no_rendering_mode=self.prev_settings.no_rendering_mode,
                    synchronous_mode=False,
                    fixed_delta_seconds=None,
                    max_substeps=self.prev_settings.max_substeps
                )
                self.world.apply_settings(new_s)
            if self.tm:
                with contextlib.suppress(Exception):
                    self.tm.set_synchronous_mode(False)

    def maybe_tick(self):
        """Call once per OMNeT++ SIMULATION_STEP. Ticks world only when in sync mode."""
        if self.enabled and self.world:
            self.world.tick()

    def restore(self):
        """Restore previous world settings and unsync TM."""
        if self.tm:
            with contextlib.suppress(Exception):
                self.tm.set_synchronous_mode(False)
        if self.prev_settings and self.world:
            with contextlib.suppress(Exception):
                self.world.apply_settings(self.prev_settings)
