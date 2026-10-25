"""
Pure-Python safety watchdog logic for the Mark Five AMR (PLAN §7, S2 + S4).

No rclpy imports: the ROS node (`watchdog_node.py`) is a thin adapter around
`SafetyWatchdog`, so everything that decides whether the robot may move is
unit-tested with plain pytest and a fake clock.

States
------
DISARMED  No heartbeat seen since start (or since the last human reset).
          Only S4 applies: commands are clamped and passed through, so teleop
          and RViz Nav2 goals behave exactly as without the watchdog.
ARMED     A heartbeat has been seen. S4 applies, and heartbeat silence longer
          than `heartbeat_timeout` trips the watchdog.
TRIPPED   Latched. Every command is replaced by zero. Heartbeats resuming do
          NOT clear it; only `reset()` (the /safety/reset service, i.e. a
          human) does, and it returns to DISARMED (re-arms on the next beat).

Release (non-latching, /safety/release): a live, cooperating orchestrator that
is finishing cleanly says "I'm done" instead of just going silent. ARMED ->
DISARMED with one zero + cancel-all goals like a trip, but no latch and no
trip_count. It never clears TRIPPED (only a human reset does), and the
heartbeat deadline is checked first: a release that arrives after the
deadline trips instead, so a late release can't paper over a missed beat.
`kill -9` can't send one, so a dead orchestrator still trips.

Invariants
----------
S4  |linear.x| <= max_linear, |angular.z| <= max_angular, every other twist
    component is zeroed, and a non-finite component anywhere yields zero.
S2  Armed + heartbeat silent > heartbeat_timeout -> trip (zero + cancel goals).
    Staleness is checked on *every* entry point (tick, command, heartbeat), not
    only on the periodic tick, so a late timer on a CPU-starved Nano cannot
    let a command through after the deadline.
S3  Geofence: NOT implemented in v1 -- see `_geofence_check()`.

Stale input: the watchdog only ever emits a command in response to an input
command, plus at most one zero when input goes silent after a nonzero command
(and one zero on a trip). It never republishes a stale nonzero command, so the
Arduino's 1 s serial-silence stop remains a working backstop.

Time is local monotonic receipt time, never message stamps: the orchestrator
runs on the workstation and the watchdog on the Jetson, and liveness must not
depend on cross-machine clock sync.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

DISARMED = 'DISARMED'
ARMED = 'ARMED'
TRIPPED = 'TRIPPED'


@dataclass(frozen=True)
class WatchdogConfig:
    max_linear: float = 0.20        # m/s   -- nav2_params velocity_smoother max_velocity[0]
    max_angular: float = 1.0        # rad/s -- nav2_params velocity_smoother max_velocity[2]
    heartbeat_timeout: float = 1.0  # s     -- PLAN §7 S2
    input_timeout: float = 0.5      # s     -- emit one zero after a nonzero command goes stale

    def __post_init__(self):
        for name in ('max_linear', 'max_angular', 'heartbeat_timeout', 'input_timeout'):
            value = getattr(self, name)
            if not (isinstance(value, (int, float)) and math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name} must be a finite number > 0, got {value!r}')


@dataclass(frozen=True)
class Twist6:
    """Plain twist (geometry_msgs/Twist without the ROS dependency)."""
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0

    def components(self) -> Tuple[float, ...]:
        return (self.linear_x, self.linear_y, self.linear_z,
                self.angular_x, self.angular_y, self.angular_z)


@dataclass(frozen=True)
class GateResult:
    """Output of filter_command(). Only linear_x/angular_z are ever nonzero."""
    linear_x: float
    angular_z: float
    clamped: bool = False   # a component was reduced to the S4 cap
    rejected: bool = False  # non-finite input -> zero
    blocked: bool = False   # TRIPPED -> zero

    @property
    def is_zero(self) -> bool:
        return self.linear_x == 0.0 and self.angular_z == 0.0


def _clamp(value: float, limit: float) -> Tuple[float, bool]:
    if value > limit:
        return limit, True
    if value < -limit:
        return -limit, True
    return value, False


class SafetyWatchdog:
    def __init__(self, config: WatchdogConfig = WatchdogConfig(),
                 clock: Callable[[], float] = time.monotonic):
        self.config = config
        self._clock = clock

        self.state = DISARMED
        self.reason = 'waiting for first heartbeat'
        self.trip_count = 0
        self.release_count = 0
        self.clamp_count = 0
        self.last_clamped = False
        self.last_heartbeat_source = ''

        self._last_heartbeat: Optional[float] = None
        self._last_input: Optional[float] = None
        self._last_output_nonzero = False
        self._trip_pending = False
        self._release_pending = False

    # ── queries ──────────────────────────────────────────────────────────

    def heartbeat_age(self) -> Optional[float]:
        """Seconds since the last heartbeat, or None if none was ever seen."""
        if self._last_heartbeat is None:
            return None
        return max(0.0, self._clock() - self._last_heartbeat)

    def consume_trip(self) -> bool:
        """True exactly once after each trip: caller publishes zero + cancels goals."""
        pending, self._trip_pending = self._trip_pending, False
        return pending

    def consume_release(self) -> bool:
        """True exactly once after each release: caller publishes zero + cancels goals."""
        pending, self._release_pending = self._release_pending, False
        return pending

    def snapshot(self) -> dict:
        age = self.heartbeat_age()
        return {
            'state': self.state,
            'reason': self.reason,
            'heartbeat_age_s': None if age is None else round(age, 3),
            'last_heartbeat_source': self.last_heartbeat_source,
            'last_cmd_clamped': self.last_clamped,
            'clamp_count': self.clamp_count,
            'trip_count': self.trip_count,
            'release_count': self.release_count,
            'max_linear': self.config.max_linear,
            'max_angular': self.config.max_angular,
            'heartbeat_timeout_s': self.config.heartbeat_timeout,
            'geofence': 'not_implemented',
        }

    # ── entry points ─────────────────────────────────────────────────────

    def heartbeat(self, source: str = '') -> None:
        """Record a heartbeat. Arms from DISARMED; never clears TRIPPED."""
        # A beat that arrives after the deadline still means the deadline was
        # missed: trip first, so a late beat can't paper over the gap.
        self._check_heartbeat()
        self._last_heartbeat = self._clock()
        self.last_heartbeat_source = source
        if self.state == DISARMED:
            self.state = ARMED
            self.reason = f'armed by heartbeat from {source or "unknown"}'

    def filter_command(self, cmd: Twist6) -> GateResult:
        """Gate one input command. The result is what goes to the motors."""
        self._check_heartbeat()
        self._last_input = self._clock()

        if self.state == TRIPPED:
            result = GateResult(0.0, 0.0, blocked=True)
        elif not all(math.isfinite(c) for c in cmd.components()):
            result = GateResult(0.0, 0.0, rejected=True)
        else:
            fence = self._geofence_check(cmd)
            if fence is not None:  # S3 hook; never taken in v1
                self.reason = fence
                result = GateResult(0.0, 0.0, blocked=True)
            else:
                lx, c1 = _clamp(cmd.linear_x, self.config.max_linear)
                az, c2 = _clamp(cmd.angular_z, self.config.max_angular)
                result = GateResult(lx, az, clamped=c1 or c2)

        self.last_clamped = result.clamped
        if result.clamped:
            self.clamp_count += 1
        self._last_output_nonzero = not result.is_zero
        return result

    def tick(self) -> bool:
        """Periodic check. Returns True if the caller should publish one zero
        because a nonzero command has gone stale (trips are reported via
        consume_trip())."""
        self._check_heartbeat()
        if (self._last_output_nonzero and self._last_input is not None
                and self._clock() - self._last_input > self.config.input_timeout):
            self._last_output_nonzero = False
            return True
        return False

    def reset(self) -> Tuple[bool, str]:
        """Human reset (/safety/reset). TRIPPED -> DISARMED; otherwise no-op."""
        if self.state != TRIPPED:
            return True, f'not tripped (state {self.state}); nothing to reset'
        old_reason = self.reason
        self.state = DISARMED
        self.reason = 'reset by human; waiting for next heartbeat'
        self._trip_pending = False
        return True, f'reset from TRIPPED ({old_reason}); now DISARMED, re-arms on next heartbeat'

    def release(self, source: str = '') -> Tuple[bool, str]:
        """Non-latching release (/safety/release) from a cleanly-stopping
        orchestrator. ARMED -> DISARMED (caller: zero + cancel goals via
        consume_release()). Never clears TRIPPED. A release arriving after the
        heartbeat deadline trips instead and returns False."""
        who = source or 'unknown'
        # Deadline first: if the beats were already overdue, this is a missed
        # deadline, not a clean stop (the trip is reported via consume_trip()).
        was_armed = self.state == ARMED
        self._check_heartbeat()
        if self.state == TRIPPED:
            if was_armed:
                return False, (f'release from {who} arrived after the heartbeat deadline; '
                               f'TRIPPED ({self.reason}). Only /safety/reset (a human) '
                               f'clears a trip.')
            return False, (f'TRIPPED ({self.reason}); release does not clear a trip. '
                           f'Only /safety/reset (a human) clears a trip.')
        if self.state == DISARMED:
            return True, f'not armed (state {self.state}); nothing to release'
        self.state = DISARMED
        self.reason = f'released by {who}; waiting for next heartbeat'
        self.release_count += 1
        self._release_pending = True
        self._last_output_nonzero = False  # the release zero supersedes any stale output
        return True, f'released by {who}: ARMED -> DISARMED, re-arms on next heartbeat'

    # ── internals ────────────────────────────────────────────────────────

    def _check_heartbeat(self) -> None:
        if self.state != ARMED or self._last_heartbeat is None:
            return
        age = self._clock() - self._last_heartbeat
        if age > self.config.heartbeat_timeout:
            self._trip(f'heartbeat lost ({age:.2f}s > {self.config.heartbeat_timeout:.2f}s)')

    def _trip(self, reason: str) -> None:
        self.state = TRIPPED
        self.reason = reason
        self.trip_count += 1
        self._trip_pending = True
        self._last_output_nonzero = False  # the trip zero supersedes any stale output

    def _geofence_check(self, cmd: Twist6) -> Optional[str]:
        """S3 hook. TODO(phase 2, S3): block motion that leaves the allowed
        bounds or enters a no-go zone. Needs the robot pose (map->base_footprint
        TF) and a fence derived from the saved map (see
        mark_five_agents/scripts/derive_geofence.py). Return a reason string to
        block this command (zero output), or None to allow. Whether a fence
        breach should also latch a trip is an open design question for S3.
        v1 allows everything."""
        return None
