"""Pure-logic tests for the safety watchdog (no ROS needed: plain pytest)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mark_five_safety.watchdog_logic import (  # noqa: E402
    ARMED, DISARMED, TRIPPED, SafetyWatchdog, Twist6, WatchdogConfig)


class FakeClock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def wd(clock):
    return SafetyWatchdog(WatchdogConfig(max_linear=0.20, max_angular=1.0,
                                         heartbeat_timeout=1.0, input_timeout=0.5),
                          clock=clock)


def arm(wd):
    wd.heartbeat('orchestrator')
    assert wd.state == ARMED


# ── S4: clamping ──────────────────────────────────────────────────────────

def test_within_limits_passes_unchanged(wd):
    r = wd.filter_command(Twist6(linear_x=0.15, angular_z=-0.5))
    assert (r.linear_x, r.angular_z) == (0.15, -0.5)
    assert not r.clamped and not wd.last_clamped


@pytest.mark.parametrize('lx, az, exp_lx, exp_az', [
    (0.5, 0.0, 0.20, 0.0),
    (-0.5, 0.0, -0.20, 0.0),
    (0.0, 3.0, 0.0, 1.0),
    (0.0, -3.0, 0.0, -1.0),
    (1.0, -2.0, 0.20, -1.0),
])
def test_clamps_to_caps(wd, lx, az, exp_lx, exp_az):
    r = wd.filter_command(Twist6(linear_x=lx, angular_z=az))
    assert r.linear_x == pytest.approx(exp_lx)
    assert r.angular_z == pytest.approx(exp_az)
    assert r.clamped and wd.last_clamped
    assert wd.clamp_count == 1


def test_exactly_at_cap_is_not_clamped(wd):
    r = wd.filter_command(Twist6(linear_x=0.20, angular_z=-1.0))
    assert (r.linear_x, r.angular_z) == (0.20, -1.0)
    assert not r.clamped


def test_other_components_are_zeroed(wd):
    r = wd.filter_command(Twist6(0.1, 0.3, 0.4, 0.5, 0.6, 0.2))
    assert (r.linear_x, r.angular_z) == (0.1, 0.2)  # result has no other components at all


def test_clamped_flag_clears_on_next_good_command(wd):
    wd.filter_command(Twist6(linear_x=0.9))
    assert wd.last_clamped
    wd.filter_command(Twist6(linear_x=0.1))
    assert not wd.last_clamped


# ── S4: NaN / inf ─────────────────────────────────────────────────────────

@pytest.mark.parametrize('cmd', [
    Twist6(linear_x=math.nan),
    Twist6(angular_z=math.inf),
    Twist6(linear_x=-math.inf, angular_z=0.3),
    Twist6(linear_x=0.1, linear_y=math.nan, angular_z=0.3),  # non-finite anywhere -> reject
])
def test_non_finite_rejected_to_zero(wd, cmd):
    r = wd.filter_command(cmd)
    assert r.rejected and r.is_zero


# ── config validation ─────────────────────────────────────────────────────

@pytest.mark.parametrize('kwargs', [
    {'max_linear': 0.0}, {'max_angular': -1.0},
    {'heartbeat_timeout': math.nan}, {'input_timeout': math.inf},
])
def test_bad_config_rejected(kwargs):
    with pytest.raises(ValueError):
        WatchdogConfig(**kwargs)


# ── S2: disarmed passthrough ──────────────────────────────────────────────

def test_starts_disarmed_and_passes_commands_without_any_heartbeat(wd, clock):
    assert wd.state == DISARMED
    for _ in range(50):  # 50 s with no heartbeat at all: teleop must keep working
        clock.advance(1.0)
        wd.tick()
        r = wd.filter_command(Twist6(linear_x=0.1, angular_z=0.2))
        assert (r.linear_x, r.angular_z) == (0.1, 0.2)
    assert wd.state == DISARMED
    assert not wd.consume_trip()
    assert wd.heartbeat_age() is None


# ── S2: arming and tripping ───────────────────────────────────────────────

def test_first_heartbeat_arms(wd):
    wd.heartbeat('orchestrator')
    assert wd.state == ARMED
    assert wd.last_heartbeat_source == 'orchestrator'
    assert wd.heartbeat_age() == 0.0


def test_steady_heartbeat_stays_armed(wd, clock):
    arm(wd)
    for _ in range(100):  # 5 Hz for 20 s
        clock.advance(0.2)
        wd.heartbeat('orchestrator')
        wd.tick()
        r = wd.filter_command(Twist6(linear_x=0.1))
        assert r.linear_x == 0.1
    assert wd.state == ARMED and wd.trip_count == 0


def test_silence_at_exactly_timeout_does_not_trip(wd, clock):
    arm(wd)
    clock.advance(1.0)
    wd.tick()
    assert wd.state == ARMED


def test_silence_trips_on_tick(wd, clock):
    arm(wd)
    clock.advance(1.05)
    wd.tick()
    assert wd.state == TRIPPED
    assert 'heartbeat lost' in wd.reason
    assert wd.consume_trip() is True       # caller publishes zero + cancels goals
    assert wd.consume_trip() is False      # exactly once
    assert wd.trip_count == 1


def test_silence_trips_on_command_even_if_tick_is_late(wd, clock):
    """CPU-starved Nano: the periodic timer may not have run yet."""
    arm(wd)
    wd.filter_command(Twist6(linear_x=0.1))
    clock.advance(1.5)
    r = wd.filter_command(Twist6(linear_x=0.1))
    assert r.blocked and r.is_zero
    assert wd.state == TRIPPED
    assert wd.consume_trip()


def test_late_heartbeat_still_trips(wd, clock):
    """A beat that arrives after the deadline must not paper over the gap."""
    arm(wd)
    clock.advance(1.5)
    wd.heartbeat('orchestrator')
    assert wd.state == TRIPPED
    assert wd.consume_trip()


def test_tripped_blocks_all_motion(wd, clock):
    arm(wd)
    clock.advance(2.0)
    wd.tick()
    for cmd in (Twist6(linear_x=0.1), Twist6(angular_z=-0.5), Twist6(linear_x=-0.05)):
        r = wd.filter_command(cmd)
        assert r.blocked and r.is_zero


def test_trip_latches_through_resumed_heartbeats(wd, clock):
    arm(wd)
    clock.advance(2.0)
    wd.tick()
    assert wd.consume_trip()
    for _ in range(50):  # orchestrator comes back and beats happily for 10 s
        clock.advance(0.2)
        wd.heartbeat('orchestrator')
        wd.tick()
        assert wd.state == TRIPPED
        assert wd.filter_command(Twist6(linear_x=0.1)).is_zero
    assert wd.trip_count == 1                # no re-trips while latched
    assert not wd.consume_trip()
    assert wd.heartbeat_age() == 0.0         # age is still reported while tripped


# ── reset ─────────────────────────────────────────────────────────────────

def test_reset_returns_to_disarmed_and_rearms_on_next_beat(wd, clock):
    arm(wd)
    clock.advance(2.0)
    wd.tick()
    ok, msg = wd.reset()
    assert ok and 'DISARMED' in msg
    assert wd.state == DISARMED
    assert not wd.consume_trip()
    # Disarmed after reset: commands flow, silence does not trip.
    clock.advance(5.0)
    wd.tick()
    assert wd.state == DISARMED
    assert wd.filter_command(Twist6(linear_x=0.1)).linear_x == 0.1
    # Next beat re-arms, and silence trips again.
    wd.heartbeat('orchestrator')
    assert wd.state == ARMED
    clock.advance(1.2)
    wd.tick()
    assert wd.state == TRIPPED and wd.trip_count == 2


def test_reset_when_not_tripped_is_a_noop(wd):
    ok, msg = wd.reset()
    assert ok and 'nothing to reset' in msg
    assert wd.state == DISARMED
    arm(wd)
    ok, _ = wd.reset()
    assert ok and wd.state == ARMED      # reset never disarms a healthy watchdog


# ── release (non-latching) ────────────────────────────────────────────────

def test_release_from_armed_disarms_without_latching_or_counting_a_trip(wd, clock):
    arm(wd)
    clock.advance(0.3)
    ok, msg = wd.release('orchestrator')
    assert ok and 'released' in msg
    assert wd.state == DISARMED
    assert 'released by orchestrator' in wd.reason
    assert wd.consume_release() is True     # caller publishes zero + cancels goals
    assert wd.consume_release() is False    # exactly once
    assert not wd.consume_trip()            # not a trip
    assert wd.trip_count == 0 and wd.release_count == 1


def test_after_release_silence_does_not_trip_and_commands_flow(wd, clock):
    arm(wd)
    wd.release('orchestrator')
    for _ in range(20):                     # 20 s of silence: disarmed, so no trip
        clock.advance(1.0)
        wd.tick()
    assert wd.state == DISARMED and not wd.consume_trip()
    assert wd.filter_command(Twist6(linear_x=0.1)).linear_x == 0.1


def test_after_release_next_heartbeat_rearms_and_silence_trips_again(wd, clock):
    arm(wd)
    wd.release('orchestrator')
    clock.advance(10.0)                     # a long gap before the next mission is fine
    wd.heartbeat('orchestrator')
    assert wd.state == ARMED and not wd.consume_trip()
    clock.advance(1.2)
    wd.tick()
    assert wd.state == TRIPPED and wd.trip_count == 1 and wd.release_count == 1


def test_release_never_clears_a_trip(wd, clock):
    arm(wd)
    clock.advance(2.0)
    wd.tick()
    assert wd.consume_trip()
    ok, msg = wd.release('orchestrator')
    assert ok is False
    assert '/safety/reset' in msg and 'human' in msg
    assert wd.state == TRIPPED
    assert not wd.consume_release() and not wd.consume_trip()
    assert wd.release_count == 0 and wd.trip_count == 1
    assert wd.filter_command(Twist6(linear_x=0.1)).is_zero


def test_release_from_disarmed_is_a_successful_noop(wd):
    ok, msg = wd.release('orchestrator')
    assert ok and 'nothing to release' in msg
    assert wd.state == DISARMED
    assert not wd.consume_release() and wd.release_count == 0


def test_late_release_trips_instead(wd, clock):
    """Beats overdue when the release arrives: the deadline was missed, so trip."""
    arm(wd)
    clock.advance(1.5)                      # no tick ran (CPU-starved Nano)
    ok, msg = wd.release('orchestrator')
    assert ok is False and 'deadline' in msg and '/safety/reset' in msg
    assert wd.state == TRIPPED and 'heartbeat lost' in wd.reason
    assert wd.consume_trip() is True
    assert not wd.consume_release()
    assert wd.trip_count == 1 and wd.release_count == 0


def test_release_at_exactly_the_deadline_is_honoured(wd, clock):
    arm(wd)
    clock.advance(1.0)
    ok, _ = wd.release('orchestrator')
    assert ok and wd.state == DISARMED


def test_release_supersedes_stale_nonzero_output(wd, clock):
    arm(wd)
    wd.filter_command(Twist6(linear_x=0.15))
    wd.release('orchestrator')              # the release zero covers it
    clock.advance(0.6)
    assert wd.tick() is False


def test_reset_does_not_consume_or_imply_a_release(wd, clock):
    arm(wd)
    clock.advance(2.0)
    wd.tick()
    wd.reset()
    assert wd.state == DISARMED and wd.release_count == 0 and not wd.consume_release()


# ── stale input ───────────────────────────────────────────────────────────

def test_stale_nonzero_input_yields_exactly_one_zero(wd, clock):
    wd.filter_command(Twist6(linear_x=0.15))
    clock.advance(0.3)
    assert wd.tick() is False             # still fresh
    clock.advance(0.3)
    assert wd.tick() is True              # stale: publish one zero
    for _ in range(40):                   # then stay silent (firmware timeout backstop)
        clock.advance(0.05)
        assert wd.tick() is False


def test_stale_zero_input_yields_nothing(wd, clock):
    wd.filter_command(Twist6())
    clock.advance(5.0)
    assert wd.tick() is False


def test_no_input_ever_yields_nothing(wd, clock):
    clock.advance(5.0)
    assert wd.tick() is False


def test_trip_supersedes_stale_zero(wd, clock):
    arm(wd)
    wd.filter_command(Twist6(linear_x=0.15))
    clock.advance(1.2)
    assert wd.tick() is False             # the trip zero (via consume_trip) covers it
    assert wd.consume_trip()


# ── state snapshot ────────────────────────────────────────────────────────

def test_snapshot_contract(wd, clock):
    snap = wd.snapshot()
    assert snap['state'] == DISARMED and snap['heartbeat_age_s'] is None
    assert snap['geofence'] == 'not_implemented'
    assert snap['release_count'] == 0
    arm(wd)
    clock.advance(0.4)
    wd.filter_command(Twist6(linear_x=1.0))
    snap = wd.snapshot()
    assert snap['state'] == ARMED
    assert snap['heartbeat_age_s'] == pytest.approx(0.4)
    assert snap['last_cmd_clamped'] is True
    assert snap['last_heartbeat_source'] == 'orchestrator'
    assert set(snap) >= {'state', 'reason', 'heartbeat_age_s', 'last_cmd_clamped',
                         'clamp_count', 'trip_count', 'release_count',
                         'max_linear', 'max_angular',
                         'heartbeat_timeout_s'}
