"""In-process ROS integration test for the watchdog node.

SAFETY: this test publishes on /cmd_vel_safe. It forces ROS_DOMAIN_ID=99 and
localhost-only discovery *before* rclpy initialises, and refuses to run
otherwise, so it can never reach the real robot (domain 5).
"""

import os
import time

os.environ['ROS_DOMAIN_ID'] = '99'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_STATIC_PEERS', None)
os.environ.pop('CYCLONEDDS_URI', None)  # interface-pinned to WiFi; not wanted here

import pytest  # noqa: E402

rclpy = pytest.importorskip('rclpy')

from action_msgs.msg import GoalInfo  # noqa: E402
from action_msgs.srv import CancelGoal  # noqa: E402
from diagnostic_msgs.msg import DiagnosticStatus  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy  # noqa: E402
from std_msgs.msg import Header  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from mark_five_safety.watchdog_node import SafetyWatchdogNode  # noqa: E402


@pytest.fixture
def harness():
    assert os.environ['ROS_DOMAIN_ID'] == '99'
    rclpy.init(args=['--ros-args', '-p', 'heartbeat_timeout:=0.5'])
    wd = SafetyWatchdogNode()
    tn = rclpy.create_node('safety_test_harness')
    ex = SingleThreadedExecutor()
    ex.add_node(wd)
    ex.add_node(tn)

    h = type('H', (), {})()
    h.wd, h.tn, h.ex = wd, tn, ex
    h.gated, h.states, h.cancels = [], [], []
    h.cmd_pub = tn.create_publisher(Twist, '/cmd_vel_safe', 10)
    h.hb_pub = tn.create_publisher(Header, '/agent/heartbeat', 10)
    tn.create_subscription(Twist, '/cmd_vel_gated', h.gated.append, 10)
    tn.create_subscription(
        DiagnosticStatus, '/safety/state', h.states.append,
        QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                   durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

    def fake_cancel(req, resp):
        h.cancels.append(req)
        resp.goals_canceling = [GoalInfo()]
        return resp
    tn.create_service(CancelGoal, '/navigate_to_pose/_action/cancel_goal', fake_cancel)
    h.reset_cli = tn.create_client(Trigger, '/safety/reset')
    h.release_cli = tn.create_client(Trigger, '/safety/release')

    def spin_for(seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            ex.spin_once(timeout_sec=0.02)
            if until is not None and until():
                return True
        return until is None
    h.spin_for = spin_for

    def send(lx=0.0, az=0.0):
        m = Twist()
        m.linear.x, m.angular.z = lx, az
        n = len(h.gated)
        h.cmd_pub.publish(m)
        assert spin_for(2.0, lambda: len(h.gated) > n), 'no gated output'
        return h.gated[-1]
    h.send = send

    def beat():
        hdr = Header()
        hdr.frame_id = 'test_orchestrator'
        h.hb_pub.publish(hdr)
    h.beat = beat

    def call(cli):
        fut = cli.call_async(Trigger.Request())
        assert spin_for(2.0, fut.done), 'no service response'
        return fut.result()
    h.call = call

    def value(key):
        return next(kv.value for kv in h.states[-1].values if kv.key == key)
    h.value = value

    def state():
        return h.states[-1].message if h.states else None
    h.state = state

    # Wait for discovery of the pubs/subs on both sides.
    assert spin_for(5.0, lambda: (h.cmd_pub.get_subscription_count() > 0
                                  and h.hb_pub.get_subscription_count() > 0
                                  and tn.count_publishers('/cmd_vel_gated') > 0
                                  and h.reset_cli.service_is_ready()
                                  and h.release_cli.service_is_ready()
                                  and wd.cancel_clients[0].service_is_ready()
                                  and h.states))
    yield h
    ex.shutdown()
    tn.destroy_node()
    wd.destroy_node()
    rclpy.shutdown()


def test_end_to_end(harness):
    h = harness
    assert h.state() == 'DISARMED'                       # latched state for late joiner

    # S4 while disarmed: clamped passthrough, other components zero.
    out = h.send(0.5, -3.0)
    assert out.linear.x == pytest.approx(0.20) and out.angular.z == pytest.approx(-1.0)
    assert h.spin_for(1.0, lambda: any(kv.key == 'last_cmd_clamped' and kv.value == 'true'
                                       for kv in h.states[-1].values))
    out = h.send(float('nan'), 0.1)
    assert out.linear.x == 0.0 and out.angular.z == 0.0

    # Arm on first beat.
    for _ in range(5):
        h.beat()
        h.spin_for(0.1)
    assert h.spin_for(1.0, lambda: h.state() == 'ARMED')
    assert h.send(0.1, 0.0).linear.x == pytest.approx(0.1)

    # Silence -> TRIPPED, zero published, Nav2 cancel-all requested.
    n = len(h.gated)
    assert h.spin_for(2.0, lambda: h.state() == 'TRIPPED')
    assert h.states[-1].level == DiagnosticStatus.ERROR
    assert h.spin_for(1.0, lambda: len(h.gated) > n)
    assert all(m.linear.x == 0.0 for m in h.gated[n:])
    assert h.spin_for(1.0, lambda: len(h.cancels) >= 1)
    assert h.cancels[0].goal_info.stamp.sec == 0           # zero id+stamp = cancel all

    # Latched: beats resuming do not clear it; motion stays blocked.
    for _ in range(5):
        h.beat()
        h.spin_for(0.1)
    assert h.state() == 'TRIPPED'
    out = h.send(0.1, 0.5)
    assert out.linear.x == 0.0 and out.angular.z == 0.0

    # Human reset -> DISARMED -> commands pass again.
    fut = h.reset_cli.call_async(Trigger.Request())
    assert h.spin_for(2.0, fut.done)
    assert fut.result().success
    assert h.spin_for(1.0, lambda: h.state() == 'DISARMED')
    assert h.send(0.1, 0.0).linear.x == pytest.approx(0.1)


def test_stale_input_gets_one_zero_then_silence(harness):
    h = harness
    h.send(0.1, 0.0)
    n = len(h.gated)
    h.spin_for(1.5)                                         # input_timeout is 0.5 s
    tail = h.gated[n:]
    assert len(tail) == 1 and tail[0].linear.x == 0.0


def test_release(harness):
    h = harness
    # DISARMED: no-op success, nothing published, nothing cancelled.
    n = len(h.gated)
    r = h.call(h.release_cli)
    assert r.success and 'nothing to release' in r.message
    h.spin_for(0.3)
    assert len(h.gated) == n and h.cancels == []

    # ARMED + moving -> release: one zero, cancel-all, DISARMED, no trip counted.
    for _ in range(3):
        h.beat()
        h.spin_for(0.1)
    assert h.spin_for(1.0, lambda: h.state() == 'ARMED')
    assert h.send(0.1, 0.0).linear.x == pytest.approx(0.1)
    n = len(h.gated)
    r = h.call(h.release_cli)
    assert r.success and 'released' in r.message
    assert h.spin_for(1.0, lambda: len(h.gated) > n)
    assert h.gated[n].linear.x == 0.0 and h.gated[n].angular.z == 0.0
    assert h.spin_for(1.0, lambda: len(h.cancels) >= 1)
    assert h.state() == 'DISARMED'
    assert h.value('release_count') == '1' and h.value('trip_count') == '0'

    # Not latched: silence well past heartbeat_timeout (0.5 s) does not trip,
    # and commands pass again.
    h.spin_for(1.2)
    assert h.state() == 'DISARMED' and h.value('trip_count') == '0'
    assert h.send(0.1, 0.0).linear.x == pytest.approx(0.1)

    # Next beat re-arms; silence then trips as usual.
    h.beat()
    assert h.spin_for(1.0, lambda: h.state() == 'ARMED')
    assert h.spin_for(2.0, lambda: h.state() == 'TRIPPED')

    # Release never clears a trip.
    r = h.call(h.release_cli)
    assert not r.success and '/safety/reset' in r.message
    h.spin_for(0.2)
    assert h.state() == 'TRIPPED' and h.value('release_count') == '1'
    out = h.send(0.1, 0.0)
    assert out.linear.x == 0.0
