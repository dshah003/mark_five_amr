#!/usr/bin/env python3
"""
Safety watchdog node for the Mark Five AMR (PLAN §7: S2 heartbeat + S4 velocity re-cap).

Last hop of the velocity pipeline, on the Jetson:

  ... collision_monitor -> /cmd_vel_safe -> [safety_watchdog] -> /cmd_vel_gated -> serial_bridge -> Arduino

All decisions live in `watchdog_logic.SafetyWatchdog`; this node only moves
messages in and out.

Interfaces
----------
Sub  /cmd_vel_safe      geometry_msgs/Twist   (param input_topic)
Sub  /agent/heartbeat   std_msgs/Header       (param heartbeat_topic)
       Contract: the orchestrator publishes at 5 Hz while it wants the robot
       to be allowed to move (PLAN §8.3). frame_id = sender name (e.g.
       "orchestrator"); stamp is informational only -- liveness uses local
       receipt time. Any QoS works (this subscription is best-effort, which
       matches both reliable and best-effort publishers).
Pub  /cmd_vel_gated     geometry_msgs/Twist   (param output_topic)
Pub  /safety/state      diagnostic_msgs/DiagnosticStatus, transient-local depth 1,
       on change and at state_publish_rate (1 Hz).
       name="mark_five_safety", message=<DISARMED|ARMED|TRIPPED>,
       level OK (DISARMED/ARMED) or ERROR (TRIPPED),
       values: state, reason, heartbeat_age_s ("none" if never),
       last_heartbeat_source, last_cmd_clamped, clamp_count, trip_count,
       release_count, max_linear, max_angular, heartbeat_timeout_s, geofence.
Srv  /safety/reset      std_srvs/Trigger -- TRIPPED -> DISARMED (human only).
Srv  /safety/release    std_srvs/Trigger -- non-latching ARMED -> DISARMED, sent by
       a cleanly-stopping orchestrator (agents/heartbeat.py Heartbeat.stop(); never
       an MCP tool). Zero once + cancel-all goals like a trip, but no latch and no
       trip_count. Never clears TRIPPED (success=false). DISARMED: no-op,
       success=true. If the heartbeat was already overdue it trips instead
       (success=false). Re-arms on the next heartbeat.

On a trip: publish zero once, then cancel all goals on each action in
`cancel_actions` via its `<action>/_action/cancel_goal` service (zero goal id +
zero stamp = cancel all). Missing servers (Nav2 not running) are skipped.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from action_msgs.srv import CancelGoal

from mark_five_safety.watchdog_logic import (
    SafetyWatchdog, WatchdogConfig, Twist6, TRIPPED)

DEFAULT_CANCEL_ACTIONS = [
    '/navigate_to_pose',
    '/navigate_through_poses',
    '/follow_waypoints',
    '/spin',
    '/backup',
    '/drive_on_heading',
]


def _fmt(value) -> str:
    if value is None:
        return 'none'
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


class SafetyWatchdogNode(Node):
    def __init__(self):
        super().__init__('safety_watchdog')

        self.declare_parameter('input_topic', '/cmd_vel_safe')
        self.declare_parameter('output_topic', '/cmd_vel_gated')
        self.declare_parameter('heartbeat_topic', '/agent/heartbeat')
        self.declare_parameter('max_linear_velocity', 0.20)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('heartbeat_timeout', 1.0)
        self.declare_parameter('input_timeout', 0.5)
        self.declare_parameter('check_rate', 20.0)
        self.declare_parameter('state_publish_rate', 1.0)
        self.declare_parameter('cancel_actions', DEFAULT_CANCEL_ACTIONS)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        config = WatchdogConfig(
            max_linear=float(p('max_linear_velocity')),
            max_angular=float(p('max_angular_velocity')),
            heartbeat_timeout=float(p('heartbeat_timeout')),
            input_timeout=float(p('input_timeout')),
        )
        self.wd = SafetyWatchdog(config, clock=time.monotonic)

        self.cmd_pub = self.create_publisher(Twist, p('output_topic'), 10)
        state_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_pub = self.create_publisher(DiagnosticStatus, '/safety/state', state_qos)

        self.create_subscription(Twist, p('input_topic'), self._on_cmd, 10)
        hb_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Header, p('heartbeat_topic'), self._on_heartbeat, hb_qos)
        self.create_service(Trigger, '/safety/reset', self._on_reset)
        self.create_service(Trigger, '/safety/release', self._on_release)

        self.cancel_clients = [
            self.create_client(CancelGoal, f'{name.rstrip("/")}/_action/cancel_goal')
            for name in p('cancel_actions') if name
        ]

        self.create_timer(1.0 / float(p('check_rate')), self._on_check)
        self.create_timer(1.0 / float(p('state_publish_rate')), self._publish_state)

        self._last_published_key = None
        self._publish_state()
        self.get_logger().info(
            f'Safety watchdog: {p("input_topic")} -> {p("output_topic")}, caps '
            f'{config.max_linear:.2f} m/s / {config.max_angular:.2f} rad/s, heartbeat '
            f'{p("heartbeat_topic")} timeout {config.heartbeat_timeout:.2f}s. DISARMED until first beat.')

    # ── callbacks ────────────────────────────────────────────────────────

    def _on_cmd(self, msg: Twist):
        result = self.wd.filter_command(Twist6(
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z))
        self._handle_trip()

        out = Twist()
        out.linear.x = float(result.linear_x)
        out.angular.z = float(result.angular_z)
        self.cmd_pub.publish(out)

        if result.clamped:
            self.get_logger().warn(
                f'Clamped command lin={msg.linear.x:.3f} ang={msg.angular.z:.3f} -> '
                f'lin={result.linear_x:.3f} ang={result.angular_z:.3f}',
                throttle_duration_sec=2.0)
        if result.rejected:
            self.get_logger().error('Rejected non-finite velocity command; sent zero',
                                    throttle_duration_sec=2.0)
        self._publish_state_if_changed()

    def _on_heartbeat(self, msg: Header):
        self.wd.heartbeat(msg.frame_id)
        self._handle_trip()
        self._publish_state_if_changed()

    def _on_check(self):
        if self.wd.tick():
            self._publish_zero()
        self._handle_trip()
        self._publish_state_if_changed()

    def _on_reset(self, request, response):
        response.success, response.message = self.wd.reset()
        self.get_logger().warn(f'/safety/reset: {response.message}')
        self._publish_state()
        return response

    def _on_release(self, request, response):
        source = self.wd.last_heartbeat_source
        response.success, response.message = self.wd.release(source)
        self._handle_trip()          # a release after the deadline trips instead
        if self.wd.consume_release():
            self._publish_zero()
            self.get_logger().warn(f'/safety/release: {response.message}; cancelling goals')
            self._cancel_all_goals()
        else:
            self.get_logger().warn(f'/safety/release: {response.message}')
        self._publish_state()
        return response

    # ── helpers ──────────────────────────────────────────────────────────

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())

    def _handle_trip(self):
        if not self.wd.consume_trip():
            return
        self._publish_zero()
        self.get_logger().error(f'SAFETY TRIP: {self.wd.reason}. Motion blocked until '
                                f'/safety/reset is called by a human.')
        self._cancel_all_goals()
        self._publish_state()

    def _cancel_all_goals(self):
        for client in self.cancel_clients:
            if not client.service_is_ready():
                self.get_logger().info(f'{client.srv_name} not available; skipping cancel')
                continue
            future = client.call_async(CancelGoal.Request())  # zero id + stamp = cancel all
            future.add_done_callback(
                lambda f, name=client.srv_name: self._on_cancel_done(name, f))

    def _on_cancel_done(self, name, future):
        try:
            resp = future.result()
            n = len(resp.goals_canceling) if resp is not None else 0
            self.get_logger().warn(f'{name}: cancel-all requested, {n} goal(s) canceling')
        except Exception as e:  # noqa: BLE001 -- never let a cancel failure kill the node
            self.get_logger().error(f'{name}: cancel failed: {e}')

    def _publish_state_if_changed(self):
        key = (self.wd.state, self.wd.reason, self.wd.last_clamped)
        if key != self._last_published_key:
            self._publish_state()

    def _publish_state(self):
        snap = self.wd.snapshot()
        msg = DiagnosticStatus()
        msg.name = 'mark_five_safety'
        msg.hardware_id = 'safety_watchdog'
        msg.level = DiagnosticStatus.ERROR if snap['state'] == TRIPPED else DiagnosticStatus.OK
        msg.message = snap['state']
        msg.values = [KeyValue(key=k, value=_fmt(v)) for k, v in snap.items()]
        self.state_pub.publish(msg)
        self._last_published_key = (self.wd.state, self.wd.reason, self.wd.last_clamped)

    def destroy_node(self):
        try:
            self._publish_zero()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
