#!/usr/bin/env python3
"""
Waypoint Mission Manager Node for Mark Five AMR

Provides a service-based API for controlling multi-waypoint autonomous missions
using Nav2's waypoint_follower infrastructure.

Services:
  /mission/start - Start loaded mission (std_srvs/Trigger)
  /mission/stop - Abort and reset mission (std_srvs/Trigger)
  /mission/pause - Pause execution (std_srvs/Trigger)
  /mission/resume - Resume from current waypoint (std_srvs/Trigger)
  /mission/set_loop - Enable/disable looping (std_srvs/SetBool)

Topics:
  Subscribers:
    /mission/waypoints (geometry_msgs/PoseArray) - Load waypoints
  Publishers:
    /mission/status (diagnostic_msgs/DiagnosticStatus) - Mission state/progress
    /mission/result (std_msgs/String) - Final result on completion/failure
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from enum import Enum

from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav2_msgs.action import NavigateThroughPoses


class MissionState(Enum):
    """Mission execution states"""
    IDLE = 0
    RUNNING = 1
    PAUSED = 2
    COMPLETED = 3
    FAILED = 4


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # Declare parameters
        self.declare_parameter('action_server_timeout', 900.0)
        self.declare_parameter('goal_response_timeout', 5.0)
        self.declare_parameter('status_publish_rate', 5.0)
        self.declare_parameter('loop_enabled_default', False)
        self.declare_parameter('min_waypoints', 1)
        self.declare_parameter('max_waypoints', 100)

        # Get parameters
        self.action_timeout = self.get_parameter('action_server_timeout').value
        self.goal_timeout = self.get_parameter('goal_response_timeout').value
        self.status_rate = self.get_parameter('status_publish_rate').value
        self.min_waypoints = self.get_parameter('min_waypoints').value
        self.max_waypoints = self.get_parameter('max_waypoints').value

        # Mission state
        self.state = MissionState.IDLE
        self.waypoints = []
        self.current_waypoint_index = 0
        self.total_waypoints = 0
        self.loop_enabled = self.get_parameter('loop_enabled_default').value
        self.loop_count = 0
        self.error_message = ""

        # Action client for Nav2
        self.nav_action_client = ActionClient(
            self,
            NavigateThroughPoses,
            '/navigate_through_poses'
        )
        self.nav_goal_handle = None

        # Service servers
        self.start_service = self.create_service(
            Trigger,
            '/mission/start',
            self.start_mission_callback
        )
        self.stop_service = self.create_service(
            Trigger,
            '/mission/stop',
            self.stop_mission_callback
        )
        self.pause_service = self.create_service(
            Trigger,
            '/mission/pause',
            self.pause_mission_callback
        )
        self.resume_service = self.create_service(
            Trigger,
            '/mission/resume',
            self.resume_mission_callback
        )
        self.set_loop_service = self.create_service(
            SetBool,
            '/mission/set_loop',
            self.set_loop_callback
        )

        # Subscribers
        self.waypoints_sub = self.create_subscription(
            PoseArray,
            '/mission/waypoints',
            self.waypoints_callback,
            10
        )

        # Publishers
        self.status_pub = self.create_publisher(
            DiagnosticStatus,
            '/mission/status',
            10
        )
        self.result_pub = self.create_publisher(
            String,
            '/mission/result',
            10
        )

        # Timer for status publishing
        timer_period = 1.0 / self.status_rate
        self.status_timer = self.create_timer(timer_period, self.publish_status)

        self.get_logger().info('Mission Manager initialized')
        self.get_logger().info(f'Waiting for waypoints on /mission/waypoints...')

    def waypoints_callback(self, msg: PoseArray):
        """Receive and validate waypoints"""
        if self.state == MissionState.RUNNING:
            self.get_logger().warn('Cannot load waypoints while mission is running')
            return

        num_waypoints = len(msg.poses)

        # Validate waypoint count
        if num_waypoints < self.min_waypoints:
            self.get_logger().error(
                f'Too few waypoints: {num_waypoints} (min: {self.min_waypoints})'
            )
            return

        if num_waypoints > self.max_waypoints:
            self.get_logger().error(
                f'Too many waypoints: {num_waypoints} (max: {self.max_waypoints})'
            )
            return

        # Convert PoseArray to list of PoseStamped
        self.waypoints = []
        for pose in msg.poses:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = pose
            self.waypoints.append(pose_stamped)

        self.total_waypoints = num_waypoints
        self.current_waypoint_index = 0
        self.state = MissionState.IDLE
        self.error_message = ""

        self.get_logger().info(
            f'Loaded {num_waypoints} waypoints in frame "{msg.header.frame_id}"'
        )

    def start_mission_callback(self, request, response):
        """Start the mission"""
        # Validate state
        if self.state not in [MissionState.IDLE, MissionState.COMPLETED, MissionState.FAILED]:
            response.success = False
            response.message = f'Cannot start from state {self.state.name}. Stop mission first.'
            self.get_logger().warn(response.message)
            return response

        # Validate waypoints
        if len(self.waypoints) == 0:
            response.success = False
            response.message = 'No waypoints loaded. Publish to /mission/waypoints first.'
            self.get_logger().error(response.message)
            return response

        # Check action server availability
        if not self.nav_action_client.wait_for_server(timeout_sec=self.goal_timeout):
            response.success = False
            response.message = 'Nav2 action server not available'
            self.get_logger().error(response.message)
            return response

        # Send navigation goal
        success = self.send_nav_goal()
        if success:
            response.success = True
            response.message = f'Mission started with {self.total_waypoints} waypoints'
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = 'Failed to send navigation goal'
            self.get_logger().error(response.message)

        return response

    def stop_mission_callback(self, request, response):
        """Stop and reset the mission"""
        if self.state == MissionState.IDLE:
            response.success = True
            response.message = 'Mission already idle'
            return response

        # Cancel active navigation
        if self.nav_goal_handle is not None:
            self.get_logger().info('Canceling active navigation goal...')
            cancel_future = self.nav_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)

        # Reset state
        self.state = MissionState.IDLE
        self.current_waypoint_index = 0
        self.loop_count = 0
        self.error_message = ""
        self.nav_goal_handle = None

        response.success = True
        response.message = 'Mission stopped and reset'
        self.get_logger().info(response.message)

        return response

    def pause_mission_callback(self, request, response):
        """Pause the mission"""
        if self.state != MissionState.RUNNING:
            response.success = False
            response.message = f'Cannot pause from state {self.state.name}'
            self.get_logger().warn(response.message)
            return response

        # Cancel current navigation
        if self.nav_goal_handle is not None:
            self.get_logger().info('Pausing mission - canceling navigation...')
            cancel_future = self.nav_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)

        self.state = MissionState.PAUSED
        response.success = True
        response.message = f'Mission paused at waypoint {self.current_waypoint_index}/{self.total_waypoints}'
        self.get_logger().info(response.message)

        return response

    def resume_mission_callback(self, request, response):
        """Resume the mission from current position"""
        if self.state != MissionState.PAUSED:
            response.success = False
            response.message = f'Cannot resume from state {self.state.name}'
            self.get_logger().warn(response.message)
            return response

        # Resume navigation from current waypoint
        success = self.send_nav_goal()
        if success:
            response.success = True
            response.message = f'Mission resumed from waypoint {self.current_waypoint_index}/{self.total_waypoints}'
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = 'Failed to resume mission'
            self.get_logger().error(response.message)

        return response

    def set_loop_callback(self, request, response):
        """Enable or disable mission looping"""
        self.loop_enabled = request.data
        response.success = True
        response.message = f'Loop mode {"enabled" if self.loop_enabled else "disabled"}'
        self.get_logger().info(response.message)
        return response

    def send_nav_goal(self):
        """Send navigation goal to Nav2"""
        if len(self.waypoints) == 0:
            self.get_logger().error('No waypoints to navigate')
            return False

        # Create goal message
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self.waypoints
        goal_msg.behavior_tree = ''  # Use default behavior tree

        # Send goal
        self.get_logger().info(f'Sending {len(self.waypoints)} waypoints to Nav2...')
        send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

        return True

    def nav_goal_response_callback(self, future):
        """Handle Nav2 action server response"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected by Nav2')
            self.state = MissionState.FAILED
            self.error_message = 'Goal rejected by Nav2'
            self.publish_result('FAILED: Goal rejected by Nav2')
            return

        self.get_logger().info('Navigation goal accepted by Nav2')
        self.nav_goal_handle = goal_handle
        self.state = MissionState.RUNNING
        self.current_waypoint_index = 0

        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        """Handle feedback from Nav2"""
        feedback = feedback_msg.feedback
        self.current_waypoint_index = feedback.current_waypoint

        # Log progress occasionally
        if self.current_waypoint_index % 5 == 0 or self.current_waypoint_index == 0:
            self.get_logger().info(
                f'Progress: waypoint {self.current_waypoint_index}/{self.total_waypoints}'
            )

    def nav_result_callback(self, future):
        """Handle Nav2 navigation result"""
        result = future.result().result
        status = future.result().status

        self.get_logger().info(f'Navigation completed with status: {status}')

        # Check for errors (error_code != 0 means failure)
        if result.error_code != 0:
            self.state = MissionState.FAILED
            self.error_message = f'Navigation failed with error code: {result.error_code}'
            self.get_logger().error(self.error_message)
            self.publish_result(f'FAILED: {self.error_message}')
            return

        # Success
        self.get_logger().info(
            f'Mission completed successfully! (Loop {self.loop_count + 1})'
        )
        self.loop_count += 1

        # Handle looping
        if self.loop_enabled:
            self.get_logger().info('Loop enabled - restarting mission...')
            self.current_waypoint_index = 0
            self.send_nav_goal()
        else:
            self.state = MissionState.COMPLETED
            self.publish_result(f'COMPLETED: Successfully navigated {self.total_waypoints} waypoints')

    def cancel_done_callback(self, future):
        """Handle goal cancellation result"""
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                self.get_logger().info('Navigation goal canceled successfully')
            else:
                self.get_logger().warn('Goal cancellation returned no goals')
        except Exception as e:
            self.get_logger().error(f'Error during goal cancellation: {e}')

    def publish_status(self):
        """Publish mission status (called by timer)"""
        status_msg = DiagnosticStatus()
        status_msg.name = 'mission_manager'
        status_msg.hardware_id = 'mark_five_amr'

        # Set level based on state
        if self.state == MissionState.RUNNING:
            status_msg.level = DiagnosticStatus.OK
            status_msg.message = f'Running waypoint {self.current_waypoint_index}/{self.total_waypoints}'
        elif self.state == MissionState.PAUSED:
            status_msg.level = DiagnosticStatus.WARN
            status_msg.message = f'Paused at waypoint {self.current_waypoint_index}/{self.total_waypoints}'
        elif self.state == MissionState.FAILED:
            status_msg.level = DiagnosticStatus.ERROR
            status_msg.message = self.error_message
        elif self.state == MissionState.COMPLETED:
            status_msg.level = DiagnosticStatus.OK
            status_msg.message = f'Completed {self.total_waypoints} waypoints'
        else:  # IDLE
            status_msg.level = DiagnosticStatus.OK
            status_msg.message = 'Idle - waiting for mission start'

        # Add key-value pairs
        status_msg.values = [
            KeyValue(key='state', value=self.state.name),
            KeyValue(key='current_waypoint', value=str(self.current_waypoint_index)),
            KeyValue(key='total_waypoints', value=str(self.total_waypoints)),
            KeyValue(key='progress_percent', value=str(
                round(100.0 * self.current_waypoint_index / self.total_waypoints, 1)
                if self.total_waypoints > 0 else 0.0
            )),
            KeyValue(key='loop_enabled', value=str(self.loop_enabled)),
            KeyValue(key='loop_count', value=str(self.loop_count)),
        ]

        self.status_pub.publish(status_msg)

    def publish_result(self, message: str):
        """Publish final mission result"""
        result_msg = String()
        result_msg.data = message
        self.result_pub.publish(result_msg)
        self.get_logger().info(f'Mission result: {message}')

    def destroy_node(self):
        """Clean up on shutdown"""
        if self.nav_goal_handle is not None:
            self.get_logger().info('Canceling active navigation on shutdown...')
            try:
                self.nav_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f'Could not cancel goal: {e}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
