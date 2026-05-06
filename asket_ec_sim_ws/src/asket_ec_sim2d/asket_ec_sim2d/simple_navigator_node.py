"""
simple_navigator_node.py — Minimal NavigateToPose action server.

Replaces the full Nav2 stack so that mission_manager can sequence waypoints
without requiring Nav2, AMCL, or a costmap.

Accepts NavigateToPose goals (map frame, metres — set by mission_manager after
the /fromLL GPS conversion) and steers the boat toward each goal using the same
pure-pursuit algorithm as waypoint_navigator_node.

Publishes /njord/cmd_vel (remapped to /cmd_vel on nav_to_pid in the launch
file, so it enters the Njord PID pipeline without conflicting with /cmd_vel
used elsewhere).
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

GOAL_TOLERANCE = 2.0    # metres — declare success when this close
MAX_SPEED      = 1.5    # m/s forward
MAX_YAW_RATE   = 1.0    # rad/s
YAW_GAIN       = 2.0    # proportional gain for heading correction
CONTROL_HZ     = 10     # navigation loop rate


def _yaw_from_quat(qz, qw):
    return 2.0 * math.atan2(qz, qw)


def _normalize(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class SimpleNavigatorNode(Node):
    """
    Minimal action server implementing the NavigateToPose interface.

    Uses a ReentrantCallbackGroup so the pose subscription keeps updating
    while the action execute loop is running in a MultiThreadedExecutor.
    """

    def __init__(self):
        super().__init__('simple_navigator')

        self._cb_group = ReentrantCallbackGroup()

        self._boat_x       = 0.0
        self._boat_y       = 0.0
        self._boat_heading = 0.0
        self._pose_ok      = False

        self.create_subscription(
            PoseStamped, '/sim2d/pose', self._pose_cb, 10,
            callback_group=self._cb_group)

        self._pub = self.create_publisher(Twist, '/njord/cmd_vel', 10)

        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            'SimpleNavigator ready — publishing on /njord/cmd_vel')

    # ------------------------------------------------------------------

    def _pose_cb(self, msg):
        self._boat_x = msg.pose.position.x
        self._boat_y = msg.pose.position.y
        self._boat_heading = _yaw_from_quat(
            msg.pose.orientation.z, msg.pose.orientation.w)
        self._pose_ok = True

    # ------------------------------------------------------------------

    def _execute(self, goal_handle):
        target = goal_handle.request.pose.pose.position
        gx, gy = target.x, target.y
        self.get_logger().info(f'New goal: ({gx:.1f}, {gy:.1f}) m')

        feedback = NavigateToPose.Feedback()
        dt = 1.0 / CONTROL_HZ

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                self.get_logger().info('Goal cancelled')
                return NavigateToPose.Result()

            if not self._pose_ok:
                time.sleep(dt)
                continue

            dx   = gx - self._boat_x
            dy   = gy - self._boat_y
            dist = math.hypot(dx, dy)

            if dist < GOAL_TOLERANCE:
                self._stop()
                goal_handle.succeed()
                self.get_logger().info(
                    f'Goal reached ({gx:.1f}, {gy:.1f}) — dist={dist:.2f} m')
                return NavigateToPose.Result()

            # Pure pursuit — slow down when badly off-heading
            desired = math.atan2(dy, dx)
            err     = _normalize(desired - self._boat_heading)

            cmd = Twist()
            cmd.linear.x  = MAX_SPEED * max(0.3, 1.0 - abs(err) / math.pi)
            cmd.angular.z = max(-MAX_YAW_RATE,
                                min(MAX_YAW_RATE, YAW_GAIN * math.sin(err)))
            self._pub.publish(cmd)

            feedback.distance_remaining = dist
            goal_handle.publish_feedback(feedback)

            time.sleep(dt)

        goal_handle.abort()
        return NavigateToPose.Result()

    def _stop(self):
        self._pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigatorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
