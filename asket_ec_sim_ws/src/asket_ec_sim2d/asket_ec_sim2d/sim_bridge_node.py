"""
sim_bridge_node.py — Topic bridge between the 2D simulator and the Njord stack.

Four responsibilities:

1. /fromLL service (robot_localization interface)
   Converts GPS (lat, lon) → local (x, y) metres using the same Barcelona
   origin as simulator_node. Required by mission_manager to convert waypoints.

2. GPS bridge
   /sim2d/navsat → /mavros/global_position/raw/fix
   Same message type (sensor_msgs/NavSatFix), different topic name.

3. IMU synthesis
   Extracts the angular velocity from /sim2d/odom and publishes it as
   sensor_msgs/Imu on /imu/data so pid_controller can close the yaw loop.

4. PID output → simulator input
   /control/effort (Twist from pid_controller) → /cmd_vel (read by simulator)
   This is the final link that feeds the Njord control output back into the sim.
"""

import math

import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix

try:
    from robot_localization.srv import FromLL
    from geographic_msgs.msg import GeoPoint  # noqa: F401 — needed by FromLL
    _FROMLL_AVAILABLE = True
except ImportError:
    _FROMLL_AVAILABLE = False

# Must match simulator_node.py and buoy_simulator_node.py
ORIGIN_LAT    = 41.3851
ORIGIN_LON    = 2.1734
EARTH_RADIUS  = 6_371_000.0


def gps_to_local(lat, lon):
    """GPS (lat, lon) → local (x, y) metres from the Barcelona origin."""
    lat_rad = math.radians(ORIGIN_LAT)
    x = (lon - ORIGIN_LON) * math.cos(lat_rad) * EARTH_RADIUS * math.pi / 180.0
    y = (lat - ORIGIN_LAT) * EARTH_RADIUS * math.pi / 180.0
    return x, y


class SimBridgeNode(Node):

    def __init__(self):
        super().__init__('sim_bridge')

        # --- /fromLL service ---
        if _FROMLL_AVAILABLE:
            self.create_service(FromLL, '/fromLL', self._fromll_cb)
            self.get_logger().info('/fromLL service ready')
        else:
            self.get_logger().warn(
                'robot_localization not found — /fromLL service unavailable. '
                'Install with: sudo apt install ros-jazzy-robot-localization')

        # --- GPS bridge ---
        self._pub_gps = self.create_publisher(
            NavSatFix, '/mavros/global_position/raw/fix', 10)
        self.create_subscription(
            NavSatFix, '/sim2d/navsat', self._navsat_cb, 10)

        # --- IMU synthesis ---
        self._pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.create_subscription(
            Odometry, '/sim2d/odom', self._odom_cb, 10)

        # --- PID effort → /cmd_vel ---
        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            Twist, '/control/effort', self._effort_cb, 10)

        self.get_logger().info('SimBridge running')

    # ------------------------------------------------------------------

    def _fromll_cb(self, request, response):
        x, y = gps_to_local(
            request.ll_point.latitude,
            request.ll_point.longitude,
        )
        response.map_point = Point(x=x, y=y, z=0.0)
        self.get_logger().debug(
            f'/fromLL ({request.ll_point.latitude:.6f}, '
            f'{request.ll_point.longitude:.6f}) → ({x:.1f}, {y:.1f}) m')
        return response

    def _navsat_cb(self, msg):
        self._pub_gps.publish(msg)

    def _odom_cb(self, msg):
        imu = Imu()
        imu.header = msg.header
        imu.angular_velocity.z      = msg.twist.twist.angular.z
        imu.orientation_covariance[0] = -1.0   # orientation unknown
        self._pub_imu.publish(imu)

    def _effort_cb(self, msg):
        self._pub_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
