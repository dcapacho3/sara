#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32

ZONE_RADIUS_M_DEFAULT = 0.3653
# The sensor's own <min> (sara.gazebo.xacro, re-measured 2026-09-25 to 0.2m)
# already excludes self-detection at the source, so this app-level filter is
# redundant defense-in-depth, not the active constraint. Must stay strictly
# below ZONE_RADIUS_M_DEFAULT or the valid window (min, radius] is empty -
# exactly the bug this file already hit once when both were set to 0.3.
MIN_VALID_RANGE_M_DEFAULT = 0.2
HYSTERESIS_DWELL_S_DEFAULT = 1.0


class ProximityStop(Node):
    def __init__(self):
        super().__init__('proximity_stop')

        self.declare_parameter('zone_radius_m', ZONE_RADIUS_M_DEFAULT)
        self.declare_parameter('min_valid_range_m', MIN_VALID_RANGE_M_DEFAULT)
        self.declare_parameter('hysteresis_dwell_s', HYSTERESIS_DWELL_S_DEFAULT)
        self.zone_radius_m = self.get_parameter('zone_radius_m').value
        self.min_valid_range_m = self.get_parameter('min_valid_range_m').value
        self.hysteresis_dwell_s = self.get_parameter('hysteresis_dwell_s').value

        self.nearest_range = None
        self.in_zone = False
        self.clear_since = None  # rclpy Time, set when the zone first clears
        self.zone_entry_count = 0

        # Control input - the only thing that decides whether to stop.
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Logging only (sec 5 trial metrics), never read by tick().
        self.robot_xy = None
        self.human_xy = None
        self.robot_odom_sub = self.create_subscription(
            Odometry, '/odom', self.robot_odom_callback, 10)
        self.human_odom_sub = self.create_subscription(
            Odometry, '/model/human_actor/odometry', self.human_odom_callback, 10)
        self.ground_truth_distance_pub = self.create_publisher(
            Float32, 'human_ground_truth_distance_m', 10)

        self.block_pub = self.create_publisher(Twist, 'cmd_vel_proximity_stop', 10)
        self.active_pub = self.create_publisher(Bool, 'proximity_stop_active', 10)
        self.nearest_range_pub = self.create_publisher(Float32, 'proximity_nearest_range_m', 10)
        self.entry_count_pub = self.create_publisher(Int32, 'proximity_stop_zone_entry_count', 10)

        self.timer = self.create_timer(0.05, self.tick)  # 20Hz, well under the 0.5s mux timeout

        self.get_logger().info(
            f'proximity_stop started: zone_radius={self.zone_radius_m}m (lidar-based), '
            f'hysteresis_dwell={self.hysteresis_dwell_s}s')

    def scan_callback(self, msg: LaserScan):
        # RE-DERIVED 2026-09-25, chasing two false triggers before landing
        # on the right fix. Chasing zone_radius/cone-angle numbers to dodge
        # specific obstacles encountered during testing (0.5m+narrow cone,
        # then 0.45m+45deg cone) was solving the wrong layer of the
        # problem: those obstacles (a wall 0.44-0.48m from spawn, aisle
        # shelving 0.49-0.61m away) were only ever close enough to matter
        # because ZONE_RADIUS_M had drifted well past the robot's own real
        # footprint, chasing the sensor's own 0.4m self-detection floor
        # (itself a "generous", not tightly measured, choice - see
        # sara.gazebo.xacro). With that floor re-measured and tightened to
        # 0.2m (true self-detection ceiling 0.172m + a real ~0.03m margin),
        # ZONE_RADIUS_M is back to its original, physically-grounded value:
        # footprint (0.2195m, farthest corner from the lidar) + a 0.08m
        # collision-imminent margin. At this radius both previous false
        # triggers clear by distance alone, angle irrelevant: the spawn
        # wall (0.44-0.48m) and the aisle shelf (0.49-0.61m) are both
        # outside 0.3m regardless of where they are.
        #
        # Rear-cone exclusion (135-225 deg, unchanged from the original
        # design) is kept as a second, coarser check - a static object
        # directly behind the robot that it's driving away from shouldn't
        # hold the zone active - but is no longer load-bearing for the
        # cases above now that the radius itself excludes them.
        rear_min, rear_max = 3 * math.pi / 4, 5 * math.pi / 4
        valid = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or not (self.min_valid_range_m < r <= self.zone_radius_m):
                continue
            angle = (msg.angle_min + i * msg.angle_increment) % (2 * math.pi)
            if rear_min <= angle <= rear_max:
                continue
            valid.append(r)
        self.nearest_range = min(valid) if valid else None

    def robot_odom_callback(self, msg: Odometry):
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def human_odom_callback(self, msg: Odometry):
        self.human_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _publish_ground_truth_distance(self):
        if self.robot_xy is None or self.human_xy is None:
            return
        dx = self.robot_xy[0] - self.human_xy[0]
        dy = self.robot_xy[1] - self.human_xy[1]
        self.ground_truth_distance_pub.publish(Float32(data=math.hypot(dx, dy)))

    def tick(self):
        self._publish_ground_truth_distance()  # logging only

        now = self.get_clock().now()
        detected = self.nearest_range is not None
        self.nearest_range_pub.publish(Float32(data=self.nearest_range if detected else -1.0))

        if detected:
            if not self.in_zone:
                self.zone_entry_count += 1
                self.get_logger().warn(
                    f'Proximity Stop: object in zone (entry #{self.zone_entry_count}), '
                    f'lidar range={self.nearest_range:.2f}m')
            self.in_zone = True
            self.clear_since = None
        else:
            if self.in_zone and self.clear_since is None:
                self.clear_since = now

        dwell_elapsed = (
            self.clear_since is not None and
            (now - self.clear_since).nanoseconds >= self.hysteresis_dwell_s * 1e9)

        if self.in_zone and dwell_elapsed:
            self.in_zone = False
            self.clear_since = None
            self.get_logger().info('Proximity Stop: hysteresis dwell elapsed, resuming')

        self.active_pub.publish(Bool(data=self.in_zone))
        self.entry_count_pub.publish(Int32(data=self.zone_entry_count))

        if self.in_zone:
            block = Twist()
            block.angular.y = 0.005  # mantiene el tópico activo, ver mux_locker.py/weight_monitor.py
            self.block_pub.publish(block)


def main(args=None):
    rclpy.init(args=args)
    node = ProximityStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
