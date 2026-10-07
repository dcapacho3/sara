#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

MIN_VALID_RANGE_M_DEFAULT = 0.2
MIN_CREEP_MPS_DEFAULT = 0.07
MAX_LINEAR_MPS_DEFAULT = 0.15
COMFORT_NEAR_M_DEFAULT = 0.3653
COMFORT_FAR_M_DEFAULT = 3.0


def smoothstep(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


class AdaptiveSeparationComfort(Node):
    def __init__(self):
        super().__init__('adaptive_separation_comfort')

        self.declare_parameter('min_valid_range_m', MIN_VALID_RANGE_M_DEFAULT)
        self.declare_parameter('min_creep_mps', MIN_CREEP_MPS_DEFAULT)
        self.declare_parameter('max_linear_mps', MAX_LINEAR_MPS_DEFAULT)
        self.declare_parameter('comfort_near_m', COMFORT_NEAR_M_DEFAULT)
        self.declare_parameter('comfort_far_m', COMFORT_FAR_M_DEFAULT)

        self.min_valid_range = self.get_parameter('min_valid_range_m').value
        self.min_creep = self.get_parameter('min_creep_mps').value
        self.v_max_platform = self.get_parameter('max_linear_mps').value
        self.near_m = self.get_parameter('comfort_near_m').value
        self.far_m = self.get_parameter('comfort_far_m').value

        self.get_logger().info(
            f'adaptive_separation_comfort started: smoothstep ramp from '
            f'{self.min_creep}m/s at S<={self.near_m}m to '
            f'{self.v_max_platform}m/s at S>={self.far_m}m '
            f'(comparison-only, not the deployed mechanism)')

        self.nearest_range = None
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.vmax_pub = self.create_publisher(
            Float32, 'adaptive_separation_comfort_v_max_mps', 10)
        self.range_pub = self.create_publisher(
            Float32, 'adaptive_separation_comfort_range_m', 10)
        self.timer = self.create_timer(0.05, self.tick)  # 20Hz, same rate

    def scan_callback(self, msg: LaserScan):
        rear_min, rear_max = 3 * math.pi / 4, 5 * math.pi / 4
        valid = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= self.min_valid_range:
                continue
            angle = (msg.angle_min + i * msg.angle_increment) % (2 * math.pi)
            if rear_min <= angle <= rear_max:
                continue
            valid.append(r)
        self.nearest_range = min(valid) if valid else None

    def _v_max(self, S):
        if S is None:
            return self.v_max_platform
        x = (S - self.near_m) / (self.far_m - self.near_m)
        return self.min_creep + (self.v_max_platform - self.min_creep) * smoothstep(x)

    def tick(self):
        S = self.nearest_range
        self.range_pub.publish(Float32(data=S if S is not None else -1.0))
        self.vmax_pub.publish(Float32(data=self._v_max(S)))


def main():
    rclpy.init()
    node = AdaptiveSeparationComfort()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
