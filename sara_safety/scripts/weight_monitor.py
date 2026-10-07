#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import WrenchStamped, Twist
from std_msgs.msg import Float32, Bool

GRAVITY = 9.80665

# Hard threshold: absolute weight -> immediate stop.
HARD_LOCK_KG = 20.0

# Soft threshold: rate of weight change -> warning + temporary speed
# reduction, not a hard stop.
RATE_SOFT_THRESHOLD_KG_S = 15.0
SOFT_SPEED_SCALE = 0.5      # speed factor applied while a soft event is active
SOFT_COOLDOWN_S = 2.0       # how long the reduction holds after the last trigger

TARE_SAMPLES = 20
RATE_SMOOTHING_SAMPLES = 3   # short moving average on dW/dt to reject sensor noise

STABILITY_WINDOW = 8
STABILITY_EPS_N = 0.03
MAX_TARE_WAIT_SAMPLES = 400  # ~20s at the sensor's 20Hz update_rate

class WeightMonitor(Node):
    def __init__(self):
        super().__init__('weight_monitor')

        self.declare_parameter('startup_payload_kg', 0.0)
        self.startup_payload_kg = float(
            self.get_parameter('startup_payload_kg').value)

        self.tare_force_z = None
        self.tare_samples = []
        self._pre_tare_window = []
        self._pre_tare_count = 0
        self._tare_stabilized = False

        self.prev_kg = None
        self.prev_stamp = None
        self.rate_history = []
        self.soft_active_until = None  # rclpy Time, sim or wall per use_sim_time

        self.wrench_sub = self.create_subscription(
            WrenchStamped,
            'cart_weight_wrench',
            self.wrench_callback,
            10)

        self.weight_pub = self.create_publisher(Float32, 'cart_cargo_weight_kg', 10)
        self.rate_pub = self.create_publisher(Float32, 'cart_cargo_weight_rate_kg_s', 10)
        self.scale_pub = self.create_publisher(Float32, 'cart_speed_scale', 10)
        self.warning_pub = self.create_publisher(Bool, 'cart_payload_warning', 10)
        self.block_pub = self.create_publisher(Twist, 'cmd_vel_block_all', 10)

        self.get_logger().info(
            f'weight_monitor started: soft rate threshold '
            f'{RATE_SOFT_THRESHOLD_KG_S}kg/s, hard lock at {HARD_LOCK_KG}kg')

    def wrench_callback(self, msg: WrenchStamped):
        force_z = abs(msg.wrench.force.z)

        if self.tare_force_z is None:
            if not self._wait_for_stable_signal(force_z):
                return
            self.tare_samples.append(force_z)
            if len(self.tare_samples) >= TARE_SAMPLES:
                settled_force_z = sum(self.tare_samples) / len(self.tare_samples)
                self.tare_force_z = settled_force_z - self.startup_payload_kg * GRAVITY
                self.get_logger().info(
                    f'Tare complete: settled={settled_force_z:.3f}N, '
                    f'startup_payload_kg={self.startup_payload_kg:.3f}kg -> '
                    f'baseline={self.tare_force_z:.3f}N '
                    f'({self.tare_force_z / GRAVITY:.3f}kg)')
            return

        net_kg = max(0.0, (force_z - self.tare_force_z) / GRAVITY)
        self.weight_pub.publish(Float32(data=net_kg))

        stamp = Time.from_msg(msg.header.stamp)
        rate = self._update_rate(net_kg, stamp)
        self.rate_pub.publish(Float32(data=rate))

        soft_active = self._update_soft_trigger(rate, stamp)
        self.warning_pub.publish(Bool(data=soft_active))
        self.scale_pub.publish(Float32(data=SOFT_SPEED_SCALE if soft_active else 1.0))

        if net_kg >= HARD_LOCK_KG:
            block = Twist()
            block.angular.y = 0.005  # mantiene el tópico activo, ver mux_locker.py
            self.block_pub.publish(block)

    def _wait_for_stable_signal(self, force_z: float) -> bool:
        if self._tare_stabilized:
            return True

        self._pre_tare_count += 1
        self._pre_tare_window.append(force_z)
        if len(self._pre_tare_window) > STABILITY_WINDOW:
            self._pre_tare_window.pop(0)

        settled = (len(self._pre_tare_window) == STABILITY_WINDOW and
                   (max(self._pre_tare_window) - min(self._pre_tare_window))
                   < STABILITY_EPS_N)
        timed_out = self._pre_tare_count >= MAX_TARE_WAIT_SAMPLES

        if not (settled or timed_out):
            return False

        if timed_out and not settled:
            self.get_logger().warn(
                f'Tare: signal never settled within {MAX_TARE_WAIT_SAMPLES} '
                f'samples (< {STABILITY_EPS_N}N over {STABILITY_WINDOW} '
                'samples) - forcing tare from the last '
                f'{len(self._pre_tare_window)} samples anyway to avoid '
                'hanging.')

        self._tare_stabilized = True
        return True

    def _update_rate(self, net_kg: float, stamp: Time) -> float:
        """Smoothed dW/dt in kg/s from consecutive tared-weight samples."""
        rate = 0.0
        if self.prev_kg is not None and self.prev_stamp is not None:
            dt = (stamp - self.prev_stamp).nanoseconds / 1e9
            if dt > 1e-3:
                rate = (net_kg - self.prev_kg) / dt
        self.prev_kg = net_kg
        self.prev_stamp = stamp

        self.rate_history.append(rate)
        if len(self.rate_history) > RATE_SMOOTHING_SAMPLES:
            self.rate_history.pop(0)
        return sum(self.rate_history) / len(self.rate_history)

    def _update_soft_trigger(self, rate: float, stamp: Time) -> bool:
        """True while a soft (rate-of-change) event is active or cooling down."""
        if abs(rate) >= RATE_SOFT_THRESHOLD_KG_S:
            self.soft_active_until = stamp + rclpy.duration.Duration(
                seconds=SOFT_COOLDOWN_S)
            self.get_logger().warn(
                f'Soft payload-interaction event: dW/dt={rate:.2f}kg/s')
        return self.soft_active_until is not None and stamp < self.soft_active_until


def main(args=None):
    rclpy.init(args=args)
    node = WeightMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
