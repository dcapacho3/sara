#!/usr/bin/env python3
# Autor: David Capacho Parra
# Descripción: Monitor de peso de carga para el robot SARA (simulación) -
# implementación de "Payload Interaction" (paper 2, docs/paper2_draft.md §3.3):
#
#   "The platform must continuously monitor its carried payload. A
#    rate-of-change in weight exceeding a soft threshold must trigger a
#    warning or speed reduction, distinguishing an active human interaction
#    (a shove, a lean, a person steadying themselves on the cart) from a
#    normal item placement or removal. An absolute weight exceeding a hard
#    threshold must trigger an immediate stop that takes precedence over
#    every other mode, including Operator Override."
#
# Two independent triggers, matching that requirement exactly (not a
# continuous magnitude-based ramp - the requirement is a rate condition and
# an absolute condition, not a blended one):
#   - SOFT (rate-of-change): |dW/dt| over a short window exceeds
#     RATE_SOFT_THRESHOLD_KG_S -> temporary speed reduction (cart_speed_scale)
#     held for a cooldown window, plus a warning flag (cart_payload_warning)
#     for trial logging (Section 5's soft/hard detection-rate and
#     false-positive-rate metrics need this to log against).
#   - HARD (absolute): net weight >= HARD_LOCK_KG -> immediate stop via
#     'cmd_vel_block_all' (twist_mux priority 255), the same mechanism
#     mux_locker.py uses for the real HX711 sensor, so no twist_mux/mux.yaml
#     changes are needed - the hard stop and the speed reduction both
#     compose through the existing mux the way the paper's §4.1 describes:
#     the hard stop is a priority-topic veto, the soft reduction is a scale
#     factor applied downstream of the mux by speed_limit.py's filter node.
#
# Reads the Gazebo force_torque sensor at cart_weight_joint (the point where,
# per the SolidWorks assembly, the real load cell sits: base_weight =
# "TopeBascula", the load cell's contact plate; base_platform = the fixed
# side it presses against), converts to a net cargo weight in kg (auto-tared
# at startup to remove the structure's own weight, same as zeroing a real
# scale).
#
# Thresholds (simulation parameters, not physical constants - Section 5's
# planned trials, "simulated payload add/remove events at varying rates and
# magnitudes," are exactly what would calibrate these; the values below are
# reasoned starting points, not final calibrated numbers):
#   - HARD_LOCK_KG = 20kg: the TurtleBot3 Waffle Pi's official structural
#     max payload is 30kg (ROBOTIS datasheet); 20kg leaves a 10kg/33% margin
#     below that, independent of the real prototype's 5kg-rated HX711 load
#     cell (this describes the simulation, not the physical prototype).
#   - RATE_SOFT_THRESHOLD_KG_S = 15kg/s: a normal gentle item placement
#     (even a multi-kg item, lowered by hand over roughly half a second to a
#     second) implies single-digit kg/s; a shove, lean, or a person
#     steadying themselves on the cart applies force far more abruptly.
#     15kg/s sits clearly above the former and below the latter as a
#     starting point.

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


class WeightMonitor(Node):
    def __init__(self):
        super().__init__('weight_monitor')

        self.tare_force_z = None
        self.tare_samples = []

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

        # Auto-tara: promedia las primeras lecturas (peso propio de
        # base_weight en reposo) para que el peso publicado sea solo la
        # carga añadida, igual que se hace con una báscula real.
        if self.tare_force_z is None:
            self.tare_samples.append(force_z)
            if len(self.tare_samples) >= TARE_SAMPLES:
                self.tare_force_z = sum(self.tare_samples) / len(self.tare_samples)
                self.get_logger().info(
                    f'Tare complete: baseline={self.tare_force_z:.3f}N '
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
