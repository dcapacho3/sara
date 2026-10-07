#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

MIN_CREEP_MPS_DEFAULT = 0.07

# Re-derivado 2026-09-26 para igualar el zone_radius_m actual de
# Proximity Stop (0.3m, footprint-derivado) - ver encabezado.
MIN_STANDOFF_M_DEFAULT = 0.3653
MAX_LINEAR_MPS_DEFAULT = 0.15  # = speed_limit.py's MAX_LINEAR_SPEED
# Re-derivado 2026-09-26 para igualar el self-detection floor actual de
# proximity_stop.py (0.2m, ver su propio encabezado) - antes en 0.3m.
MIN_VALID_RANGE_M_DEFAULT = 0.2

REACTION_TIME_S_DEFAULT = 0.3
HUMAN_APPROACH_MPS_DEFAULT = 1.4

# ---------------------------------------------------------------------------
# Empirical braking-deceleration model (docs/data/braking_battery.csv,
# docs/data/braking_decel_model.json - see the module docstring above for
# the full derivation). OLS fit of max_decel_mps2 ~ payload_mass_kg over all
# 70 trials across six masses (n=16 @0kg, n=8 @5kg, n=15 @10kg, n=8 @12kg,
# n=15 @15kg, n=8 @18kg):
#   decel_fit(m) = DECEL_INTERCEPT_MPS2 + DECEL_SLOPE_MPS2_PER_KG * m
# Control uses a one-sided 95% lower prediction bound on that fit, not the
# fit itself - see safe_decel_mps2().
# ---------------------------------------------------------------------------
DECEL_INTERCEPT_MPS2 = 5.916203793018501
DECEL_SLOPE_MPS2_PER_KG = -0.1346629270344021
DECEL_RESID_STD_MPS2 = 0.4268896728727216    # s, residual std error, dof=68
DECEL_MASS_MEAN_KG = 9.357142857142858       # x-bar over the 70 trials
DECEL_SXX = 2690.071428571429                # sum((m_i - x-bar)^2)
DECEL_N_TRIALS = 70
DECEL_TCRIT_95 = 1.6675722806611726          # scipy.stats.t.ppf(0.95, df=68)

DECEL_FIT_MAX_MASS_KG = 18.0  # heaviest mass the braking battery tested
MIN_DECEL_MPS2 = 0.3  # defensive floor; not reached within 0-18kg by the fit


def safe_decel_mps2(mass_kg):
    """Conservative (one-sided 95% lower prediction bound) achievable
    braking deceleration at a given carried mass, from the empirical
    braking battery. See module header for the full derivation."""
    m = max(0.0, min(mass_kg, DECEL_FIT_MAX_MASS_KG))
    fit = DECEL_INTERCEPT_MPS2 + DECEL_SLOPE_MPS2_PER_KG * m
    se_pred = DECEL_RESID_STD_MPS2 * math.sqrt(
        1.0 + 1.0 / DECEL_N_TRIALS + (m - DECEL_MASS_MEAN_KG) ** 2 / DECEL_SXX)
    return max(MIN_DECEL_MPS2, fit - DECEL_TCRIT_95 * se_pred)


class AdaptiveSeparation(Node):
    def __init__(self):
        super().__init__('adaptive_separation')

        self.declare_parameter('reaction_time_s', REACTION_TIME_S_DEFAULT)
        self.declare_parameter('human_approach_mps', HUMAN_APPROACH_MPS_DEFAULT)
        self.declare_parameter('min_standoff_m', MIN_STANDOFF_M_DEFAULT)
        self.declare_parameter('max_linear_mps', MAX_LINEAR_MPS_DEFAULT)
        self.declare_parameter('min_valid_range_m', MIN_VALID_RANGE_M_DEFAULT)
        self.declare_parameter('min_creep_mps', MIN_CREEP_MPS_DEFAULT)

        self.t_r = self.get_parameter('reaction_time_s').value
        self.v_h = self.get_parameter('human_approach_mps').value
        self.d_min = self.get_parameter('min_standoff_m').value
        self.v_max_platform = self.get_parameter('max_linear_mps').value
        self.min_creep = self.get_parameter('min_creep_mps').value
        self.min_valid_range = self.get_parameter('min_valid_range_m').value

        a_unloaded = safe_decel_mps2(0.0)
        crossover_s_unloaded = self._solve_crossover_distance(a_unloaded)
        s_zero = self.d_min + self.t_r * self.v_h
        self.get_logger().info(
            f'adaptive_separation started: a(m)=empirical 95% lower prediction '
            f'bound from braking_battery.csv (a(0kg)={a_unloaded:.3f}m/s^2, '
            f're-evaluated every tick from live cart_cargo_weight_kg), '
            f'T_r={self.t_r}s, v_h={self.v_h}m/s, '
            f'd_min={self.d_min}m (= Proximity Stop zone radius), '
            f'v_max=min_creep({self.min_creep}m/s) below S={s_zero:.2f}m, '
            f'v_max saturates at {self.v_max_platform}m/s beyond '
            f'S~={crossover_s_unloaded:.2f}m unloaded (rises with carried mass)')

        self.nearest_range = None
        self.cargo_kg = None

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.weight_sub = self.create_subscription(
            Float32, 'cart_cargo_weight_kg', self.weight_callback, 10)

        self.vmax_pub = self.create_publisher(Float32, 'adaptive_separation_v_max_mps', 10)
        self.range_pub = self.create_publisher(Float32, 'adaptive_separation_nearest_range_m', 10)

        self.timer = self.create_timer(0.05, self.tick)  # 20Hz

    def _solve_crossover_distance(self, a):
        """S at which the v_max formula (at a given deceleration a) reaches
        v_max_platform - purely informative, not used in the control loop."""
        target = self.v_max_platform
        rhs = target + a * self.t_r + self.v_h
        return self.d_min + (rhs ** 2 - (a * self.t_r) ** 2 - self.v_h ** 2) / (2 * a)

    def scan_callback(self, msg: LaserScan):
        # Rear-cone exclusion, same convention proximity_stop.py and
        # naive_obstacle_avoidance.py already use (135-225 degrees ignored):
        # a static object directly behind the robot that it's driving away
        # from shouldn't scale down the speed limit.
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

    def weight_callback(self, msg: Float32):
        self.cargo_kg = msg.data

    def _v_max(self, a, S):
        if S is None:
            return self.v_max_platform
        inner = (a * self.t_r) ** 2 + self.v_h ** 2 + 2 * a * (S - self.d_min)
        if inner < 0:
            return self.min_creep
        v = -(a * self.t_r + self.v_h) + math.sqrt(inner)
        return max(self.min_creep, min(v, self.v_max_platform))

    def tick(self):
        S = self.nearest_range
        self.range_pub.publish(Float32(data=S if S is not None else -1.0))

        a = safe_decel_mps2(self.cargo_kg or 0.0)
        v_max = self._v_max(a, S)
        self.vmax_pub.publish(Float32(data=v_max))


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveSeparation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
