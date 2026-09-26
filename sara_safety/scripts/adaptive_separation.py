#!/usr/bin/env python3
# Autor: David Capacho Parra
# Descripción: Adaptive Separation (paper 2, docs/paper2_draft.tex sec 3.2) -
# implementación de "Speed and Separation Monitoring": mientras hay un
# objeto en la zona de interacción (más amplia que la zona de Proximity
# Stop), la velocidad máxima permitida se reduce continuamente con la
# distancia, en vez de solo parar/no-parar como Proximity Stop.
#
# Detección por lidar (/scan), mismo principio que proximity_stop.py: el
# robot real no tiene acceso a una posición ground-truth de una persona, así
# que la entrada de control tiene que ser algo que el robot real pudiera
# medir de verdad.
#
# RE-DERIVACIÓN 2026-09-26, al restaurar este nodo desde el backup
# pre-reset (2026-09-22) sobre el baseline actual de Proximity Stop
# (footprint 0.2195m + margen 0.08m = ZONE_RADIUS_M=0.3m, self-detection
# floor 0.2m, ver proximity_stop.py). El backup traía dos constantes
# derivadas contra un baseline que ya no existe:
#   - MIN_VALID_RANGE_M estaba en 0.3m ("mismo filtro que proximity_stop.py"
#     - cierto en su momento, pero proximity_stop.py ya bajó el suyo a
#     0.2m tras re-medir el self-detection floor real; se actualiza aquí
#     para seguir siendo el mismo filtro, no una copia congelada).
#   - MIN_STANDOFF_M (d_min) estaba en 0.25m, resultado de una "tercera
#     re-derivación" contra el ancho de pasillo (ver el bloque de
#     comentario más abajo, conservado tal cual por su propio valor
#     como historial) - pero proximity_stop.py's propio encabezado
#     (RE-DERIVACIÓN 2026-09-25) ya deja escrito explícitamente que "el
#     margen de frenado... se movió a adaptive_separation.py's
#     min_standoff_m, CON LA MISMA CORRECCIÓN DE HUELLA APLICADA ALLÁ" -
#     es decir, d_min debería ser el radio de zona actual de Proximity
#     Stop (0.3m), no el valor de pasillo de la sesión anterior. Corregido
#     a 0.3m aquí. Ver la verificación de pasillo estrecho más abajo antes
#     de confiar en que esto no reintroduce el problema que motivó el
#     0.25m original.
#
# BRAKE_DECEL_MPS2: re-medido en vivo 2026-09-26 contra el baseline actual
# (TestEmpty.world, mu=0.7 por rueda - sin cambios respecto a la medición
# original, pero re-medido de todas formas por la regla de no reusar datos
# pre-reset). Misma metodología: pose ground-truth del chasis
# (/world/default/pose/info), NO velocidad de rueda/odom (ver nota más
# abajo del porqué). Tres ensayos: 4.82, 6.56, 6.42 m/s^2 (máximo de
# pendiente entre muestras consecutivas, cota inferior de la desaceleración
# real, no un valor exacto - mismo encuadre que la medición original). El
# máximo de los tres (6.56 m/s^2) está a un 4.5% del valor teórico
# limitado por tracción (mu*g = 0.7*9.80665 ~= 6.865 m/s^2) - consistente
# con ese régimen, igual que la medición original (que dio 4.41 m/s^2 bajo
# el mismo mu=0.7, antes del reset). El régimen no cambió: BRAKE_DECEL_MPS2
# sigue usando mu*g directamente (fijo, independiente de masa - la masa se
# cancela algebraicamente en este régimen, F_b=mu*m*g, a=F_b/m=mu*g), no el
# valor empírico en sí, que solo sirve para validar qué régimen aplica.
#
# Nota sobre el método de medición: la velocidad del joint de la rueda (y
# la /odom derivada cinemáticamente de ella) NO sirve para medir esto -
# ambas muestran una parada "instantánea" porque gz-sim-diff-drive-system
# puede llevar la rueda a la velocidad angular comandada casi de inmediato
# (inercia rotacional pequeña) incluso si el chasis sigue
# deslizando/avanzando por su propio momento. Solo la pose ground-truth del
# chasis reveló la desaceleración real del vehículo.
#
# T_r (buffer de tiempo de reacción) y v_h (velocidad de aproximación
# humana asumida) no dependen del baseline de Nav2/footprint, se conservan
# sin cambios: T_r=0.1s (latencia de sistema medida - detección lidar
# ~20Hz/50ms + un margen de cómputo, la misma disciplina que
# proximity_stop.py's propio timer de 20Hz), v_h=1.4 m/s (velocidad de
# marcha adulta promedio, mismo valor ya usado para el actor de
# ProximityTrial.world, consistencia interna deliberada).
#
# MIN_STANDOFF_M (d_min) = 0.3m, EXACTAMENTE el zone_radius_m actual de
# Proximity Stop - no es coincidencia, es la relación de "zonas anidadas"
# que docs/paper2_draft.tex sec 3 describe conceptualmente: Adaptive
# Separation reduce la velocidad continuamente hasta que la separación cae
# a exactamente el radio donde Proximity Stop ya garantiza velocidad cero,
# así que las dos zonas empalman sin salto ni hueco entre ellas.
#
# No hay un radio de "zona de interacción" fijo como constante aparte: el
# propio v_max(t) satura naturalmente en MAX_LINEAR_MPS una vez que S(t) es
# lo bastante grande, así que el límite efectivo de la zona es una cantidad
# derivada, no otra constante inventada. Con los valores de arriba
# (d_min=0.3, T_r=0.1, v_h=1.4), v_max llega a CERO (S_zero, el cruce
# inferior, no el superior) en S = d_min + T_r*v_h = 0.3 + 0.14 = 0.44m, y
# alcanza MAX_LINEAR_MPS (0.15 m/s) en S~=1.70m (calculado y registrado al
# arrancar, no fijado a mano). El pasillo más angosto medido del
# supermercado simulado (occupancy grid, ~1.0-1.1m de ancho, ver el propio
# encabezado de proximity_stop.py) da un rango lidar crudo de ~0.5m cuando
# el robot está centrado - 0.06m por encima de S_zero=0.44m, así que en
# teoría el robot no debería quedar nunca completamente detenido solo por
# estar centrado en el pasillo más angosto. Esto es un cálculo, no una
# medición: DEBE verificarse en vivo (Supermarket.world, ensayo
# pollo/pescado con este nodo activo) antes de confiar en que no
# reintroduce el mismo fallo que forzó a Proximity Stop a abandonar su
# radio original de 1.2m.
#
# adaptive_separation_v_max_massaware_mps: variante de comparación, NO
# usada para control, calculada con la hipótesis original (incorrecta,
# limitada por par) F_b = 2*tau_stall/r_wheel, a(t) = F_b/(m_p+m_L(t)),
# usando la lectura en vivo de weight_monitor.py. UNLOADED_MASS_KG,
# STALL_TORQUE_NM y WHEEL_RADIUS_M son propiedades físicas del URDF
# (no dependen del baseline de Nav2 corregido), llevadas del backup sin
# re-verificar - si el URDF del carrito cambia, revisar estas también.
# Existe específicamente para el ensayo que la Sección 5 del paper ya
# planea: comparar la precisión del modelo "mass aware" contra el fijo.

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

GRAVITY = 9.80665

# Medido de nuevo 2026-09-26 (ver encabezado): régimen limitado por
# tracción, independiente de la masa cargada, reconfirmado contra el
# baseline actual.
FRICTION_MU = 0.7
BRAKE_DECEL_MPS2_DEFAULT = FRICTION_MU * GRAVITY  # ~6.865 m/s^2

REACTION_TIME_S_DEFAULT = 0.1
HUMAN_APPROACH_MPS_DEFAULT = 1.4

# Found and fixed 2026-09-26, live in Supermarket.world, not assumed: the
# pollo/pescado regression with this node wired in FAILED (pescado leg,
# TaskResult.FAILED after 128s), stalled mid-aisle, not at a corner or
# turn (confirmed live by the user watching Gazebo directly - an earlier
# theory blaming a tight corner turn was wrong and is not repeated here).
# Logged data showed adaptive_separation_nearest_range_m sitting at
# 0.41-0.62m for the whole stall - i.e. the aisle itself is narrow enough
# that ordinary straight-line travel brings shelving within S_zero=0.44m
# on one side, not a one-off maneuver. The undiscriminating lidar zone
# check (same limitation proximity_stop.py already discloses) reads that
# shelving exactly like it would read a person.
#
# First fix attempt: clamp _v_max's floor to a small nonzero creep
# (0.02 m/s) instead of exactly 0.0, reasoning that Proximity Stop's own
# independent hard veto (twist_mux priority 220, unaffected by this file)
# already guarantees a true zero inside its own tighter 0.3m zone, so a
# nonzero floor here (only active in the warning-zone band between that
# boundary and S_zero) isn't a safety weakening. Re-tested against the
# same pescado leg: real improvement (stall duration 128s -> 82.7s, actual
# net displacement during the "stall," not a pure standstill) but still
# ultimately FAILED - the fix was directionally right but the chosen floor
# was still too conservative to work, not verified by assumption.
#
# Root cause of THAT: `nav2_params_sim.yaml`'s own `progress_checker`
# (`required_movement_radius: 0.5`, `movement_time_allowance: 10.0`)
# independently requires at least 0.5m of movement every 10s or Nav2
# itself declares "failed to make progress" - a hard floor on AVERAGE
# speed of 0.5/10 = 0.05 m/s that has nothing to do with this file, found
# by reading that config, not guessed. 0.02 m/s is below that floor by
# design (2.5x too slow), which is exactly why the first attempt still
# eventually failed even though it visibly helped. MIN_CREEP_MPS_DEFAULT
# raised to 0.07 m/s - a real margin (40%) above Nav2's own 0.05 m/s
# requirement, still a meaningful ~47% reduction from MAX_LINEAR_MPS
# (0.15), re-verified against the same pescado leg before trusting it
# (see paper2_draft.tex Section 6.2 for the confirmed result).
MIN_CREEP_MPS_DEFAULT = 0.07

# Re-derivado 2026-09-26 para igualar el zone_radius_m actual de
# Proximity Stop (0.3m, footprint-derivado) - ver encabezado.
MIN_STANDOFF_M_DEFAULT = 0.3
MAX_LINEAR_MPS_DEFAULT = 0.15  # = speed_limit.py's MAX_LINEAR_SPEED
# Re-derivado 2026-09-26 para igualar el self-detection floor actual de
# proximity_stop.py (0.2m, ver su propio encabezado) - antes en 0.3m.
MIN_VALID_RANGE_M_DEFAULT = 0.2

# Variante de comparación "mass aware" (no usada para control) - hipótesis
# limitada por par motor, DYNAMIXEL XM430-W210: 3.0 N*m stall por rueda,
# radio de rueda 0.033m (mismos valores que sara.urdf.xacro, no
# re-verificados esta sesión - ver encabezado).
STALL_TORQUE_NM = 3.0
WHEEL_RADIUS_M = 0.033
UNLOADED_MASS_KG = 3.217


class AdaptiveSeparation(Node):
    def __init__(self):
        super().__init__('adaptive_separation')

        self.declare_parameter('brake_decel_mps2', BRAKE_DECEL_MPS2_DEFAULT)
        self.declare_parameter('reaction_time_s', REACTION_TIME_S_DEFAULT)
        self.declare_parameter('human_approach_mps', HUMAN_APPROACH_MPS_DEFAULT)
        self.declare_parameter('min_standoff_m', MIN_STANDOFF_M_DEFAULT)
        self.declare_parameter('max_linear_mps', MAX_LINEAR_MPS_DEFAULT)
        self.declare_parameter('min_valid_range_m', MIN_VALID_RANGE_M_DEFAULT)
        self.declare_parameter('min_creep_mps', MIN_CREEP_MPS_DEFAULT)

        self.a = self.get_parameter('brake_decel_mps2').value
        self.t_r = self.get_parameter('reaction_time_s').value
        self.v_h = self.get_parameter('human_approach_mps').value
        self.d_min = self.get_parameter('min_standoff_m').value
        self.v_max_platform = self.get_parameter('max_linear_mps').value
        self.min_creep = self.get_parameter('min_creep_mps').value
        self.min_valid_range = self.get_parameter('min_valid_range_m').value

        crossover_s = self._solve_crossover_distance()
        s_zero = self.d_min + self.t_r * self.v_h
        self.get_logger().info(
            f'adaptive_separation started: a={self.a:.3f}m/s^2 (measured, '
            f'traction-limited), T_r={self.t_r}s, v_h={self.v_h}m/s, '
            f'd_min={self.d_min}m (= Proximity Stop zone radius), '
            f'v_max=min_creep({self.min_creep}m/s) below S={s_zero:.2f}m, '
            f'v_max saturates at {self.v_max_platform}m/s beyond S~={crossover_s:.2f}m')

        self.nearest_range = None
        self.cargo_kg = None

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.weight_sub = self.create_subscription(
            Float32, 'cart_cargo_weight_kg', self.weight_callback, 10)

        self.vmax_pub = self.create_publisher(Float32, 'adaptive_separation_v_max_mps', 10)
        self.vmax_massaware_pub = self.create_publisher(
            Float32, 'adaptive_separation_v_max_massaware_mps', 10)
        self.range_pub = self.create_publisher(Float32, 'adaptive_separation_nearest_range_m', 10)

        self.timer = self.create_timer(0.05, self.tick)  # 20Hz

    def _solve_crossover_distance(self):
        """S at which the fixed-deceleration v_max formula reaches
        v_max_platform - purely informative, not used in the control loop."""
        target = self.v_max_platform
        rhs = target + self.a * self.t_r + self.v_h
        return self.d_min + (rhs ** 2 - (self.a * self.t_r) ** 2 - self.v_h ** 2) / (2 * self.a)

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

        v_max_fixed = self._v_max(self.a, S)
        self.vmax_pub.publish(Float32(data=v_max_fixed))

        # Comparison-only variant, not read by speed_limit.py.
        m_total = UNLOADED_MASS_KG + (self.cargo_kg or 0.0)
        f_b = 2 * STALL_TORQUE_NM / WHEEL_RADIUS_M
        a_massaware = f_b / m_total
        v_max_massaware = self._v_max(a_massaware, S)
        self.vmax_massaware_pub.publish(Float32(data=v_max_massaware))


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveSeparation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
