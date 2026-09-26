#!/usr/bin/env python3
# Autor: David Capacho Parra
# Descripción: Proximity Stop (paper 2, docs/paper2_draft.md sec 3.1) -
# implementación de "Safety-rated Monitored Stop" para SARA:
#
#   "While a human is present in the defined, task-relevant collaboration
#    zone, commanded velocity must be zero. Once the zone clears, motion may
#    resume only after a hysteresis dwell time, to prevent chatter at the
#    zone boundary."
#
# Detección por lidar (/scan), no por pose ground-truth: cualquier mecanismo
# que forme parte del control tiene que ser algo que el robot real pudiera
# ejecutar de verdad (mismo lidar que ya usa naive_obstacle_avoidance.py, en
# sim y en hardware real, según el propio README del repo), no un atajo que
# solo funciona porque el simulador regala la posición exacta de la
# "persona". Una versión anterior de este nodo sí leía la odometría
# ground-truth del actor humano directamente - descartada por esto mismo,
# no por un problema técnico: no sería aplicable al robot real.
#
# Ground truth (la odometría del <actor> en ProximityTrial.world) se sigue
# publicando y bridgeando, pero solo para instrumentación de ensayos
# (registrar la distancia real y así calcular métricas de la Sección 5 -
# tiempo de respuesta, distancia de parada - comparándolas contra lo que el
# lidar detectó), nunca como entrada de control. Esto cambia la
# "ground-truth simplification" que docs/paper2_draft.md sec 5/7 describía
# como alcance del método: ya no aplica a Proximity Stop (si se termina
# aplicando a Adaptive Separation es una decisión aparte, pendiente). A
# cambio, esta versión no distingue una persona de cualquier otro objeto
# dentro de la zona - el lidar no clasifica - una limitación real que hay
# que declarar explícitamente, pero es la misma limitación que ya tiene
# ISO 3691-4 (detección-y-parada genérica, no reconocimiento de persona), y
# el mismo nivel de percepción que ya usa el resto de la capa de seguridad
# de SARA (naive_obstacle_avoidance.py tampoco distingue). No es peor que
# lo que ya existe, y sí es honesto sobre lo que realmente hace.
#
# ZONE_RADIUS_M: dos valores intermedios probados y descartados esta
# sesión antes de este, por razones que vale la pena dejar registradas:
#   1) 1.2m original: límite exterior del "personal space" de Hall (Hall,
#      E.T., "The Hidden Dimension", 1966) - una zona de COMODIDAD
#      social, no una distancia de frenado derivada de la física real de
#      la plataforma. Encontrado en vivo (paper2_draft.tex sec 6/7): con
#      1.2m, CUALQUIER objeto estático a esa distancia o menos - una
#      estantería, no solo una persona - detiene el robot
#      permanentemente; en un pasillo real de supermercado (1.0-1.1m de
#      ancho, medido directamente del occupancy grid) esto vuelve la
#      navegación autónoma inutilizable.
#   2) 0.5m vía ISO 13855 (S=K*T+C con K=1.6m/s, T=0.1s, C=0.3m = 0.46m,
#      redondeado): un error distinto, encontrado después - zone_radius_m
#      se mide desde el LIDAR, no desde el borde físico del robot, y el
#      carrito (base_platform, ver abajo) se extiende hasta 0.22m del
#      lidar en su esquina más lejana. Ese 0.46m de "margen real" nunca
#      fue realmente 0.46m de espacio libre frente al carrito - una
#      fracción ya la ocupaba el propio robot.
#
# Corrección final: Proximity Stop y Adaptive Separation NO deberían
# cargar la misma responsabilidad. Por diseño (ISO/TS 15066, sec 2 del
# paper), Proximity Stop es el equivalente a "Safety-rated Monitored
# Stop" - un backstop simple, binario, de corto alcance, deliberadamente
# NO escalado por velocidad. Esa escala por velocidad/distancia ya existe
# en el framework: es exactamente lo que hace adaptive_separation.py's
# v_max(t) (Ecuación 2 del paper). Cargarle a Proximity Stop también el
# margen de frenado tipo ISO 13855 duplicaba el trabajo de Adaptive
# Separation con un mecanismo binario mucho menos flexible.
#
# ZONE_RADIUS_M ahora es solo "huella física del robot + margen de
# colisión inminente", no una distancia de frenado:
#   footprint = 0.2195m - la esquina más lejana de base_platform
#     (sara.urdf.xacro: caja de colisión 0.315x0.2655m, centrada en offset
#     local (-0.0173, 0.0) respecto al lidar, que comparte el mismo x,y
#     que base_platform en la cadena de montaje) respecto al propio lidar
#     - calculado geométricamente, no estimado.
#   margin = 0.08m de margen de "algo está a punto de tocar el carrito"
#   ZONE_RADIUS_M = 0.2195 + 0.08 ≈ 0.3m
# El margen de frenado dependiente de velocidad/distancia (antes mal
# ubicado aquí) se movió a adaptive_separation.py's min_standoff_m, con
# la misma corrección de huella aplicada allá.
#
# MIN_VALID_RANGE_M: el lidar simulado (sara.gazebo.xacro, mismo modelo que
# el LDS-01 real del TurtleBot3 Waffle: rango 0.12-3.5m, ruido gaussiano
# stddev=0.01m) tiene un límite de hardware en 0.12m, pero el filtro real no
# es por ese límite - es por auto-detección: el lidar ve la propia
# estructura del carrito. El piso de auto-detección se centralizó a nivel
# de sensor (sara.gazebo.xacro <min>, re-medido 2026-09-25 en TestEmpty.world:
# máximo real 0.172m, piso puesto en 0.2m), así que este filtro de
# aplicación ya es defensa en profundidad, no la restricción activa -
# tiene que quedar estrictamente por debajo de ZONE_RADIUS_M o la ventana
# válida (min, radius] queda vacía (ver la re-derivación más abajo, el
# mismo bug que ya se dio una vez con 0.3/0.3).
#
# HYSTERESIS_DWELL_S: tiempo de espera tras salir de la zona antes de
# reanudar el movimiento, para evitar "chatter" (parada/arranque repetido)
# si el humano se queda justo en el borde de la zona - exactamente el caso
# que el propio §3.1 nombra y que el "boundary-chatter check" de la Sección
# 5 está pensado para medir.
#
# Los tres umbrales de arriba son parámetros ROS (no solo constantes de
# módulo), con estos mismos valores como default: para que un futuro batch
# trial runner (paper_outline.md Phase 5) pueda variarlos por ensayo sin
# tocar el código fuente, igual que cart_speed_scale/etc en weight_monitor.py
# se calibran con datos, no adivinando.
#
# RE-DERIVACIÓN 2026-09-25, tras restaurar este nodo sobre el baseline de
# Nav2 reconstruido (4 fixes, commit 9bede409): al restaurar, el self-
# detection floor del sensor (sara.gazebo.xacro <min>) estaba en 0.4m, un
# valor "generoso" decidido durante el reset sin conocer el footprint real
# de este archivo - con eso, ZONE_RADIUS_M=0.3m era una ventana vacía
# (`min_valid_range_m < r <= zone_radius_m` nunca se cumplía) y Proximity
# Stop no disparaba nunca, silenciosamente. Subir ZONE_RADIUS_M para
# compensar (probado hasta 0.5m) resultó ser la corrección equivocada:
# perseguía el piso del sensor en vez de corregirlo, y un radio mayor al
# footprint real volvió a enganchar objetos estáticos cercanos (una pared
# junto al spawn, estanterías de pasillo) - el mismo problema que ya había
# descartado el radio de 1.2m originalmente. El piso del sensor se
# re-midió empíricamente (TestEmpty.world, <min> temporalmente en 0.12m
# para ver la auto-detección cruda): máximo real 0.172m, no los 0.186m
# documentados antes. Piso reajustado a 0.2m (margen real ~0.03m, no
# duplicado) y ZONE_RADIUS_M devuelto a su valor original, fundamentado en
# el footprint (0.3m) - MIN_VALID_RANGE_M bajado a 0.2m en consecuencia
# (debe quedar estrictamente por debajo de ZONE_RADIUS_M, ver la constante
# de módulo). Verificado de nuevo contra navegación Nav2 normal (sin
# actor) sin ningún disparo falso y contra el propio self-detection
# re-medido, antes de confiar en estos valores.
#
# Tópicos de diagnóstico publicados, pensados para la Sección 5 / Fase 6 de
# paper_outline.md (tiempo de respuesta, distancia de parada, tasa de
# éxito/fallo, chatter-event count) sin tener que reconstruirlos a mano de
# los logs después:
#   - proximity_nearest_range_m: la señal de control real (lidar). -1.0
#     cuando no hay nada en la zona (evita confundir "vacío" con "muy
#     lejos" en un Float32 sin exponer un NaN al graficar).
#   - proximity_stop_active: bool de estado (con hysteresis ya aplicada).
#   - proximity_stop_zone_entry_count: contador monótono, incrementa en
#     cada transición False->True - da el chatter-event count directamente,
#     sin tener que detectar flancos en proximity_stop_active a mano.
#   - human_ground_truth_distance_m: NO es señal de control (ver más
#     arriba) - solo para comparar contra proximity_nearest_range_m y medir
#     qué tan bien el lidar detectó al actor realmente.

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32

ZONE_RADIUS_M_DEFAULT = 0.3
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
