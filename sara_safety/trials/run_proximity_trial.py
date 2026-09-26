#!/usr/bin/env python3
# Autor: David Capacho Parra
# Descripción: Ejecuta un ensayo de Proximity Stop (paper 2, docs/paper2_draft.tex
# sec 5, Table 2) de principio a fin: genera el mundo, lanza la pila
# completa vía sara_safety/launch/proximity_trial.launch.py (mismo launch
# ya probado manualmente esta sesión, no reinventa el spawn/bridge/mux),
# conduce el robot, mide, y cierra todo limpio. Corre DENTRO del
# contenedor (necesita rclpy).
#
# Métricas usadas, y por qué cada una viene de una señal ya confirmada
# robusta esta sesión, no de la odometría ground-truth del actor (que se
# sabe intermitente bajo la pila completa, ver sara_paper2_framework):
#   - response_time_s: desde que /proximity_stop_active pasa a True hasta
#     que la velocidad real de la rueda (joint_states) llega a ~0 - mide
#     la latencia real de actuación, no solo el tick del nodo. SOLO válido
#     así quando hay una transición real dentro de la ventana de
#     observación del TrialNode (condición "moving": el actor empieza
#     fuera de la zona - 0.5m, re-derivada esta sesión, era 1.2m). En
#     "static_*" y "boundary_chatter" el
#     actor YA está dentro de la zona desde antes de que el robot spawnee
#     (spawn_wait_s=9.0s antes de que este nodo se suscriba), así que
#     active_since captura el primer mensaje ya-True que llega, no una
#     transición - el número resultante mide latencia de arranque/
#     descubrimiento de ROS (DDS), no la latencia de seguridad real.
#     Confirmado: los outliers de response_time_s en esas condiciones
#     coinciden con overshoot_m vacío (robot_xy también llegó tarde) -
#     mismo root cause. Ver paper2_draft.tex Sección 6 para cómo se
#     reporta esto honestamente (solo "moving" se usa como medida de
#     latencia real).
#   - stop_distance_m: la lectura de /proximity_nearest_range_m (lidar, la
#     misma señal de control) en el instante del stop - es la distancia que
#     el robot REALMENTE detectó al parar, con la limitación honesta de que
#     el lidar solo ve las piernas del actor (ver ProximityTrial.world).
#   - success: activó Y llegó a v~0 - el requisito propio del modo (sec
#     3.1), no si el actor con su propia trayectoria guionada se acercó o
#     no, que pasa igual sin importar si el robot paró bien o mal (el actor
#     no reacciona al robot).
#   - overshoot_m: cuánto siguió avanzando el robot (odometría propia, no
#     la del actor) después de que /proximity_stop_active se activara -
#     mide directamente si obedeció el veto, sin contaminar con el
#     movimiento independiente del actor.
#   - chatter_events: incremento de /proximity_stop_zone_entry_count durante
#     la ventana del ensayo. El actor camina en loop (ver run_one_trial:
#     <loop>false</loop> hace que gz-sim8 descarte el actor del mundo por
#     completo, un bug real de esa versión, no algo a pelear), así que en
#     los ensayos "moving" un valor >1 puede ser simplemente más de un
#     acercamiento real dentro de la ventana, no chatter en el sentido de
#     la Sección 5 (oscilación en el borde de la zona). La prueba de
#     chatter real usa un ensayo dedicado con el actor estático
#     (start==end) exactamente en el borde de la zona, donde cualquier
#     entrada adicional sí es chatter genuino - con la salvedad honesta de
#     que entry_count_start se toma del primer mensaje que llega tras la
#     suscripción (~9s después del spawn), así que cualquier chatter
#     ANTES de ese punto es invisible: "chatter_events=0" en boundary_chatter
#     respalda "no se observó chatter una vez alcanzado el estado estable,
#     en una ventana de 12s" - no "no hubo chatter en absoluto".
#
# El ground-truth del actor humano NO está disponible: se intentaron dos
# vías (plugin gz-sim-odometry-publisher-system en el <actor>, y bridgear
# pose/info o dynamic_pose/info) y ambas confirmaron, en vivo, que gz-sim8
# simplemente no expone entidades <actor> en ningún topic de pose estándar
# del scene broadcaster (a diferencia de <model>, que sí). human_gt_samples
# y final_gt_distance_m quedan en el CSV por continuidad de esquema pero
# siempre son 0/None - ver paper2_draft.tex Sección 6 para cómo se reporta
# esta limitación honestamente en el paper.

import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Int32

sys.path.insert(0, os.path.dirname(__file__))
from generate_world import generate_world  # noqa: E402

DRIVE_SPEED_MPS = 0.15


class TrialNode(Node):
    def __init__(self, duration_s, drive=True):
        super().__init__('trial_runner')
        self.duration_s = duration_s
        self.drive = drive
        self.t0 = time.time()

        self.wheel_v = None
        self.active = False
        self.nearest_range = None
        self.entry_count = None
        self.entry_count_start = None
        self.robot_xy = None

        self.active_since = None      # sim wall-clock time active first became True
        self.stopped_since = None     # wall-clock time wheel first reached ~0 after active
        self.response_time_s = None
        self.stop_distance_m = None
        self.robot_x_at_active = None
        self.max_robot_x_after_active = None

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_key', 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Bool, 'proximity_stop_active', self.active_cb, 10)
        self.create_subscription(Float32, 'proximity_nearest_range_m', self.range_cb, 10)
        self.create_subscription(Int32, 'proximity_stop_zone_entry_count', self.entry_cb, 10)
        self.create_subscription(Odometry, '/odom', self.robot_cb, 10)

        self.drive_timer = self.create_timer(0.05, self.drive_tick)
        self.check_timer = self.create_timer(0.05, self.check_tick)

    def joint_cb(self, msg: JointState):
        try:
            i = msg.name.index('wheel_left_joint')
            self.wheel_v = msg.velocity[i]
        except (ValueError, IndexError):
            pass

    def active_cb(self, msg: Bool):
        now = time.time()
        if msg.data and not self.active:
            self.active_since = now
            if self.robot_xy is not None:
                self.robot_x_at_active = self.robot_xy[0]
                self.max_robot_x_after_active = self.robot_xy[0]
        self.active = msg.data

    def range_cb(self, msg: Float32):
        self.nearest_range = msg.data if msg.data >= 0 else None

    def entry_cb(self, msg: Int32):
        if self.entry_count_start is None:
            self.entry_count_start = msg.data
        self.entry_count = msg.data

    def robot_cb(self, msg: Odometry):
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.robot_x_at_active is not None:
            self.max_robot_x_after_active = max(self.max_robot_x_after_active, self.robot_xy[0])

    def drive_tick(self):
        msg = Twist()
        msg.linear.x = DRIVE_SPEED_MPS if self.drive else 0.0
        msg.angular.y = 0.005  # keeps the mux topic alive either way
        self.cmd_pub.publish(msg)

    def check_tick(self):
        now = time.time()
        if (self.active_since is not None and self.stopped_since is None
                and self.wheel_v is not None and abs(self.wheel_v) < 0.05):
            self.stopped_since = now
            self.response_time_s = now - self.active_since
            self.stop_distance_m = self.nearest_range

        if now - self.t0 > self.duration_s:
            rclpy.shutdown()

    def result(self):
        # success = the mode's own core requirement: detection happened and
        # the robot actually came to rest (not "the actor's independent
        # scripted trajectory never got close" - the actor doesn't react to
        # the robot, so it reaches its scripted waypoint regardless of
        # whether Proximity Stop worked, that's not a meaningful check).
        triggered = self.active_since is not None
        stopped = self.stopped_since is not None
        success = triggered and stopped

        overshoot_m = None
        if self.robot_x_at_active is not None and self.max_robot_x_after_active is not None:
            overshoot_m = self.max_robot_x_after_active - self.robot_x_at_active

        chatter = None
        if self.entry_count is not None and self.entry_count_start is not None:
            chatter = self.entry_count - self.entry_count_start

        return {
            'response_time_s': self.response_time_s,
            'stop_distance_m': self.stop_distance_m,
            'overshoot_m': overshoot_m,
            'success': success,
            'chatter_events': chatter,
            # Ground-truth actor-position cross-validation was attempted
            # (gz-sim-odometry-publisher-system on the actor, then bridging
            # dynamic_pose/info and pose/info) and abandoned: confirmed
            # empirically, live, that gz-sim8's scene broadcaster does not
            # expose <actor> entities on ANY of its standard per-entity pose
            # topics (unlike <model> entities - the robot's own /odom ground
            # truth is unaffected). These two fields are kept in the schema
            # for continuity but are always None/0 - not a bug, a documented
            # limitation (see paper2_draft.tex Section 6 discussion).
            'human_gt_samples': 0,
            'final_gt_distance_m': None,
            'triggered': triggered,
            'stopped': stopped,
        }


def launch_stack(world_path, launch_log_path, payload_mass_kg=None, zone_radius_m=None,
                  max_obstacle_distance=None, min_obstacle_distance=None, min_standoff_m=None,
                  reaction_time_s=None, human_approach_mps=None,
                  spawn_x=None, spawn_z=None, spawn_pitch=None):
    """Starts the proven proximity_trial.launch.py with a world override, in
    its own process group so it can be torn down cleanly as a unit.
    payload_mass_kg (optional): extra mass added to base_weight at spawn
    time via sara.urdf.xacro's payload_mass_kg arg - see that file and
    run_braking_trial.py for why this exists (real mass for a momentum/
    braking test, not a wrench).
    zone_radius_m (optional): override for Proximity Stop's zone radius -
    see mux.launch.py and run_slope_trial.py for why this exists (a large
    static object deliberately close to the robot, e.g. a ramp, would
    otherwise be correctly detected and vetoed).
    max_obstacle_distance/min_obstacle_distance (optional): same reasoning,
    for the SEPARATE lidar-based naive_obstacle_avoidance.py node (twist_mux
    priority 200, above keyboard teleop's 100) - discovered the hard way
    that overriding zone_radius_m alone was not enough for slope/tip-over
    trials: this second node also sees the ramp as a frontal obstacle and
    takes evasive control, silently blocking all forward progress with no
    error logged anywhere - see run_slope_trial.py.
    min_standoff_m (optional): same reasoning again, for Adaptive
    Separation - a THIRD independent lidar-based layer that composes its
    own v_max cap into speed_limit.py's output downstream of twist_mux
    entirely, so even overriding the two topics above was not enough;
    speed_limit.py silently capped cmd_vel_out to 0 with no log line at
    all - see run_slope_trial.py."""
    env = os.environ.copy()
    log = open(launch_log_path, 'w')
    extra_args = ''
    if payload_mass_kg is not None:
        extra_args += f' payload_mass_kg:={payload_mass_kg}'
    if zone_radius_m is not None:
        extra_args += f' zone_radius_m:={zone_radius_m}'
    if max_obstacle_distance is not None:
        extra_args += f' max_obstacle_distance:={max_obstacle_distance}'
    if min_obstacle_distance is not None:
        extra_args += f' min_obstacle_distance:={min_obstacle_distance}'
    if min_standoff_m is not None:
        extra_args += f' min_standoff_m:={min_standoff_m}'
    if reaction_time_s is not None:
        extra_args += f' reaction_time_s:={reaction_time_s}'
    if human_approach_mps is not None:
        extra_args += f' human_approach_mps:={human_approach_mps}'
    if spawn_x is not None:
        extra_args += f' spawn_x:={spawn_x}'
    if spawn_z is not None:
        extra_args += f' spawn_z:={spawn_z}'
    if spawn_pitch is not None:
        extra_args += f' spawn_pitch:={spawn_pitch}'
    proc = subprocess.Popen(
        ['bash', '-c',
         f'source /opt/ros/jazzy/setup.bash && source ~/ros_ws/install/setup.bash && '
         f'exec ros2 launch sara_safety proximity_trial.launch.py world:={world_path}{extra_args}'],
        stdout=log, stderr=subprocess.STDOUT, env=env,
        preexec_fn=os.setsid)
    return proc


def teardown(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass
    proc.wait(timeout=5)
    # proximity_trial.launch.py spawns gz/robot_state_publisher/bridge/mux
    # stack as its own children outside this process group in some cases
    # (ros2 launch's own process management) - belt-and-suspenders cleanup.
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'ros_gz_bridge'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'ros_gz_sim'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'ros2 launch sara_safety'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'sara_safety/proximity_stop.py'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'sara_safety/adaptive_separation.py'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'sara_safety/speed_limit.py'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'sara_safety/weight_monitor.py'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'sara_safety/naive_obstacle_avoidance.py'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'sara_safety/joy_teleop.py'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'robot_state_publisher'], check=False)
    time.sleep(1.0)


def run_one_trial(trial_id, start_xy, end_xy, speed_mps, spawn_wait_s,
                   drive_duration_s, results_dir, loop=True, drive=True):
    world_path = f'/tmp/trial_{trial_id}.world'
    # loop=True always, deliberately: confirmed empirically that
    # <loop>false</loop> makes gz-sim8 drop the actor from the world
    # entirely (it stops appearing in /world/default/pose/info at all,
    # silently, no error) - a real gz-sim8 quirk with non-looping actors,
    # not worth fighting. A one-shot approach was the original plan (to
    # avoid multi-cycle ambiguity in chatter_events) but isn't achievable
    # this way; see result()'s docstring for how chatter_events is
    # interpreted honestly given the actor keeps cycling regardless of
    # this trial's own observation window.
    world = generate_world(start_xy, end_xy, speed_mps=speed_mps, pause_s=1.0,
                            turn_s=0.3, loop=loop)
    with open(world_path, 'w') as f:
        f.write(world)

    log_path = os.path.join(results_dir, f'{trial_id}_launch.log')
    proc = launch_stack(world_path, log_path)
    try:
        time.sleep(spawn_wait_s)
        rclpy.init()
        node = TrialNode(drive_duration_s, drive=drive)
        try:
            rclpy.spin(node)
        except Exception:
            pass
        result = node.result()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    finally:
        teardown(proc)

    result['trial_id'] = trial_id
    result['start_xy'] = start_xy
    result['end_xy'] = end_xy
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--trial-id', required=True)
    p.add_argument('--start', type=float, nargs=2, required=True)
    p.add_argument('--end', type=float, nargs=2, required=True)
    p.add_argument('--speed', type=float, default=1.4)
    p.add_argument('--spawn-wait', type=float, default=9.0)
    p.add_argument('--duration', type=float, default=14.0)
    p.add_argument('--results-dir', default='/tmp/trial_results')
    p.add_argument('--no-drive', action='store_true')
    args = p.parse_args()

    os.makedirs(args.results_dir, exist_ok=True)
    result = run_one_trial(
        args.trial_id, tuple(args.start), tuple(args.end), args.speed,
        args.spawn_wait, args.duration, args.results_dir, drive=not args.no_drive)
    print(json.dumps(result))


if __name__ == '__main__':
    main()
