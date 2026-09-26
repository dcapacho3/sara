#!/usr/bin/env python3
"""Fresh braking-deceleration measurement against the CURRENT (corrected)
baseline - reproduces the same ground-truth-chassis-pose methodology
adaptive_separation.py's own header documents (wheel odom is blind to
slip and shows an unphysical instant stop; only /world/default/pose/info
reveals the real deceleration), but self-contained, no dependency on the
pre-reset trials/generate_world.py machinery.

Used to produce the 2026-09-26 re-measurement cited in
adaptive_separation.py's own header and paper2_draft.tex Section 6.2 (raw
pose logs: docs/data/adaptive_sep_braking_{1,2,3}_2026-09-26.jsonl,
analysis: the windowed max-consecutive-slope method below, same as the
pre-reset run_braking_trial.py this replaces).

How it was run: launch `ros2 launch sara_safety proximity_trial.launch.py
world:=<path to sara_gazebo/worlds/TestEmpty.world>` (a plain robot-only
world, no actor needed), wait for spawn/settle (~10s), then run this
script inside the container: `python3 braking_trial_2026-09-26.py
<out_path.jsonl>`. Drives forward at DRIVE_SPEED_MPS for DRIVE_DURATION_S,
commands zero, logs ground-truth chassis pose via a background `gz topic
-e -t /world/default/pose/info --json-output` process for POST_BRAKE_S
after. Repeat 3x with a full `docker restart` between each (container
hygiene - see sara_docker_workflow memory) for independent trials, then
feed the resulting pose logs through analyze_braking_2026-09-26.py
(steepest slope between consecutive chassis x-position samples, windowed
to [brake_sim_time-0.3s, brake_sim_time+2.0s])."""
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

DRIVE_SPEED_MPS = 0.15
DRIVE_DURATION_S = 3.0
POST_BRAKE_S = 2.0
PRE_BRAKE_BUFFER_S = 0.3

OUT_PATH = sys.argv[1] if len(sys.argv) > 1 else '/tmp/trial_results/braking_pose.jsonl'


class BrakingNode(Node):
    def __init__(self):
        super().__init__('braking_trial_runner')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Clock, '/clock', self.clock_cb, 10)
        self.t0 = time.time()
        self.braked = False
        self.sim_time_now = None
        self.brake_sim_time = None
        self.timer = self.create_timer(0.05, self.tick)

    def clock_cb(self, msg):
        self.sim_time_now = msg.clock.sec + msg.clock.nanosec / 1e9

    def tick(self):
        elapsed = time.time() - self.t0
        msg = Twist()
        if elapsed < DRIVE_DURATION_S:
            msg.linear.x = DRIVE_SPEED_MPS
        else:
            msg.linear.x = 0.0
            if not self.braked:
                self.braked = True
                self.brake_sim_time = self.sim_time_now
                self.get_logger().info(f'braking commanded, sim_t={self.brake_sim_time}')
        msg.angular.y = 0.005
        self.cmd_pub.publish(msg)
        if elapsed > DRIVE_DURATION_S + POST_BRAKE_S:
            rclpy.shutdown()


def main():
    proc = subprocess.Popen(
        ['bash', '-c', 'source /opt/ros/jazzy/setup.bash && exec gz topic -e '
                        '-t /world/default/pose/info --json-output'],
        stdout=open(OUT_PATH, 'w'), stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(0.5)

    rclpy.init()
    node = BrakingNode()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    brake_sim_time = node.brake_sim_time
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass
    proc.wait(timeout=5)

    print(json.dumps({'brake_sim_time': brake_sim_time}))


if __name__ == '__main__':
    main()
