#!/usr/bin/env python3
"""Log Proximity Stop's response to the cylinder actor's approach/retreat cycle.
Logs: detected range, active/veto state, zone-entry count, and the actual
mux output velocity (cmd_vel_in) - the last one only meaningful if a dummy
drive command is also running, to show a genuine command override, not just
internal state."""
import json
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float32, Int32
from geometry_msgs.msg import Twist

import sys
OUT_PATH = sys.argv[1] if len(sys.argv) > 1 else "/tmp/trial_results/proximity_response_log.jsonl"
DURATION_S = float(sys.argv[2]) if len(sys.argv) > 2 else 45.0


class Logger(Node):
    def __init__(self):
        super().__init__("proximity_response_logger")
        self.f = open(OUT_PATH, "w")
        self.range_m = -1.0
        self.active = False
        self.entry_count = 0
        self.cmd_x = 0.0
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Float32, "/proximity_nearest_range_m", self.cb_range, qos)
        self.create_subscription(Bool, "/proximity_stop_active", self.cb_active, qos)
        self.create_subscription(Int32, "/proximity_stop_zone_entry_count", self.cb_count, qos)
        self.create_subscription(Twist, "/cmd_vel_in", self.cb_cmd, qos)
        self.timer = self.create_timer(0.05, self.tick)  # 20Hz sample
        self.t0 = None

    def cb_range(self, msg):
        self.range_m = msg.data

    def cb_active(self, msg):
        self.active = msg.data

    def cb_count(self, msg):
        self.entry_count = msg.data

    def cb_cmd(self, msg):
        self.cmd_x = msg.linear.x

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        rec = {
            "t": now - self.t0,
            "range_m": self.range_m,
            "active": self.active,
            "entry_count": self.entry_count,
            "cmd_vel_x": self.cmd_x,
        }
        self.f.write(json.dumps(rec) + "\n")

    def close(self):
        self.f.close()


def main():
    rclpy.init()
    node = Logger()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    time.sleep(DURATION_S)
    node.close()
    executor.shutdown()
    rclpy.shutdown()
    print("[log] done")


if __name__ == "__main__":
    main()
