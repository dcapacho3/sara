import json, sys, threading, time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32

OUT_PATH = sys.argv[1] if len(sys.argv) > 1 else '/tmp/adaptive_sep_log.jsonl'
DURATION_S = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0

class Logger(Node):
    def __init__(self):
        super().__init__('adaptive_sep_logger')
        self.f = open(OUT_PATH, 'w')
        self.range_m = -1.0
        self.v_max = -1.0
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Float32, '/adaptive_separation_nearest_range_m', self.cb_range, qos)
        self.create_subscription(Float32, '/adaptive_separation_v_max_mps', self.cb_vmax, qos)
        self.timer = self.create_timer(0.1, self.tick)
        self.t0 = None
    def cb_range(self, msg): self.range_m = msg.data
    def cb_vmax(self, msg): self.v_max = msg.data
    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None: self.t0 = now
        self.f.write(json.dumps({'t': now - self.t0, 'range_m': self.range_m, 'v_max': self.v_max}) + '\n')
    def close(self): self.f.close()

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
    print('[log] done')

if __name__ == '__main__':
    main()
