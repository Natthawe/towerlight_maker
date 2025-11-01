#!/usr/bin/env python3
import json, socket
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Int32, Bool, Empty

class TowerUdpClient(Node):
    """
    ส่ง UDP JSON ไปยัง ESP32 ตามสคีม:
      {"ver":1,"light":{"color":"red","mode":"blink","period_ms":300},"audio":{"action":"play","track":6}}
    พารามิเตอร์:
      target_ip (default: 10.1.100.200)
      target_port (default: 9000)
      qos_reliable (bool)  : ถ้าจริงจะใช้ Reliable สำหรับซับสคริปชัน
    ท็อปปิคอินพุต (subscribe):
      - /tower/udp_json            (std_msgs/String)  # ผ่าน JSON ตรงๆ
      - /tower/audio/play          (std_msgs/Int32)   # absolute track (>0)
      - /tower/audio/stop          (std_msgs/Empty)   # stop
      - /tower/audio/volume        (std_msgs/Int32)   # 0..30
      - /tower/audio/volume_delta  (std_msgs/Int32)   # +/-int
      - /tower/audio/repeat        (std_msgs/Bool)    # true/false
      - /tower/audio/lof_once      (std_msgs/Bool)    # light_off_on_finish_once
      - /tower/light/json          (std_msgs/String)  # {"color":"red","mode":"blink","period_ms":300}
      - /tower/heartbeat           (std_msgs/Bool)    # true/false -> {"ver":1,"heartbeat":true}
    """

    def __init__(self):
        super().__init__('tower_udp_client')

        self.declare_parameter('target_ip', '10.1.100.200')
        self.declare_parameter('target_port', 9000)
        self.declare_parameter('qos_reliable', False)

        self.ip = self.get_parameter('target_ip').get_parameter_value().string_value
        self.port = int(self.get_parameter('target_port').get_parameter_value().integer_value)
        qos_rel = self.get_parameter('qos_reliable').get_parameter_value().bool_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if qos_rel else ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (self.ip, self.port)
        self.get_logger().info(f'UDP → {self.addr}')

        # Subscriptions
        self.sub_json = self.create_subscription(String, '/tower/udp_json', self.cb_json, qos)
        self.sub_play = self.create_subscription(Int32,  '/tower/audio/play', self.cb_play, qos)
        self.sub_stop = self.create_subscription(Empty,  '/tower/audio/stop', self.cb_stop, qos)
        self.sub_vol  = self.create_subscription(Int32,  '/tower/audio/volume', self.cb_volume, qos)
        self.sub_vd   = self.create_subscription(Int32,  '/tower/audio/volume_delta', self.cb_volume_delta, qos)
        self.sub_rep  = self.create_subscription(Bool,   '/tower/audio/repeat', self.cb_repeat, qos)
        self.sub_lof  = self.create_subscription(Bool,   '/tower/audio/lof_once', self.cb_lof, qos)
        self.sub_ljson= self.create_subscription(String, '/tower/light/json', self.cb_light_json, qos)
        self.sub_hb   = self.create_subscription(Bool,   '/tower/heartbeat', self.cb_heartbeat, qos)

    # ---- helpers ----
    def send_json(self, d: dict):
        try:
            d.setdefault('ver', 1)
            payload = json.dumps(d, separators=(',',':')).encode('utf-8')
            self.sock.sendto(payload, self.addr)
        except Exception as e:
            self.get_logger().error(f'UDP send error: {e}')

    # ---- callbacks ----
    def cb_json(self, msg: String):
        # ส่งตรง (ถ้าผู้ใช้ประกอบ JSON เอง)
        try:
            d = json.loads(msg.data)
            if 'ver' not in d:
                d['ver'] = 1
            self.send_json(d)
        except Exception as e:
            self.get_logger().warn(f'Invalid JSON on /tower/udp_json: {e}')

    def cb_play(self, msg: Int32):
        tr = int(msg.data)
        if tr <= 0:
            self.get_logger().warn('play: track must be > 0')
            return
        self.send_json({"ver":1, "audio":{"action":"play","track":tr}})

    def cb_stop(self, _msg: Empty):
        self.send_json({"ver":1, "audio":{"action":"stop"}})

    def cb_volume(self, msg: Int32):
        v = int(msg.data)
        if not (0 <= v <= 30):
            self.get_logger().warn('volume must be 0..30')
            return
        self.send_json({"ver":1, "audio":{"volume":v}})

    def cb_volume_delta(self, msg: Int32):
        self.send_json({"ver":1, "audio":{"volume_delta":int(msg.data)}})

    def cb_repeat(self, msg: Bool):
        self.send_json({"ver":1, "audio":{"repeat":bool(msg.data)}})

    def cb_lof(self, msg: Bool):
        self.send_json({"ver":1, "audio":{"light_off_on_finish":bool(msg.data)}})

    def cb_light_json(self, msg: String):
        # คาดหวัง {"color":"red|yellow|green|off","mode":"off|solid|blink","period_ms":300}
        try:
            lj = json.loads(msg.data)
            # validate เบื้องต้น
            if 'mode' not in lj or not isinstance(lj['mode'], str):
                raise ValueError('missing/invalid mode')
            # period_ms optional
            d = {"ver":1, "light": lj}
            self.send_json(d)
        except Exception as e:
            self.get_logger().warn(f'Invalid light JSON: {e}')

    def cb_heartbeat(self, msg: Bool):
        self.send_json({"ver":1, "heartbeat":bool(msg.data)})

def main():
    rclpy.init()
    node = TowerUdpClient()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
