#!/usr/bin/env python3
import json
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, Float32, Int32, String

from gmr_msgs.msg import ControllerStatus


class S(Enum):
    SAFE = 0
    WARN = 1
    EMER = 2


class TowerRulesBridge(Node):

    def __init__(self):
        super().__init__('tower_rules_bridge')

        # --- Declare params ---
        self.declare_parameter('bumper_topic', '/bumper_active')

        # แกน emergency: เลือกแหล่ง
        self.declare_parameter('use_controller_status', True)                           # True = ใช้ gmr_controllers/status, False = ใช้ emergency_topic
        self.declare_parameter('controller_status_topic', '/gmr_controllers/status')    # แหล่ง ControllerStatus
        self.declare_parameter('emergency_topic', '/emergency')                         # fallback: std_msgs/Bool

        self.declare_parameter('distance_topic', '/closest_object_distance')            # topic ระยะทาง
        self.declare_parameter('distance_msg_type', 'float32')                          # 'float32' | 'int32'

        self.declare_parameter('distance_thresh', 0.5)                                  # ระยะทางเกณฑ์ (เมตร)

        self.declare_parameter('warn_track', 2)                                         # track เสียงเตือน
        self.declare_parameter('emergency_track', 1)                                    # track เสียงฉุกเฉิน
        self.declare_parameter('warn_blink_ms', 460)                                    # ช่วง warn blink ms
        self.declare_parameter('emer_blink_ms', 150)                                    # ช่วง emer blink ms

        self.declare_parameter('retrigger_cooldown_sec', 3.0)                           # หน่วงเวลา retrigger เสียง

        # refresh รายสถานะ (0.0 = ปิด เพื่อไม่ให้เฟส blink รีเซ็ต)
        self.declare_parameter('safe_light_refresh_sec', 5.0)                           # SAFE
        self.declare_parameter('warn_light_refresh_sec', 0.0)                           # WARN
        self.declare_parameter('emer_light_refresh_sec', 0.0)                           # EMER

        # ทิศทางค่า emergency (ทั่วไป True=EMER)
        self.declare_parameter('emergency_true_is_emergency', True)                     # True=EMER, False=OK
        self.declare_parameter('bumper_true_is_ok', True)                               # True=OK, False=EMER

        # เก็บ period ล่าสุดที่ส่ง (เพื่อยิง light-only เมื่อค่าพารามิเตอร์เปลี่ยน)
        self.last_sent_warn_period = None
        self.last_sent_emer_period = None

        # --- Read params ---
        gp = self.get_parameter

        self.bumper_topic = gp('bumper_topic').value

        self.use_controller_status = bool(gp('use_controller_status').value)
        self.ctrl_status_topic     = gp('controller_status_topic').value
        self.emer_topic_fallback   = gp('emergency_topic').value

        self.dist_topic = gp('distance_topic').value
        self.dist_type  = str(gp('distance_msg_type').value).lower()

        self.d_thresh = float(gp('distance_thresh').value)

        self.warn_tr  = int(gp('warn_track').value)
        self.emer_tr  = int(gp('emergency_track').value)

        self.warn_blink_ms = self._as_int_ms(gp('warn_blink_ms').value)
        self.emer_blink_ms = self._as_int_ms(gp('emer_blink_ms').value)

        self.cooldown = float(gp('retrigger_cooldown_sec').value)

        self.safe_light_refresh = float(gp('safe_light_refresh_sec').value)
        self.warn_light_refresh = float(gp('warn_light_refresh_sec').value)
        self.emer_light_refresh = float(gp('emer_light_refresh_sec').value)

        self.emergency_true_is_emergency = bool(gp('emergency_true_is_emergency').value)
        self.bumper_true_is_ok           = bool(gp('bumper_true_is_ok').value)

        # --- Pub/Sub ---
        self.pub_json = self.create_publisher(String, '/tower/udp_json', 10)

        # bumper
        self.create_subscription(Bool, self.bumper_topic, self.cb_bumper, 10)

        # emergency source: ControllerStatus หรือ fallback Bool
        if self.use_controller_status:
            self.create_subscription(ControllerStatus, self.ctrl_status_topic,
                                     self.cb_emergency_from_controller, 10)
            self.get_logger().info(f'Using ControllerStatus: {self.ctrl_status_topic}')
        else:
            self.create_subscription(Bool, self.emer_topic_fallback, self.cb_emergency_bool, 10)
            self.get_logger().info(f'Using emergency Bool: {self.emer_topic_fallback}')

        # distance
        if self.dist_type == 'float32':
            self.create_subscription(Float32, self.dist_topic, self.cb_distance_f, 10)
        elif self.dist_type == 'int32':
            self.create_subscription(Int32,   self.dist_topic, self.cb_distance_i, 10)
        else:
            self.get_logger().warn(f'Unknown distance_msg_type="{self.dist_type}", fallback to float32')
            self.create_subscription(Float32, self.dist_topic, self.cb_distance_f, 10)

        # --- Runtime states ---
        self.emergency_ok = True
        self.bumper_ok = True
        self.distance_m = 9999.0

        self.state = S.SAFE
        self.last_light_refresh_ts = 0.0
        self.last_command_ts = 0.0

        # dynamic parameter updates (เปลี่ยนช่วง blink ได้ทันที)
        self.add_on_set_parameters_callback(self._on_param_update)

        # 10 Hz loop
        self.timer = self.create_timer(0.1, self.tick)

        self.get_logger().info(
            f'Rules ready: use_controller_status={self.use_controller_status}, '
            f'warn_ms={self.warn_blink_ms}, emer_ms={self.emer_blink_ms}, thresh={self.d_thresh}'
        )

    # ----------------- Utils -----------------
    def _as_int_ms(self, v) -> int:
        """รับได้ทั้ง int/float/str -> คืน int ms (bound 50..5000 เพื่อสอดคล้องฝั่ง ESP32)"""
        try:
            iv = int(float(v))
        except Exception:
            iv = 300
        if iv < 50:
            iv = 50
        if iv > 5000:
            iv = 5000
        return iv

    # ----------------- Dynamic params -----------------
    def _on_param_update(self, params):
        updated = {}
        force_light = None  # ("warn"|"emer", new_period)

        for p in params:
            name, val = p.name, p.value
            if name == 'warn_blink_ms':
                newv = self._as_int_ms(val)
                if newv != self.warn_blink_ms:
                    self.warn_blink_ms = newv
                    self.last_sent_warn_period = None
                    updated[name] = newv
                    if self.state == S.WARN:
                        force_light = ("warn", newv)
            elif name == 'emer_blink_ms':
                newv = self._as_int_ms(val)
                if newv != self.emer_blink_ms:
                    self.emer_blink_ms = newv
                    self.last_sent_emer_period = None
                    updated[name] = newv
                    if self.state == S.EMER:
                        force_light = ("emer", newv)
            elif name == 'distance_thresh':
                self.d_thresh = float(val)
                updated[name] = self.d_thresh

        # ยิง light-only ทันทีถ้าค่าที่เกี่ยวกับสถานะปัจจุบันเปลี่ยน
        if force_light:
            kind, period = force_light
            if kind == "warn":
                self._send_light("yellow", "blink", int(period))
                self.last_sent_warn_period = int(period)
                self.get_logger().info(f"APPLY WARN blink -> {int(period)}ms")
            else:
                self._send_light("red", "blink", int(period))
                self.last_sent_emer_period = int(period)
                self.get_logger().info(f"APPLY EMER blink -> {int(period)}ms")

        if updated:
            self.get_logger().info(f'Params updated: {updated}')
        return SetParametersResult(successful=True)

    # ----------------- Helpers -----------------
    def send(self, d: dict):
        """Publish JSON to /tower/udp_json"""
        msg = String()
        d.setdefault('ver', 1)
        msg.data = json.dumps(d, separators=(',', ':'))
        self.pub_json.publish(msg)

    def _send_light(self, color: str, mode: str, period_ms: int | None = None):
        payload = {"ver": 1, "light": {"color": color, "mode": mode}}
        if period_ms is not None:
            payload["light"]["period_ms"] = int(period_ms)
        self.send(payload)

    def _try_play_once(self, track: int, period_ms: int):
        """Play once per state entry (edge-trigger) with cooldown."""
        now = time.time()
        if now - self.last_command_ts < self.cooldown:
            return
        period_ms = int(self._as_int_ms(period_ms))
        self.send({
            "ver": 1,
            "light": {"color": "red" if self.state == S.EMER else "yellow",
                      "mode": "blink",
                      "period_ms": period_ms},
            "audio": {"action": "play", "track": int(track), "repeat": True}
        })
        self.last_command_ts = now

    def _compute_state(self) -> S:
        # Priority: EMER > WARN > SAFE
        if not self.emergency_ok:
            return S.EMER
        if not self.bumper_ok:
            return S.EMER
        if self.distance_m < self.d_thresh:
            return S.WARN
        return S.SAFE

    # ----------------- Callbacks -----------------
    def cb_emergency_from_controller(self, msg: ControllerStatus):
        """
        emergency_status == True → ถือว่า EMER (เมื่อ emergency_true_is_emergency=True)
        """
        emer_active = bool(msg.emergency_status)
        self.emergency_ok = (not emer_active) if self.emergency_true_is_emergency else emer_active

    def cb_emergency_bool(self, msg: Bool):
        """fallback: /emergency (Bool)"""
        self.emergency_ok = (not bool(msg.data)) if self.emergency_true_is_emergency else bool(msg.data)

    def cb_bumper(self, msg: Bool):
        # True = OK หาก bumper_true_is_ok=True
        self.bumper_ok = bool(msg.data) if self.bumper_true_is_ok else (not bool(msg.data))

    def cb_distance_f(self, msg: Float32):
        self.distance_m = float(msg.data)

    def cb_distance_i(self, msg: Int32):
        # สมมติหน่วยเป็น "เมตร" (ถ้าเป็นหน่วยอื่น ปรับฝั่ง publisher ชัดกว่า)
        self.distance_m = float(int(msg.data))

    # ----------------- Main loop -----------------
    def tick(self):
        new_state = self._compute_state()

        # Enter new state → one-shot
        if new_state != self.state:
            self.state = new_state
            self.last_light_refresh_ts = 0.0

            if self.state == S.EMER:
                self._try_play_once(self.emer_tr, self.emer_blink_ms)
                self.last_sent_emer_period = int(self.emer_blink_ms)
                return

            if self.state == S.WARN:
                self._try_play_once(self.warn_tr, self.warn_blink_ms)
                self.last_sent_warn_period = int(self.warn_blink_ms)
                return

            # SAFE
            self.send({"audio": {"action": "stop"}})
            self._send_light("green", "solid")
            self.last_sent_warn_period = None
            self.last_sent_emer_period = None
            return

        # Same state → minimal work (ไม่รีเซ็ตเฟส)
        now = time.time()
        if self.state == S.SAFE:
            if self.safe_light_refresh > 0.0 and (now - self.last_light_refresh_ts > self.safe_light_refresh):
                self._send_light("green", "solid")
                self.last_light_refresh_ts = now
            return

        if self.state == S.WARN:
            desired = int(self.warn_blink_ms)
            if self.last_sent_warn_period is None or self.last_sent_warn_period != desired:
                self._send_light("yellow", "blink", desired)
                self.last_sent_warn_period = desired
                self.last_light_refresh_ts = time.time()
                return
            if self.warn_light_refresh > 0.0 and (now - self.last_light_refresh_ts > self.warn_light_refresh):
                self._send_light("yellow", "blink", desired)
                self.last_light_refresh_ts = now
            return

        if self.state == S.EMER:
            desired = int(self.emer_blink_ms)
            if self.last_sent_emer_period is None or self.last_sent_emer_period != desired:
                self._send_light("red", "blink", desired)
                self.last_sent_emer_period = desired
                self.last_light_refresh_ts = time.time()
                return
            if self.emer_light_refresh > 0.0 and (now - self.last_light_refresh_ts > self.emer_light_refresh):
                self._send_light("red", "blink", desired)
                self.last_light_refresh_ts = now
            return


def main():
    rclpy.init()
    node = TowerRulesBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
