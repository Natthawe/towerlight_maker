#!/usr/bin/env python3
import json
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, Float32, Int32, String
from rcl_interfaces.msg import SetParametersResult


class S(Enum):
    SAFE = 0
    WARN = 1
    EMER = 2


class TowerRulesBridge(Node):

    def __init__(self):
        super().__init__('tower_rules_bridge')

        # --- Declare params ---
        self.declare_parameter('bumper_topic', '/bumper_active')
        self.declare_parameter('emergency_topic', '/emergency')
        self.declare_parameter('distance_topic', '/distance')
        self.declare_parameter('distance_msg_type', 'float32')  # 'float32' | 'int32'

        self.declare_parameter('distance_thresh', 0.5)

        self.declare_parameter('warn_track', 2)
        self.declare_parameter('emergency_track', 1)
        self.declare_parameter('warn_blink_ms', 1000)
        self.declare_parameter('emer_blink_ms', 150)

        self.declare_parameter('retrigger_cooldown_sec', 3.0)

        # refresh รายสถานะ (0.0 = ปิด เพื่อไม่ให้เฟส blink รีเซ็ต)
        self.declare_parameter('safe_light_refresh_sec', 5.0)
        self.declare_parameter('warn_light_refresh_sec', 0.0)
        self.declare_parameter('emer_light_refresh_sec', 0.0)

        self.declare_parameter('emergency_true_is_emergency', True)
        self.declare_parameter('bumper_true_is_ok', True)

        # เก็บ period ที่ส่งล่าสุด (สำหรับอัปเดตครั้งเดียวเมื่อค่าเปลี่ยน)
        self.last_sent_warn_period = None
        self.last_sent_emer_period = None

        # --- Read params (robust) ---
        gp = self.get_parameter

        self.bumper_topic = gp('bumper_topic').value
        self.emer_topic   = gp('emergency_topic').value
        self.dist_topic   = gp('distance_topic').value
        self.dist_type    = str(gp('distance_msg_type').value).lower()

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

        self.create_subscription(Bool, self.emer_topic,   self.cb_emergency, 10)
        self.create_subscription(Bool, self.bumper_topic, self.cb_bumper,    10)

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

        # dynamic parameter updates
        self.add_on_set_parameters_callback(self._on_param_update)

        # 10 Hz loop
        self.timer = self.create_timer(0.1, self.tick)

        self.get_logger().info(
            f'Rules ready: warn_ms={self.warn_blink_ms}, emer_ms={self.emer_blink_ms}, '
            f'thresh={self.d_thresh}'
        )

    # ----------------- Utils -----------------
    def _as_int_ms(self, v) -> int:
        """รับได้ทั้ง int/float/str -> คืน int ms (bound 50..5000 เพื่อสอดคล้อง ESP32)"""
        try:
            iv = int(float(v))
        except Exception:
            iv = 300
        if iv < 50:
            iv = 50
        if iv > 5000:
            iv = 5000
        return iv

    def _on_param_update(self, params):
        updated = {}
        force_light = None  # ("warn", period) หรือ ("emer", period)

        for p in params:
            name = p.name
            val  = p.value
            if name == 'warn_blink_ms':
                newv = self._as_int_ms(val)
                if newv != self.warn_blink_ms:
                    self.warn_blink_ms = newv
                    self.last_sent_warn_period = None
                    updated[name] = newv
                    if self.state == S.WARN:
                        force_light = ("warn", newv)   # ยิงทันที
            elif name == 'emer_blink_ms':
                newv = self._as_int_ms(val)
                if newv != self.emer_blink_ms:
                    self.emer_blink_ms = newv
                    self.last_sent_emer_period = None
                    updated[name] = newv
                    if self.state == S.EMER:
                        force_light = ("emer", newv)   # ยิงทันที
            elif name == 'distance_thresh':
                self.d_thresh = float(val)
                updated[name] = self.d_thresh
            # ...รองรับพารามิเตอร์อื่น ๆ ได้ตามต้องการ...

        # ยิง light-only ทันทีถ้าค่าที่เกี่ยวกับสถานะปัจจุบันเปลี่ยน
        if force_light:
            kind, period = force_light
            if kind == "warn":
                self._send_light("yellow", "blink", int(period))
                self.last_sent_warn_period = int(period)
                self.get_logger().info(f"Applied WARN blink period immediately: {period} ms")
            elif kind == "emer":
                self._send_light("red", "blink", int(period))
                self.last_sent_emer_period = int(period)
                self.get_logger().info(f"Applied EMER blink period immediately: {period} ms")

        if updated:
            self.get_logger().info(f'Params updated: {updated}')
        return SetParametersResult(successful=True)

    # ----------------- Helpers -----------------
    def send(self, d: dict):
        """Publish JSON to /tower/udp_json พร้อม log ยืนยัน period"""
        msg = String()
        d.setdefault('ver', 1)
        msg.data = json.dumps(d, separators=(',', ':'))
        self.pub_json.publish(msg)

        # log สั้น ๆ
        light = d.get('light', {})
        audio = d.get('audio', {})
        if light:
            self.get_logger().info(
                f"TX LIGHT color={light.get('color')} mode={light.get('mode')} "
                f"period={light.get('period_ms')}"
            )
        if audio:
            self.get_logger().info(
                f"TX AUDIO action={audio.get('action')} track={audio.get('track')} "
                f"repeat={audio.get('repeat')}"
            )

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
        self.send({
            "light": {"color": "red" if self.state == S.EMER else "yellow",
                      "mode": "blink",
                      "period_ms": int(period_ms)},
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
    def cb_emergency(self, msg: Bool):
        # True = ฉุกเฉิน หาก emergency_true_is_emergency=True
        self.emergency_ok = (not bool(msg.data)) if self.emergency_true_is_emergency else bool(msg.data)

    def cb_bumper(self, msg: Bool):
        # True = OK หาก bumper_true_is_ok=True
        self.bumper_ok = bool(msg.data) if self.bumper_true_is_ok else (not bool(msg.data))

    def cb_distance_f(self, msg: Float32):
        self.distance_m = float(msg.data)

    def cb_distance_i(self, msg: Int32):
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
            # reset remembered periods (ไม่บังคับ แต่ช่วยให้ clean)
            self.last_sent_warn_period = None
            self.last_sent_emer_period = None
            return

        # Same state → only do minimal work (avoid resetting blink phase)
        now = time.time()
        if self.state == S.SAFE:
            if self.safe_light_refresh > 0.0 and (now - self.last_light_refresh_ts > self.safe_light_refresh):
                self._send_light("green", "solid")
                self.last_light_refresh_ts = now
            return

        if self.state == S.WARN:
            desired = int(self.warn_blink_ms)
            if self.last_sent_warn_period is None or self.last_sent_warn_period != desired:
                # ค่าที่ต้องการเปลี่ยน → ส่ง light-only 1 ครั้งเพื่ออัปเดตจังหวะ
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
