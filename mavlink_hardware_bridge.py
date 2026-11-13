#!/usr/bin/env python3
"""
جسر بروتوكول MAVLink لربط نظام Unified-Stealth-BWB بمتحكم طيران حقيقي (Pixhawk / PX4 / ArduPilot)
أو بيئة اختبار Hardware-in-the-Loop (HIL).
"""

import json
import math
import threading
import time

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False

try:
    import websocket
    WS_AVAILABLE = True
except ImportError:
    WS_AVAILABLE = False


EARTH_RADIUS_M = 6378137.0


def local_ned_to_gps(home_lat: float, home_lon: float, home_alt: float,
                     north_m: float, east_m: float, down_m: float) -> tuple[float, float, float]:
    """
    تحويل الإحداثيات المحلية (NED بالمتر) إلى إحداثيات GPS عالمية (WGS84)
    لإرسالها إلى متحكم Pixhawk الحقيقي.
    """
    d_lat = (north_m / EARTH_RADIUS_M) * (180.0 / math.pi)
    d_lon = (east_m / (EARTH_RADIUS_M * math.cos(math.radians(home_lat)))) * (180.0 / math.pi)

    target_lat = home_lat + d_lat
    target_lon = home_lon + d_lon
    target_alt = home_alt - down_m  # down_m سالب في الأعلى
    return target_lat, target_lon, target_alt


class StealthMAVLinkBridge:
    """
    جسر ثنائي الاتجاه بين خادم القيادة والسيطرة (C2) ومتحكم الطيران (Pixhawk).
    """

    def __init__(self, mav_connection_str: str = "udp:127.0.0.1:14550",
                 c2_ws_url: str = "ws://localhost:8000/ws/telemetry"):
        self.mav_connection_str = mav_connection_str
        self.c2_ws_url = c2_ws_url
        self.master = None
        self.running = True

        # نقطة الإقلاع المرجعية (تُحدّث تلقائياً من GPS الطائرة عند الاتصال)
        self.home_lat = 35.7595   # إحداثيات افتراضية (طنجة) حتى قفل GPS الحقيقي
        self.home_lon = -5.8340
        self.home_alt = 50.0
        self.home_locked = False

        # حالة الطائرة الحقيقية المقروءة من Pixhawk
        self.real_telemetry = {
            "lat": self.home_lat,
            "lon": self.home_lon,
            "alt": 0.0,
            "airspeed": 0.0,
            "battery_pct": 100.0
        }
        self._last_sent_wp = None
        self._last_mode = None

    def connect_pixhawk(self) -> bool:
        """الاتصال بمتحكم الطيران وانتظار نبضة القلب (MAVLink Heartbeat)."""
        print(f"[MAVLink] جاري الاتصال بمتحكم الطيران على: {self.mav_connection_str} ...")
        self.master = mavutil.mavlink_connection(self.mav_connection_str)
        hb = self.master.wait_heartbeat(timeout=10)
        if not hb:
            print("[MAVLink ERROR] لم يتم استقبال Heartbeat من متحكم الطيران.")
            return False

        print(f"[MAVLink OK] متصل بالنظام رقم {self.master.target_system} (المكون {self.master.target_component})")
        return True

    def send_gps_waypoint(self, lat: float, lon: float, alt_amsl: float):
        """إرسال نقطة ملاحة GPS مباشرة إلى متحكم الطيران في وضع GUIDED / OFFBOARD."""
        if self.master is None:
            return

        # قناع تجاهل السرعة والتسارع والاعتماد على الموقع الجغرافي فقط
        type_mask = 0b0000111111111000
        self.master.mav.set_position_target_global_int_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask,
            int(lat * 1e7),
            int(lon * 1e7),
            float(alt_amsl),
            0, 0, 0,  # vx, vy, vz
            0, 0, 0,  # afx, afy, afz
            0, 0      # yaw, yaw_rate
        )

    def send_rtl_command(self):
        """إرسال أمر العودة الاضطرارية للقاعدة (RTL) إلى متحكم Pixhawk."""
        if self.master is None:
            return
        print("[MAVLink SAFETY] إرسال أمر العودة للقاعدة RTL إلى Pixhawk!")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def _pixhawk_reader_loop(self):
        """قراءة بيانات الحساسات الحقيقية من Pixhawk باستمرار."""
        while self.running and self.master is not None:
            msg = self.master.recv_match(
                type=["GLOBAL_POSITION_INT", "VFR_HUD", "SYS_STATUS"],
                blocking=True,
                timeout=1.0
            )
            if not msg:
                continue

            msg_type = msg.get_type()
            if msg_type == "GLOBAL_POSITION_INT":
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.alt / 1000.0
                if not self.home_locked and lat != 0.0 and lon != 0.0:
                    self.home_lat, self.home_lon, self.home_alt = lat, lon, alt
                    self.home_locked = True
                    print(f"[GPS HOME LOCKED] Lat: {lat:.6f}, Lon: {lon:.6f}, Alt: {alt:.1f}m")

                self.real_telemetry["lat"] = lat
                self.real_telemetry["lon"] = lon
                self.real_telemetry["alt"] = msg.relative_alt / 1000.0

            elif msg_type == "VFR_HUD":
                self.real_telemetry["airspeed"] = float(msg.airspeed)

            elif msg_type == "SYS_STATUS":
                if msg.battery_remaining >= 0:
                    self.real_telemetry["battery_pct"] = float(msg.battery_remaining)

    def _on_c2_message(self, ws, message: str):
        """استقبال الأوامر ونقاط المسار من لوحة التحكم وترجمتها إلى أوامر MAVLink."""
        try:
            data = json.loads(message)
            mode = data.get("mode")
            target_wp = data.get("target_wp")

            if mode == "RTL" and self._last_mode != "RTL":
                self.send_rtl_command()
            elif target_wp and len(target_wp) == 3:
                if target_wp != self._last_sent_wp:
                    lat, lon, alt = local_ned_to_gps(
                        self.home_lat, self.home_lon, self.home_alt,
                        float(target_wp[0]), float(target_wp[1]), float(target_wp[2])
                    )
                    self.send_gps_waypoint(lat, lon, alt)
                    self._last_sent_wp = list(target_wp)

            self._last_mode = mode
        except Exception as exc:
            print(f"[Bridge Warn] {exc}")

    def run(self):
        """تشغيل الجسر ثنائي الاتجاه."""
        if not self.connect_pixhawk():
            return

        threading.Thread(target=self._pixhawk_reader_loop, daemon=True).start()

        while self.running:
            try:
                ws = websocket.WebSocketApp(
                    self.c2_ws_url,
                    on_open=lambda w: print("[C2 Link] متصل بخادم القيادة والسيطرة بنجاح."),
                    on_message=self._on_c2_message
                )
                ws.run_forever(ping_interval=10, ping_timeout=5)
            except Exception as exc:
                print(f"[C2 Link Error] {exc}")
            time.sleep(2.0)


if __name__ == "__main__":
    if not MAVLINK_AVAILABLE or not WS_AVAILABLE:
        print("لتشغيل جسر العتاد الحقيقي، ثبّت المكتبات المطلوبة:")
        print("pip install pymavlink websocket-client")
    else:
        # أمثلة الاتصال:
        # 1. محاكاة SITL / HIL عبر UDP: "udp:127.0.0.1:14550"
        # 2. متحكم Pixhawk حقيقي عبر USB في ويندوز: "COM3" (بسرعة baud=57600)
        # 3. متحكم Pixhawk حقيقي على Jetson/Raspberry Pi: "/dev/ttyACM0"
        bridge = StealthMAVLinkBridge(mav_connection_str="udp:127.0.0.1:14550")
        bridge.run()