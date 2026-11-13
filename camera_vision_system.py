import cv2
import numpy as np


class UAVVisionSystem:
    """
    نظام الرؤية الحاسوبية وشاشة العرض التكتيكية (Tactical HUD) لطائرة BWB.
    يدعم الكاميرات الحقيقية (USB / CSI / RTSP) أو وضع المحاكاة البصرية.
    """

    def __init__(self, camera_source=0, width: int = 640, height: int = 360):
        self.camera_source = camera_source
        self.width = width
        self.height = height
        self.cap = cv2.VideoCapture(self.camera_source)

    def _draw_tactical_hud(self, frame: np.ndarray, telemetry: dict, is_synthetic: bool) -> np.ndarray:
        """
        إسقاط بيانات القياس الحية والأفق الاصطناعي على إطار الفيديو.
        """
        h, w, _ = frame.shape
        cx, cy = w // 2, h // 2

        # استخراج البيانات بأمان مع دعم التوافق الكامل مع مفاتيح محرك الفيزياء
        pos = telemetry.get("pos", telemetry.get("position", [0.0, 0.0, 0.0]))
        altitude = float(telemetry.get("altitude", abs(float(pos[2])) if len(pos) > 2 else 0.0))
        airspeed = float(telemetry.get("airspeed", 0.0))

        euler_deg = telemetry.get("euler_deg", telemetry.get("euler", [0.0, 0.0, 0.0]))
        roll_deg = float(euler_deg[0]) if len(euler_deg) > 0 else 0.0
        pitch_deg = float(euler_deg[1]) if len(euler_deg) > 1 else 0.0
        yaw_deg = float(euler_deg[2]) if len(euler_deg) > 2 else 0.0

        rcs = float(telemetry.get("rcs_dBsm", telemetry.get("rcs_dbsm", -26.0)))
        battery = float(telemetry.get("battery_pct", 100.0))

        hud_color = (0, 255, 204)  # أخضر تكتيكي (BGR)
        bat_color = (0, 255, 0) if battery > 30.0 else (0, 0, 255)

        # 1. الإطار الخارجي للشاشة
        cv2.rectangle(frame, (10, 10), (w - 10, h - 10), (0, 80, 90), 1)

        # 2. خط الأفق الاصطناعي الديناميكي (Artificial Horizon) بناءً على Roll و Pitch
        roll_rad = np.radians(roll_deg)
        pitch_offset = int(np.clip(pitch_deg * 2.5, -80, 80))
        horizon_len = 95
        dx = int(horizon_len * np.cos(roll_rad))
        dy = int(horizon_len * np.sin(roll_rad))

        p1 = (cx - dx, cy + pitch_offset - dy)
        p2 = (cx + dx, cy + pitch_offset + dy)
        cv2.line(frame, p1, p2, (0, 200, 255), 1, cv2.LINE_AA)

        # 3. مؤشر التصويب المركزي الثابت (Boresight Crosshair)
        cv2.line(frame, (cx - 22, cy), (cx - 6, cy), hud_color, 2)
        cv2.line(frame, (cx + 6, cy), (cx + 22, cy), hud_color, 2)
        cv2.line(frame, (cx, cy - 16), (cx, cy - 5), hud_color, 2)
        cv2.circle(frame, (cx, cy), 3, hud_color, -1)

        # 4. شريط البيانات العلوي (حالة الكاميرا، البوصلة، البصمة الرادارية)
        mode_text = "OPTICAL CAM [SYNTHETIC]" if is_synthetic else "OPTICAL CAM [LIVE EO]"
        cv2.putText(frame, mode_text, (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.5, hud_color, 1, cv2.LINE_AA)
        cv2.putText(frame, f"HDG: {yaw_deg % 360:05.1f} deg", (cx - 55, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.5, hud_color, 1, cv2.LINE_AA)
        cv2.putText(frame, f"RCS: {rcs:.1f} dBsm", (w - 165, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.5, hud_color, 1, cv2.LINE_AA)

        # 5. شريط البيانات السفلي (الارتفاع، السرعة، البطارية)
        cv2.putText(frame, f"ALT: {altitude:.1f} m", (20, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, f"SPD: {airspeed:.1f} m/s", (cx - 60, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, f"BAT: {battery:.0f}%", (w - 115, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, bat_color, 2, cv2.LINE_AA)

        return frame

    def process_frame(self, telemetry: dict) -> np.ndarray:
        """
        التقاط إطار من الكاميرا أو توليد شاشة رادارية موحدة الأبعاد مع إسقاط البيانات.
        """
        # 1. القراءة من الكاميرا الحقيقية وتوحيد أبعادها
        if self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret and frame is not None:
                frame = cv2.resize(frame, (self.width, self.height))
                return self._draw_tactical_hud(frame, telemetry, is_synthetic=False)

        # 2. توليد شاشة رادار بصري افتراضية عند غياب الكاميرا
        synthetic_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        cv2.circle(synthetic_frame, (self.width // 2, self.height // 2), 65, (0, 45, 55), 1)
        cv2.circle(synthetic_frame, (self.width // 2, self.height // 2), 125, (0, 35, 45), 1)

        return self._draw_tactical_hud(synthetic_frame, telemetry, is_synthetic=True)

    def release(self):
        """
        تحرير مورد الكاميرا بأمان عند إيقاف النظام.
        """
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()