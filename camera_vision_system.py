import threading
import time
import cv2
import numpy as np


class UAVVisionSystem:
    """
    نظام الرؤية الحاسوبية وشاشة العرض التكتيكية (Tactical HUD) لطائرة BWB.
    يعمل التقاط الفيديو في خيط مستقل (Non-Blocking Thread) لضمان عدم تأخير حلقة الطيران.
    """

    def __init__(self, camera_source=0, width: int = 640, height: int = 360):
        self.camera_source = camera_source
        self.width = width
        self.height = height
        self.cap = cv2.VideoCapture(self.camera_source)

        self._lock = threading.Lock()
        self._latest_frame = None
        self._running = True
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

    def _capture_loop(self):
        """خيط مستقل لقراءة إطارات الكاميرا في الخلفية دون تعطيل حلقة الطيران."""
        while self._running:
            if self.cap is not None and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    resized = cv2.resize(frame, (self.width, self.height))
                    with self._lock:
                        self._latest_frame = resized
                else:
                    with self._lock:
                        self._latest_frame = None
                    time.sleep(0.05)
            else:
                time.sleep(0.2)

    def _draw_tactical_hud(self, frame: np.ndarray, telemetry: dict, is_synthetic: bool) -> np.ndarray:
        """إسقاط بيانات القياس الحية والأفق الاصطناعي على إطار الفيديو."""
        h, w, _ = frame.shape
        cx, cy = w // 2, h // 2

        pos = telemetry.get("pos", telemetry.get("position", [0.0, 0.0, 0.0]))
        altitude = float(telemetry.get("altitude", abs(float(pos[2])) if len(pos) > 2 else 0.0))
        airspeed = float(telemetry.get("airspeed", 0.0))

        euler_deg = telemetry.get("euler_deg", telemetry.get("euler", [0.0, 0.0, 0.0]))
        roll_deg = float(euler_deg[0]) if len(euler_deg) > 0 else 0.0
        pitch_deg = float(euler_deg[1]) if len(euler_deg) > 1 else 0.0
        yaw_deg = float(euler_deg[2]) if len(euler_deg) > 2 else 0.0

        rcs = float(telemetry.get("rcs_dBsm", telemetry.get("rcs_dbsm", -26.0)))
        battery = float(telemetry.get("battery_pct", 100.0))

        hud_color = (0, 255, 204)
        bat_color = (0, 255, 0) if battery > 30.0 else (0, 0, 255)

        # 1. الإطار الخارجي للشاشة
        cv2.rectangle(frame, (10, 10), (w - 10, h - 10), (0, 80, 90), 1)

        # 2. خط الأفق الاصطناعي الديناميكي (Artificial Horizon)
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

        # 4. شريط البيانات العلوي
        mode_text = "OPTICAL CAM [SYNTHETIC]" if is_synthetic else "OPTICAL CAM [LIVE EO]"
        cv2.putText(frame, mode_text, (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.5, hud_color, 1, cv2.LINE_AA)
        cv2.putText(frame, f"HDG: {yaw_deg % 360:05.1f} deg", (cx - 55, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.5, hud_color, 1, cv2.LINE_AA)
        cv2.putText(frame, f"RCS: {rcs:.1f} dBsm", (w - 165, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.5, hud_color, 1, cv2.LINE_AA)

        # 5. شريط البيانات السفلي
        cv2.putText(frame, f"ALT: {altitude:.1f} m", (20, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, f"SPD: {airspeed:.1f} m/s", (cx - 60, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, f"BAT: {battery:.0f}%", (w - 115, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, bat_color, 2, cv2.LINE_AA)

        return frame

    def process_frame(self, telemetry: dict) -> np.ndarray:
        """دمج أحدث إطار متاح من الكاميرا (أو الشاشة الافتراضية) مع بيانات القياس فوراً."""
        with self._lock:
            frame = self._latest_frame.copy() if self._latest_frame is not None else None

        if frame is not None:
            return self._draw_tactical_hud(frame, telemetry, is_synthetic=False)

        synthetic_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        cv2.circle(synthetic_frame, (self.width // 2, self.height // 2), 65, (0, 45, 55), 1)
        cv2.circle(synthetic_frame, (self.width // 2, self.height // 2), 125, (0, 35, 45), 1)
        return self._draw_tactical_hud(synthetic_frame, telemetry, is_synthetic=True)

    def release(self):
        """إيقاف خيط الكاميرا وتحرير العتاد بأمان عند إغلاق النظام."""
        self._running = False
        if self._capture_thread.is_alive():
            self._capture_thread.join(timeout=1.0)
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()