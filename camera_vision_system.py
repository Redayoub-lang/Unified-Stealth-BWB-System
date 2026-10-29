import cv2
import numpy as np


class UAVVisionSystem:
    """
    Onboard Computer Vision Engine for Target Tracking & Optical HUD.
    """
    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        self.target_box = None

    def generate_synthetic_frame(self, telemetry_data):
        """توليد بث بصري محاكي مع تراكب بيانات HUD عند عدم توفر كاميرا حقيقية."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[:] = (15, 23, 42)  # خلفية تكتيكية داكنة

        # رسم الأفق البصري (Artificial Horizon)
        pitch = telemetry_data.get("euler", [0, 0, 0])[1]
        roll = telemetry_data.get("euler", [0, 0, 0])[0]
        
        center_y = int(240 + pitch * 3)
        cv2.line(frame, (160, center_y), (480, center_y), (56, 189, 248), 2)
        cv2.circle(frame, (320, 240), 10, (74, 222, 128), 1)

        # تراكب بيانات الـ HUD
        cv2.putText(frame, f"AIRSPEED: {telemetry_data.get('airspeed', 0):.1f} m/s", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (56, 189, 248), 1)
        cv2.putText(frame, f"ALTITUDE: {-telemetry_data.get('position', [0,0,0])[2]:.1f} m", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (56, 189, 248), 1)
        cv2.putText(frame, f"RCS: {telemetry_data.get('rcs_db', 0):.2f} dBsm", (460, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (245, 158, 11), 1)
        cv2.putText(frame, f"BATTERY: {telemetry_data.get('battery_percent', 100):.1f}%", (460, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (74, 222, 128), 1)

        return frame

    def process_frame(self, telemetry_data=None):
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                return frame
        return self.generate_synthetic_frame(telemetry_data or {})

    def release(self):
        if self.cap.isOpened():
            self.cap.release()