import cv2
import numpy as np


class UAVVisionSystem:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)

    def process_frame(self, telemetry):
        # استخراج الارتفاع والسرعة بأمان لتفادي الخطأ
        position = telemetry.get("position", [0, 0, 0])
        altitude = telemetry.get("altitude", abs(position[2]))
        airspeed = telemetry.get("airspeed", 0.0)

        # 1. استخدام كاميرا الويب إن وُجدت
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                cv2.putText(frame, f"ALT: {altitude:.1f}m", (20, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                return frame

        # 2. توليد شاشة رادار بصري افتراضية عند غياب الكاميرا
        frame = np.zeros((200, 300, 3), dtype=np.uint8)
        
        cv2.rectangle(frame, (10, 10), (290, 190), (0, 51, 68), 1)
        cv2.line(frame, (150, 80), (150, 120), (0, 255, 204), 1)
        cv2.line(frame, (130, 100), (170, 100), (0, 255, 204), 1)
        cv2.circle(frame, (150, 100), 25, (0, 255, 204), 1)

        cv2.putText(frame, "OPTICAL CAM [SYNTHETIC]", (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 204), 1)
        cv2.putText(frame, f"ALT: {altitude:.1f}m", (15, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.putText(frame, f"SPD: {airspeed:.1f}m/s", (15, 180),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        return frame