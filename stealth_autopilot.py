import numpy as np


class StealthAutopilot:
    """
    نظام الطيار الآلي والملاحة الذاتية لطائرة BWB الشبكية
    مع السياج الجغرافي (3D Geofence) وحماية مغلف الطيران والعودة التلقائية RTL.
    """

    def __init__(self, stealth_mode: bool = True):
        self.stealth_mode = stealth_mode

        # 1. معاملات PID للتحكم بالارتفاع (Altitude -> Pitch / Elevator)
        self.kp_alt, self.ki_alt, self.kd_alt = 0.045, 0.004, 0.10
        self.alt_err_int = 0.0
        self.prev_alt_err = 0.0

        # 2. معاملات PID للتحكم بالاتجاه (Heading -> Roll / Aileron)
        self.kp_head, self.ki_head, self.kd_head = 1.15, 0.01, 0.25
        self.head_err_int = 0.0
        self.prev_head_err = 0.0

        # قائمة نقاط المسار التكتيكية بنظام NED [(X_north, Y_east, Z_down)]
        self.waypoints = [
            [200.0, 300.0, -120.0],
            [-300.0, 500.0, -150.0],
            [-400.0, -200.0, -100.0],
            [0.0, 0.0, -80.0]
        ]
        self.home_position = [0.0, 0.0, -60.0]
        self.current_wp_index = 0
        self.acceptance_radius = 35.0

        # حدود السياج الجغرافي والأمان الميداني (Field Safety Geofence)
        self.max_geofence_radius = 1200.0  # أقصى مسافة أفقية مسموحة من القاعدة (متر)
        self.max_altitude = 300.0          # أقصى ارتفاع مسموح (متر)
        self.min_battery_pct = 20.0        # عتبة البطارية للعودة الاضطرارية (%)

        self.rtl_triggered = False
        self.failsafe_reason = None

    def set_waypoints(self, waypoints_list: list):
        """تحديث قائمة النقاط التكتيكية وتصفير ذاكرة المتحكم."""
        if waypoints_list:
            self.waypoints = waypoints_list
            self.current_wp_index = 0
            self.clear_failsafe()
            self.head_err_int = 0.0
            self.prev_head_err = 0.0

    def trigger_rtl(self, reason: str = "MANUAL_RTL"):
        """تفعيل وضع العودة الاضطرارية إلى نقطة الإقلاع مع تسجيل السبب."""
        if not self.rtl_triggered:
            self.rtl_triggered = True
            self.failsafe_reason = reason

    def clear_failsafe(self):
        """إلغاء حالة الطوارئ عند استعادة السيطرة اليدوية أو إعادة الضبط."""
        self.rtl_triggered = False
        self.failsafe_reason = None

    def check_geofence(self, current_pos: np.ndarray) -> bool:
        """التحقق من بقاء الطائرة داخل السياج الجغرافي الأفقي والرأسي."""
        home_xy = np.array(self.home_position[:2], dtype=np.float64)
        dist_from_home = float(np.linalg.norm(current_pos[:2] - home_xy))
        current_alt = abs(float(current_pos[2]))

        if dist_from_home > self.max_geofence_radius or current_alt > self.max_altitude:
            self.trigger_rtl(reason="GEOFENCE_BREACH")
            return False
        return True

    def compute_control_surface_commands(self, telemetry: dict, dt: float = 0.03) -> tuple[float, float, float]:
        """حساب أوامر أسطح التوجيه (Elevon Left, Elevon Right) ونسبة الدفع (Thrust)."""
        dt = max(float(dt), 1e-4)

        raw_pos = telemetry.get("pos", telemetry.get("position", [0.0, 0.0, 0.0]))
        current_pos = np.array(raw_pos, dtype=np.float64)

        raw_euler = telemetry.get("euler_deg", telemetry.get("euler", [0.0, 0.0, 0.0]))
        current_heading = np.radians(float(raw_euler[2]))
        battery_pct = float(telemetry.get("battery_pct", 100.0))

        # 1. فحوصات الأمان الميداني (البطارية + السياج الجغرافي)
        if battery_pct < self.min_battery_pct:
            self.trigger_rtl(reason="LOW_BATTERY")

        self.check_geofence(current_pos)

        # تحديد الهدف الحالي (القاعدة في حالة الطوارئ أو النقطة التالية في المسار)
        if self.rtl_triggered:
            target_wp = np.array(self.home_position, dtype=np.float64)
        elif not self.waypoints or self.current_wp_index >= len(self.waypoints):
            return 0.0, 0.0, 0.45
        else:
            target_wp = np.array(self.waypoints[self.current_wp_index], dtype=np.float64)

        # 2. حساب المسافة الأفقية والتبديل للنقطة التالية عند الوصول
        horiz_dist = float(np.linalg.norm(target_wp[:2] - current_pos[:2]))
        if not self.rtl_triggered and horiz_dist < self.acceptance_radius:
            self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)
            target_wp = np.array(self.waypoints[self.current_wp_index], dtype=np.float64)
            self.head_err_int = 0.0
            self.prev_head_err = 0.0

        # 3. التحكم بالارتفاع (Altitude PID مع Anti-Windup)
        target_alt = abs(float(target_wp[2]))
        current_alt = abs(float(current_pos[2]))
        alt_error = target_alt - current_alt

        self.alt_err_int = float(np.clip(self.alt_err_int + alt_error * dt, -40.0, 40.0))
        alt_deriv = (alt_error - self.prev_alt_err) / dt
        self.prev_alt_err = alt_error

        delta_elevator = (
            self.kp_alt * alt_error
            + self.ki_alt * self.alt_err_int
            + self.kd_alt * alt_deriv
        )
        delta_elevator = float(np.clip(delta_elevator, -0.32, 0.32))

        # 4. التحكم بالاتجاه (Heading PID)
        dx = float(target_wp[0] - current_pos[0])
        dy = float(target_wp[1] - current_pos[1])
        target_heading = float(np.arctan2(dy, dx))

        heading_error = target_heading - current_heading
        heading_error = float(np.arctan2(np.sin(heading_error), np.cos(heading_error)))

        self.head_err_int = float(np.clip(self.head_err_int + heading_error * dt, -1.5, 1.5))
        head_deriv = (heading_error - self.prev_head_err) / dt
        self.prev_head_err = heading_error

        max_aileron = 0.18 if self.stealth_mode else 0.28
        delta_aileron = (
            self.kp_head * heading_error
            + self.ki_head * self.head_err_int
            + self.kd_head * head_deriv
        )
        delta_aileron = float(np.clip(delta_aileron, -max_aileron, max_aileron))

        # 5. خلط أوامر الـ Elevons وتقييدها ضمن الحدود الميكانيكية
        max_servo_rad = 0.44
        elevon_left = float(np.clip(delta_elevator + delta_aileron, -max_servo_rad, max_servo_rad))
        elevon_right = float(np.clip(delta_elevator - delta_aileron, -max_servo_rad, max_servo_rad))

        # 6. إدارة الدفع الذكية
        if current_alt < (target_alt - 10.0) or abs(heading_error) > 0.35:
            thrust_cmd = 0.65
        else:
            thrust_cmd = 0.45

        return elevon_left, elevon_right, thrust_cmd