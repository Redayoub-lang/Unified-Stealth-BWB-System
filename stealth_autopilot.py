import numpy as np


class StealthAutopilot:
    """
    Autonomous Flight Controller & Waypoint Navigation System for BWB UAV.
    """
    def __init__(self):
        # معامل التحكم بالارتفاع (Pitch PID)
        self.kp_alt, self.ki_alt, self.kd_alt = 0.08, 0.005, 0.12
        self.alt_err_int = 0.0
        self.prev_alt_err = 0.0

        # معامل التحكم بالاتجاه (Heading PID)
        self.kp_head, self.ki_head, self.kd_head = 1.2, 0.01, 0.25
        self.head_err_int = 0.0
        self.prev_head_err = 0.0

        # قائمة نقاط المسار التكتيكية [(X, Y, Z_target)] (Z بالسالب للارتفاع)
        self.waypoints = [
            [200.0, 300.0, -120.0],
            [-300.0, 500.0, -150.0],
            [-400.0, -200.0, -100.0],
            [0.0, 0.0, -80.0]
        ]
        self.current_wp_index = 0
        self.acceptance_radius = 35.0  # نصف قطر الوصول للنقطة (متر)

    def set_waypoints(self, waypoints_list):
        """تحديث قائمة النقاط التكتيكية."""
        self.waypoints = waypoints_list
        self.current_wp_index = 0

    def compute_control_surface_commands(self, telemetry, dt=0.03):
        """حساب أوامر الأسطح والسرعة للوصول للـ Waypoint الحالية."""
        if not self.waypoints or self.current_wp_index >= len(self.waypoints):
            return 0.0, 0.0, 0.45  # وضع الإبحار الافتراضي (Elevon L, Elevon R, Thrust)

        target_wp = np.array(self.waypoints[self.current_wp_index])
        current_pos = np.array(telemetry["position"])

        # حساب المسافة المتبقية للنقطة
        dist_to_wp = np.linalg.norm(target_wp - current_pos)
        if dist_to_wp < self.acceptance_radius:
            self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)
            target_wp = np.array(self.waypoints[self.current_wp_index])

        # 1. التحكم بالارتفاع (Altitude Control)
        target_alt = target_wp[2]
        current_alt = current_pos[2]
        alt_error = target_alt - current_alt
        self.alt_err_int += alt_error * dt
        alt_deriv = (alt_error - self.prev_alt_err) / dt if dt > 0 else 0.0
        self.prev_alt_err = alt_error

        # أمر الـ Pitch المطول (Elevon Sum)
        delta_elevator = -(self.kp_alt * alt_error + self.ki_alt * self.alt_err_int + self.kd_alt * alt_deriv)
        delta_elevator = np.clip(delta_elevator, -0.35, 0.35)

        # 2. التحكم بالاتجاه (Heading / Yaw Control)
        dx = target_wp[0] - current_pos[0]
        dy = target_wp[1] - current_pos[1]
        target_heading = np.arctan2(dy, dx)
        current_heading = np.radians(telemetry["euler"][2])

        heading_error = target_heading - current_heading
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))  # تطبيع [-pi, pi]

        self.head_err_int += heading_error * dt
        head_deriv = (heading_error - self.prev_head_err) / dt if dt > 0 else 0.0
        self.prev_head_err = heading_error

        # أمر الـ Aileron لمالان الجناح والتوجيه (Elevon Difference)
        delta_aileron = self.kp_head * heading_error + self.ki_head * self.head_err_int + self.kd_head * head_deriv
        delta_aileron = np.clip(delta_aileron, -0.25, 0.25)

        # دمج الأوامر لأسطح الـ Elevons (Left/Right)
        elevon_left = delta_elevator + delta_aileron
        elevon_right = delta_elevator - delta_aileron

        # تعديل نسبة الدفع بناءً على الانعطاف
        thrust_cmd = 0.55 if abs(heading_error) > 0.3 else 0.42

        return elevon_left, elevon_right, thrust_cmd