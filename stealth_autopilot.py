import numpy as np


class StealthAutopilot:
    """
    نظام الطيار الآلي والملاحة الذاتية لطائرة BWB الشبكية
    مزود بخوارزمية تجنب الرادار المماسية (Tangential Radar Evasion) وتقييد البصمة الرادارية.
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

        # قائمة نقاط المسار بنظام NED [(X_north, Y_east, Z_down)]
        # المسار بين النقطة 1 و 2 يمر مباشرة فوق موقع رادار SAM [300, 400] لاختبار الالتفاف الذكي
        self.waypoints = [
            [80.0, 100.0, -120.0],
            [540.0, 640.0, -135.0],
            [-260.0, 480.0, -130.0],
            [0.0, 0.0, -80.0]
        ]
        self.home_position = [0.0, 0.0, -60.0]
        self.current_wp_index = 0
        self.acceptance_radius = 35.0

        # بيانات مواقع الرادارات المعادية (SAM Sites) وحزام الأمان
        self.sam_sites = [
            {"pos": np.array([300.0, 400.0, 0.0], dtype=np.float64), "radius": 180.0}
        ]
        self.evasion_margin = 95.0  # مسافة الأمان الإضافية حول قبة الرادار (180 + 95 = 275m)
        self.radar_evasion_active = False

        # حدود السياج الجغرافي والأمان الميداني (Field Safety Geofence)
        self.max_geofence_radius = 1200.0
        self.max_altitude = 300.0
        self.min_battery_pct = 20.0

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

    def _sanitize_waypoint_against_radars(self, target_wp: np.ndarray) -> np.ndarray:
        """إزاحة نقطة المسار خارج نطاق الرادار إذا قام المستخدم بوضعها داخل منطقة الخطر."""
        if not self.stealth_mode:
            return target_wp

        safe_wp = target_wp.copy()
        for sam in self.sam_sites:
            sam_xy = sam["pos"][:2]
            safe_radius = sam["radius"] + 45.0
            diff = safe_wp[:2] - sam_xy
            dist = float(np.linalg.norm(diff))
            if dist < safe_radius:
                direction = diff / dist if dist > 1e-3 else np.array([-1.0, 0.0])
                safe_wp[:2] = sam_xy + direction * safe_radius
        return safe_wp

    def _compute_evasive_heading(self, current_pos: np.ndarray, target_wp: np.ndarray) -> float:
        """
        حساب زاوية الاتجاه المثلى باستخدام الحقول الكامنة المماسية (Tangential Potential Field)
        للالتفاف السلس حول قباب الرادار دون اختراقها.
        """
        pos_xy = current_pos[:2]
        goal_xy = target_wp[:2]
        vec_to_goal = goal_xy - pos_xy
        dist_to_goal = float(np.linalg.norm(vec_to_goal))

        if dist_to_goal < 1e-3:
            return 0.0

        u_goal = vec_to_goal / dist_to_goal
        combined_dir = u_goal.copy()
        evading = False

        if self.stealth_mode:
            for sam in self.sam_sites:
                sam_xy = sam["pos"][:2]
                radar_r = sam["radius"]
                buffer_r = radar_r + self.evasion_margin

                vec_from_radar = pos_xy - sam_xy
                dist_to_radar = float(np.linalg.norm(vec_from_radar))

                if dist_to_radar < buffer_r:
                    evading = True
                    u_rep = vec_from_radar / max(dist_to_radar, 1e-3)

                    # حساب المتجهين المماسيين حول دائرة الرادار (يمين ويسار) واختيار الأقرب للهدف
                    tangent_ccw = np.array([-u_rep[1], u_rep[0]], dtype=np.float64)
                    tangent_cw = np.array([u_rep[1], -u_rep[0]], dtype=np.float64)
                    best_tangent = (
                        tangent_ccw
                        if float(np.dot(tangent_ccw, u_goal)) >= float(np.dot(tangent_cw, u_goal))
                        else tangent_cw
                    )

                    # تصعيد قوة المناورة كلما اقتربت الطائرة من حافة الكشف الراداري
                    penetration = (buffer_r - dist_to_radar) / max(self.evasion_margin, 1.0)
                    evasion_weight = float(np.clip(penetration, 0.0, 1.6))

                    combined_dir += evasion_weight * (1.45 * best_tangent + 0.90 * u_rep)

        self.radar_evasion_active = evading
        return float(np.arctan2(combined_dir[1], combined_dir[0]))

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

        # تحديد الهدف الحالي
        if self.rtl_triggered:
            raw_target = np.array(self.home_position, dtype=np.float64)
        elif not self.waypoints or self.current_wp_index >= len(self.waypoints):
            self.radar_evasion_active = False
            return 0.0, 0.0, 0.45
        else:
            raw_target = np.array(self.waypoints[self.current_wp_index], dtype=np.float64)

        target_wp = self._sanitize_waypoint_against_radars(raw_target)

        # 2. حساب المسافة الأفقية والتبديل للنقطة التالية عند الوصول
        horiz_dist = float(np.linalg.norm(target_wp[:2] - current_pos[:2]))
        if not self.rtl_triggered and horiz_dist < self.acceptance_radius:
            self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)
            raw_target = np.array(self.waypoints[self.current_wp_index], dtype=np.float64)
            target_wp = self._sanitize_waypoint_against_radars(raw_target)
            self.head_err_int = 0.0
            self.prev_head_err = 0.0

        # 3. التحكم بالارتفاع (Altitude PID مع خفض الارتفاع التكتيكي الطفيف أثناء التخفي)
        target_alt = abs(float(target_wp[2]))
        if self.radar_evasion_active and self.stealth_mode:
            target_alt = max(75.0, target_alt - 25.0)

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

        # 4. التحكم بالاتجاه عبر خوارزمية تجنب الرادار الذكية
        target_heading = self._compute_evasive_heading(current_pos, target_wp)

        heading_error = target_heading - current_heading
        heading_error = float(np.arctan2(np.sin(heading_error), np.cos(heading_error)))

        self.head_err_int = float(np.clip(self.head_err_int + heading_error * dt, -1.5, 1.5))
        head_deriv = (heading_error - self.prev_head_err) / dt
        self.prev_head_err = heading_error

        # تقييد زاوية الالتفاف أثناء المناورة حول الرادار للحفاظ على أقل بصمة رادارية (RCS)
        if self.radar_evasion_active:
            max_aileron = 0.13
        elif self.stealth_mode:
            max_aileron = 0.18
        else:
            max_aileron = 0.28

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
            thrust_cmd = 0.48 if self.radar_evasion_active else 0.45

        return elevon_left, elevon_right, thrust_cmd