import numpy as np


class StealthBWBPhysics:
    """
    المحرك الفيزيائي ثلاثي الأبعاد لطائرة BWB الشبكية بنظام الإحداثيات NED
    (X: شمال، Y: شرق، Z: للأسفل - الارتفاع قيمة سالبة).
    """

    def __init__(self, dt: float = 0.03):
        self.dt = dt
        self.pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)       # [X, Y, Z] (m)
        self.velocity = np.array([0.0, 0.0, 0.0], dtype=np.float64)  # [Vx, Vy, Vz] (m/s)
        self.euler = np.array([0.0, 0.0, 0.0], dtype=np.float64)     # [Roll, Pitch, Yaw] (rad)

        self.mass = 45.0
        self.gravity = 9.81
        self.airspeed = 0.0
        self.battery_pct = 100.0

        # إحداثيات محطة الرادار الأرضية المعادية (SAM Radar Site)
        self.sam_radar_pos = np.array([300.0, 400.0, 0.0], dtype=np.float64)

    def compute_rcs(self) -> float:
        """حساب البصمة الرادارية اللحظية بناءً على زاوية الميل والانعطاف."""
        base_rcs = -26.0  # dBsm
        roll_deg = abs(np.degrees(self.euler[0]))
        pitch_deg = abs(np.degrees(self.euler[1]))
        exposure_penalty = (roll_deg * 0.42) + (pitch_deg * 0.35)
        return float(np.clip(base_rcs + exposure_penalty, -26.0, 0.0))

    def evaluate_radar_threat(self, rcs_dBsm: float) -> dict:
        """تطبيق معادلة الرادار لحساب مسافة وخطر الكشف الراداري."""
        rcs_m2 = 10.0 ** (rcs_dBsm / 10.0)
        dist = float(np.linalg.norm(self.pos - self.sam_radar_pos))

        k_radar = 8.5e7  # معامل قدرة الرادار والحساسية
        snr_factor = (k_radar * rcs_m2) / (max(dist, 1.0) ** 4)

        is_detected = bool(snr_factor > 1.0)
        risk_percentage = min(100.0, float(snr_factor * 100.0))

        return {
            "radar_pos": self.sam_radar_pos.tolist(),
            "distance_to_radar": round(dist, 1),
            "is_detected": is_detected,
            "threat_risk_pct": round(risk_percentage, 1)
        }

    def step(self, delta_elevon_left: float = 0.0, delta_elevon_right: float = 0.0, delta_thrust: float = 0.0) -> dict:
        """تحديث الحالة الفيزيائية ثلاثية الأبعاد للطائرة خلال خطوة زمنية dt."""
        throttle = float(np.clip(delta_thrust, 0.0, 1.0))

        # 1. تحليل أوامر أسطح التحكم الخلفية (Elevon Mixing)
        elevator_cmd = 0.5 * (delta_elevon_left + delta_elevon_right)  # التحكم بالصعود/الهبوط (Pitch)
        aileron_cmd = 0.5 * (delta_elevon_left - delta_elevon_right)   # التحكم بالميلان الجانبي (Roll)

        # 2. ديناميكا زوايا الطائرة (استقرار هوائي واقعي بدلاً من الدوران اللانهائي)
        tau_attitude = 0.25  # ثابت زمن استجابة الهيكل (ثانية)
        alpha = self.dt / (tau_attitude + self.dt)

        target_roll = float(np.clip(aileron_cmd * 1.8, -0.78, 0.78))   # أقصى ميلان ±45 درجة
        # إذا كانت السرعة أقل من سرعة الانهيار (14 m/s)، تميل مقدمة الطائرة للأسفل طبيعياً
        stall_pitch = -0.15 if (self.airspeed < 14.0 and self.pos[2] < -1.0) else 0.0
        target_pitch = float(np.clip(elevator_cmd * 1.3 + stall_pitch, -0.52, 0.52))  # ±30 درجة

        self.euler[0] += alpha * (target_roll - self.euler[0])
        self.euler[1] += alpha * (target_pitch - self.euler[1])

        # معدل الالتفاف الأفقي المنسق (Coordinated Turn + Differential Drag)
        turn_rate = (self.gravity / max(self.airspeed, 12.0)) * np.tan(self.euler[0]) + (0.35 * aileron_cmd)
        self.euler[2] = (self.euler[2] + turn_rate * self.dt + np.pi) % (2.0 * np.pi) - np.pi

        # 3. حساب قوى الدفع والسحب والجاذبية على محور الطيران
        max_thrust = 350.0
        thrust_force = throttle * max_thrust

        air_density = 1.225
        wing_area = 3.2
        drag_coeff = 0.025 + 0.08 * (self.euler[1] ** 2) + 0.03 * (self.airspeed / 100.0)
        drag_force = 0.5 * air_density * (self.airspeed ** 2) * wing_area * drag_coeff
        gravity_climb_drag = self.mass * self.gravity * np.sin(self.euler[1])

        forward_acc = np.clip((thrust_force - drag_force - gravity_climb_drag) / self.mass, -40.0, 40.0)
        self.airspeed = float(np.clip(self.airspeed + forward_acc * self.dt, 0.0, 120.0))

        # 4. تفكيك السرعة إلى متجه ثلاثي الأبعاد في نظام NED وتحديث الموقع [X, Y, Z]
        roll, pitch, yaw = self.euler
        self.velocity[0] = self.airspeed * np.cos(pitch) * np.cos(yaw)   # الشمال (X)
        self.velocity[1] = self.airspeed * np.cos(pitch) * np.sin(yaw)   # الشرق (Y)
        self.velocity[2] = -self.airspeed * np.sin(pitch)                # للأسفل (Z سالب عند الصعود)

        self.pos += self.velocity * self.dt

        # 5. منع اختراق سطح الأرض (Ground Collision Protection عند Z >= 0)
        if self.pos[2] >= 0.0:
            self.pos[2] = 0.0
            if self.velocity[2] > 0.0:
                self.velocity[2] = 0.0
            if self.euler[1] < 0.0:
                self.euler[1] = 0.0
            # احتكاك المدرج عند إيقاف المحرك
            if throttle < 0.05:
                self.airspeed *= 0.96

        # 6. تحديث استهلاك البطارية
        self.battery_pct = max(0.0, self.battery_pct - (0.002 + 0.012 * throttle) * self.dt)

        current_rcs = self.compute_rcs()
        threat_info = self.evaluate_radar_threat(current_rcs)

        return {
            "position": self.pos.tolist(),
            "velocity": self.velocity.tolist(),
            "euler": np.degrees(self.euler).tolist(),
            "airspeed": round(self.airspeed, 2),
            "altitude": round(abs(float(self.pos[2])), 2),
            "rcs_dBsm": round(current_rcs, 2),
            "radar_threat": threat_info,
            "battery_pct": round(self.battery_pct, 2)
        }