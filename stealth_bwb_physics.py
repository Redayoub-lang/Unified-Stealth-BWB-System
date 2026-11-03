import numpy as np


class StealthBWBPhysics:
    def __init__(self, dt=0.03):
        self.dt = dt
        self.pos = np.array([0.0, 0.0, 0.0])      # [X, Y, Z]
        self.velocity = np.array([0.0, 0.0, 0.0]) # [Vx, Vy, Vz]
        self.euler = np.array([0.0, 0.0, 0.0])    # [Roll, Pitch, Yaw]
        self.mass = 45.0
        self.airspeed = 0.0

        # إحداثيات محطة الرادار الأرضية المعادية (SAM Radar Site)
        self.sam_radar_pos = np.array([300.0, 400.0, 0.0])

    def compute_rcs(self):
        """حساب البصمة الرادارية بناءً على زاوية الميل والانعطاف."""
        base_rcs = -26.0  # dBsm
        roll_deg = abs(np.degrees(self.euler[0]))
        pitch_deg = abs(np.degrees(self.euler[1]))
        exposure_penalty = (roll_deg * 0.42) + (pitch_deg * 0.35)
        return float(np.clip(base_rcs + exposure_penalty, -26.0, 0.0))

    def evaluate_radar_threat(self, rcs_dBsm):
        """تطبيق معادلة الرادار لحساب مسافة واختراق الكشف الراداري (Radar Cross Section)."""
        # تحويل RCS من dBsm إلى متر مربع (m²)
        rcs_m2 = 10.0 ** (rcs_dBsm / 10.0)
        
        # المسافة بين الطائرة والرادار الأرضي
        dist = float(np.linalg.norm(self.pos - self.sam_radar_pos))
        
        # معادلة الإشارة الرادارية (Signal-to-Noise Ratio Factor)
        k_radar = 8.5e7  # معامل قدرة الرادار والحساسية
        snr_factor = (k_radar * rcs_m2) / (max(dist, 1.0) ** 4)

        # عتبة الاكتشاف (Detection Threshold)
        is_detected = bool(snr_factor > 1.0)
        risk_percentage = min(100.0, float(snr_factor * 100.0))

        return {
            "radar_pos": self.sam_radar_pos.tolist(),
            "distance_to_radar": round(dist, 1),
            "is_detected": is_detected,
            "threat_risk_pct": round(risk_percentage, 1)
        }

    def step(self, delta_elevon_left=0.0, delta_elevon_right=0.0, delta_thrust=0.0):
        max_thrust = 350.0
        thrust_force = np.clip(delta_thrust, 0.0, 1.0) * max_thrust

        speed = np.linalg.norm(self.velocity)
        air_density = 1.225
        wing_area = 3.2
        drag_coeff = 0.025 + 0.05 * (speed / 100.0)
        drag_force = 0.5 * air_density * (speed**2) * wing_area * drag_coeff

        forward_acc = np.clip((thrust_force - drag_force) / self.mass, -50.0, 50.0)
        self.velocity[0] += forward_acc * self.dt
        self.velocity = np.clip(self.velocity, -300.0, 300.0)
        self.airspeed = float(np.linalg.norm(self.velocity))

        self.euler[0] += (delta_elevon_left - delta_elevon_right) * 0.1 * self.dt
        self.euler[1] += (delta_elevon_left + delta_elevon_right) * 0.05 * self.dt
        self.euler[2] += (delta_elevon_left - delta_elevon_right) * 0.02 * self.dt

        if delta_thrust > 0.4 and self.airspeed > 15.0:
            self.pos[2] -= (2.5 + self.euler[1] * 10.0) * self.dt

        current_rcs = self.compute_rcs()
        threat_info = self.evaluate_radar_threat(current_rcs)

        return {
            "position": self.pos.tolist(),
            "velocity": self.velocity.tolist(),
            "euler": np.degrees(self.euler).tolist(),
            "airspeed": round(self.airspeed, 2),
            "altitude": round(abs(self.pos[2]), 2),
            "rcs_dBsm": round(current_rcs, 2),
            "radar_threat": threat_info,
            "battery_pct": 99.5
        }