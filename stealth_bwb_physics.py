import json
import os
import numpy as np

AERO_PROFILE_PATH = "aero_profile.json"


class StealthBWBPhysics:
    """
    محرك الفيزياء ثلاثي الأبعاد (3D NED Kinematics & Aerodynamics) لطائرة BWB الشبكية.
    مرتبط مباشرة بمعاملات الرفع والمقاومة المستخرجة من نفق الرياح الافتراضي.
    """

    def __init__(self, dt: float = 0.03):
        self.dt = float(dt)

        # الخصائص البنيوية والديناميكية الهوائية الافتراضية (مطابقة لنفق الرياح)
        self.mass = 45.0          # الكتلة الكلية (kg)
        self.wing_area = 3.2      # مساحة الجناح المدمج (m^2)
        self.max_thrust = 350.0   # أقصى قوة دفع (N)
        self.rho = 1.225          # كثافة الهواء عند سطح البحر (kg/m^3)
        self.g = 9.81

        # معاملات الرفع والمقاومة المستخرجة من نفق الرياح (Wind Tunnel Aero Coefficients)
        self.cl0 = 0.15           # معامل الرفع الأساسي عند زاوية هجوم 0°
        self.cl_alpha = 0.085     # ميل منحنى الرفع لكل درجة (1/deg)
        self.cd0 = 0.022          # معامل المقاومة الطفيلية (Zero-Lift Drag)
        self.induced_k = 0.045    # معامل المقاومة المستحثة (Induced Drag Factor)

        # تحميل آخر معاملات مُعايرة من نفق الرياح إن وُجدت
        self.load_aero_profile()

        # متجهات الحالة بنظام NED: [North (X), East (Y), Down (Z)]
        self.pos = np.array([0.0, 0.0, -60.0], dtype=np.float64)
        self.euler = np.array([0.0, 0.02, 0.0], dtype=np.float64)  # [roll, pitch, yaw] بالراديان
        self.airspeed = 48.0
        self.battery_pct = 100.0

        # موقع رادار التهديد SAM بنظام NED (مطابق للمشهد ثلاثي الأبعاد)
        self.sam_position = np.array([300.0, 400.0, 0.0], dtype=np.float64)
        self.base_rcs_dbsm = -26.0

    def load_aero_profile(self):
        """تحميل معاملات نفق الرياح المحفوظة إن وُجدت."""
        if os.path.exists(AERO_PROFILE_PATH):
            try:
                with open(AERO_PROFILE_PATH, "r", encoding="utf-8") as f:
                    data = json.load(f)
                self.update_aero_coefficients(data, save=False)
            except Exception:
                pass

    def update_aero_coefficients(self, params: dict, save: bool = True) -> dict:
        """تحديث معاملات الرفع والمقاومة والكتلة لحظياً من نفق الرياح."""
        if "cl0" in params:
            self.cl0 = float(params["cl0"])
        if "cl_alpha" in params:
            self.cl_alpha = float(params["cl_alpha"])
        if "cd0" in params:
            self.cd0 = float(params["cd0"])
        if "induced_k" in params:
            self.induced_k = float(params["induced_k"])
        if "wing_area" in params:
            self.wing_area = max(0.2, float(params["wing_area"]))
        if "mass" in params:
            self.mass = max(1.0, float(params["mass"]))
        if "max_thrust" in params:
            self.max_thrust = max(10.0, float(params["max_thrust"]))

        current_profile = {
            "cl0": self.cl0,
            "cl_alpha": self.cl_alpha,
            "cd0": self.cd0,
            "induced_k": self.induced_k,
            "wing_area": self.wing_area,
            "mass": self.mass,
            "max_thrust": self.max_thrust,
        }

        if save:
            try:
                with open(AERO_PROFILE_PATH, "w", encoding="utf-8") as f:
                    json.dump(current_profile, f, indent=2)
            except Exception:
                pass

        return current_profile

    def compute_dynamic_rcs(self, roll_rad: float, pitch_rad: float) -> float:
        """حساب البصمة الرادارية الديناميكية (dBsm) بناءً على زاوية ميلان الجناح."""
        bank_penalty = 14.0 * (abs(np.sin(roll_rad)) ** 1.4)
        pitch_penalty = 6.0 * abs(np.sin(pitch_rad))
        return float(self.base_rcs_dbsm + bank_penalty + pitch_penalty)

    def step(self, delta_elevon_left: float = 0.0, delta_elevon_right: float = 0.0, delta_thrust: float = 0.45) -> dict:
        """تحديث الحالة الفيزيائية لخطوة زمنية واحدة (dt)."""
        dt = self.dt
        thrust_ratio = float(np.clip(delta_thrust, 0.0, 1.0))

        # 1. استخلاص أوامر الرفع (Elevator) والالتفاف (Aileron) من أسطح الـ Elevons
        delta_elevator = 0.5 * (float(delta_elevon_left) + float(delta_elevon_right))
        delta_aileron = 0.5 * (float(delta_elevon_left) - float(delta_elevon_right))

        # 2. تحديث زوايا التوجيه (Roll & Pitch) باستجابة ديناميكية مستقرة
        target_roll = float(np.clip(delta_aileron * 2.4, -0.75, 0.75))
        target_pitch = float(np.clip(delta_elevator * 1.15, -0.35, 0.40))

        self.euler[0] += (target_roll - self.euler[0]) * min(1.0, 6.0 * dt)
        self.euler[1] += (target_pitch - self.euler[1]) * min(1.0, 5.0 * dt)

        roll, pitch, yaw = self.euler

        # 3. الحسابات الديناميكية الهوائية المطابقة لنفق الرياح
        # زاوية الهجوم الفعلية بالدرجات مرتبطة بزاوية الميلان وانحراف الـ Elevator
        alpha_deg = float(np.degrees(pitch) + np.degrees(delta_elevator) * 0.25)
        cl = float(np.clip(self.cl0 + self.cl_alpha * alpha_deg, -0.5, 1.6))
        cd = float(self.cd0 + self.induced_k * (cl ** 2))

        q = 0.5 * self.rho * (self.airspeed ** 2)
        lift_n = q * self.wing_area * cl
        drag_n = q * self.wing_area * cd
        thrust_n = thrust_ratio * self.max_thrust
        weight_n = self.mass * self.g

        # 4. معادلات الحركة الانتقالية والدورانية
        # التسارع الأمامي
        accel_forward = (thrust_n * np.cos(pitch) - drag_n - weight_n * np.sin(pitch)) / self.mass
        self.airspeed = float(np.clip(self.airspeed + accel_forward * dt, 18.0, 95.0))

        # معدل تغير الارتفاع الرأسي الناتج عن التوازن بين مركبة الرفع الرأسية والوزن
        vertical_lift = lift_n * np.cos(roll) * np.cos(pitch) + thrust_n * np.sin(pitch)
        vertical_accel = (vertical_lift - weight_n) / self.mass
        climb_rate = float(np.clip(self.airspeed * np.sin(pitch) + vertical_accel * dt * 2.5, -22.0, 22.0))

        # معدل الدوران الأفقي (Coordinated Turn)
        turn_rate = (self.g * np.tan(roll)) / max(self.airspeed, 15.0)
        self.euler[2] = float((yaw + turn_rate * dt) % (2.0 * np.pi))

        # 5. تحديث الموقع ثلاثي الأبعاد بنظام NED
        horiz_speed = self.airspeed * np.cos(pitch)
        vx = horiz_speed * np.cos(self.euler[2])
        vy = horiz_speed * np.sin(self.euler[2])
        vz = -climb_rate  # سالب للأعلى في نظام NED

        self.pos[0] += vx * dt
        self.pos[1] += vy * dt
        self.pos[2] = min(0.0, self.pos[2] + vz * dt)

        # 6. استهلاك البطارية وتقييم التهديد الراداري
        self.battery_pct = max(0.0, self.battery_pct - (0.012 + 0.025 * thrust_ratio) * dt)

        rcs_dbsm = self.compute_dynamic_rcs(roll, pitch)
        dist_to_sam = float(np.linalg.norm(self.pos - self.sam_position))
        # كلما انخفضت البصمة الرادارية تقل مسافة كشف الرادار الفعالة
        effective_detection_range = 180.0 * (10.0 ** ((rcs_dbsm - (-20.0)) / 40.0))
        is_detected = bool(dist_to_sam < effective_detection_range)

        euler_deg = np.degrees(self.euler).tolist()
        pos_list = self.pos.tolist()

        return {
            "position": pos_list,
            "pos": pos_list,
            "euler": euler_deg,
            "euler_deg": euler_deg,
            "airspeed": float(self.airspeed),
            "altitude": float(-self.pos[2]),
            "battery_pct": float(self.battery_pct),
            "rcs_dBsm": rcs_dbsm,
            "aero": {
                "cl": cl,
                "cd": cd,
                "lift_n": float(lift_n),
                "drag_n": float(drag_n),
                "ld_ratio": float(cl / max(cd, 1e-4)),
            },
            "radar_threat": {
                "is_detected": is_detected,
                "distance_to_radar": dist_to_sam,
            },
        }