import numpy as np


class StealthBWBPhysics:
    """
    6-DOF Aerodynamic, Low-RCS & Power Consumption Physics Engine for BWB UAV.
    """
    def __init__(self, dt=0.03):
        self.dt = dt
        self.rho = 1.225  # كثافة الهواء (kg/m^3)

        # الخصائص الكتلية والهندسية للهيكل المدمج
        self.m = 3.2        # الكتلة (kg)
        self.S = 0.58       # مساحة الجناح (m^2)
        self.b = 2.1        # باع الجناح (m)
        self.c = 0.32       # متوسط الوتر (m)
        self.g = 9.81

        # مصفوفة العطالة الهيكلية
        self.J = np.array([
            [0.18, 0.0, -0.005],
            [0.0, 0.09, 0.0],
            [-0.005, 0.0, 0.26]
        ])
        self.J_inv = np.linalg.inv(self.J)

        # المعاملات الأيروديناميكية (High L/D Design)
        self.CL0, self.CL_alpha, self.CL_de = 0.16, 4.3, 0.30
        self.CD0, self.e = 0.013, 0.92
        self.AR = (self.b**2) / self.S

        # معامل الاستقرار لأسطح الـ Elevons
        self.Cm0, self.Cm_alpha, self.Cm_de, self.Cm_q = 0.01, -0.85, -0.68, -5.5
        self.Cl_beta, self.Cl_da, self.Cl_p = -0.06, 0.19, -0.42
        self.Cn_beta, self.Cn_dr, self.Cn_r = 0.04, -0.03, -0.12

        # منظومة الطاقة (Li-Ion 4S)
        self.battery_capacity_mah = 5000.0
        self.battery_level = 100.0

        # الحالة الابتدائية
        self.pos = np.array([0.0, 0.0, -80.0], dtype=np.float64)
        self.vel_body = np.array([22.0, 0.0, 0.0], dtype=np.float64)
        self.euler = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.omega = np.array([0.0, 0.0, 0.0], dtype=np.float64)

    def calculate_rcs(self, azimuth_deg=0.0, elevation_deg=0.0):
        """حساب المقطع الراداري (RCS) بناءً على زوايا توجيه الهيكل."""
        az = np.radians(azimuth_deg)
        el = np.radians(elevation_deg)
        rcs_base = 0.004  # 0.004 m^2
        spike = 0.06 * (np.exp(-((abs(az) - np.radians(30))**2) / 0.004))
        rcs_m2 = rcs_base + spike * (np.cos(el)**2)
        return rcs_m2, 10.0 * np.log10(rcs_m2)

    def step(self, delta_elevon_left=0.0, delta_elevon_right=0.0, delta_thrust=0.5):
        """تحديث الديناميكا والحركة بخطوة زمنية dt."""
        delta_a = 0.5 * (delta_elevon_left - delta_elevon_right)
        delta_e = 0.5 * (delta_elevon_left + delta_elevon_right)

        u, v, w = self.vel_body
        p, q, r = self.omega

        Va = max(np.sqrt(u**2 + v**2 + w**2), 1.0)
        alpha = np.arctan2(w, u)
        beta = np.arcsin(np.clip(v / Va, -1.0, 1.0))
        q_bar = 0.5 * self.rho * (Va**2)

        CL = self.CL0 + self.CL_alpha * alpha + self.CL_de * delta_e
        CD = self.CD0 + (CL**2) / (np.pi * self.e * self.AR)
        CY = self.Cl_beta * beta

        f_lift = q_bar * self.S * CL
        f_drag = q_bar * self.S * CD
        f_side = q_bar * self.S * CY

        ca, sa = np.cos(alpha), np.sin(alpha)
        Fx_aero = -f_drag * ca + f_lift * sa
        Fz_aero = -f_drag * sa - f_lift * ca
        Fy_aero = f_side
        Fx_thrust = delta_thrust * 18.0

        # حساب استهلاك البطارية
        current_draw = (delta_thrust ** 1.8) * 22.0
        mah_consumed = (current_draw * 1000.0) * (self.dt / 3600.0)
        self.battery_level = max(0.0, self.battery_level - (mah_consumed / self.battery_capacity_mah) * 100.0)

        # العزوم الأيروديناميكية
        l_aero = q_bar * self.S * self.b * (self.Cl_beta * beta + self.Cl_da * delta_a + (self.b / (2 * Va)) * (self.Cl_p * p))
        m_aero = q_bar * self.S * self.c * (self.Cm0 + self.Cm_alpha * alpha + self.Cm_de * delta_e + (self.c / (2 * Va)) * self.Cm_q * q)
        n_aero = q_bar * self.S * self.b * (self.Cn_beta * beta + (self.b / (2 * Va)) * (self.Cn_r * r))

        # تكامل نيوتن-أويلر
        phi, theta, psi = self.euler
        c_ph, s_ph = np.cos(phi), np.sin(phi)
        c_th, s_th = np.cos(theta), np.sin(theta)
        c_ps, s_ps = np.cos(psi), np.sin(psi)

        R = np.array([
            [c_th*c_ps, s_ph*s_th*c_ps - c_ph*s_ps, c_ph*s_th*c_ps + s_ph*s_ps],
            [c_th*s_ps, s_ph*s_th*s_ps + c_ph*c_ps, c_ph*s_th*s_ps - s_ph*c_ps],
            [-s_th, s_ph*c_th, c_ph*c_th]
        ])

        gravity_body = R.T @ np.array([0.0, 0.0, self.m * self.g])
        Total_Forces = np.array([Fx_aero + Fx_thrust, Fy_aero, Fz_aero]) + gravity_body

        self.vel_body += ((Total_Forces / self.m) - np.cross(self.omega, self.vel_body)) * self.dt
        self.omega += (self.J_inv @ (np.array([l_aero, m_aero, n_aero]) - np.cross(self.omega, self.J @ self.omega))) * self.dt
        self.pos += (R @ self.vel_body) * self.dt

        tan_th = np.tan(theta) if np.abs(np.cos(theta)) > 1e-4 else 0.0
        sec_th = 1.0 / np.cos(theta) if np.abs(np.cos(theta)) > 1e-4 else 1.0
        euler_dot = np.array([
            p + q * s_ph * tan_th + r * c_ph * tan_th,
            q * c_ph - r * s_ph,
            q * s_ph * sec_th + r * c_ph * sec_th
        ])
        self.euler += euler_dot * self.dt

        rcs_m2, rcs_db = self.calculate_rcs(azimuth_deg=0.0, elevation_deg=0.0)
        ld_ratio = float(CL / CD) if CD > 0 else 0.0

        return {
            "position": self.pos.tolist(),
            "velocity": self.vel_body.tolist(),
            "euler": np.degrees(self.euler).tolist(),
            "airspeed": float(Va),
            "alpha_deg": float(np.degrees(alpha)),
            "lift_to_drag": round(ld_ratio, 2),
            "rcs_db": round(float(rcs_db), 2),
            "battery_percent": round(float(self.battery_level), 1)
        }