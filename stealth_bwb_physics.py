import numpy as np

class StealthBWBPhysics:
    def __init__(self, dt=0.03):
        self.dt = dt
        self.pos = np.array([0.0, 0.0, 0.0])      # [X, Y, Z]
        self.velocity = np.array([0.0, 0.0, 0.0]) # [Vx, Vy, Vz]
        self.euler = np.array([0.0, 0.0, 0.0])    # [Roll, Pitch, Yaw]
        self.mass = 45.0                          # kg
        self.airspeed = 0.0

    def step(self, delta_elevon_left=0.0, delta_elevon_right=0.0, delta_thrust=0.0):
        # 1. قوة الدفع والسحب
        max_thrust = 350.0 # Newton
        thrust_force = np.clip(delta_thrust, 0.0, 1.0) * max_thrust

        # 2. حساب السرعة الحالية وتقييدها لمنع الانفجار العددي
        speed = np.linalg.norm(self.velocity)
        air_density = 1.225
        wing_area = 3.2
        drag_coeff = 0.025 + 0.05 * (speed / 100.0)
        drag_force = 0.5 * air_density * (speed**2) * wing_area * drag_coeff

        # 3. صافي القوة والتسارع
        forward_acc = (thrust_force - drag_force) / self.mass
        forward_acc = np.clip(forward_acc, -50.0, 50.0) # تقييد التسارع [-50, +50] m/s²

        # 4. تحديث السرعة والاتجاه
        self.velocity[0] += forward_acc * self.dt
        self.velocity = np.clip(self.velocity, -300.0, 300.0) # حد أقصى للسرعة 300 m/s
        self.airspeed = float(np.linalg.norm(self.velocity))

        # 5. تحديث الموقع (ارتفاع الإقلاع)
        if delta_thrust > 0.5 and self.airspeed > 15.0:
            self.pos[2] -= 2.5 * self.dt # الارتفاع بالسالب في الإحداثيات

        return {
            "position": self.pos.tolist(),
            "velocity": self.velocity.tolist(),
            "euler": np.degrees(self.euler).tolist(),
            "airspeed": round(self.airspeed, 2),
            "altitude": round(abs(self.pos[2]), 2),
            "rcs_dBsm": -24.0,
            "battery_pct": 99.5
        }