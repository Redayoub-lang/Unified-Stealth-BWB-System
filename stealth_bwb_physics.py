import numpy as np


class StealthBWBPhysics:
    def __init__(self, dt=0.03):
        self.dt = dt
        self.pos = np.array([0.0, 0.0, 0.0])      # [X, Y, Z]
        self.velocity = np.array([0.0, 0.0, 0.0]) # [Vx, Vy, Vz]
        self.euler = np.array([0.0, 0.0, 0.0])    # [Roll, Pitch, Yaw] (rad)
        self.mass = 45.0
        self.airspeed = 0.0

    def compute_rcs(self):
        """Calcul dynamique du RCS en fonction de l'attitude (Roll / Pitch exposure)."""
        base_rcs = -26.0  # Signature spectrale minimale (Vue frontale furtive)
        
        roll_deg = abs(np.degrees(self.euler[0]))
        pitch_deg = abs(np.degrees(self.euler[1]))

        # Penalty d'exposition de la surface sous fort virage (Roll) ou cabrage (Pitch)
        exposure_penalty = (roll_deg * 0.42) + (pitch_deg * 0.35)
        rcs_val = base_rcs + exposure_penalty
        
        # Plafonnement entre -26 dBsm (discret) et 0 dBsm (fortement détectable)
        return float(np.clip(rcs_val, -26.0, 0.0))

    def step(self, delta_elevon_left=0.0, delta_elevon_right=0.0, delta_thrust=0.0):
        # 1. Calcul des forces & poussée
        max_thrust = 350.0
        thrust_force = np.clip(delta_thrust, 0.0, 1.0) * max_thrust

        speed = np.linalg.norm(self.velocity)
        air_density = 1.225
        wing_area = 3.2
        drag_coeff = 0.025 + 0.05 * (speed / 100.0)
        drag_force = 0.5 * air_density * (speed**2) * wing_area * drag_coeff

        # 2. Accélération et mise à jour dynamique
        forward_acc = np.clip((thrust_force - drag_force) / self.mass, -50.0, 50.0)
        self.velocity[0] += forward_acc * self.dt
        self.velocity = np.clip(self.velocity, -300.0, 300.0)
        self.airspeed = float(np.linalg.norm(self.velocity))

        # 3. Évolution des angles d'attitude (Effet des Élevons)
        self.euler[0] += (delta_elevon_left - delta_elevon_right) * 0.1 * self.dt  # Roll
        self.euler[1] += (delta_elevon_left + delta_elevon_right) * 0.05 * self.dt # Pitch
        self.euler[2] += (delta_elevon_left - delta_elevon_right) * 0.02 * self.dt # Yaw

        if delta_thrust > 0.4 and self.airspeed > 15.0:
            self.pos[2] -= (2.5 + self.euler[1] * 10.0) * self.dt

        return {
            "position": self.pos.tolist(),
            "velocity": self.velocity.tolist(),
            "euler": np.degrees(self.euler).tolist(),
            "airspeed": round(self.airspeed, 2),
            "altitude": round(abs(self.pos[2]), 2),
            "rcs_dBsm": round(self.compute_rcs(), 2),
            "battery_pct": 99.5
        }