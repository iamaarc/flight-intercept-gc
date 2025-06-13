# src/target.py

import numpy as np

class TargetProfile:
    def __init__(self, start_pos, start_vel):
        self.position = np.array(start_pos, dtype=float)
        self.velocity = np.array(start_vel, dtype=float)
        self.time = 0.0

    def update(self, dt):
        """Straight-line, constant velocity."""
        self.position += self.velocity * dt
        self.time += dt
        return self.position.copy()

class ConstantTurnTarget(TargetProfile):
    def __init__(self, start_pos, speed, turn_rate_rad_s, heading_deg=0):
        heading = np.deg2rad(heading_deg)
        velocity = [speed * np.cos(heading), speed * np.sin(heading), 0]
        super().__init__(start_pos, velocity)
        self.speed = speed
        self.turn_rate = turn_rate_rad_s  # rad/s
        self.heading = heading

    def update(self, dt):
        self.heading += self.turn_rate * dt
        self.velocity = np.array([
            self.speed * np.cos(self.heading),
            self.speed * np.sin(self.heading),
            0
        ])
        self.position += self.velocity * dt
        self.time += dt
        return self.position.copy()
