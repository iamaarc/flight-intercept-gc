# src/target.py

import numpy as np

class TargetProfile:
    """
    Straight-line, constant-velocity target (used for both Day 2 and Day 3).
    """
    def __init__(self, position, heading_vec, speed=1.0):
        self.position = np.array(position, dtype=float)
        self.heading_vec = np.array(heading_vec, dtype=float)
        self.heading_vec /= np.linalg.norm(self.heading_vec)
        self.speed = speed

    def update(self, dt):
        """
        Update position using current heading and speed.
        """
        self.position += self.heading_vec * self.speed * dt

    def get_state(self):
        return self.position.copy()

class ConstantHeadingTarget:
    """
    Target moving with constant turn rate (circular/arc motion).
    """
    def __init__(self, position, heading_vec, speed=1.0, turn_rate=0.3):
        self.position = np.array(position, dtype=float)
        self.heading = np.arctan2(heading_vec[1], heading_vec[0])
        self.speed = speed
        self.turn_rate = turn_rate  # radians per second

    def update(self, dt):
        """
        Update heading and move in the current heading direction.
        """
        self.heading += self.turn_rate * dt
        dx = self.speed * np.cos(self.heading) * dt
        dy = self.speed * np.sin(self.heading) * dt
        self.position += np.array([dx, dy])

    def get_state(self):
        return self.position.copy()

