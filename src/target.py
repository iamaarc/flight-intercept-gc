import numpy as np

class TargetProfile:
    """Straight-line target profile."""
    def __init__(self, position, velocity, speed=1.0):
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.velocity = self.velocity / np.linalg.norm(self.velocity) * speed

    def update(self, dt):
        self.position += self.velocity * dt

class ConstantHeadingTarget:
    """Target moving with constant turn rate (circular arc)."""
    def __init__(self, position, velocity, speed=1.0, turn_rate=np.deg2rad(45)):
        self.position = np.array(position, dtype=float)
        self.heading = np.arctan2(velocity[1], velocity[0])
        self.speed = speed
        self.turn_rate = turn_rate  # radians per second

    def update(self, dt):
        self.heading += self.turn_rate * dt
        dx = self.speed * np.cos(self.heading) * dt
        dy = self.speed * np.sin(self.heading) * dt
        self.position += np.array([dx, dy])
