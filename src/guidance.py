# src/guidance.py

import numpy as np

class PurePursuitGuidance:
    """
    Simple pure pursuit guidance law: given pursuer and target positions,
    returns velocity vector pointing directly at the target.
    """
    def __init__(self, gain=1.0):
        self.gain = gain

    def compute_command(self, pursuer_pos, target_pos):
        # Direction vector from pursuer to target
        vec = np.array(target_pos) - np.array(pursuer_pos)
        dist = np.linalg.norm(vec)
        if dist < 1e-6:
            return np.zeros_like(vec)
        direction = vec / dist
        # Output is desired velocity vector (can scale by gain)
        return self.gain * direction

