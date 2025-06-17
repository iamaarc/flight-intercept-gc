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

class ProportionalNavigationGuidance:
    """
    Proportional Navigation (PN) guidance law.
    Computes an acceleration command based on LOS rate and pursuer velocity.
    """
    def __init__(self, nav_constant=3.0):
        self.N = nav_constant

    def compute_command(self, pursuer_pos, pursuer_vel, target_pos, target_vel):
        r_rel = np.array(target_pos) - np.array(pursuer_pos)
        v_rel = np.array(target_vel) - np.array(pursuer_vel)
        r_norm = np.linalg.norm(r_rel)

        if r_norm < 1e-6:
            return np.zeros_like(r_rel)

        los = r_rel / r_norm
        los_rate = np.cross(r_rel, v_rel) / (r_norm**2 + 1e-6)
        acc_cmd = self.N * np.cross(los_rate, pursuer_vel)
        return acc_cmd
