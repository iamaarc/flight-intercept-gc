# src/position_controller.py

import numpy as np

class PositionController:
    def __init__(self, kp=1.0, kd=0.5, max_acc=3.0):
        self.kp = kp  # Proportional gain
        self.kd = kd  # Derivative gain
        self.max_acc = max_acc  # Maximum acceleration (m/s^2)

    def compute_acceleration(self, current_pos, current_vel, desired_pos, desired_vel):
        # PD law: acceleration = kp * pos_error + kd * vel_error
        pos_error = np.array(desired_pos) - np.array(current_pos)
        vel_error = np.array(desired_vel) - np.array(current_vel)
        acc_cmd = self.kp * pos_error + self.kd * vel_error
        # Limit acceleration
        norm = np.linalg.norm(acc_cmd)
        if norm > self.max_acc:
            acc_cmd = acc_cmd / norm * self.max_acc
        return acc_cmd

