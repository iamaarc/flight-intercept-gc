import numpy as np

class AttitudeController3D:
    """
    PID controller for 3D attitude (roll, pitch, yaw).
    Each axis has its own PID gains and output limits (radians/sec).
    """
    def __init__(self, gains):
        self.gains = gains
        self.integral = np.zeros(3)
        self.prev_error = np.zeros(3)

    def compute(self, att_des, att, dt):
        u = np.zeros(3)
        for i, axis in enumerate(['roll', 'pitch', 'yaw']):
            kp, ki, kd, lim = self.gains[axis]
            error = att_des[i] - att[i]
            self.integral[i] += error * dt
            d_error = (error - self.prev_error[i]) / dt if dt > 0 else 0.0
            out = kp * error + ki * self.integral[i] + kd * d_error
            out = np.clip(out, -lim, lim)
            u[i] = out
            self.prev_error[i] = error
        return u

