class PositionController:
    """
    PD Position Controller for multi-dimensional position control (2D/3D).
    """
    def __init__(self, kp=1.0, kd=0.5, max_acc=3.0):
        self.kp = kp
        self.kd = kd
        self.max_acc = max_acc

    def compute_acceleration(self, current_pos, current_vel, desired_pos, desired_vel=None):
        """
        Compute PD acceleration command.
        Args:
            current_pos (np.array): Current position.
            current_vel (np.array): Current velocity.
            desired_pos (np.array): Target position.
            desired_vel (np.array or None): Target velocity (default: zero vector).
        Returns:
            np.array: Acceleration command, clipped to max_acc.
        """
        if desired_vel is None:
            desired_vel = np.zeros_like(current_vel)
        pos_error = np.array(desired_pos) - np.array(current_pos)
        vel_error = np.array(desired_vel) - np.array(current_vel)
        acc_cmd = self.kp * pos_error + self.kd * vel_error
        norm = np.linalg.norm(acc_cmd)
        if norm > self.max_acc:
            acc_cmd = acc_cmd / norm * self.max_acc
        return acc_cmd

