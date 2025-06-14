# src/main.py

from target import TargetProfile, ConstantHeadingTarget
import numpy as np
import matplotlib.pyplot as plt

print("Starting simulation for Day 2: Target Motion Profiles")

dt = 0.1
steps = int(20 / dt)

# --- Profile 1: Straight line ---
tgt1 = TargetProfile([0, 0], [1, 0], speed=1.0)  # Start at (0,0), heading right, 1 m/s
traj1 = [tgt1.position.copy()]
for _ in range(steps):
    tgt1.update(dt)
    traj1.append(tgt1.position.copy())
traj1 = np.array(traj1)

# --- Profile 2: Constant turn rate (circle/arc) ---
tgt2 = ConstantHeadingTarget([0, 0], [1, 0], speed=1.0, turn_rate=np.deg2rad(45))  # 45 deg/s
traj2 = [tgt2.position.copy()]
for _ in range(steps):
    tgt2.update(dt)
    traj2.append(tgt2.position.copy())
traj2 = np.array(traj2)

# --- Plot both profiles ---
plt.plot(traj1[:, 0], traj1[:, 1], label='Straight Line')
plt.plot(traj2[:, 0], traj2[:, 1], label='Constant Turn (Arc)')
plt.xlabel('X position [m]')
plt.ylabel('Y position [m]')
plt.axis('equal')
plt.legend()
plt.title('Target Trajectories')
plt.show()
