# src/main.py
from guidance import PurePursuitGuidance
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

# ========== DAY 3: Simple Guidance & Pursuer ==========
print("=== Day 3: Guidance (Pursuer) ===")

target = ConstantHeadingTarget([0, 0], [1, 0], speed=1.0, turn_rate=np.deg2rad(45))
pursuer_pos = np.array([-5.0, -5.0])
pursuer_traj = [pursuer_pos.copy()]
target_traj = [target.position.copy()]

guidance = PurePursuitGuidance(gain=1.2)

for _ in range(steps):
    target.update(dt)
    target_traj.append(target.position.copy())
    pursuer_vel = guidance.compute_command(pursuer_pos, target.position)
    pursuer_pos += pursuer_vel * dt
    pursuer_traj.append(pursuer_pos.copy())

target_traj = np.array(target_traj)
pursuer_traj = np.array(pursuer_traj)

plt.figure()
plt.plot(target_traj[:, 0], target_traj[:, 1], label="Target")
plt.plot(pursuer_traj[:, 0], pursuer_traj[:, 1], label="Pursuer")
plt.xlabel('X position [m]')
plt.ylabel('Y position [m]')
plt.legend()
plt.title('Day 3: Pursuer Guidance')
plt.savefig("doc/day3_guidance_demo.png")  # Save the plot to file
plt.show()

