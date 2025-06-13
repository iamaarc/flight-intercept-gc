from target import TargetProfile, ConstantTurnTarget
import matplotlib.pyplot as plt

# Example usage:
dt = 0.1
N = 200

# Straight-line target
tgt1 = TargetProfile([0, 0, 0], [5, 0, 0])
traj1 = [tgt1.position.copy()]
for _ in range(N):
    traj1.append(tgt1.update(dt))
traj1 = np.array(traj1)

# Constant-turn target
tgt2 = ConstantTurnTarget([0, 0, 0], speed=5, turn_rate_rad_s=0.2, heading_deg=0)
traj2 = [tgt2.position.copy()]
for _ in range(N):
    traj2.append(tgt2.update(dt))
traj2 = np.array(traj2)

plt.plot(traj1[:, 0], traj1[:, 1], label='Straight-line')
plt.plot(traj2[:, 0], traj2[:, 1], label='Constant-turn')
plt.axis('equal')
plt.legend()
plt.title("Target Motion Profiles")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.show()
