from target import TargetProfile, ConstantTurnTarget
import numpy as np
import matplotlib.pyplot as plt

print("Starting simulation...")  # This should ALWAYS print

dt = 0.1
steps = int(20 / dt)

# Profile 1: straight-line
tgt1 = TargetProfile([0, 0, 0], [1.0, 0.0, 0.0])
traj1 = [tgt1.position.copy()]

# Profile 2: constant-turn
tgt2 = ConstantTurnTarget([0, 0, 0], speed=2.0, turn_rate_rad_s=0.3, heading_deg=45)
traj2 = [tgt2.position.copy()]

for _ in range(steps):
    traj1.append(tgt1.update(dt))
    traj2.append(tgt2.update(dt))

traj1 = np.array(traj1)
traj2 = np.array(traj2)

plt.plot(traj1[:, 0], traj1[:, 1], label='Straight-line')
plt.plot(traj2[:, 0], traj2[:, 1], label='Constant-turn')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend()
plt.title('Target Trajectories')
plt.axis('equal')
print("Simulation finished, showing plot...")
plt.show()
