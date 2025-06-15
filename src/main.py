# src/main.py

from target import TargetProfile, ConstantHeadingTarget
from guidance import PurePursuitGuidance
from position_controller import PositionController
import numpy as np
import matplotlib.pyplot as plt

dt = 0.1
steps = int(20 / dt)

##############################
# Day 2: Target Motion Profiles
##############################

tgt1 = TargetProfile([0, 0], [1, 0], speed=1.0)
traj1 = [tgt1.position.copy()]
for _ in range(steps):
    tgt1.update(dt)
    traj1.append(tgt1.position.copy())
traj1 = np.array(traj1)

tgt2 = ConstantHeadingTarget([0, 0], [1, 0], speed=1.0, turn_rate=np.deg2rad(45))
traj2 = [tgt2.position.copy()]
for _ in range(steps):
    tgt2.update(dt)
    traj2.append(tgt2.position.copy())
traj2 = np.array(traj2)

plt.figure(figsize=(7,6))
plt.plot(traj1[:, 0], traj1[:, 1], label='Straight Line')
plt.plot(traj2[:, 0], traj2[:, 1], label='Constant Turn (Arc)')
plt.xlabel('X position [m]')
plt.ylabel('Y position [m]')
plt.axis('equal')
plt.legend()
plt.title('Day 2: Target Motion Profiles')
plt.tight_layout()
plt.savefig("doc/day2_target_profiles.png")
plt.show()

##############################
# Day 3: Pure Pursuit Guidance (Kinematic)
##############################

print("\n=== Day 3: Guidance (Pure Pursuit, Kinematic) ===")
target = ConstantHeadingTarget([0, 0], [1, 0], speed=1.0, turn_rate=np.deg2rad(45))
pursuer_pos = np.array([-5.0, -5.0])
pursuer_traj = [pursuer_pos.copy()]
target_traj = [target.position.copy()]
guidance = PurePursuitGuidance()

for _ in range(steps):
    target.update(dt)
    target_traj.append(target.position.copy())
    # Guidance: "chase" target directly (constant speed)
    pursuer_vel = guidance.compute_command(pursuer_pos, target.position)
    pursuer_pos += pursuer_vel * dt
    pursuer_traj.append(pursuer_pos.copy())

target_traj = np.array(target_traj)
pursuer_traj = np.array(pursuer_traj)

plt.figure(figsize=(7,6))
plt.plot(target_traj[:, 0], target_traj[:, 1], label='Target (arc)', linewidth=2)
plt.plot(pursuer_traj[:, 0], pursuer_traj[:, 1], label='Pursuer (pursuit)', linewidth=2)
plt.scatter([pursuer_traj[0,0]], [pursuer_traj[0,1]], c='green', s=80, label="Pursuer Start")
plt.scatter([target_traj[0,0]], [target_traj[0,1]], c='red', s=80, label="Target Start")
plt.xlabel('X position [m]')
plt.ylabel('Y position [m]')
plt.title('Day 3: Pure Pursuit Guidance')
plt.axis('equal')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("doc/day3_guidance_demo.png")
plt.show()

##############################
# Day 4: PD Position Controller (with Guidance Law)
##############################

print("\n=== Day 4: Outer-Loop Position Controller Demo ===")
target = ConstantHeadingTarget([0, 0], [1, 0], speed=1.0, turn_rate=np.deg2rad(45))
pursuer_pos = np.array([-5.0, -5.0])
pursuer_vel = np.zeros(2)
target_traj = [target.position.copy()]
pursuer_traj = [pursuer_pos.copy()]
guidance = PurePursuitGuidance()
pos_controller = PositionController(kp=1.2, kd=0.6, max_acc=3.0)

for _ in range(steps):
    target.update(dt)
    target_traj.append(target.position.copy())

    # Guidance: desired position is current target position
    desired_pos = target.position.copy()
    desired_vel = np.zeros(2)
    acc_cmd = pos_controller.compute_acceleration(
        current_pos=pursuer_pos,
        current_vel=pursuer_vel,
        desired_pos=desired_pos,
        desired_vel=desired_vel
    )
    pursuer_vel += acc_cmd * dt
    pursuer_pos += pursuer_vel * dt
    pursuer_traj.append(pursuer_pos.copy())

target_traj = np.array(target_traj)
pursuer_traj = np.array(pursuer_traj)

plt.figure(figsize=(7,6))
plt.plot(target_traj[:, 0], target_traj[:, 1], label='Target (arc)', linewidth=2)
plt.plot(pursuer_traj[:, 0], pursuer_traj[:, 1], label='Pursuer (PD-controlled)', linewidth=2)
plt.scatter([pursuer_traj[0,0]], [pursuer_traj[0,1]], c='green', s=80, label="Pursuer Start")
plt.scatter([target_traj[0,0]], [target_traj[0,1]], c='red', s=80, label="Target Start")
plt.xlabel('X position [m]')
plt.ylabel('Y position [m]')
plt.title('Day 4: Outer-Loop Position Control (PD)')
plt.axis('equal')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("doc/day4_position_controller_demo.png")
plt.show()

