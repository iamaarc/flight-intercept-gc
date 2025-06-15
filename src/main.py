# src/main.py
# src/full_demo.py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from attitude_controller import AttitudeController3D

# ========== Utility: 3D Helical Target ==========
def helical_target(t, radius=5, z_rate=0.2, speed=1.0):
    x = radius * np.cos(speed * t)
    y = radius * np.sin(speed * t)
    z = z_rate * t
    return np.array([x, y, z])

# ========== Utility: Map desired acceleration to attitude ==========
def compute_desired_attitude(acc_des, g=9.81):
    ax, ay, az = acc_des
    thrust = np.sqrt(ax**2 + ay**2 + (az + g)**2)
    pitch = np.arctan2(ax, az + g)
    roll  = -np.arctan2(ay, az + g)
    yaw   = np.arctan2(ay, ax)
    return roll, pitch, yaw

# ========== Day 2: Target Trajectory Only (3D) ==========
dt = 0.05
N = 400

traj_target = []
for i in range(N):
    t = i * dt
    target = helical_target(t)
    traj_target.append(target)
traj_target = np.array(traj_target)

fig = plt.figure(figsize=(7,6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(traj_target[:,0], traj_target[:,1], traj_target[:,2], label='Target (helix)')
ax.scatter(traj_target[0,0], traj_target[0,1], traj_target[0,2], c='r', label='Start', s=40)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
ax.set_title("Day 2: 3D Target Trajectory")
plt.tight_layout()
plt.savefig("doc/day2_target_trajectory.png")
plt.show()

# ========== Day 3: Pure Pursuit (Kinematic, 3D) ==========
pursuer_pos = np.array([-7.0, -7.0, 0.0])
traj_pursuer = [pursuer_pos.copy()]
traj_target = []

for i in range(N):
    t = i * dt
    target = helical_target(t)
    traj_target.append(target.copy())
    # Pure pursuit: move directly toward target at constant speed
    direction = target - pursuer_pos
    if np.linalg.norm(direction) > 1e-3:
        direction = direction / np.linalg.norm(direction)
    speed = 1.5
    pursuer_pos += direction * speed * dt
    traj_pursuer.append(pursuer_pos.copy())

traj_target = np.array(traj_target)
traj_pursuer = np.array(traj_pursuer)

fig = plt.figure(figsize=(7,6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(traj_target[:,0], traj_target[:,1], traj_target[:,2], label='Target (helix)')
ax.plot(traj_pursuer[:,0], traj_pursuer[:,1], traj_pursuer[:,2], label='Pursuer')
ax.scatter(traj_pursuer[0,0], traj_pursuer[0,1], traj_pursuer[0,2], c='g', label='Pursuer Start', s=40)
ax.scatter(traj_target[0,0], traj_target[0,1], traj_target[0,2], c='r', label='Target Start', s=40)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
ax.set_title("Day 3: 3D Pure Pursuit (Kinematic)")
plt.tight_layout()
plt.savefig("doc/day3_pure_pursuit_3d.png")
plt.show()

# ========== Day 4: Outer & Inner Loop Control (Position & Attitude) ==========

# ====== Target Trajectory (Helix) ======
def helical_target(t, radius=5, z_rate=0.2, speed=1.0):
    x = radius * np.cos(speed * t)
    y = radius * np.sin(speed * t)
    z = z_rate * t
    return np.array([x, y, z])

# ====== Map desired acceleration to attitude (roll, pitch, yaw) ======
def compute_desired_attitude(acc_des, g=9.81):
    ax, ay, az = acc_des
    pitch = np.arctan2(ax, az + g)
    roll  = -np.arctan2(ay, az + g)
    yaw   = np.arctan2(ay, ax)
    return roll, pitch, yaw

# ====== Attitude Controller Gains ======
gains = {
    'roll':  (0.2, 0.01, 0.01, np.deg2rad(10)),
    'pitch': (0.2, 0.01, 0.01, np.deg2rad(10)),
    'yaw':   (0.1, 0.00, 0.01, np.deg2rad(20))
}
att_ctrl = AttitudeController3D(gains)

# ====== Simulation Parameters ======
dt = 0.05
N = 400
max_thrust = 3.0  # [m/s^2], max allowed acceleration (lowered for stability)
max_vel = 3.0     # [m/s], max allowed speed

# ====== Initial states ======
pursuer_pos = np.array([-7.0, -7.0, 0.0])
pursuer_vel = np.zeros(3)
pursuer_att = np.zeros(3)  # roll, pitch, yaw

traj_pursuer = [pursuer_pos.copy()]
traj_target = []

# ====== Main Simulation Loop ======
for i in range(N):
    t = i * dt
    target = helical_target(t)
    traj_target.append(target.copy())

    # Outer loop PD control for position
    pos_error = target - pursuer_pos
    des_vel = pos_error * 0.6     # Proportional velocity command
    des_vel = np.clip(des_vel, -max_vel, max_vel)
    vel_error = des_vel - pursuer_vel
    acc_cmd = vel_error * 0.8     # Proportional acceleration command

    # Limit commanded acceleration (thrust)
    acc_cmd = np.clip(acc_cmd, -max_thrust, max_thrust)

    # Map desired acc to attitude (ignore full 6DOF for simplicity)
    att_des = compute_desired_attitude(acc_cmd)
    att_cmd = att_ctrl.compute(att_des, pursuer_att, dt)

    # For this demo, simply apply acc_cmd as thrust in world frame
    pursuer_vel += acc_cmd * dt
    pursuer_vel = np.clip(pursuer_vel, -max_vel, max_vel)
    pursuer_pos += pursuer_vel * dt

    traj_pursuer.append(pursuer_pos.copy())
    pursuer_att += att_cmd * dt

    # Print debug info every 50 steps
    if i % 50 == 0:
        print(f"Step {i}: pos={pursuer_pos}, vel={pursuer_vel}, acc={acc_cmd}")

traj_target = np.array(traj_target)
traj_pursuer = np.array(traj_pursuer)

# ====== Plot ======
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(traj_target[:,0], traj_target[:,1], traj_target[:,2], label='Target (helix)', color='dodgerblue')
ax.plot(traj_pursuer[:,0], traj_pursuer[:,1], traj_pursuer[:,2], label='Pursuer (Controlled)', color='orange')
ax.scatter(traj_pursuer[0,0], traj_pursuer[0,1], traj_pursuer[0,2], c='g', label='Pursuer Start', s=50)
ax.scatter(traj_target[0,0], traj_target[0,1], traj_target[0,2], c='r', label='Target Start', s=50)
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.legend()
ax.set_title("Day 4: 3D Pursuit with Position + Attitude Control")
plt.tight_layout()
plt.savefig("doc/day4_position_attitude_control_demo.png")
plt.show()

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
import numpy as np

# Assuming traj_target and traj_pursuer are already created as Nx3 arrays

fig = plt.figure(figsize=(8, 7))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(np.min(traj_target[:,0])-1, np.max(traj_target[:,0])+1)
ax.set_ylim(np.min(traj_target[:,1])-1, np.max(traj_target[:,1])+1)
ax.set_zlim(np.min(traj_target[:,2])-1, np.max(traj_target[:,2])+1)
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.set_title("Day 4: 3D Pursuit with Position + Attitude Control")

# Plot start points
pursuer_start = ax.scatter(traj_pursuer[0,0], traj_pursuer[0,1], traj_pursuer[0,2], c='g', s=50, label='Pursuer Start')
target_start = ax.scatter(traj_target[0,0], traj_target[0,1], traj_target[0,2], c='r', s=50, label='Target Start')

# Initial lines
target_line, = ax.plot([], [], [], 'b', lw=2, label='Target (helix)')
pursuer_line, = ax.plot([], [], [], 'orange', lw=2, label='Pursuer (Controlled)')

ax.legend()

def init():
    target_line.set_data([], [])
    target_line.set_3d_properties([])
    pursuer_line.set_data([], [])
    pursuer_line.set_3d_properties([])
    return target_line, pursuer_line

def animate(i):
    target_line.set_data(traj_target[:i+1,0], traj_target[:i+1,1])
    target_line.set_3d_properties(traj_target[:i+1,2])
    pursuer_line.set_data(traj_pursuer[:i+1,0], traj_pursuer[:i+1,1])
    pursuer_line.set_3d_properties(traj_pursuer[:i+1,2])
    return target_line, pursuer_line

anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=len(traj_target), interval=30, blit=True)

# --- To Save as GIF ---
anim.save('doc/day4_pursuit_animation.gif', writer='pillow', fps=30)

plt.show()

