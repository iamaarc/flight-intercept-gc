
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import argparse
import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))
from position_controller import PositionController
from guidance import PurePursuitGuidance, ProportionalNavigationGuidance

def helical_target(t, radius=5, z_rate=0.2, speed=1.0):
    x = radius * np.cos(speed * t)
    y = radius * np.sin(speed * t)
    z = z_rate * t
    return np.array([x, y, z])

def simulate(guidance_type, noise=0.0, disturbance=False, N=400, dt=0.05):
    pursuer_pos = np.array([-7.0, -7.0, 0.0])
    pursuer_vel = np.zeros(3)
    traj_pursuer = []
    traj_target = []

    if guidance_type == "pp":
        guidance = PurePursuitGuidance()
    elif guidance_type == "pn":
        guidance = ProportionalNavigationGuidance()
    else:
        raise ValueError("Invalid guidance type")

    for i in range(N):
        t = i * dt
        target = helical_target(t)
        traj_target.append(target.copy())

        pos_noise = np.random.normal(0, noise, size=3)
        current_pos = pursuer_pos + pos_noise

        if guidance_type == "pp":
            vel_cmd = guidance.compute_command(current_pos, target)
            acc = vel_cmd - pursuer_vel
        elif guidance_type == "pn":
            target_vel = np.array([-np.sin(t), np.cos(t), 0.2])
            acc = guidance.compute_command(current_pos, pursuer_vel, target, target_vel)

        if disturbance and np.random.rand() < 0.05:
            acc += np.random.uniform(-1, 1, size=3)

        pursuer_vel += acc * dt
        pursuer_pos += pursuer_vel * dt
        traj_pursuer.append(pursuer_pos.copy())

    return np.array(traj_pursuer), np.array(traj_target)

def animate_3d(pursuer, target, out_file):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Guidance Animation")
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(0, 6)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    pursuer_line, = ax.plot([], [], [], 'b-', label='Pursuer')
    target_line, = ax.plot([], [], [], 'k--', label='Target')
    pursuer_dot, = ax.plot([], [], [], 'bo')
    target_dot, = ax.plot([], [], [], 'ko')

    def init():
        pursuer_line.set_data([], [])
        pursuer_line.set_3d_properties([])
        return pursuer_line, target_line, pursuer_dot, target_dot

    def update(frame):
        pursuer_line.set_data(pursuer[:frame, 0], pursuer[:frame, 1])
        pursuer_line.set_3d_properties(pursuer[:frame, 2])
        target_line.set_data(target[:frame, 0], target[:frame, 1])
        target_line.set_3d_properties(target[:frame, 2])
        pursuer_dot.set_data([pursuer[frame, 0]], [pursuer[frame, 1]])
        pursuer_dot.set_3d_properties([pursuer[frame, 2]])
        target_dot.set_data([target[frame, 0]], [target[frame, 1]])
        target_dot.set_3d_properties([target[frame, 2]])
        return pursuer_line, target_line, pursuer_dot, target_dot

    ani = animation.FuncAnimation(fig, update, frames=len(pursuer),
                                  init_func=init, blit=True, interval=40)

    ani.save(out_file, writer='pillow')
    plt.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--guidance", choices=["pp", "pn"], default="pp", help="Guidance law: pp or pn")
    parser.add_argument("--noise", type=float, default=0.0, help="Sensor noise stddev")
    parser.add_argument("--disturbance", action="store_true", help="Add random disturbance")
    args = parser.parse_args()

    pursuer, target = simulate(args.guidance, args.noise, args.disturbance)
    os.makedirs("doc", exist_ok=True)
    out_path = f"doc/guidance_comparison_{args.guidance}.gif"
    animate_3d(pursuer, target, out_path)
    print(f"Animation saved to {out_path}")
