
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import argparse
import os
import sys
import pandas as pd

# Add src/ to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

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
        guidance = PurePursuitGuidance(gain=1.0)
    elif guidance_type == "pn":
        guidance = ProportionalNavigationGuidance(nav_constant=3.0)
    else:
        raise ValueError("Unknown guidance type")

    for i in range(N):
        t = i * dt
        target_pos = helical_target(t)
        traj_target.append(target_pos.copy())

        pos_noise = np.random.normal(0, noise, size=3)
        sensed_pos = pursuer_pos + pos_noise

        if guidance_type == "pp":
            cmd = guidance.compute_command(sensed_pos, target_pos)
            acc = PositionController(2.0, 1.0).compute_acceleration(sensed_pos, pursuer_vel, sensed_pos + cmd)
        else:
            target_vel = np.array([-np.sin(t), np.cos(t), 0.2])
            cmd = guidance.compute_command(sensed_pos, pursuer_vel, target_pos, target_vel)
            acc = cmd

        if disturbance and np.random.rand() < 0.05:
            acc += np.random.uniform(-1, 1, size=3)

        pursuer_vel += acc * dt
        pursuer_pos += pursuer_vel * dt
        traj_pursuer.append(pursuer_pos.copy())

    return np.array(traj_pursuer), np.array(traj_target)

def plot_and_animate(traj_pp, traj_pn, traj_target, out_path):
    fig = plt.figure(figsize=(10, 5))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(traj_target[:, 0], traj_target[:, 1], traj_target[:, 2], 'k--', label="Target")
    ax.plot(traj_pp[:, 0], traj_pp[:, 1], traj_pp[:, 2], 'b', label="pure_pursuit")
    ax.plot(traj_pn[:, 0], traj_pn[:, 1], traj_pn[:, 2], 'r', label="proportional_navigation")

    ax.set_title("3D Trajectories - Guidance Comparison")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    plt.tight_layout()
    plt.savefig(out_path.replace(".gif", ".png"))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--noise", type=float, default=0.0, help="Sensor noise stddev")
    parser.add_argument("--disturbance", action="store_true", help="Enable random disturbance")
    args = parser.parse_args()

    traj_pp, traj_target = simulate("pp", noise=args.noise, disturbance=args.disturbance)
    traj_pn, _ = simulate("pn", noise=args.noise, disturbance=args.disturbance)

    out_path = "doc/guidance_comparison.gif"
    plot_and_animate(traj_pp, traj_pn, traj_target, out_path)

    # Save summary metrics
    metrics = {
        "miss_distance_pp": float(np.linalg.norm(traj_pp[-1] - traj_target[-1])),
        "miss_distance_pn": float(np.linalg.norm(traj_pn[-1] - traj_target[-1])),
    }
    df = pd.DataFrame([metrics])
    os.makedirs("doc", exist_ok=True)
    df.to_csv("doc/guidance_comparison_metrics.csv", index=False)

    print("Comparison complete. Plot and metrics saved to /doc.")

if __name__ == "__main__":
    main()
