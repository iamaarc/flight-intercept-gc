# src/run.py
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import os
import pandas as pd

from position_controller import PositionController
from guidance import PurePursuitGuidance, ProportionalNavigationGuidance

# --- Target Trajectory (3D helix) ---
def helical_target(t, radius=5, z_rate=0.2, speed=1.0):
    x = radius * np.cos(speed * t)
    y = radius * np.sin(speed * t)
    z = z_rate * t
    return np.array([x, y, z])

# --- Simulation Function ---
def run_sim(guidance_type="pure_pursuit", noise=0.0, disturbance=False, N=400, dt=0.05, capture_radius=0.5):
    pursuer_pos = np.array([-7.0, -7.0, 0.0])
    pursuer_vel = np.zeros(3)
    traj_pursuer = [pursuer_pos.copy()]
    traj_target = []
    acc_history = []

    # Guidance Law
    if guidance_type == "pure_pursuit":
        guidance = PurePursuitGuidance(gain=1.0)
    elif guidance_type == "proportional_navigation":
        guidance = ProportionalNavigationGuidance(nav_constant=3.0)
    else:
        raise ValueError("Unknown guidance type")

    controller = PositionController(kp=2.0, kd=1.0)

    for i in range(N):
        t = i * dt
        target = helical_target(t)
        traj_target.append(target.copy())

        # Sensor noise
        noisy_pos = pursuer_pos + np.random.normal(0, noise, size=3)

        # Guidance command
        if isinstance(guidance, PurePursuitGuidance):
            desired_dir = guidance.compute_command(noisy_pos, target)
        elif isinstance(guidance, ProportionalNavigationGuidance):
            target_vel = np.array(helical_target(t + dt)) - np.array(target)
            desired_dir = guidance.compute_command(noisy_pos, pursuer_vel, target, target_vel)

        # Convert to acceleration
        acc = controller.compute_acceleration(noisy_pos, pursuer_vel, pursuer_pos + desired_dir)

        if disturbance and np.random.rand() < 0.05:
            acc += np.random.uniform(-1, 1, size=3)

        acc_history.append(acc.copy())

        pursuer_vel += acc * dt
        pursuer_pos += pursuer_vel * dt
        traj_pursuer.append(pursuer_pos.copy())

    # Metrics
    traj_pursuer = np.array(traj_pursuer)
    traj_target = np.array(traj_target)
    distances = np.linalg.norm(traj_pursuer[:len(traj_target)] - traj_target, axis=1)
    min_dist = np.min(distances)

    time_to_intercept = None
    for i, d in enumerate(distances):
        if d < capture_radius:
            time_to_intercept = i * dt
            break

    settling_time = None
    settle_window = int(0.1 * len(distances))
    if np.all(distances[-settle_window:] < capture_radius):
        settling_time = (len(distances) - settle_window) * dt

    energy = np.sum(np.linalg.norm(acc_history, axis=1)**2) * dt

    return traj_pursuer, traj_target, {
        "guidance": guidance_type,
        "miss_distance": min_dist,
        "time_to_intercept": time_to_intercept,
        "settling_time": settling_time,
        "energy": energy,
    }

# --- Main Comparison ---
if __name__ == "__main__":
    os.makedirs("doc", exist_ok=True)
    runs = []
    colors = {"pure_pursuit": "blue", "proportional_navigation": "red"}

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for guidance_type in ["pure_pursuit", "proportional_navigation"]:
        start = time.time()
        traj_pursuer, traj_target, metrics = run_sim(
            guidance_type=guidance_type,
            noise=0.1,
            disturbance=True,
            N=400,
            dt=0.05
        )
        metrics["cpu_time"] = time.time() - start
        runs.append(metrics)

        ax.plot(traj_pursuer[:, 0], traj_pursuer[:, 1], traj_pursuer[:, 2],
                label=guidance_type, color=colors[guidance_type], linestyle='-')

    # Plot target
    traj_target = np.array([helical_target(i * 0.05) for i in range(400)])
    ax.plot(traj_target[:, 0], traj_target[:, 1], traj_target[:, 2], 'k--', label="Target")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3D Trajectories - Guidance Comparison")
    ax.legend()
    plt.tight_layout()
    plt.savefig("doc/guidance_comparison_3d.png")
    plt.show()

    # Save metrics
    df = pd.DataFrame(runs)
    df.to_csv("doc/guidance_comparison_metrics.csv", index=False)
    print("\n✅ Comparison complete. Results saved to:")
    print(" - 📊 doc/guidance_comparison_metrics.csv")
    print(" - 🌀 doc/guidance_comparison_3d.png")
