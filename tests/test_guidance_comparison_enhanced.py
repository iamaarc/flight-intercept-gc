
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
    from position_controller import PositionController
    from guidance import PurePursuitGuidance, ProportionalNavigationGuidance
    from simulation_utils import run_sim  # Adjust path if needed

    # --- Sim config ---
    N = 400
    dt = 0.05
    noise = 0.1
    disturbance = True

    # --- Pure Pursuit run ---
    guidance_pp = PurePursuitGuidance(gain=1.0)
    pursuer_pp, target_pp, metrics_pp = run_sim(guidance_pp, N=N, dt=dt, noise=noise, disturbance=disturbance)

    # --- Proportional Navigation run ---
    guidance_pn = ProportionalNavigationGuidance(nav_constant=3.0)
    pursuer_pn, target_pn, metrics_pn = run_sim(guidance_pn, N=N, dt=dt, noise=noise, disturbance=disturbance)

    # --- Extract metrics ---
    miss_pp = metrics_pp["miss_distance"]
    miss_pn = metrics_pn["miss_distance"]
    tti_pp = metrics_pp["time_to_intercept"]
    tti_pn = metrics_pn["time_to_intercept"]
    energy_pp = metrics_pp["energy"]
    energy_pn = metrics_pn["energy"]

    # --- Save CSV ---
    metrics = {
    "miss_distance_pp": miss_pp,
    "miss_distance_pn": miss_pn,
    "time_to_intercept_pp": tti_pp,
    "time_to_intercept_pn": tti_pn,
    "energy_pp": energy_pp,
    "energy_pn": energy_pn
    }
    df = pd.DataFrame([metrics])
    os.makedirs("doc", exist_ok=True)
    df.to_csv("doc/guidance_comparison_metrics.csv", index=False)

    print("✅ Comparison complete. Plot and metrics saved to /doc.")

# Add src/ to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../src")))

from position_controller import PositionController
from guidance import PurePursuitGuidance, ProportionalNavigationGuidance

# --- Helical target ---
def helical_target(t, radius=5, z_rate=0.2, speed=1.0):
    x = radius * np.cos(speed * t)
    y = radius * np.sin(speed * t)
    z = z_rate * t
    return np.array([x, y, z])

# --- Simulator ---
def simulate(guidance_type, noise=0.0, disturbance=False, N=400, dt=0.05):
    pursuer_pos = np.array([-7.0, -7.0, 0.0])
    pursuer_vel = np.zeros(3)
    traj_pursuer, traj_target, acc_hist = [], [], []
    
    controller = PositionController(kp=2.0, kd=1.0)
    guidance = PurePursuitGuidance() if guidance_type == "pp" else ProportionalNavigationGuidance()

    for i in range(N):
        t = i * dt
        target = helical_target(t)
        traj_target.append(target.copy())

        pos_noise = np.random.normal(0, noise, size=3)
        current_pos = pursuer_pos + pos_noise
        current_vel = pursuer_vel

        if guidance_type == "pp":
            desired_vel = guidance.compute_command(current_pos, target)
            acc = controller.compute_acceleration(current_pos, current_vel, current_pos + desired_vel)
        elif guidance_type == "pn":
            target_vel = np.array([-np.sin(t), np.cos(t), 0.2])  # Approx helix tangent
            acc = guidance.compute_command(current_pos, current_vel, target, target_vel)
        else:
            raise ValueError("Unknown guidance type")

        if disturbance and np.random.rand() < 0.05:
            acc += np.random.uniform(-1, 1, size=3)

        acc_hist.append(acc.copy())

        pursuer_vel += acc * dt
        pursuer_pos += pursuer_vel * dt
        traj_pursuer.append(pursuer_pos.copy())

    traj_pursuer = np.array(traj_pursuer)
    traj_target = np.array(traj_target)
    acc_hist = np.array(acc_hist)

    distances = np.linalg.norm(traj_pursuer[:len(traj_target)] - traj_target, axis=1)
    miss_distance = distances[-1]
    time_to_intercept = next((i * dt for i, d in enumerate(distances) if d < 1.0), None)
    energy = np.sum(np.linalg.norm(acc_hist, axis=1)**2) * dt

    return traj_pursuer, traj_target, {
        "miss_distance": float(miss_distance),
        "time_to_intercept": float(time_to_intercept) if time_to_intercept else None,
        "energy": float(energy)
    }

# --- Run comparisons ---
results = []
for method in ["pp", "pn"]:
    traj_p, traj_t, metrics = simulate(method)
    print(f"[{method.upper()}] -> {metrics}")
    results.append({
        "method": method,
        "miss_distance": metrics["miss_distance"],
        "time_to_intercept": metrics["time_to_intercept"],
        "energy": metrics["energy"]
    })

# --- Save CSV ---
df = pd.DataFrame(results)
os.makedirs("doc", exist_ok=True)
df.to_csv("doc/guidance_comparison_metrics.csv", index=False)
print("✅ Saved metrics to doc/guidance_comparison_metrics.csv")
