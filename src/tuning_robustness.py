import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import time
import os
from position_controller import PositionController

# --- Target trajectory ---
def helical_target(t, radius=5, z_rate=0.2, speed=1.0):
    x = radius * np.cos(speed * t)
    y = radius * np.sin(speed * t)
    z = z_rate * t
    return np.array([x, y, z])

# --- Run single simulation ---
def run_sim(kp=2.0, kd=1.0, noise=0.0, max_acc=None, disturbance=False, N=400, dt=0.05, capture_radius=0.5):
    pursuer_pos = np.array([-7.0, -7.0, 0.0])
    pursuer_vel = np.zeros(3)
    controller = PositionController(kp, kd, max_acc=max_acc if max_acc else 100)
    traj_pursuer = [pursuer_pos.copy()]
    traj_target = []
    acc_history = []

    for i in range(N):
        t = i * dt
        target = helical_target(t)
        traj_target.append(target.copy())

        # Add noise to sensor
        pos_noise = np.random.normal(0, noise, size=3)
        current_pos = pursuer_pos + pos_noise

        # Control
        acc = controller.compute_acceleration(current_pos, pursuer_vel, target)
        if max_acc:
            acc = np.clip(acc, -max_acc, max_acc)

        # Random disturbance (spikes)
        if disturbance and np.random.rand() < 0.05:
            acc += np.random.uniform(-1, 1, size=3)

        acc_history.append(acc.copy())

        # Physics
        pursuer_vel += acc * dt
        pursuer_pos += pursuer_vel * dt
        traj_pursuer.append(pursuer_pos.copy())

    traj_pursuer = np.array(traj_pursuer)
    traj_target = np.array(traj_target)
    acc_history = np.array(acc_history)

    # Metrics
    distances = np.linalg.norm(traj_pursuer[:len(traj_target)] - traj_target, axis=1)
    min_dist = np.min(distances)
    time_to_intercept = None
    for i, d in enumerate(distances):
        if d < capture_radius:
            time_to_intercept = i * dt
            break
    settle_steps = int(len(distances) * 0.1)
    settling_time = None
    if np.all(distances[-settle_steps:] < capture_radius):
        settling_time = (len(distances) - settle_steps) * dt

    energy = np.sum(np.linalg.norm(acc_history, axis=1)**2) * dt

    return traj_pursuer, traj_target, {
        'time_to_intercept': time_to_intercept,
        'miss_distance': min_dist,
        'settling_time': settling_time,
        'energy': energy
    }

# --- Sweeps ---

results = []
outdir = "../doc" if os.path.isdir("../doc") else "./doc"
os.makedirs(outdir, exist_ok=True)

def record_and_print(label, metrics, extras=None):
    metrics_dict = dict(metrics)
    if extras:
        metrics_dict.update(extras)
    print(f"{label} -> {metrics_dict}")
    results.append(metrics_dict)

# Gain sweep
for kp in [1.0, 2.0, 4.0]:
    start = time.time()
    _, _, metrics = run_sim(kp=kp, kd=1.0)
    cpu_time = time.time() - start
    record_and_print(f"[Gain Sweep] Kp={kp}, Kd=1.0", metrics, {"type": "Gain", "Kp": kp, "Kd": 1.0, "cpu_time": cpu_time})

# Noise sweep
for noise in [0.0, 0.1, 0.3]:
    start = time.time()
    _, _, metrics = run_sim(noise=noise)
    cpu_time = time.time() - start
    record_and_print(f"[Noise Sweep] σ={noise}", metrics, {"type": "Noise", "noise": noise, "cpu_time": cpu_time})

# Actuator limit sweep
for max_acc in [1.0, 2.0, 3.0]:
    start = time.time()
    _, _, metrics = run_sim(max_acc=max_acc)
    cpu_time = time.time() - start
    record_and_print(f"[Actuator Limit] max_acc={max_acc}", metrics, {"type": "Actuator", "max_acc": max_acc, "cpu_time": cpu_time})

# Disturbance test
for disturbance in [False, True]:
    start = time.time()
    _, _, metrics = run_sim(disturbance=disturbance)
    cpu_time = time.time() - start
    record_and_print(f"[Disturbance] {disturbance}", metrics, {"type": "Disturbance", "disturbance": disturbance, "cpu_time": cpu_time})

# Save as DataFrame
df = pd.DataFrame(results)
df.to_csv(os.path.join(outdir, "tuning_robustness_metrics.csv"), index=False)

# --- Plot Bar Charts ---
def plot_bar(metric, group, x_label, y_label, fname):
    subset = df[df["type"] == group]
    if group == "Gain":
        x = subset["Kp"].astype(str)
    elif group == "Noise":
        x = subset["noise"].astype(str)
    elif group == "Actuator":
        x = subset["max_acc"].astype(str)
    elif group == "Disturbance":
        x = subset["disturbance"].astype(str)
    else:
        x = np.arange(len(subset))
    y = subset[metric].astype(float)
    plt.figure(figsize=(6,4))
    plt.bar(x, y)
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.title(f"{y_label} vs {x_label} ({group})")
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, fname))
    plt.close()

plot_bar("miss_distance", "Gain", "Kp", "Miss Distance (m)", "miss_distance_vs_gain.png")
plot_bar("energy", "Gain", "Kp", "Energy Used", "energy_vs_gain.png")
plot_bar("miss_distance", "Noise", "Noise σ", "Miss Distance (m)", "miss_distance_vs_noise.png")
plot_bar("miss_distance", "Actuator", "Max Acc", "Miss Distance (m)", "miss_distance_vs_maxacc.png")
plot_bar("energy", "Actuator", "Max Acc", "Energy Used", "energy_vs_maxacc.png")
plot_bar("miss_distance", "Disturbance", "Disturbance", "Miss Distance (m)", "miss_distance_vs_disturbance.png")

print("\nAll experiments complete. Plots saved to /doc and metrics to tuning_robustness_metrics.csv")

