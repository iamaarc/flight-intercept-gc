import sys
import os
sys.path.append(os.path.abspath("src"))

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from guidance import PurePursuitGuidance, ProportionalNavigationGuidance
from position_controller import PositionController

def helical_target(t, radius=5, z_rate=0.2, speed=1.0):
    x = radius * np.cos(speed * t)
    y = radius * np.sin(speed * t)
    z = z_rate * t
    return np.array([x, y, z])

def run_guidance_sim(guidance, noise=0.1, disturbance=True, dt=0.05, N=400, capture_radius=0.5):
    pursuer_pos = np.array([-7.0, -7.0, 0.0])
    pursuer_vel = np.zeros(3)
    traj_pursuer = []
    traj_target = []
    acc_history = []

    for i in range(N):
        t = i * dt
        target = helical_target(t)
        traj_target.append(target)

        noisy_pos = pursuer_pos + np.random.normal(0, noise, 3)
        noisy_vel = pursuer_vel + np.random.normal(0, noise * 0.1, 3)

        if isinstance(guidance, PurePursuitGuidance):
            acc = guidance.compute_command(noisy_pos, target)
        else:
            target_vel = np.array(helical_target(t + dt)) - np.array(target)
            acc = guidance.compute_command(noisy_pos, noisy_vel, target, target_vel)

        if disturbance and np.random.rand() < 0.05:
            acc += np.random.uniform(-1, 1, 3)

        pursuer_vel += acc * dt
        pursuer_pos += pursuer_vel * dt
        traj_pursuer.append(pursuer_pos.copy())
        acc_history.append(acc)

    traj_pursuer = np.array(traj_pursuer)
    traj_target = np.array(traj_target)
    acc_history = np.array(acc_history)

    distances = np.linalg.norm(traj_pursuer[:len(traj_target)] - traj_target, axis=1)
    min_dist = np.min(distances)
    energy = np.sum(np.linalg.norm(acc_history, axis=1)**2) * dt

    time_to_intercept = None
    for i, d in enumerate(distances):
        if d < capture_radius:
            time_to_intercept = i * dt
            break

    return {
        "miss_distance": min_dist,
        "time_to_intercept": time_to_intercept,
        "energy": energy
    }

def monte_carlo_run(label, guidance, runs=100):
    print(f"Running Monte Carlo for {label} ({runs} runs)...")
    results = []
    for i in range(runs):
        metrics = run_guidance_sim(guidance)
        results.append(metrics)
    df = pd.DataFrame(results)
    df["method"] = label
    return df

def main():
    pp = PurePursuitGuidance(gain=1.0)
    pn = ProportionalNavigationGuidance(nav_constant=3.0)

    df_pp = monte_carlo_run("Pure Pursuit", pp)
    df_pn = monte_carlo_run("Proportional Navigation", pn)

    full_df = pd.concat([df_pp, df_pn], ignore_index=True)
    os.makedirs("doc", exist_ok=True)
    full_df.to_csv("doc/monte_carlo_results.csv", index=False)
    print("✅ Saved doc/monte_carlo_results.csv")

    # --- Boxplots ---
    for metric in ["miss_distance", "time_to_intercept", "energy"]:
        plt.figure(figsize=(6, 4))
        full_df.boxplot(column=metric, by="method")
        plt.title(f"{metric.replace('_', ' ').title()} by Guidance Method")
        plt.suptitle("")
        plt.ylabel(metric.replace('_', ' ').title())
        out_path = f"doc/monte_carlo_{metric}_boxplot.png"
        plt.savefig(out_path)
        print(f"[✓] Saved {out_path}")
        plt.close()

    # --- Failure count (no intercept) ---
    failures = full_df[full_df["time_to_intercept"].isna()]
    fail_counts = failures["method"].value_counts()
    plt.figure(figsize=(5, 4))
    fail_counts.plot(kind="bar", color="red")
    plt.title("Failure Count (No Intercept)")
    plt.ylabel("Count")
    plt.xticks(rotation=0)
    out_path = "doc/monte_carlo_failure_count.png"
    plt.tight_layout()
    plt.savefig(out_path)
    print(f"[✓] Saved {out_path}")

if __name__ == "__main__":
    main()
