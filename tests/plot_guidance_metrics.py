import pandas as pd
import matplotlib.pyplot as plt
import os

# Load CSV
df = pd.read_csv("doc/guidance_comparison_metrics.csv")

# Ensure output directory exists
os.makedirs("doc", exist_ok=True)

# Metrics to plot
metrics = {
    "miss_distance": "Miss Distance (m)",
    "time_to_intercept": "Time to Intercept (s)",
    "energy": "Energy Used"
}

# Plot each metric
for key, label in metrics.items():
    if key not in df.columns:
        print(f"[!] Skipping '{key}' — missing in CSV.")
        continue

    values = df[key].values
    methods = df["method"].values

    plt.figure(figsize=(6, 4))
    plt.bar(methods, values, color=["skyblue", "salmon"])
    plt.ylabel(label)
    plt.title(f"{label} Comparison")
    plt.tight_layout()
    out_path = f"doc/guidance_{key}_comparison.png"
    plt.savefig(out_path)
    plt.close()
    print(f"[✓] Saved {out_path}")


