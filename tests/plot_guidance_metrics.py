import pandas as pd
import matplotlib.pyplot as plt
import os

# Load CSV
df = pd.read_csv("doc/guidance_comparison_metrics.csv")

# Ensure output directory exists
os.makedirs("doc", exist_ok=True)

# Define each metric to plot
metrics = {
    "miss_distance": {
        "pp": "miss_distance_pp",
        "pn": "miss_distance_pn",
        "ylabel": "Miss Distance (m)"
    },
    "time_to_intercept": {
        "pp": "time_to_intercept_pp",
        "pn": "time_to_intercept_pn",
        "ylabel": "Time to Intercept (s)"
    },
    "energy": {
        "pp": "energy_pp",
        "pn": "energy_pn",
        "ylabel": "Energy Used"
    }
}

# Plot loop
for metric, cols in metrics.items():
    if cols["pp"] in df.columns and cols["pn"] in df.columns:
        values = [df[cols["pp"]][0], df[cols["pn"]][0]]
        labels = ["Pure Pursuit", "Proportional Navigation"]

        plt.figure(figsize=(6, 4))
        plt.bar(labels, values, color=["skyblue", "salmon"])
        plt.ylabel(cols["ylabel"])
        plt.title(f"{cols['ylabel']} Comparison")
        plt.tight_layout()
        out_path = f"doc/guidance_{metric}_comparison.png"
        plt.savefig(out_path)
        plt.close()
        print(f"[✓] Saved {out_path}")
    else:
        print(f"[!] Skipping '{metric}' — missing expected columns.")
