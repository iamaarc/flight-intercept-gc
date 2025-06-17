# ✈️ Flight-Intercept Guidance & Control – System Sandbox

This repository contains a modular Python simulation for evaluating and comparing guidance and control strategies for a pursuit/intercept scenario. It includes support for:

- Multiple guidance laws (Pure Pursuit, Proportional Navigation)
- Outer-loop and simplified inner-loop controllers
- Monte Carlo experiments with noise/disturbance
- Metric logging (miss distance, energy, time to intercept)
- Visualizations (plots, 3D animations, GIFs)
- Clean CLI workflows for reproducibility

---

## 📁 Repository Structure

| Path                          | Description                                              |
|-------------------------------|----------------------------------------------------------|
| `src/`                        | Core simulation, control, and guidance modules           |
| `tests/`                      | Scripts for evaluation, animation, Monte Carlo runs      |
| `doc/`                        | All generated plots, GIFs, and visual results            |
| `guidance_comparison_metrics.csv` | Summary metrics from guidance comparison (PP vs PN) |
| `monte_carlo_results.csv`     | Aggregated metrics from Monte Carlo experiments          |
| `tuning_robustness_metrics.csv` | Results from parameter sweep experiments               |
| `tech_note.md`                | Technical note (≤ 3 pages) covering all models/results   |
| `README.md`                   | This file                                                |
| `requirements.txt`            | Python dependencies for the environment                  |

---

# 🚀 Quickstart

### 1. Install Dependencies

bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

2. Run Sample Simulation
python tests/test_guidance_comparison_enhanced.py

3. Generate Visuals

# Metrics bar plots
python tests/plot_guidance_metrics.py

# Run and save animated intercept
python tests/test_guidance_animation.py --guidance pp

🧠 Features & Highlights

| Capability                    | Status | Description                    |
| ----------------------------- | ------ | ------------------------------ |
| Pure Pursuit Guidance         | ✅      | Simple directional intercept   |
| Proportional Navigation       | ✅      | LOS-based acceleration law     |
| Comparative Evaluation        | ✅      | Metrics + 3D plots + GIF       |
| Tuning & Robustness Sweeps    | ✅      | Gain/noise/actuator tests      |
| Monte Carlo (100+ runs)       | ✅      | Stats across noise/disturbance |
| Failure Rate / StdDev Metrics | ✅      | Boxplots and CSV output        |
| Animated Intercepts (GIF)     | ✅      | Optional visualization         |
| Self-contained structure      | ✅      | One-command execution & setup  |



## 📊 Output Overview

All result files are located in the `doc/` folder and root CSV files.

| File / Pattern                     | Type        | Description                                              |
|------------------------------------|-------------|----------------------------------------------------------|
| `doc/*.png`                        | Plot Images | Bar charts, boxplots, performance visualizations         |
| `doc/*.gif`                        | Animations  | Intercept animations (e.g., Pure Pursuit vs PN)          |
| `guidance_comparison_metrics.csv` | CSV         | Single-run metrics for PP and PN                         |
| `monte_carlo_results.csv`         | CSV         | 100-run summary with noise/disturbance (mean, std, fail) |
| `tuning_robustness_metrics.csv`   | CSV         | Parameter sweep results for control sensitivity          |

📝 Tech Note

See tech_note.md for:

Guidance law descriptions
Simulation setup & assumptions
Key results and interpretations
Observations from robustness/Monte Carlo tests
📌 Notes

Code runs on Python ≥ 3.8
Compatible with macOS and Linux
Requires ~10s per Monte Carlo test batch
PNG/GIF outputs auto-save in /doc
✅ Submission Checklist

 Self-contained code repo
 Tech note (≤ 3 pages)
 Visual result package (GIFs + plots)
 Requirements file
 GitHub-hosted + documented
Authored by: Aayush Chugh
Submitted to: [Take-Home Challenge — Control Engineer (IMAARC)]
