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


## 🚀 Quickstart Summary

| Step | Command | Description |
|------|---------|-------------|
| ✅ 1. **Install Dependencies** | `python3 -m venv .venv`<br>`source .venv/bin/activate`<br>`pip install -r requirements.txt` | Set up Python virtual environment and install all required packages |
| ✅ 2. **Run Sample Simulation** | `python tests/test_guidance_comparison_enhanced.py` | Compares Pure Pursuit vs Proportional Navigation and saves metrics |
| ✅ 3. **Generate Visuals** | `python tests/plot_guidance_metrics.py` | Bar plots for miss distance, energy, and intercept time |
| &nbsp; | `python tests/test_guidance_animation.py --guidance pp` | Saves 3D animated GIF for Pure Pursuit (`pp`) or PN (`pn`) |
| ✅ 4. **Monte Carlo Simulation** | `python tests/test_monte_carlo_guidance.py` | Runs 100 randomized trials per method with noise/disturbance |
| &nbsp; | *(auto)* | Saves boxplots, failure rate plots, and CSV summary |
| ✅ 5. **Robustness Tuning Sweep** | `python tests/test_tuning_sweep.py` | Varies controller gains/disturbance levels and logs results |



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

## 📝 Tech Note

📄 Refer to [`tech_note.md`](tech_note.md) for full technical details, including:

- 🔄 **Guidance Laws**: Pure Pursuit, Proportional Navigation
- 🛠️ **Simulation Setup**: Assumptions, initial conditions, controller design
- 📊 **Key Results**: Miss distance, energy, intercept time comparisons
- 🧪 **Robustness & Monte Carlo**: Insights from param sweeps and noisy trials

---

## 📌 Notes

- ✅ Runs on **Python ≥ 3.8**
- 🖥️ Compatible with **macOS and Linux**
- ⏱️ Approx. **10s per Monte Carlo test batch**
- 📁 All PNGs, GIFs, and CSV outputs auto-save into `doc/`

---

## ✅ Submission Checklist

| Deliverable                    | Status | Location |
|-------------------------------|--------|----------|
| 🧠 Self-contained code repo    | ✅     | GitHub |
| 📄 Tech note (≤ 3 pages)       | ✅     | [`tech_note.md`](tech_note.md) |
| 📊 Visual results (GIFs, plots) | ✅     | `doc/` |
| 📦 requirements.txt            | ✅     | [`requirements.txt`](requirements.txt) |
| 🌐 Hosted + documented         | ✅     | [`github.com/iamaarc/flight-intercept-gc`](https://github.com/iamaarc/flight-intercept-gc) |

---

**👤 Authored by**: Aayush Chugh  
**📨 Submitted to**: Take-Home Challenge to LENDURAI — *Control Engineer (IAMAARC)*
