# ✈️ Flight-Intercept Guidance & Control – System Sandbox

This project simulates and evaluates 3D intercept scenarios using different guidance laws under noisy and disturbed conditions. It includes physics-based simulation, comparative evaluation (Pure Pursuit vs Proportional Navigation), and Monte Carlo testing for robustness.

---

flight-intercept-gc/
├── src/ # Core simulation, control, and guidance modules
├── tests/ # Scripts for evaluation, animation, Monte Carlo
├── doc/ # All generated plots, GIFs, and visual results
├── guidance_comparison_metrics.csv
├── monte_carlo_results.csv
├── tuning_robustness_metrics.csv
├── tech_note.md # Technical summary (≤ 3 pages)
├── README.md # You're here!
├── requirements.txt # Python dependencies

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

Capability	Status	Description
Pure Pursuit Guidance	✅	Simple directional intercept
Proportional Navigation	✅	LOS-based acceleration law
Comparative Evaluation	✅	Metrics + 3D plots + GIF
Tuning & Robustness Sweeps	✅	Gain/noise/actuator tests
Monte Carlo (100+ runs)	✅	Stats across noise/disturbance
Failure Rate / StdDev Metrics	✅	Boxplots and CSV output
Animated Intercepts (GIF)	✅	Optional visualization
Self-contained structure	✅	One-command execution & setup



📊 Output Overview

doc/ contains:
*.png — bar charts, boxplots, trajectory plots
*.gif — intercept animations for PP and PN
CSV logs:
guidance_comparison_metrics.csv — single-run metrics
monte_carlo_results.csv — 100-run statistical summary
tuning_robustness_metrics.csv — controller sensitivity analysis
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
