
# 🛠️ Technical Note: Flight-Intercept Guidance & Control

**Author**: Aayush Chugh  
**Date**: June 2025

---

## 1. Problem Definition

Design, simulate, and evaluate guidance and control strategies to intercept a moving target using a modular and extensible Python sandbox. The system should:

- Support multiple guidance laws (e.g., Pure Pursuit, Proportional Navigation).
- Be robust to disturbances and sensor noise.
- Allow for comparative performance metrics and visualizations.

---

## 2. Simulation Sandbox

| Component     | Details                                                                 |
|---------------|-------------------------------------------------------------------------|
| Language      | Python 3.12+                                                             |
| Environment   | CLI-driven, using `numpy`, `matplotlib`, `pandas`                        |
| Vehicle       | 3D kinematic model with heading control (simplified quadrotor)           |
| Sensors       | Ideal and noisy position sensors                                         |
| Target Types  | Linear motion and curved (helical) motion                                |
| Guidance      | Modular framework for plugging in multiple guidance laws                |
| Controller    | Outer-loop PD controller for position control                            |

---

## 3. Target Motion Profiles

```python
# Linear Motion
target_pos += target_vel * dt

# Helical Motion
x = R * cos(ωt)
y = R * sin(ωt)
z = Vz * t
```

| Parameter        | Value     |
| ---------------- | --------- |
| Radius           | 10 m      |
| Angular Velocity | 0.5 rad/s |
| Vertical Speed   | 1.0 m/s   |

---

## 4. Guidance Laws

| Guidance Method              | Description                                        |
| ---------------------------- | -------------------------------------------------- |
| Pure Pursuit (PP)            | Commands velocity towards target position directly |
| Proportional Navigation (PN) | Commands acceleration based on LOS rate            |

**Tuning Parameter Example:**

```python
pn_gain = 3.0
```

> **Observation**: Pure Pursuit works well under general conditions. PN is more sensitive to tuning and less robust in curved trajectories.

---

## 5. Control Design

### Outer-Loop: Position Controller

A PD controller converts guidance commands into acceleration vectors:

```python
acc_cmd = kp * (target_pos - drone_pos) + kd * (target_vel - drone_vel)
```

- Operates in world-frame.
- Tuned to stabilize pursuit and reduce oscillations.

### Inner-Loop: Attitude Controller

| Component         | Status        | Notes                                          |
| ----------------- | ------------- | ---------------------------------------------- |
| Attitude Control  | ⚠️ Simplified | Attitude assumed instantly achieved            |
| Actuator Dynamics | ❌ Not modeled | No motors or angular rate feedback             |

Stub exists in [`src/controllers/attitude_controller.py`](./src/controllers/attitude_controller.py).  
Future versions can implement PID or LQR for more realism.

---

## 6. Integration and Testing

### Scenario

- Initial pursuer position: `[0, 0, 0]`
- Target follows a 3D helix
- 10s simulation or until intercept (within 1m)

### Evaluation Metrics

| Metric            | Description                        |
| ----------------- | ---------------------------------- |
| Miss Distance     | Final Euclidean distance to target |
| Time to Intercept | Time when within 1m of target      |
| Energy Used       | Sum of squared control inputs      |
| Failure Rate      | % of simulations with no intercept |

---

## 7. Results Summary

### 📈 Sample Metrics (Single Run)

| Method | Miss Distance (m) | Time to Intercept (s) | Energy |
| ------ | ----------------- | --------------------- | ------ |
| PP     | 5.92              | 3.55                  | 36.43  |
| PN     | 15.26             | N/A (fail)            | 0.00   |

> **Interpretation**: PP reaches target reliably; PN often overshoots or fails in this scenario due to lack of curvature compensation.

---

### 📊 Monte Carlo Results (100 runs, noise + disturbance)

| Metric            | PP (mean ± std) | PN (mean ± std)   |
| ----------------- | --------------- | ----------------- |
| Miss Distance     | 6.1 ± 2.0 m     | 13.7 ± 5.5 m      |
| Time to Intercept | 3.6 ± 0.3 s     | — (mostly failed) |
| Energy Used       | 37.0 ± 3.2      | ~0.0              |
| Failures          | 1/100           | 78/100            |

> **Interpretation**: Pure Pursuit is significantly more robust under noise. PN requires precise conditions to succeed.

---

### 📁 Visual Outputs

| Output Type            | File                                                                |
|------------------------|---------------------------------------------------------------------|
| Bar Charts             | `doc/guidance_*_comparison.png`                                     |
| Monte Carlo Boxplots   | `doc/monte_carlo_*_boxplot.png`                                     |
| Failure Statistics     | `doc/monte_carlo_failure_count.png`                                 |
| Intercept Animations   | `doc/guidance_comparison_pp.gif`, `doc/guidance_comparison_pn.gif`  |

---

## 8. Conclusion & Next Steps

### Key Findings

- **PP** is more adaptable to changing trajectories.
- **PN** fails often in noisy or curved motion scenarios unless finely tuned.
- The system design supports modularity and extension.

### Future Extensions

- Add MPC or RL-based guidance
- Implement actuator dynamics and cascaded control
- ROS or WebGL integration for real-time visualization
- Expand noise/disturbance profiles

---

## 📎 Appendix

### Simulation Files

| File | Description |
|------|-------------|
| [`tests/test_guidance_comparison_enhanced.py`](./tests/test_guidance_comparison_enhanced.py) | Single-run comparison |
| [`tests/test_monte_carlo_guidance.py`](./tests/test_monte_carlo_guidance.py) | Statistical robustness test |
| [`tests/test_guidance_animation.py`](./tests/test_guidance_animation.py) | 3D visualization of run |

### Output Logs

- `guidance_comparison_metrics.csv`
- `monte_carlo_results.csv`
- `tuning_robustness_metrics.csv`
