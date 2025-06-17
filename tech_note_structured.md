
## 4. Guidance Laws

| Guidance Method              | Description                                        |
| ---------------------------- | -------------------------------------------------- |
| Pure Pursuit (PP)            | Commands velocity towards target position directly |
| Proportional Navigation (PN) | Commands acceleration based on LOS rate            |

**Parameters:**
```python
pn_gain = 3.0  # For PN
```

---

## 5. Control Design

**Outer-Loop: Position Controller**  
Implemented as a Proportional-Derivative (PD) controller.  
Converts desired position (from guidance law) into acceleration commands.  
Operates in world-frame, assumes simple translational model.

```python
acc_cmd = kp * (target_pos - drone_pos) + kd * (target_vel - drone_vel)
```

- Integrated into simulation
- Tuned to provide stable, responsive intercept behavior

**Inner-Loop: Attitude Controller**  
Stub exists: `attitude_controller.py`  
Intended to map acceleration commands to attitude setpoints.  
Currently idealized: assumes attitude is immediately achieved (no actuator dynamics, no angle-rate feedback loop).

| Component         | Status        | Notes                                          |
| ----------------- | ------------- | ---------------------------------------------- |
| Attitude Control  | ⚠️ Simplified | No real motor control or PID attitude loop yet |
| Actuator Dynamics | ❌ Not modeled | Treated as perfect instantaneous orientation   |

---

## 6. Integration and Testing

**Comparison Scenario:**
- Initial pursuer: `[0, 0, 0]`
- Target follows 3D helical trajectory
- No wind in baseline case
- Simulations run for up to 10s or until intercept

**Metrics:**

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

### 📊 Monte Carlo (100 runs, with noise)

| Metric            | PP (mean ± std) | PN (mean ± std)   |
| ----------------- | --------------- | ----------------- |
| Miss Distance     | 6.1 ± 2.0 m     | 13.7 ± 5.5 m      |
| Time to Intercept | 3.6 ± 0.3 s     | — (failed mostly) |
| Energy Used       | 37.0 ± 3.2      | ~0.0              |
| Failures          | 1/100           | 78/100            |

### 📁 Visuals

| Type                 | File                                                               |
| -------------------- | ------------------------------------------------------------------ |
| Bar Charts           | `doc/guidance_*_comparison.png`                                    |
| GIFs                 | `doc/guidance_comparison_pp.gif`, `doc/guidance_comparison_pn.gif` |
| Monte Carlo Boxplots | `doc/monte_carlo_*_boxplot.png`                                    |
| Failure Stats        | `doc/monte_carlo_failure_count.png`                                |

---

## 8. Conclusion & Future Work

**Findings:**
- Pure Pursuit is more robust and consistent under sensor noise.
- PN fails often in 3D curvature scenarios unless tightly tuned.
- Modular design allows rapid testing and expansion.

**🔭 Next Steps:**
- Add MPC or deep RL-based guidance
- Integrate nonlinear quadrotor dynamics
- Real-time visualization (WebGL, ROS)
- Expand Monte Carlo to more disturbance models

---

## 📎 Appendix

- `src/`: Core modules (guidance, controllers, simulation)
- `tests/`: CLI scripts for evaluation:
  - `test_guidance_comparison_enhanced.py`
  - `test_monte_carlo_guidance.py`
  - `test_guidance_animation.py`
- `doc/`: All outputs (.png, .gif) and visual logs

**Root CSVs:**
- `guidance_comparison_metrics.csv`
- `monte_carlo_results.csv`
- `tuning_robustness_metrics.csv`
