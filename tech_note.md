
# 🛠️ Technical Note: Flight-Intercept Guidance & Control

**Author**: Aayush Chugh  
**Date**: 18th June, 2025

## 1. Problem Definition

Design, simulate, and evaluate guidance and control strategies to intercept a moving target using a modular and extensible Python sandbox. The system should:

- Support different guidance laws (e.g., Pure Pursuit, Proportional Navigation).
- Be robust to disturbances and sensor noise.
- Allow comparison across performance metrics and visualize behavior.

---

## 2. Simulation Sandbox

| **Component**          | **Details**                                                                                                                                                                |
| ---------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Language**           | Python 3.12+ with core scientific libraries (`numpy`, `matplotlib`, `pandas`). Chosen for its modularity, clarity, and speed of prototyping.                               |
| **Simulator**          | Custom-built, discrete-time simulation engine. Kinematics-only modeling ensures lightweight, transparent evaluation of outer-loop G\&C logic.                              |
| **Vehicle Model**      | 3D thrust-vectoring rigid body modeled via ideal kinematics. No inertia, actuator lag, or drag forces included — sufficient for guidance law benchmarking.                 |
| **Actuators**          | Control inputs are assumed to be perfectly executed (no delay, saturation, or dynamics). This idealization is acceptable for guidance evaluation.                          |
| **Environment**        | Disturbance-free by default. Gravity, ground collision, and wind are not modeled. Monte Carlo simulations can inject noise to test robustness.                             |
| **Sensors**            | Ground-truth access to position in baseline mode. Gaussian noise (zero-mean) optionally added to simulate degraded GPS/IMU. No estimator (e.g., EKF) currently integrated. |
| **Target Types**       | Two motion profiles implemented: <br>• **Straight-line** constant velocity <br>• **Helical** (curved) path with tunable radius, turn rate, and vertical speed.             |
| **Guidance Framework** | Pluggable interface supporting multiple laws (e.g., Pure Pursuit, Proportional Navigation). Easy to extend to others (e.g., MPC, RL).                                      |
| **Controller**         | Outer-loop **PD position controller** converts guidance outputs into body-frame accelerations. Inner-loop is simplified: instantaneous attitude is assumed.                |

---

## 3. Target Motion Profiles

```python
### Linear Trajectory:
target_pos += target_vel * dt

3D Helical Motion:
x = R * cos(ωt), y = R * sin(ωt), z = Vz * t
```
| Parameter        | Value     |
| ---------------- | --------- |
| Radius           | 10 m      |
| Angular velocity | 0.5 rad/s |
| Vertical speed   | 1.0 m/s   |



## 4. Guidance Laws

| Guidance Method              | Description                                        |
| ---------------------------- | -------------------------------------------------- |
| Pure Pursuit (PP)            | Commands velocity towards target position directly |
| Proportional Navigation (PN) | Commands acceleration based on LOS rate            |

**Parameters:**
```python
pn_gain = 3.0  # For PN
```
> **Observation**: Pure Pursuit works well under general conditions. PN is more sensitive to tuning and less robust in curved trajectories.
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

Stub exists in [`src/controllers/attitude_controller.py`](./src/controllers/attitude_controller.py).  
Future versions can implement PID or LQR for more realism.
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
> **Interpretation**: PP reaches target reliably; PN often overshoots or fails in this scenario due to lack of curvature compensation.
---

### 📊 Monte Carlo (100 runs, with noise)

| Metric            | PP (mean ± std) | PN (mean ± std)   |
| ----------------- | --------------- | ----------------- |
| Miss Distance     | 6.1 ± 2.0 m     | 13.7 ± 5.5 m      |
| Time to Intercept | 3.6 ± 0.3 s     | — (failed mostly) |
| Energy Used       | 37.0 ± 3.2      | ~0.0              |
| Failures          | 1/100           | 78/100            |
> **Interpretation**: Pure Pursuit is significantly more robust under noise. PN requires precise conditions to succeed.
---

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
