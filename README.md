# Flight-Intercept Guidance & Control – System Sandbox

Day 1: Define sandbox, pick simulator, finalize quadrotor/environment/sensors, document in README.
## Simulator
- **Choice:** Custom Python-based ODE simulator.
- **Justification:** 
  - Rapid prototyping, transparent for G&C research.
  - Easy to inspect, modify, and debug.
  - (Optional: Can migrate to AirSim/Flightmare for richer visuals if time allows.)

## Vehicle Dynamics
- **Model:** 6-DOF rigid-body quadrotor (Newton-Euler equations).
- **Control Inputs:** Total thrust and 3-axis torques (roll, pitch, yaw).
- **Actuator Limits:** Thrust: [0, 40 N], Torques: ±1 Nm per axis.
- **Actuator Dynamics:** First-order lag (time constant: 50 ms).

## Environment
- **Gravity:** \(g = 9.81\) m/s² (downward, constant).
- **Ground Plane:** \(z = 0\) (no negative altitude).
- **Wind/Disturbances:** None for baseline (can add later for robustness).
- **Atmosphere:** Standard, no drag by default.

## Sensors
- **IMU:** 3-axis accel & gyro (sample rate: 200 Hz, noise: σ=0.05 m/s², 0.005 rad/s).
- **GPS:** Position & velocity (10 Hz, σ=0.1 m).
- **Barometer:** Altitude (50 Hz, σ=0.05 m).
- **Magnetometer:** Heading (50 Hz, σ=0.01 rad).
- **Estimation:** Start with ground-truth; stretch: add EKF or complementary filter.

## Documentation & Reproducibility
- All parameters and modeling choices documented here and in `/doc/tech_note.md`.
- Simulation reproducible with one command:

Day 2: Implement target motion (two profiles), integrate into your sim.
## Target Motion Profiles

1. **Static/Low-Speed Drift**
   - Type: Straight-line, constant velocity
   - Example bounds: Speed = 0–2 m/s, Acceleration ≈ 0

2. **Maneuvering Trajectory**
   - Type: Constant turn-rate (circular), can be extended to random bursts
   - Example bounds: Speed = 2–5 m/s, Turn-rate = 0.1–0.5 rad/s, Acceleration ≤ 2 m/s²

Both profiles are implemented in `src/target.py` and can be selected in the simulator.

