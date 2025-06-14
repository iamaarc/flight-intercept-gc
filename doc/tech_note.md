
---

# 📝 **Tech Note (`doc/tech_note.md`) Structure**

```markdown
# Technical Note: Flight-Intercept Guidance & Control

## 1. Problem Definition
- Summarize the challenge: Intercepting arbitrary moving targets with a quadrotor using modular, testable code.

## 2. Sandbox Definition
- **Simulator**: Python (numpy, matplotlib).
- **Vehicle**: 2D kinematics, extendable to quadrotor dynamics.
- **Sensors**: Ideal/noisy position, IMU.
- **Targets**: Two types (straight line, maneuver).

## 3. Target Motion Profiles
- Detailed equations/logic for:
    - Straight-line motion
    - Constant turn-rate (arc)
- Bounds for speed, position, and heading

## 4. Guidance Laws
- Explain selected law (e.g., Proportional Navigation or Pursuit)
- Tuning parameters, rationale

## 5. Control Design
- Outer-loop: Position control (PID or equivalent)
- Inner-loop: Attitude/heading control
- Controller parameters and tuning

## 6. Integration & Test
- Simulation scenarios (steps, plots)
- Metrics: Intercept time, miss distance
- Robustness: Sensor noise, disturbances (if tried)

## 7. Results
- Trajectory plots
- Quantitative results (e.g., time to intercept, error)
- Discussion

## 8. Conclusion & Next Steps
- What worked well, limitations, possible extensions

---

## Appendix
- Code snippets, equations, references

