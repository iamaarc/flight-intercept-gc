# Flight-Intercept Guidance & Control – System Sandbox

## Overview

A modular Python sandbox for **interception guidance & control**, supporting both 2D and 3D simulations.  
Key goals: rapid prototyping, transparent logic, and reproducibility for research or teaching.

---

## Day 1: Sandbox Setup & Simulator

- **Simulator:** Custom Python-based ODE (Ordinary Differential Equation) integrator.
- **Vehicle Model:** 6-DOF quadrotor (3D) and planar point-mass (2D) supported.
- **Justification:** 
  - Easy to debug/extend.
  - Allows fast iterations on G&C logic.
  - 3D for realism; 2D for rapid concept validation.

**Vehicle Dynamics**
- **3D:** Newton-Euler rigid body, thrust + 3-axis torque inputs, actuator dynamics.
- **2D:** Planar motion, simplified kinematics.
- **Limits:** Thrust: [0, 40 N], Torques: ±1 Nm (3D).
- **Sensors:** IMU, GPS, barometer, magnetometer (configurable noise/latency).

**Environment**
- Gravity, ground plane, (optional) wind/disturbance.

**Reproducibility:**  
All parameters in `/doc/tech_note.md`.  
Run everything with:  
```sh
python src/main.py --demo [day] --mode [2d|3d]

Day 2: Target Motion Profiles

Implemented Profiles (in src/target.py):
2D & 3D:
Straight-Line Motion: Constant velocity, user-selectable speed/direction.
Constant Turn/Circular Motion: User-defined turn rate/radius.
Results
2D:


3D:


Simulated trajectories: Blue = straight/circular (2D), Helix (3D).

Day 3: Guidance & Pursuit Demo

Goal:
Implement a simple guidance law (Pure Pursuit) where a pursuer tracks a moving target.

2D:
Target follows circular/arc.
Pursuer chases using heading/velocity commands.
Baseline for more advanced interception strategies.
3D:
Target follows 3D helix or circular path.
Pursuer initialized at offset, uses direct pursuit.
Results
2D:


3D:


Orange = pursuer, Blue = target.

Day 4: Outer-Loop Position Controller

Goal:
Add a PD position controller so the pursuer tracks smoothly, even with noise/disturbances.

2D:
Converts guidance commands into position/velocity.
Smoother and more robust than simple pursuit.
3D:
Outer loop PD controller.
Attitude control for roll/pitch/yaw (with demo plots).
Results
2D:


3D:


Day 5: Animation, Robustness, Extensions

Animated Simulations:
All major demos now animated with matplotlib.animation, outputting GIFs in /doc/.
Example:


Noise/Disturbance Injection:
Optional: Add wind, sensor noise, actuator lag.
Advanced Guidance:
Placeholder for Proportional Navigation, optimal control, or swarm logic.
3D Visualization:
Future work: Real-time 3D plotting, AirSim/Flightmare bridge.
How to Run

Clone the repo and install requirements:

sh
Copy
Edit
git clone https://github.com/iamaarc/flight-intercept-gc.git
cd flight-intercept-gc
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
To run main simulation (change --demo and --mode as needed):

sh
Copy
Edit
python src/main.py --demo day4 --mode 3d
Or for animation/GIF:

sh
Copy
Edit
python src/main.py --demo day4 --mode 3d --animate
Structure

src/
main.py – main simulation launcher
target.py – target trajectory models
guidance.py – pursuit logic
position_controller.py – PD/outer-loop control
attitude_controller.py – inner-loop (3D only)
doc/
All figures, animations, technical notes
Citation & Contact

Made by Aayush Chugh.
For questions or collaboration, open an issue or contact via GitHub.

