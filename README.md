# ROBODOG

## Project Overview
ROBODOG is a modular, 12-Degree-of-Freedom (12-DOF) quadruped robot designed and engineered for the IEEE RAS Tunisia Build Challenge. Built around a split-spine 3D-printed chassis, dual-shear ball-bearing joints, a Star-Topology power distribution system, and a Raspberry Pi 5 edge compute core, the platform bridges physics simulation (MuJoCo) directly to real-time physical actuation via a browser-based Web Control App.

## Core Capabilities
* **Posture & Balance:** Stand up from rest, sit, and maintain a stable static/dynamic stance using feedback from a 6-axis IMU (MPU-6050).
* **Omnidirectional Locomotion:** Walk forward, turn on the spot, and steer dynamically using inverse-kinematics (IK) gait generation.
* **Web-Based Command Center:** Real-time browser UI (`index.html` + `server.py`) for remote ON/OFF power/torque enable, walking, rotating, and telemetry monitoring.
* **Sim-to-Real Parity:** Identical kinematic solver (`gait_core.py`) driving both the live MuJoCo physics twin (`sim_live.py`) and the physical PCA9685 PWM hardware driver (`hardware_walker.py`).

## Repository StructureROBODOG/
├── ELECTRICAL/
│   └── ELECTRICAL BLOCK DIAGRAM.png      # Star-bus power & I2C/PWM wiring architecture
├── MECHANICAL/
│   ├── MECHANICAL BLOCK DIAGRAM.png      # Chassis split-spine & 12-DOF kinematic chain
│   ├── CAD/                              # Native CAD & editable STEP assemblies
│   └── STL/                              # Print-ready STL/3MF parts (numbered with print specs)
├── SIMULATION/
│   ├── mujoco_dog_cad.py                 # MuJoCo physics environment using exact CAD STL inertias
│   ├── cad_run_real_servo.png            # Torque & joint tracking plot (realistic servo model)
│   ├── cad_run_servo20.png               # Torque & gait response plot (20 kg·cm actuator limit)
│   ├── robodog_cad_servo20.mp4           # Recorded MuJoCo walking & turning simulation video
│   └── summary_real_servo.txt            # Automated simulation metrics (peak torque, velocity, CoM)
└── SOFTWARE/
└── FINAL SERVER PI AND HTML/
├── index.html                    # Responsive Web Control Dashboard (Walk, Rotate, ON/OFF)
├── server.py                     # Web server & WebSocket/HTTP command router
├── gait_core.py                  # 12-DOF Inverse Kinematics, gait generator & IMU feedback
├── hardware_walker.py            # PCA9685 PWM driver & servo calibration offsets
├── oled_display.py               # 1.3" I2C OLED real-time IP, battery & state display
└── sim_live.py                   # Live MuJoCo visualizer coupled to gait_core.py
[🔗 Watch the MuJoCo Walking Simulation Video](https://drive.google.com/file/d/1xey-RBdzLEuBOvsU3O_ST1kFibfGtx57/view?usp=sharing)

