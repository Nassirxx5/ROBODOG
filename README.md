# ROBODOG

**A small, modular 12-DOF quadruped robot**
IEEE RAS Tunisia Build Challenge

---

## Table of Contents

1. [Overview](#overview)
2. [Key Features](#key-features)
3. [System at a Glance](#system-at-a-glance)
4. [Specifications](#specifications)
5. [Repository Structure](#repository-structure)
6. [Design Highlights](#design-highlights)
7. [Simulation](#simulation)
8. [Getting Started](#getting-started)
9. [Documentation](#documentation)
10. [Team](#team)

---

## Overview

ROBODOG is a small, modular quadruped robot with 12 degrees of freedom (three per leg). It was developed for the IEEE RAS Tunisia Build Challenge and covers the full design chain: mechanical design, electrical architecture, physics simulation and software control.

The robot is driven from a web app running on a Raspberry Pi 5, and the same walking code runs in both the simulations (MuJoCo and Gazebo Harmonic) and on the real hardware.

---

## Key Features

| Feature | Description |
|---|---|
| Stand, sit and balance | Uses inertial feedback |
| Walk and turn | Inverse-kinematics gait generator |
| Browser-based control | From any phone or laptop on the same Wi-Fi network |
| Dual simulation | Validated in both MuJoCo and Gazebo Harmonic |
| Sim-to-real consistency | One locomotion engine for both the simulators and the physical robot |
| Real-time torque monitoring | Live joint torque graphs in the Gazebo simulation |
| Safety first | Emergency stop and status display on the robot |

---

## System at a Glance
<img width="732" height="382" alt="image" src="https://github.com/user-attachments/assets/a1799880-0629-4f7b-8b78-818c878031ef" />

---

## Specifications

| Parameter | Value |
|---|---|
| Degrees of freedom | 12 (3 per leg) |
| Mass | ≈ 2.6 kg |
| Controller | Raspberry Pi 5 |
| Actuators | 12 × metal-gear servos |
| Sensors | IMU, distance sensor |
| Power | 3S LiPo with isolated regulated rails |
| Runtime | ≈ 50 min of active walking (estimated) |
| Frame | 3D-printed split-spine chassis |
| Simulation | MuJoCo and Gazebo Harmonic |

---

## Repository Structure

```
ROBODOG/
├── MECHANICAL/            Chassis and leg design (CAD, STL files, block diagram)
├── ELECTRICAL/            Power distribution and wiring (block diagram)
├── SIMULATION/
│   ├── MUJOCO/            MuJoCo physics model, plots and demo video
│   └── GAZEBO/            Gazebo Harmonic setup scripts, data logger, videos and torque graphs
│       ├── robodog_all.sh
│       ├── patch_rotation.sh
│       ├── patch_turn_camera.sh
│       └── data_logger.py
├── SOFTWARE/              Web interface, gait engine and hardware drivers
└── README.md
```

---

## Design Highlights

| Domain | Highlight |
|---|---|
| Mechanical | A rigid split-spine chassis and joints keep loads off the servo shafts. Parts are designed for 3D printing and easy servicing. |
| Electrical | A star-topology power layout separates the servos from the computer, so motor current spikes do not reset the Raspberry Pi. |
| Simulation | The robot was validated in MuJoCo and Gazebo Harmonic using the real CAD geometry and mass properties before fabrication, including torque margins and walking stability. |
| Software | A single Python locomotion core converts commands from the web app into joint angles, then sends them either to a simulator or to the servos. |

---

## Simulation

The robot is simulated in two complementary environments, both using the same locomotion code as the real robot.

| Simulator | Purpose | Contents |
|---|---|---|
| MuJoCo | Physics validation with exact CAD geometry and mass properties | Simulation code, torque and tracking plots, demo video |
| Gazebo Harmonic | Full robot simulation with real-time monitoring | Simulation code, demo video, real-time joint torque graphs |

---

## Getting Started

### Simulation: MuJoCo (PC)

```sh
cd SIMULATION/MUJOCO
pip install mujoco numpy matplotlib
python mujoco_dog_cad.py
```

### Simulation: Gazebo Harmonic (PC)

```sh
cd SIMULATION/GAZEBO
chmod +x *.sh

# 1. Set up and build the full RoboDog simulation
./robodog_all.sh

# 2. Apply the rotation and turning/camera patches
./patch_rotation.sh
./patch_turn_camera.sh

# 3. Log joint data for the real-time torque graphs
python3 data_logger.py
```

### Physical robot (Raspberry Pi 5)

```sh
cd "SOFTWARE/FINAL SERVER PI AND HTML"
python server.py
```

Then open `http://<ROBOT_IP>:8000` on a device connected to the same network. The IP address is shown on the robot's OLED screen.

> Detailed wiring, parts list and calculations are available in the **Design Book** (see `docs/`).

---

## Documentation

| Document | Content |
|---|---|
| Design Book | Full design rationale, calculations and bill of materials |
| `MECHANICAL/` | CAD files and printable parts |
| `ELECTRICAL/` | Power and wiring architecture |
| `SIMULATION/` | MuJoCo and Gazebo Harmonic code, demo videos and torque graphs |

---

## Team

| # | Name |
|---|---|
| 1 | Mohamed Yassine Jouini |
| 2 | Hayder Hachaichi |
| 3 | Mariem Marzouk |
| 4 | Nassir Manouchi |
| 5 | Iheb Cherif |
