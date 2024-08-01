# ACSL Flight Stack
[![BSD License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE.txt)

To clone this repo with all the needed submodules you can run the command:

```bash
git clone --recurse-submodules https://github.com/andrealaffly/ACSL_flightstack_X8.git
```

## Introduction

The **Advanced Control Systems Lab (ACSL) Flight Stack** is a PX4-compatible offboard flight stack designed for multi-rotor UAVs (Uncrewed Aerial Vehicles). This software supports the UAV control community by offering a freeware, open-source solution that serves as a shared platform, facilitating comparison of research results. 

We strongly encourage control practitioners, researchers, and UAV enthusiasts to:

- **Use this code**
- **Improve it**
- **Provide constructive feedback**
- **Propose edits through [GitHub](https://github.com/andrealaffly/ACSL_flightstack_X8.git)**

This flight stack is tailored for autonomous UAVs with collinear propellers, such as quadcopters, X8-copters, and hexacopters. Currently, it supports quadcopters and X8-copters. However, by modifying the mixer matrix—which defines the relationship between total thrust, moment of thrust, and thrust produced by each motor—this software can be extended to other UAV configurations with collinear propellers.

## Outlook on the Control Architecture

Autonomous UAVs with collinear propellers are inherently under-actuated. To manage this, the software includes:

- **Inner Loop**: Handles rotational dynamics.
- **Outer Loop**: Manages translational dynamics.

Both loops are governed by nonlinear equations of motion.

### Available Control Solutions

This software currently offers two control solutions for the inner and outer loops:

1. **Continuous-Time Feedback-Linearizing Control Law** combined with a **PID (Proportional-Integral-Derivative) Control Law**.
2. The above control law is augmented by a **Robust Model Reference Adaptive Control (MRAC) System**, incorporating a simplified quadratic-in-the-velocity aerodynamic model.

For further details on these control architectures, refer to the following publications:

- M. Gramuglia, G. M. Kumar, and A. L'Afflitto, ["Two-Layer Adaptive Funnel MRAC with Applications to the Control of Multi-Rotor UAVs,"](https://doi.org/10.1109/RoMoCo60539.2024.10604361 target="_blank") *2024 13th International Workshop on Robot Motion and Control (RoMoCo),* Poznań, Poland, 2024, pp. 31-36.
- M. Gramuglia, G. M. Kumar, and A. L’Afflitto, ["A Hybrid Model Reference Adaptive Control System for Multi-Rotor Unmanned Aerial Vehicles,"](https://doi.org/10.2514/6.2024-0755) *AIAA SCITECH 2024 Forum, 2024.*
- E. Lavretsky and K. Wise, *"Robust and Adaptive Control: With Aerospace Applications,"*, London, UK: Springer, 2012.

Future versions of the software will include additional control systems.

## Hardware Requirements

This flight stack is compatible with **ROS2 (Foxy or Galactic)** and has been tested on an **ODROID M1S** companion computer interfaced with a **Pixhawk 6c**. The companion computer runs **Ubuntu 20.04 Linux**. The firmware version of the flight controller is **PX4 v1.15.0**.

### Assumed UAV State

- Provided by a motion capture system over UDP (or, alternatively, GNSS).
- IMU (Inertial Measurement Unit) from the Pixhawk flight controller.

## Maintenance Team

- [**Andrea L'Afflitto**](https://github.com/andrealaffly)
- [**Mattia Gramuglia**](https://github.com/mattia-gramuglia)
- [**Giri M. Kumar**](https://github.com/girimugundankumar)

For more information, visit [https://lafflitto.com](https://lafflitto.com).

[![ACSL Flight Stack Logo](https://lafflitto.com/images/ACSL_Logo.jpg)](https://lafflitto.com/ACSL.html)


---

This software is distributed under a permissive **3-Clause BSD License**.

