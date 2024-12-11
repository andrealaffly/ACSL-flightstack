# ACSL Flight Stack
[![BSD License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE.txt)
[![Website](https://img.shields.io/badge/Website-acslstack.com-green)](https://www.acslstack.com/)

To clone this repo with all the needed submodules you can run the command:

```bash
git clone --recurse-submodules https://github.com/andrealaffly/ACSL-flightstack.git
```

## Introduction

The **Advanced Control Systems Lab (ACSL) Flight Stack** is a PX4-compatible offboard flight stack designed for multi-rotor UAVs (Uncrewed Aerial Vehicles). This software supports the UAV control community by offering a freeware, open-source solution that serves as a shared platform, facilitating comparison of research results. 

We strongly encourage control practitioners, researchers, and UAV enthusiasts to:

- **Use this code**
- **Improve it**
- **Provide constructive feedback**
- **Propose edits through [GitHub](https://github.com/andrealaffly/ACSL-flightstack.git)**

This flight stack is tailored for autonomous UAVs with collinear propellers, such as quadcopters, X8-copters, and hexacopters. Currently, it supports quadcopters and X8-copters. However, by modifying the mixer matrix, which defines the relationship between total thrust, moment of thrust, and thrust produced by each motor, this software can be extended to other UAV configurations with collinear propellers.

## Outlook on the Control Architecture

Autonomous UAVs with collinear propellers are inherently under-actuated. For this reason, the software includes:

- **Inner Loop**: Handles the rotational dynamics.
- **Outer Loop**: Handles the translational dynamics.

Both loops are governed by nonlinear equations of motion.

### Available Control Solutions

This software currently offers two control solutions for the inner and outer loops:

1. **Continuous-Time Feedback-Linearizing Control Law** combined with a **PID (Proportional-Integral-Derivative) Control Law**.
2. The above control law is augmented by a **Robust Model Reference Adaptive Control (MRAC) System**, incorporating a simplified quadratic-in-the-velocity aerodynamic model.

For further details on these control architectures, refer to the publications found [here](https://www.acslstack.com/Journals).

Future versions of the software will include additional control systems.

## Hardware Requirements

This flight stack is compatible with **[ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html)** or **[Galactic](https://docs.ros.org/en/galactic/Installation.html)** and has been tested on an **[ODROID M1S](https://www.hardkernel.com/shop/odroid-m1s-with-8gbyte-ram-io-header/)** companion computer interfaced with a **[Pixhawk 6c](https://docs.px4.io/main/en/flight_controller/pixhawk6c.html)**. The companion computer runs **Ubuntu 20.04 Linux**. The firmware version of the flight controller is **[PX4 v1.15.0](https://docs.px4.io/v1.15/en/)**.

### Assumed UAV State

- Provided by a motion capture system over UDP (or, alternatively, GNSS).
- IMU (Inertial Measurement Unit) from the Pixhawk flight controller.

## How to Plot Log Data

The GitHub repository **[ACSL-flightstack-accessories](https://github.com/andrealaffly/ACSL-flightstack-accessories)** provides MATLAB scripts to plot flight log data and examples of flight tests data.
  
## Demo
A demo of this flight stack is available on **[YouTube](https://youtu.be/Ykjjg21iAm0)**

## Maintenance Team

- [**Andrea L'Afflitto**](https://github.com/andrealaffly)
- [**Mattia Gramuglia**](https://github.com/mattia-gramuglia)

For more information, visit [acslstack.com](https://www.acslstack.com/).

[![ACSL Flight Stack Logo](https://lafflitto.com/images/ACSL_Logo.jpg)](https://lafflitto.com/ACSL.html)


---

This software is distributed under a permissive **3-Clause BSD License**.

