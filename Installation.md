# Installation

This page guides you through installing the ACSL Flight Stack.

## 1. Clone the Repository

```bash
git clone https://github.com/andrealaffly/ACSL-flightstack.git
```

## 2. Install Dependencies

- ROS2 Foxy or Galactic (see [ROS2 Install Guide](https://docs.ros.org/en/foxy/Installation.html))
- Eigen3 (usually `sudo apt install libeigen3-dev`)
- MAVROS, MAVLink, GeographicLib (for offboard control)

```bash
sudo apt install ros-foxy-mavros ros-foxy-mavlink ros-foxy-geographic-msgs
```

- Additional dependencies may be required based on your hardware and OS.

## 3. Build the Workspace

```bash
cd ACSL-flightstack
colcon build --symlink-install
source install/setup.bash
```

## 4. Configure Your Hardware

- Ensure your Pixhawk is flashed with PX4 firmware
- Confirm your motion capture or GNSS system is operational
- Connect your onboard computer (e.g., Jetson) to Pixhawk via serial/USB

## 5. Test the Installation

- Run:

```bash
ros2 launch acsl_flightstack bringup.launch.py
```

- If everything is set up correctly, the UAV should be ready for software-in-the-loop (SITL) or hardware tests
