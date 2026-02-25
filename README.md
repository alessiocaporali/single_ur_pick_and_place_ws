# UR ROS2 Jazzy Workspace

This repository contains a ROS 2 workspace for Universal Robots (UR) simulation and control. It includes control and description packages for a single `ur_cell`, Docker compose files for simulators, and typical ROS 2 build artifacts.

## Quick overview

- **Main packages (in `src/`):**
  - `ur_cell/ur_cell_control`
  - `ur_cell/ur_cell_description`
  - `ur_rtde_interface` (UR RTDE interface for controlling UR robots)
  - `ur_rtde_srv` (UR RTDE service definitions)
- **Docker compose:** `docker_compose_sim.yml` (for running simulators/URSims)


## Building the workspace

1. Source your ROS 2 install (replace `<ROS_DISTRO>`):

```bash
source /opt/ros/<ROS_DISTRO>/setup.bash
```

2. From the workspace root, install package dependencies and build:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Notes:

- If you modify Python packages, `--symlink-install` makes iterative development faster.

## Running

**Using Docker Compose to start UR-simulators:**

```bash
docker compose -f docker_compose_sim.yml up
```

### Start Robot and RTDE Control Interface
```bash

# spawn robot and gripper
ros2 launch ur_cell_control start_robot.launch.py ur_type:=ur5e use_mock_gripper_hardware:=false  gripper_spawn:=true tty_port:=/dev/ttyUSB0

# RTDE controller interface for robot
ros2 launch ur_rtde_interface controller.launch.py robot_ip:=172.25.0.2

```

# Start Gripper 

in case of problems check grant permission with
```bash
sudo chmod 777 /dev/ttyUSB0 
```

```bash
# real gripper over USB
ros2 launch robotiq_hande_driver gripper.launch.py tty_port:=/dev/ttyUSB0 use_fake_hardware:=false
