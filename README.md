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

**UR robot state + rtde interface + robotiq gripper:**
```bash

# no gripper (UR robot only)
ros2 launch ur_rtde_interface robot.launch.py gripper_mode:=false

# real gripper over USB Modbus
ros2 launch ur_rtde_interface robot.launch.py gripper_mode:=modbus tty_port:=/dev/ttyUSB0

# gripper over UR RTDE socket service (model in RViz, no gripper controller spawn, no Modbus connection)
ros2 launch ur_rtde_interface robot.launch.py gripper_mode:=rtde

# fake gripper (simulation)
ros2 launch ur_rtde_interface robot.launch.py gripper_mode:=simulation

```

## Gripper controller parameters

When launching with:

```bash
ros2 launch ur_rtde_interface robot.launch.py
```

the `gripper_action_controller` parameters are loaded from:

```bash
src/ur_cell/ur_cell_control/config/ros2_controllers.yaml
```

To expose the RTDE gripper service (`/<namespace>/move_gripper`) through the UR robot socket, enable:

```bash
ros2 launch ur_rtde_interface robot.launch.py gripper_mode:=rtde
```

Launch only the gripper:

```bash
# real gripper over USB
ros2 launch ur_cell_control start_gripper.launch.py use_sim_gripper:=false tty_port:=/dev/ttyUSB0

# sim gripper
ros2 launch ur_cell_control start_gripper.launch.py use_sim_gripper:=true
```
