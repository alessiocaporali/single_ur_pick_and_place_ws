

# docker with UR sim

docker run --rm -it -p 5900:5900 -p 6080:6080 --name ursim universalrobots/ursim_e-series


# ROS2 launch

ros2 launch ur_rtde_interface controller.launch.py


# AGX ROS2 launch

python sim_ur.py 