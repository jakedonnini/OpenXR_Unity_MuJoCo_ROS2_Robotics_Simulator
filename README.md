# OpenXR_Unity_MuJoCo_ROS2_Robotics_Simulator
This project aims to combine OpenXR through Unity, MuJoCo physics engine and ROS2 to create a simulator that allows a user to interact in VR using a real developing pipeline

# Installation
- Follow this Guide for using ROS2: https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md
- Download Unity, OpenXR, Meta Quest Link to for the Quest 3, Add this package for unity to talk to ROS2 https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
- Install MuJoCo using this for Unity: https://mujoco.readthedocs.io/en/latest/unity.html

# Start the Docker Container
- Be in OpenXR_Unity_MuJoCo_ROS2_Robotics_Simulator\ros2 directory and docker is open
- build the container: docker build -t ros2_unity_image -f docker/Dockerfile .
- run the container: docker run -it --rm -p 10000:10000 ros2_unity_image /bin/bash
- run a new window in the same container: docker exec -it <container_name_or_id> /bin/bash

# Run ROS2 Nodes
- Get info from unity: ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
- Run kinematic controller: ros2 run panda_kinematics panda_arm_controller
