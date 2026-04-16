#!/bin/bash
# RoboRacer WS - Launch Gazebo Simulation
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/.."
source install/setup.bash
ros2 launch roboracer_gazebo gz_sim.launch.py "$@"
