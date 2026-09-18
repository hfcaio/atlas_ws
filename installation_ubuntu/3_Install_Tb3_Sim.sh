#!/bin/bash

# Define workspace
WORKSPACE=~/turtlebot3_ws
SRC_DIR=$WORKSPACE/src

# Ensure workspace exists
mkdir -p $SRC_DIR

# Navigate to src and clone simulation package
cd $SRC_DIR
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# Build the workspace
cd $WORKSPACE
colcon build --symlink-install --parallel-workers 10

# Source the setup file
source install/setup.bash

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Launch Gazebo with empty world
ros2 launch turtlebot3_gazebo empty_world.launch.py


# Operate TurtleBot3
# Run the teleoperation node in a new terminal window
echo "To control TurtleBot3, open a new terminal and run:"
echo "$ ros2 run turtlebot3_teleop teleop_keyboard"
