#!/bin/bash

echo "Updating package list..."
sudo apt update && sudo apt upgrade -y

echo "Installing Gazebo..."
sudo apt install -y ros-humble-gazebo-*

echo "Installing Cartographer..."
sudo apt install -y ros-humble-cartographer ros-humble-cartographer-ros

echo "Installing Navigation2..."
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

echo "Setting up TurtleBot3 workspace..."
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/

echo "Cloning TurtleBot3 repositories..."
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git

echo "Installing colcon..."
sudo apt install -y python3-colcon-common-extensions

echo "Building TurtleBot3 packages..."
cd ~/turtlebot3_ws
colcon build --symlink-install

echo "Configuring environment..."
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
source ~/.bashrc

echo "Installation and setup complete!"
