#!/bin/bash

# Exit on error
set -e

# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

# Setup Sources
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package index and upgrade system
sudo apt update
sudo apt upgrade -y

# Install ROS 2 Humble (Desktop version)
sudo apt install -y ros-humble-desktop

# Install ROS-Base (if minimal setup is needed, uncomment below line)
# sudo apt install -y ros-humble-ros-base

# Install development tools
sudo apt install -y ros-dev-tools

# Environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Instructions to verify installation
echo "ROS 2 Humble installation complete!"
echo "To verify, open a new terminal and run:"
echo "source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp talker"
echo "Then in another terminal:"
echo "source /opt/ros/humble/setup.bash && ros2 run demo_nodes_py listener"
