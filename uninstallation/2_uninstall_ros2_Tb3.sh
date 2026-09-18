#!/bin/bash

echo "Removing Gazebo..."
sudo apt remove --purge -y ros-humble-gazebo-*

echo "Removing Cartographer..."
sudo apt remove --purge -y ros-humble-cartographer ros-humble-cartographer-ros

echo "Removing Navigation2..."
sudo apt remove --purge -y ros-humble-navigation2 ros-humble-nav2-bringup

# echo "Removing colcon..."
# sudo apt remove --purge -y python3-colcon-common-extensions

echo "Removing TurtleBot3 workspace..."
rm -rf ~/turtlebot3_ws

echo "Cleaning up ROS environment variables from .bashrc..."
sed -i '/source ~\/turtlebot3_ws\/install\/setup.bash/d' ~/.bashrc
sed -i '/export ROS_DOMAIN_ID=30 #TURTLEBOT3/d' ~/.bashrc
sed -i '/source \/usr\/share\/gazebo\/setup.sh/d' ~/.bashrc

echo "Updating package list and removing unused dependencies..."
sudo apt autoremove -y
sudo apt autoclean -y

echo "Uninstallation complete!"
