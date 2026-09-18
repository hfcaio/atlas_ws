#!/bin/bash

# Add user to dialout group
sudo usermod -a -G dialout $USER

# Remove modemmanager
sudo apt-get remove modemmanager -y

# Install necessary GStreamer plugins
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y

# Install libfuse2
sudo apt install libfuse2 -y

# Install necessary XCB libraries
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y

# Prompt user to logout and login again
echo "Please logout and login again to enable the change to user permissions."

# Download and install QGroundControl
cd ~ || exit
echo "Downloading QGroundControl..."
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage -O QGroundControl.AppImage

# Make the AppImage executable
chmod +x ./QGroundControl.AppImage

# Run QGroundControl
echo "Running QGroundControl..."
./QGroundControl.AppImage
