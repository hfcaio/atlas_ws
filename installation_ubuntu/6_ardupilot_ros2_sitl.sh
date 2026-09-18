#!/bin/bash

# Ensure rosdep is installed
sudo apt install -y python3-rosdep
sudo apt install python3-vcstool


sudo apt update

# Create workspace directory
mkdir -p ~/ardu_ws/src
cd ~/ardu_ws

# Clone required repositories
vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src

# Update dependencies
sudo apt update

rosdep update

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

rm -rf ~/.gradle/caches
rm -rf ~/.gradle/daemon
rm -rf ~/.gradle/wrapper
rm -rf thirdparty/IDL-Parser/.gradle
rm -rf thirdparty/IDL-Test-Types/.gradle


# Install MicroXRCEDDSGen build dependency
sudo apt install -y default-jre
cd ~/ardu_ws
git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen
./gradlew assemble

# Add MicroXRCEDDSGen to PATH
echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc
source ~/.bashrc

# Test microxrceddsgen installation
microxrceddsgen -help

# Warning about FastDDS or FastDDSGen
echo "⚠️ If you have installed FastDDS or FastDDSGen globally, consider removing it to avoid conflicts."

# Install colcon if not already installed
sudo apt install -y python3-colcon-common-extensions

# Build the workspace
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_dds_tests --parallel-workers 10 || true
colcon build --packages-up-to ardupilot_dds_tests --parallel-workers 10 --event-handlers=console_cohesion+ || true


# If build fails, run in verbose mode
echo "If the build fails, re-run with verbose mode:"
echo "colcon build --packages-up-to ardupilot_dds_tests --event-handlers=console_cohesion+"

# Test ArduPilot ROS 2 installation
cd ~/ardu_ws
source ./install/setup.bash
colcon test --executor sequential --parallel-workers 0 --base-paths src/ardupilot --event-handlers=console_cohesion+ || true
colcon test-result --all --verbose

echo "🎉 ArduPilot ROS 2 SITL installation complete!"
######################ros with sitl######################


source /opt/ros/humble/setup.bash
cd ~/ardu_ws/
colcon build --packages-up-to ardupilot_sitl --allow-overriding ardupilot_msgs ardupilot_sitl|| true
source install/setup.bash





