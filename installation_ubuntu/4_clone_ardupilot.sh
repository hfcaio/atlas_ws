#!/bin/bash
# Ensure environment variables are properly loaded
source ~/.bashrc
source ~/.profile

# Update package list
sudo apt-get update

# Install Git and related tools
sudo apt-get install -y git gitk git-gui

# Clone the ArduPilot repository
cd ~ || exit
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git


cd ardupilot || exit

# Install required dependencies
Tools/environment_install/install-prereqs-ubuntu.sh -y


source ~/.profile
# Reload the path
# . ~/.profile

# Install pymavlink and mavproxy
sudo pip install pymavlink mavproxy
sudo rm -rf /var/lib/docker
sudo rm -rf /var/lib/containerd

echo "Setup complete. Log out and log back in to finalize the environment setup."
echo "You can now build ArduPilot using waf. Example for MatekH743 board:"
echo "./waf configure --board MatekH743"
echo "./waf copter"

# Build ArduPilot for MatekH743
./waf configure --board MatekH743
./waf copter

# Start the simulation (Make sure you're in the Tools/autotest directory)
cd ~/ardupilot/Tools/autotest || exit

# Run the simulation with ArduCopter vehicle type
#./sim_vehicle.py --console --map -v ArduCopter -w

# End of script



