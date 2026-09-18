# Created By Sam
# 20-03-2025

# Use ROS Humble desktop as the base image
FROM osrf/ros:humble-desktop

# Update and install necessary packages
RUN apt-get update && apt-get install -y \
    nano vim git curl lsb-release gnupg sudo python3-colcon-common-extensions \
    # ros-humble-gazebo-* \
    # ros-humble-cartographer ros-humble-cartographer-ros \
    # ros-humble-navigation2 ros-humble-nav2-bringup \
    x11-apps mesa-utils gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev \
    gcc-arm-none-eabi default-jre gitk git-gui \
    && rm -rf /var/lib/apt/lists/*

# Install GUI-related dependencies
RUN apt-get update && apt-get install -y \
    libxcb1-dev libxcb-keysyms1-dev libxcb-image0-dev libxcb-shm0-dev \
    libxcb-icccm4-dev libxcb-shape0-dev libxcb-xfixes0-dev \
    libxcb-render-util0-dev libxcb-randr0-dev libxcb-xinerama0-dev \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Add user to dialout group for serial port access
RUN usermod -a -G dialout $USERNAME

# Switch to the new user
USER $USERNAME
WORKDIR /home/$USERNAME

# # Install TurtleBot3 Packages
# ENV ROS_WS=/home/$USERNAME/turtlebot3_ws
# RUN mkdir -p $ROS_WS/src && cd $ROS_WS/src/ \
#     && git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git \
#     && git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git \
#     && git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git \
#     && git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# WORKDIR $ROS_WS
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --parallel-workers 2"

# # Environment Configuration
# RUN echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc \
#     && echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc \
#     && echo 'if [ -f /usr/share/gazebo/setup.sh ]; then source /usr/share/gazebo/setup.sh; fi' >> ~/.bashrc \
#     && echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc

# Install ArduPilot
ENV ARDUPILOT_DIR=/home/$USERNAME/ardupilot
RUN git clone https://github.com/ArduPilot/ardupilot.git $ARDUPILOT_DIR \
    && cd $ARDUPILOT_DIR \
    && git submodule update --init --recursive

RUN /bin/bash -c "cd $ARDUPILOT_DIR && USER=$USERNAME Tools/environment_install/install-prereqs-ubuntu.sh -y"

# Install the arm-none-eabi toolchain
RUN echo 'export PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH' >> ~/.bashrc \
    && echo 'export PATH=/home/ros/ardupilot/Tools/autotest:$PATH' >> ~/.bashrc \
    && echo 'export PATH=/usr/lib/ccache:$PATH' >> ~/.bashrc

# Build ArduPilot
RUN cd ~/ardupilot \
    && ./waf distclean \
    && ./waf configure --board MatekF405-Wing \
    && ./waf copter

USER root
RUN apt-get update && apt-get install -y fuse libfuse2 && rm -rf /var/lib/apt/lists/*
#RUN apt-get update && apt-get install -y fuse

USER $USERNAME

# Download and install QGroundControl
WORKDIR /home/$USERNAME
RUN curl -L -o QGroundControl.AppImage https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage \
    && chmod +x QGroundControl.AppImage

# Set environment variables to run QGroundControl
RUN echo 'export DISPLAY=:1' >> ~/.bashrc

# Install pymavlink and MAVProxy
RUN pip install --upgrade pip setuptools
RUN echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc

# Setup ArduPilot ROS 2 workspace
RUN mkdir -p ~/ardu_ws/src \
    && cd ~/ardu_ws \
    && vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src \
    && sudo apt update \
    && rosdep update \
    && /bin/bash -c "source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src -r -y" 


RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-skip ardupilot_dds_tests ardupilot_msgs ardupilot_sitl --parallel-workers 8"

RUN colcon build --packages-up-to ardupilot_dds_tests --parallel-workers 8 || true

# Clone and build Micro-XRCE-DDS-Gen
RUN cd ~/ardu_ws \
    && git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git \
    && cd Micro-XRCE-DDS-Gen \
    && ./gradlew assemble \
    && echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc

    # gazebo classic
# Install necessary tools for Gazebo Harmonic
RUN sudo apt-get update && sudo apt-get install -y curl lsb-release gnupg \
    && sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list \
    && sudo apt-get update && sudo apt-get install -y gz-harmonic gazebo-common \
    && sudo rm -rf /var/lib/apt/lists/*

     # gazebo garden
    # Install necessary tools for Gazebo Garden
# RUN apt-get update && apt-get install -y lsb-release curl gnupg \
#     && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
#     && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
#     && apt-get update && apt-get install -y gz-garden \
#     && rm -rf /var/lib/apt/lists/*

# Set the Gazebo version to harmonic
RUN echo 'export GZ_VERSION=harmonic' >> ~/.bashrc


# Import gz repos — continua mesmo se algum falhar por DNS
RUN cd ~/ardu_ws \
    && vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src || true

# Clonar manualmente os repos que o vcs costuma falhar por DNS
RUN cd ~/ardu_ws/src \
    && ([ -d ardupilot_gazebo/.git ] || git clone https://github.com/ArduPilot/ardupilot_gazebo.git ardupilot_gazebo) \
    && ([ -d ros_gz/.git ]          || git clone -b humble https://github.com/gazebosim/ros_gz.git ros_gz)
    


## repeated from above
#Add Gazebo APT sources
RUN sudo apt-get update && sudo apt-get install -y wget \
    && sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list \
    && sudo apt-get update

# Add Gazebo sources to rosdep for the non-default pairing of ROS 2 Humble and Gazebo Harmonic

RUN sudo wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list \
    -O /etc/ros/rosdep/sources.list.d/00-gazebo.list \
    && rosdep update

# install mavros
RUN sudo apt install -y ros-humble-mavros 

# Install dependencies for the workspace
RUN cd /home/$USERNAME/ardu_ws \
    && /bin/bash -c "source /opt/ros/humble/setup.bash && sudo apt update && rosdep update && rosdep install --from-paths src --ignore-src -y"

RUN cd ~/ardu_ws/src \
    && git clone https://github.com/ArduPilot/ardupilot_ros.git
    
RUN cd ~/ardu_ws \
    && /bin/bash -c "source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src -y"

RUN cd ~/ardu_ws \
    && /bin/bash -c "source ./install/setup.bash && colcon build --packages-up-to ardupilot_ros ardupilot_gz_bringup --parallel-workers 8 || true"


# Copy the entrypoint and bashrc scripts
COPY --chmod=755 entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/bashrc_custom
RUN cat /home/${USERNAME}/bashrc_custom >> /home/${USERNAME}/.bashrc && rm /home/${USERNAME}/bashrc_custom

# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["/bin/bash"]
#CMD ["/home/ros/QGroundControl.AppImage"]
