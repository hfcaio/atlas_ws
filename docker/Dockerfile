# syntax=docker/dockerfile:1
#
# atlas-sim : AMBIENTE para ROS 2 Humble + Gazebo Harmonic + ArduPilot SITL + MAVROS
#
# A imagem NAO carrega o teu workspace. Ela traz apenas o ambiente:
#   ROS 2 Humble, Gazebo Harmonic, ros_gz, MAVROS(+extras), deps de build do
#   ArduPilot, MAVProxy, microxrceddsgen e as regras de rosdep para o par
#   Humble+Harmonic.
# O workspace (src/ build/ install/) vive no volume montado e e compilado com
# colcon, como manda o fluxo normal de ROS 2.
#
# Build:
#   DOCKER_BUILDKIT=1 docker build --network=host -t atlas-sim:humble .
#
ARG ROS_DISTRO=humble

# ---------------------------------------------------------------------------
# Stage 1: microxrceddsgen (necessario para compilar o ArduPilot com AP_DDS,
#          que e o que o ardupilot_sitl/ardupilot_gz usam)
# ---------------------------------------------------------------------------
FROM eclipse-temurin:19-jdk-jammy AS dds-gen-builder

RUN apt-get update && apt-get install --no-install-recommends -y git \
    && rm -rf /var/lib/apt/lists/*

RUN git clone -b master --recurse-submodules --depth 1 \
      https://github.com/ArduPilot/Micro-XRCE-DDS-Gen.git dds-gen \
 && cd dds-gen && ./gradlew assemble

# ---------------------------------------------------------------------------
# Stage 2: ambiente
# ---------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS main

ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8 TZ=Europe/Lisbon

# --- pacotes de sistema / toolchain -----------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      sudo curl wget gnupg2 lsb-release ca-certificates software-properties-common \
      git vim nano tmux screen less htop iputils-ping net-tools socat unzip zip rsync \
      build-essential ccache cmake ninja-build g++ gdb gawk make pkg-config \
      libtool libtool-bin libxml2-dev libxml2-utils libxslt1-dev default-jre \
      python3-dev python3-pip python3-setuptools python3-wheel python-is-python3 \
      python3-numpy python3-pyparsing python3-psutil python3-serial \
      python3-matplotlib python3-lxml python3-opencv python3-wxgtk4.0 \
      rapidjson-dev libopencv-dev \
      libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
      x11-apps mesa-utils libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# --- Gazebo Harmonic + ros_gz (repo da OSRF; Humble+Harmonic nao e o par padrao)
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y --no-install-recommends \
      gz-harmonic libgz-sim8-dev ros-${ROS_DISTRO}-ros-gzharmonic \
    && rm -rf /var/lib/apt/lists/*

ENV GZ_VERSION=harmonic

# --- ferramentas ROS + deps do MAVROS ----------------------------------------
# NOTA (2026-09): os binarios ros-humble-mavros / -extras / libmavconn sumiram do
# packages.ros.org (buraco de sync do buildfarm; so mavros-msgs e mavlink ficaram).
# Por isso o MAVROS e' compilado da fonte no passo seguinte -> imagem self-contained,
# imune ao buraco. Ver docker/patches/README.md.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-mavlink \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-rqt-common-plugins \
      ros-${ROS_DISTRO}-tf2-tools \
      ros-${ROS_DISTRO}-topic-tools \
      ros-${ROS_DISTRO}-launch-pytest \
      ros-dev-tools geographiclib-tools libgeographic-dev \
    && rm -rf /var/lib/apt/lists/*

# --- MAVROS da fonte (mavros + mavros_msgs + libmavconn + mavros_extras) ------
# Tag fixa (nao a branch ros2): o HEAD usa enums de MAVLink mais novos que o
# ros-humble-mavlink do apt (ex. MAV_AUTOPILOT::FLIX). A 2.15.1 bate com o
# ros-humble-mavros-msgs do apt e compila contra o mavlink disponivel.
ARG MAVROS_REF=2.15.1
ARG WAF_JOBS=4
ENV MAKEFLAGS="-j${WAF_JOBS}"
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && mkdir -p /opt/mavros_ws/src && cd /opt/mavros_ws \
 && git clone -b ${MAVROS_REF} --depth 1 https://github.com/mavlink/mavros.git src/mavros \
 && apt-get update \
 && rosdep update --rosdistro ${ROS_DISTRO} \
 && rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
      --skip-keys "mavros mavros_msgs mavros_extras libmavconn" \
 && MAKEFLAGS="-j${WAF_JOBS}" colcon build --merge-install --install-base /opt/mavros_ws/install \
      --parallel-workers "${WAF_JOBS}" \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && geographiclib-get-geoids egm96-5 \
 && rm -rf build log src /var/lib/apt/lists/*

# --- rosdep: regras do Gazebo para o par nao-default -------------------------
RUN wget -q https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list \
      -O /etc/ros/rosdep/sources.list.d/00-gazebo.list \
 && rosdep update --rosdistro ${ROS_DISTRO} || true

# --- deps Python do ArduPilot / MAVProxy + da mavros_library (Dinis) ---------
# geographiclib e pyproj sao usados por mavros_library/utils (nao declarados no
# package.xml da lib) -> entram aqui para o nó da missao importar.
RUN python3 -m pip install --no-cache-dir -U \
      pip "empy==3.3.4" future lxml pexpect ptyprocess \
      pymavlink MAVProxy pyserial intelhex dronecan \
      pyelftools tabulate junitparser flake8 pytest \
      geographiclib pyproj

# --- microxrceddsgen (AP_DDS) ------------------------------------------------
COPY --from=dds-gen-builder /dds-gen/scripts /dds-gen/scripts
COPY --from=dds-gen-builder /dds-gen/share   /dds-gen/share
ENV PATH="/dds-gen/scripts:/usr/lib/ccache:${PATH}" \
    DISABLE_MAVNATIVE=True \
    BUILDLOGS=/tmp/buildlogs \
    PYTHONUNBUFFERED=1

# --- utilizador --------------------------------------------------------------
ARG USERNAME=ros
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} ${USERNAME} 2>/dev/null || true \
 && useradd -m -o -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && usermod -aG dialout,video,plugdev ${USERNAME} \
 && mkdir -p /opt/ardupilot && chown ${UID}:${GID} /opt/ardupilot \
 && mkdir -p /home/${USERNAME}/.cache/ccache \
 && chown -R ${UID}:${GID} /home/${USERNAME}/.cache
# ^ o dir do ccache e' pre-criado com dono do user para o volume nomeado
#   (ap_ccache) herdar a permissao ao inicializar; senao ccache da' Permission denied.

# --- ArduPilot de fallback ---------------------------------------------------
# So e usado se NAO existir src/ardupilot no teu workspace. Serve para a imagem
# ja arrancar SITL sem compilar nada. Se puseres o ardupilot no workspace (que e
# o que o ardupilot_gz exige), esse passa a ter prioridade no PATH.
ARG BUILD_ARDUPILOT=1
ARG ARDUPILOT_REF=master
ARG WAF_JOBS=4

USER ${USERNAME}
ENV HOME=/home/${USERNAME}

RUN if [ "${BUILD_ARDUPILOT}" = "1" ]; then \
      git clone --depth 1 --recurse-submodules --shallow-submodules \
        -b ${ARDUPILOT_REF} https://github.com/ArduPilot/ardupilot.git /opt/ardupilot \
      && cd /opt/ardupilot \
      && ./waf configure --board sitl --enable-dds \
      && ./waf copter plane -j${WAF_JOBS} ; \
    fi

# --- scripts + entrypoint ----------------------------------------------------
USER root
COPY scripts/ /usr/local/bin/
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/ap-* /usr/local/bin/atlas-* /usr/local/bin/entrypoint.sh
USER ${USERNAME}

# workspace montado por volume; ATLAS_WS pode ser mudado no run/compose
ENV ATLAS_WS=/home/ros/atlas_ws \
    ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    QT_X11_NO_MITSHM=1

RUN mkdir -p ${ATLAS_WS}/src \
 && echo 'source /usr/local/bin/atlas-env' >> ${HOME}/.bashrc \
 && rosdep update --rosdistro ${ROS_DISTRO} || true

WORKDIR /home/${USERNAME}/atlas_ws
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
