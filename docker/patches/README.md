# Patches em repos upstream

O `CLAUDE.md` proíbe edições silenciosas nos repos upstream (`ardupilot`,
`ardupilot_gazebo`, `ardupilot_gz`, `SITL_Models`). Patches **necessários** ficam
aqui, documentados, e podem ser reaplicados se o repo for re-clonado.

## `ardupilot_gz-robot-launch-rviz.patch`

**Alvo:** `src/ardupilot_gz/ardupilot_gz_bringup/launch/robots/robot.launch.py`

**Porquê:** no launch oficial `iris_runway.launch.py`, o `robot_state_publisher`
converte o SDF com o plugin `sdformat_urdf`, que **crasha** ao encontrar os sensores
do ArduPilot (`air_pressure`, `navsat`, `camera`, …). Como esse nó publica o
`robot_description` que o `create` usa para spawnar o iris, o crash impede **tanto o
RViz quanto o spawn no Gazebo**.

**O patch faz:**
1. `create` passa a spawnar do **arquivo** completo (`-file`), com sensores → Gazebo/SITL OK.
2. `robot_state_publisher` recebe o SDF **achatado (`gz sdf -p`) e sem `<sensor>`** →
   parseia a árvore cinemática, não crasha → publica `/robot_description` + `/tf` → RViz OK.

**Só é preciso para o caminho alternativo** `ros2 launch ardupilot_gz_bringup ...`.
O fluxo primário (`ap-gazebo` + `ap-sitl`) **não** usa este launch e não precisa do patch.

**Reaplicar** (após re-clonar ardupilot_gz):
```bash
cd $ATLAS_WS/src/ardupilot_gz
git apply ../../docker/patches/ardupilot_gz-robot-launch-rviz.patch
```

## MAVROS compilado da fonte (Dockerfile)

**Porquê:** em 2026-09 os binários `ros-humble-mavros`, `ros-humble-mavros-extras` e
`ros-humble-libmavconn` sumiram do `packages.ros.org` (buraco de sync do buildfarm; só
`ros-humble-mavros-msgs` e `ros-humble-mavlink` ficaram). Confirmado com
`apt-cache policy ros-humble-mavros` → sem candidato.

**O Dockerfile faz:** em vez do `apt install ros-humble-mavros*`, clona `mavlink/mavros`
(branch `ros2`) e faz `colcon build --merge-install --install-base /opt/mavros_ws/install`.
O `atlas-env` sourceia esse overlay; o `atlas-deps` pula as chaves `mavros mavros_msgs
mavros_extras libmavconn` no rosdep (vêm do overlay, não do apt).

**Se o repo voltar a ter mavros:** dá para reverter para o `apt install` e apagar o passo de
build (mais rápido). Enquanto não voltar, a fonte é o caminho seguro.
