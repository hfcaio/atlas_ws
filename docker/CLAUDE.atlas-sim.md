# CLAUDE.md — atlas-sim

Repositório do **ambiente Docker** para simulação ArduPilot do ATLAS. Aqui não vive
código de drone: vive a imagem e os scripts que a operam.

## Princípio

A imagem é **só ambiente**. O workspace ROS 2 (`atlas_ws`) vem de fora, por volume, e
é compilado com colcon dentro do container. Se uma alteração fizer a imagem carregar
pacotes ROS do utilizador, ou duplicar `ardupilot_gazebo`/`SITL_Models` que já estão
no `src/` do workspace, está errada — quebra o overlay do colcon.

## Ficheiros

```
Dockerfile           # 2 stages: microxrceddsgen (temurin) + ambiente (ros:humble-ros-base)
entrypoint.sh        # faz source do atlas-env e exec "$@"
docker-compose.yml   # serviços gazebo / sitl / mavros / dev, todos network_mode: host
scripts/
  atlas-env          # SOURCED: ROS + overlay install_docker + PATH do ArduPilot + fallback GZ_*
  atlas-ws-init      # clona no src/ só o que faltar (ardupilot, ardupilot_gz, ...)
  atlas-deps         # rosdep install
  atlas-build        # colcon build --build-base build_docker --install-base install_docker
  ap-gazebo          # gz sim <mundo do veículo>
  ap-sitl            # sim_vehicle.py --model JSON ligado ao Gazebo
  ap-mavros          # ros2 launch mavros apm.launch
  ap-sim             # os três em tmux
  ap-check           # diagnóstico
```

## Invariantes que não se mexem sem motivo forte

- **Humble + Harmonic**: `gz-harmonic`, `libgz-sim8-dev` e `ros-humble-ros-gzharmonic`
  vêm do repo `packages.osrfoundation.org` (não do packages.ros.org), e
  `GZ_VERSION=harmonic` tem de estar no ambiente. Nada de Gazebo Classic / `gazebo_ros`.
- A regra de rosdep `00-gazebo.list` da OSRF é obrigatória: sem ela o
  `rosdep install` não resolve `ros_gz_sim` nos `package.xml` do `ardupilot_gz`.
- `microxrceddsgen` no PATH é o que permite compilar o ArduPilot com AP_DDS
  (`--enable-dds`), que o `ardupilot_sitl` exige.
- Utilizador `ros`, UID/GID por build-arg, home `/home/ros` — o workspace monta em
  `/home/ros/atlas_ws` e há comandos antigos da equipa que assumem esse caminho.
- Compilação do workspace vai para `build_docker`/`install_docker`, nunca para
  `build`/`install` (essas são do host; o install space guarda caminhos absolutos).
- `/opt/ardupilot` é apenas **fallback**: se existir `src/ardupilot`, é esse que entra
  no PATH via `$ARDUPILOT_HOME`. Desligável com `--build-arg BUILD_ARDUPILOT=0`.

## Build e teste

```bash
DOCKER_BUILDKIT=1 docker build --network=host --build-arg WAF_JOBS=$(nproc) -t atlas-sim:humble .
docker run --rm atlas-sim:humble ap-check
docker compose run --rm dev          # shell para atlas-ws-init / atlas-deps / atlas-build
VEHICLE=vtol docker compose up       # gazebo + sitl + mavros
```

O build é caro (compila o SITL de copter e plane). Antes de propor mudanças no
Dockerfile, verifica se dá para resolver num script — mexer em camadas iniciais
invalida tudo o que vem a seguir.

## Ao mexer nos scripts

- Bash com `set -euo pipefail`, exceto o `atlas-env`, que é *sourced* e não pode matar
  a shell do utilizador.
- Cada script aceita o veículo como `$1` e cai para `$VEHICLE` e depois `copter`.
- Mensagens de erro dizem o comando seguinte a correr (ex.: "corre atlas-ws-init").
- Qualquer script novo entra em `scripts/`, é copiado para `/usr/local/bin` e tem de
  ser coberto pelo `chmod +x` do Dockerfile (globs `ap-*` e `atlas-*`).
