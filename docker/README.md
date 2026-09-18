# atlas-sim — ambiente ROS 2 Humble + Gazebo Harmonic + ArduPilot SITL + MAVROS

## Início rápido (TL;DR)

Tudo mora em `~/git_projects/atlas_ws`; o ambiente em `atlas_ws/docker/`.

```bash
# --- 1x: construir imagem + compilar workspace ---
cd ~/git_projects/atlas_ws/docker
docker compose build                                                  # imagem atlas-sim:humble
docker compose run --rm dev bash -lc 'atlas-ws-init && atlas-deps && atlas-build'
#   (só repetir atlas-build ao mexer no código; --packages-select <pkg> p/ um só)

# --- toda vez: subir a simulação (Gazebo GUI + SITL + MAVROS) ---
xhost +local:docker                                                   # libera o X p/ o container
cd ~/git_projects/atlas_ws/docker
docker compose run --rm dev ap-sim copter                             # tmux: Gazebo | SITL | MAVROS

# --- noutro terminal: rodar a missão ---
cd ~/git_projects/atlas_ws/docker
docker compose run --rm dev ros2 launch atlas_mission mission.launch.py
```

`ap-sim` abre tmux (`Ctrl-b →` troca painel, `Ctrl-b d` destaca). `vehicle` = `copter|plane|vtol`.
A GUI precisa do `xhost` + GPU (o compose usa `privileged` → `/dev/dri`); sem GPU o render é por
software (lento). Headless (sem janela, p/ CI): `HEADLESS=1 docker compose up -d gazebo sitl mavros`.

**Onde fica cada coisa:** código de missão em `src/atlas_mission/` (nós + `launch/` + `fake_target`);
missão icarus em `src/icarus_tests/` (`ros2 run icarus_tests mission`); launch de simulação em
`src/atlas_bringup/`; ambiente/scripts em `docker/`.

---

A imagem é **só o ambiente**. O workspace (`src/`, `build_docker/`, `install_docker/`)
fica no host e entra por volume, e é compilado com `colcon` lá dentro — o fluxo normal
de ROS 2, igual ao que o `ardupilot_gz` documenta.

```
host: /home/caio/git_projects/atlas_ws        container: /home/ros/atlas_ws
├── src/
│   ├── ardupilot              <- necessário para ardupilot_sitl / ardupilot_gz
│   ├── ardupilot_gazebo       (plugin + iris/zephyr)
│   ├── ardupilot_gz           (launch files ROS 2)
│   ├── SITL_Models            (modelos extra, inclui o quadplane VTOL)
│   ├── mavros_library
│   ├── atlas_mission
│   ├── icarus_tests
│   └── missions
├── build_docker/   install_docker/   log_docker/     <- gerados pelo container
└── build/          install/          log/            <- os teus, do host
```

`build_docker`/`install_docker` existem porque o install space do colcon guarda
caminhos absolutos: no host o workspace é `/home/caio/...` e no container
`/home/ros/atlas_ws`. Partilhar as mesmas pastas quebraria os dois lados. O
`atlas-env` faz source do `install_docker` sozinho em cada shell nova.

## O que a imagem traz

| Componente | Origem |
|---|---|
| ROS 2 Humble | `ros:humble-ros-base` + rviz2, rqt, tf2-tools |
| Gazebo Harmonic + ros_gz | `gz-harmonic`, `libgz-sim8-dev`, `ros-humble-ros-gzharmonic` (repo da OSRF) |
| MAVROS | `mavros`, `mavros-extras`, `mavros-msgs` + datasets GeographicLib |
| Deps do ArduPilot | waf/gcc/ccache, MAVProxy, pymavlink, empy 3.3.4, wxPython |
| `microxrceddsgen` | build do `Micro-XRCE-DDS-Gen` (para compilar com AP_DDS) |
| rosdep | regra `00-gazebo.list` da OSRF, para o par Humble+Harmonic resolver |
| ArduPilot (fallback) | `/opt/ardupilot`, SITL de copter+plane já compilado |

Sobre o fallback: se existir `src/ardupilot` no workspace, é esse que entra no PATH
(`$ARDUPILOT_HOME`). O `/opt/ardupilot` só serve para a imagem já arrancar SITL sem
compilar nada. Para desligá-lo: `--build-arg BUILD_ARDUPILOT=0`.

Humble + Harmonic não é o par padrão do ROS (o padrão do Humble é Fortress). Por isso
o `ros_gz` vem do repositório da OSRF, o `GZ_VERSION=harmonic` já está no ambiente e a
regra de rosdep do Gazebo está instalada — sem isso o `rosdep install` falha a resolver
`ros_gz_sim` nos `package.xml`.

## Setup

O ambiente vive em `atlas_ws/docker/`; o workspace é o próprio `atlas_ws` (montado em
`/home/ros/atlas_ws`). Todos os comandos abaixo correm de dentro de `atlas_ws/docker/`.

```bash
cd atlas_ws/docker
cp .env.example .env                                    # UID=1000 GID=100 já preenchidos
# se o teu id diferir: printf "UID=%s\nGID=%s\n" "$(id -u)" "$(id -g)" >> .env

DOCKER_BUILDKIT=1 docker compose build                  # ou: docker build -t atlas-sim:humble .
```

Primeira entrada no container:

```bash
xhost +local:docker
docker compose run --rm dev

# dentro do container:
ap-check            # o que está e o que falta
atlas-ws-init       # clona no src/ só o que faltar (ardupilot, ardupilot_gz, ...)
atlas-deps          # rosdep install --from-paths src
atlas-build         # colcon build -> build_docker/ install_docker/
```

`atlas-ws-init` não toca no que já existe. Se preferires o caminho oficial:

```bash
cd $ATLAS_WS
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src
```

`atlas-build` aceita os argumentos do colcon:

```bash
atlas-build --packages-select atlas_mission
atlas-build --packages-up-to ardupilot_gz_bringup
```

## Rodar

Um container só, três painéis tmux (Gazebo | SITL | MAVROS):

```bash
docker run -it --rm --name atlas \
  --network=host --ipc=host --privileged \
  --env=DISPLAY=$DISPLAY --env=QT_X11_NO_MITSHM=1 \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume=$HOME/.ssh:/home/ros/.ssh:ro \
  --volume=/home/caio/git_projects/atlas_ws:/home/ros/atlas_ws \
  atlas-sim:humble ap-sim copter
```

Containers separados:

```bash
VEHICLE=vtol docker compose up        # gazebo + sitl + mavros
docker compose run --rm dev           # noutro terminal, para o teu código
```

Manual, 3 terminais (é só o que os scripts fazem):

```bash
ap-gazebo copter     # gz sim -v4 -r iris_runway.sdf
ap-sitl copter       # sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
                     #   + --out=udp:127.0.0.1:14550 (MAVROS) e :14551 (QGC)
ap-mavros            # ros2 launch mavros apm.launch fcu_url:=udp://:14550@
```

| | copter | plane | vtol |
|---|---|---|---|
| Mundo | `iris_runway.sdf` | `zephyr_runway.sdf` | `alti_transition_runway.sdf` |
| Frame | `-f gazebo-iris` | `-f gazebo-zephyr` | param file do Alti Transition |
| Binário | arducopter | arduplane | arduplane |

VTOL sem os scripts:

```bash
gz sim -v4 -r alti_transition_runway.sdf
sim_vehicle.py -v ArduPlane --model JSON --console --map \
  --add-param-file=$ATLAS_WS/src/SITL_Models/Gazebo/config/alti_transition_quad.param
```

No MAVProxy: `mode QLOITER` → `arm throttle` → `takeoff 20`, depois `mode FBWA` ou
`mode AUTO` para a transição para asa fixa.

### Caminho alternativo: launch do ardupilot_gz

Com `ardupilot` e `ardupilot_gz` no `src/` compilados, dá para usar os launch files
oficiais em vez do `ap-gazebo` + `ap-sitl` — sobem Gazebo e SITL juntos:

```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py rviz:=true use_gz_tf:=true
ap-mavros    # MAVROS por cima, se quiseres MAVROS além do DDS
```

Esses launch usam AP_DDS (tópicos `/ap/...`). MAVROS e DDS podem correr ao mesmo
tempo na mesma simulação; são só duas pontes diferentes para o mesmo SITL.

### Portas

| Porta | Quem usa |
|---|---|
| 9002 / 9003 | JSON entre SITL e o plugin do Gazebo |
| 5760 (TCP) | MAVLink direto do SITL (com `NO_MAVPROXY=1`) |
| 14550 (UDP) | MAVProxy → MAVROS |
| 14551 (UDP) | MAVProxy → QGroundControl (roda a QGC no host) |

Multi-veículo: `INSTANCE=1 ap-sitl copter` e `INSTANCE=1 ap-mavros` deslocam tudo em +10.

## Variáveis úteis

| Var | Efeito |
|---|---|
| `HEADLESS=1` | `gz sim -s` e SITL sem `--map/--console` |
| `GUI_ONLY=1` | `gz sim -g`, anexa a um servidor já a correr |
| `RENDER=ogre` / `LIBGL_ALWAYS_SOFTWARE=1` | render sem GPU decente |
| `NO_MAVPROXY=1` | SITL puro; usa `ap-mavros tcp://127.0.0.1:5760` |
| `WIPE=1` | apaga parâmetros salvos (`-w`) |
| `SPEEDUP=5` | acelera a simulação |
| `LOCATION=CMAC` | ponto de partida (`Tools/autotest/locations.txt`) |
| `ATLAS_WS` | caminho do workspace dentro do container |

## Problemas comuns

**`rosdep install` não resolve `ros_gz_sim`** — falta a regra do Gazebo. Já vem na
imagem; se editaste algo, refaz: `sudo wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list && rosdep update`.

**`colcon build` falha em `ardupilot_sitl`** — normalmente é `GZ_VERSION` ou o
`microxrceddsgen` fora do PATH. Ambos estão no ambiente; confirma com `ap-check`.

**Gazebo abre preto / crasha no ogre2** — `RENDER=ogre ap-gazebo copter` ou
`LIBGL_ALWAYS_SOFTWARE=1`. Com NVIDIA, `--gpus all` + runtime da NVIDIA.

**SITL preso em "waiting for connection"** — o Gazebo tem de estar no ar antes, com o
modelo carregado. Sobe `WAIT_GAZEBO` para 15–20 em máquina lenta.

**MAVROS conecta mas não publica** — quase sempre o `fcu_url`. Com MAVProxy,
`udp://:14550@`; sem MAVProxy, `tcp://127.0.0.1:5760`. Testa com `ros2 topic hz /mavros/state`.

**Nós em containers diferentes não se veem** — mesmo `ROS_DOMAIN_ID` e
`network_mode: host` nos dois.

**O host já tinha `build/` e `install/`** — ficam intocados; o container usa
`build_docker/`/`install_docker/`. Vale a pena pôr os dois no `.gitignore`.
