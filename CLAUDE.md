# CLAUDE.md

Contexto do workspace ROS 2 do ATLAS. Lê isto antes de mexer em qualquer coisa.

## O que é este repositório

Workspace colcon (ROS 2 **Humble**) com o código de missão e a biblioteca MAVROS do
ATLAS, mais os pacotes de simulação do ArduPilot. O alvo é drone ArduPilot
(**copter**, **plane** e **quadplane VTOL**) testado em SITL + Gazebo **Harmonic**,
com **MAVROS** como ponte MAVLink↔ROS 2.

Todo o desenvolvimento e execução acontecem **dentro do container** `atlas-sim:humble`
(ver secção "Ambiente"). Não assumas que o host tem ROS, Gazebo ou ArduPilot.

## Layout

```
atlas_ws/
├── CLAUDE.md               # este guia
├── docker/                 # AMBIENTE: Dockerfile, docker-compose, scripts ap-*/atlas-*
│   ├── scripts/            # ap-check ap-gazebo ap-sitl ap-mavros ap-sim atlas-*
│   ├── patches/            # patches documentados p/ repos upstream (ver patches/README.md)
│   └── .env(.example)      # UID/GID, ATLAS_WS_HOST, VEHICLE, ...
├── src/
│   ├── atlas_mission/      # nossos nós de missão (Python) + launch/ + fake_target
│   ├── atlas_bringup/      # nossos launch de simulação (Gazebo+SITL+MAVROS)
│   ├── mavros_library/     # nossa lib: commands/, sequences/, utils/, config/
│   ├── icarus_tests/       # missão icarus (entry point `mission`) + test_0..8
│   ├── ardupilot/          # UPSTREAM - não editar (traz Tools/ros2/ardupilot_sitl)
│   ├── ardupilot_gazebo/   # UPSTREAM - não editar (plugin + modelos iris/zephyr) [branch ros2]
│   ├── ardupilot_gz/       # UPSTREAM - não editar (launch files ROS 2)
│   └── SITL_Models/        # UPSTREAM - não editar (modelos extra, VTOL Alti Transition)
├── build_docker/ install_docker/ log_docker/   # gerados DENTRO do container
└── build/ install/ log/                        # gerados no host (não usar no container)
```

**Regra dura:** só editamos `atlas_mission`, `atlas_bringup`, `mavros_library` e
`icarus_tests`. Os quatro repos upstream são dependências — se algo não funciona neles,
a solução é configuração/versão do nosso lado ou um **patch documentado em
`docker/patches/`**, nunca uma edição silenciosa no `src/`.

## Ambiente (container)

Imagem `atlas-sim:humble`. Traz ROS 2 Humble, Gazebo Harmonic (`gz sim`),
`ros-humble-ros-gzharmonic`, MAVROS + mavros_extras + datasets GeographicLib,
dependências de build do ArduPilot (waf, MAVProxy, pymavlink, empy 3.3.4),
`microxrceddsgen` e a regra de rosdep do Gazebo (`00-gazebo.list`).

Factos que mudam decisões:
- Humble + Harmonic **não é o par padrão** do ROS (o padrão do Humble é Fortress).
  `ros_gz` vem do repo da OSRF e `GZ_VERSION=harmonic` tem de estar no ambiente.
  Se uma sugestão implicar Fortress ou `gazebo_ros` (Gazebo Classic), está errada.
- O workspace do container compila em `build_docker/` / `install_docker/`, **nunca**
  em `build/` / `install/` (essas são do host; o install space do colcon grava
  caminhos absolutos e os dois ambientes têm prefixos diferentes).
- `$ATLAS_WS` = `/home/ros/atlas_ws`. `$ARDUPILOT_HOME` aponta para
  `src/ardupilot` se existir, senão `/opt/ardupilot` (cópia da imagem).
- Cada shell nova já faz source de ROS + `install_docker` via `atlas-env`.

## Comandos

**Fluxo de uso (resumo — detalhe em `docker/README.md`):**
```bash
cd ~/git_projects/atlas_ws/docker
docker compose build                                                  # 1x: imagem
docker compose run --rm dev bash -lc 'atlas-ws-init && atlas-deps && atlas-build'   # 1x: workspace
xhost +local:docker && docker compose run --rm dev ap-sim copter      # sobe Gazebo+SITL+MAVROS
docker compose run --rm dev ros2 launch atlas_mission mission.launch.py  # noutro terminal: missão
```

```bash
ap-check                                   # diagnóstico do ambiente e do workspace
atlas-deps                                 # rosdep install --from-paths src
atlas-build                                # colcon build -> build_docker/install_docker
atlas-build --packages-select atlas_mission
atlas-build --packages-up-to ardupilot_gz_bringup

ap-gazebo copter|plane|vtol                # gz sim com o mundo certo
ap-sitl   copter|plane|vtol                # sim_vehicle.py ligado ao modelo do Gazebo
ap-mavros [fcu_url]                        # ros2 launch mavros apm.launch
ap-sim    copter|plane|vtol                # os três acima em painéis tmux
```

Alternativa oficial ao par `ap-gazebo` + `ap-sitl` (sobe Gazebo e SITL juntos, com
AP_DDS e tópicos `/ap/...`):

```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py rviz:=true use_gz_tf:=true
```

Correr um nó nosso:

```bash
atlas-build --packages-select atlas_mission
source $ATLAS_WS/install_docker/setup.bash   # só se a shell for anterior ao build
ros2 run atlas_mission <nó>
```

## Veículos

| | copter | plane | vtol |
|---|---|---|---|
| Mundo | `iris_runway.sdf` | `zephyr_runway.sdf` | `alti_transition_runway.sdf` |
| Frame SITL | `-f gazebo-iris` | `-f gazebo-zephyr` | param file do Alti Transition |
| Binário | arducopter | arduplane | arduplane |

O VTOL não usa `-f`: usa `--model JSON` + `--add-param-file=$ATLAS_WS/src/SITL_Models/Gazebo/config/alti_transition_quad.param`.
Transição no MAVProxy: `mode QLOITER` → `arm throttle` → `takeoff 20` → `mode FBWA`/`AUTO`.

## Portas e ligação

| Porta | Uso |
|---|---|
| 9002 / 9003 | JSON entre SITL e o plugin do Gazebo |
| 5760 TCP | MAVLink direto do SITL (quando corre com `NO_MAVPROXY=1`) |
| 14550 UDP | MAVProxy → MAVROS (`fcu_url=udp://:14550@`) |
| 14551 UDP | MAVProxy → QGroundControl |

Multi-veículo: `INSTANCE=n` desloca tudo em `+10*n`. Todos os containers usam
`network_mode: host`, logo `127.0.0.1` é comum ao host e aos containers, e o
`ROS_DOMAIN_ID` (default 0) tem de ser igual entre eles.

## Convenções de código

- Python 3.10, `rclpy`. Pacotes nossos são `ament_python`.
- MAVROS: usar a `mavros_library` em vez de falar com os tópicos/serviços crus.
  Se algo lá não existe, estender a lib — não duplicar lógica no nó de missão.
- Subscrições a tópicos MAVROS de sensor/estado (`/mavros/state`,
  `/mavros/local_position/pose`, …) precisam de QoS **BEST_EFFORT / VOLATILE**;
  com QoS default não chega nada. É o erro mais comum aqui.
- Serviços MAVROS retornam `success`/`mode_sent` — verificar sempre, e tolerar
  falha na primeira tentativa (o FCU pode ainda não estar pronto).
- Nada de `time.sleep()` bloqueante dentro de callbacks; usar timers do rclpy.
- Comandos que armam ou movem o veículo ficam atrás de uma máquina de estados
  explícita, nunca disparados direto num callback de sensor.

## Ao trabalhar aqui

- Antes de propor mudanças de build, corre `ap-check` e olha o que já existe.
- Ao mexer num pacote, recompila só ele (`--packages-select`), não o workspace todo:
  `ardupilot` e `ardupilot_gazebo` demoram muito.
- Não faças `colcon build` sem `--build-base build_docker --install-base install_docker`
  (ou seja: usa `atlas-build`).
- Testar mudanças de missão em SITL antes de qualquer coisa; `SPEEDUP=5` acelera.
- Não commitar `build*/`, `install*/`, `log*/`, `*.tlog`, `*.BIN`, `logs/`.

## TODO para o dono do repo preencher

- [ ] Nós existentes em `atlas_mission` e o que cada um faz
- [ ] API pública da `mavros_library` (classes/funções principais)
- [ ] Como correr os `icarus_tests` (pytest? `colcon test`?)
- [ ] Formato das missões em `missions/`
- [ ] Parâmetros/airframe reais do drone alvo
