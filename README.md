# atlas_ws — workspace ROS 2 do ATLAS

Workspace colcon (ROS 2 **Humble**) com o código de missão e a biblioteca MAVROS do ATLAS.
O alvo é drone ArduPilot (**copter**, **plane**, **quadplane VTOL**) testado em SITL + Gazebo
**Harmonic**, com **MAVROS** como ponte MAVLink↔ROS 2.

Todo o desenvolvimento e execução acontecem **dentro do container** `atlas-sim:humble`.
O host não precisa de ROS, Gazebo nem ArduPilot — só Docker.

> Detalhe do ambiente e dos comandos: [`docker/README.md`](docker/README.md).
> Contexto para trabalhar no código: [`CLAUDE.md`](CLAUDE.md).
> Guia de uso da missão Icarus: [`docs/index.html`](docs/index.html).

---

## Setup (primeira vez)

```bash
# 1. clonar este repositório COM os submódulos (mavros_library e icarus_tests)
git clone --recurse-submodules git@github.com:hfcaio/atlas_ws.git
cd atlas_ws/docker

# (se já clonaste sem --recurse-submodules:)
#   git submodule update --init --recursive

# 2. configurar o .env (UID/GID/paths) — ver docker/.env.example
cp .env.example .env    # e edita se necessário

# 3. construir a imagem do container
docker compose build

# 4. trazer os repos upstream e compilar o workspace (1x)
docker compose run --rm dev bash -lc 'atlas-ws-init && atlas-deps && atlas-build'
```

O passo `atlas-ws-init` **clona as dependências upstream** (ArduPilot, ardupilot_gazebo,
ardupilot_gz, SITL_Models) para dentro de `src/`. Estes repos **não estão versionados aqui**
(são ~5 GB e têm o seu próprio git) — por isso o setup tem de os trazer.

## Correr

```bash
cd atlas_ws/docker

# subir a simulação (Gazebo + SITL + MAVROS) em painéis tmux
xhost +local:docker && docker compose run --rm dev ap-sim copter

# noutro terminal: lançar a missão
docker compose run --rm dev ros2 launch atlas_mission mission.launch.py
```

Ao mexer no código, recompila só o pacote afetado:

```bash
docker compose run --rm dev atlas-build --packages-select icarus_tests
```

## Layout

```
atlas_ws/
├── CLAUDE.md            # contexto do projeto (ler antes de mexer)
├── README.md           # este ficheiro
├── docs/               # guia de uso da missão (index.html)
├── docker/             # AMBIENTE: Dockerfile, compose, scripts ap-*/atlas-*
└── src/
    ├── atlas_mission/   # nós de missão + launch + fake_target   (nosso)
    ├── atlas_bringup/   # launch de simulação                    (nosso)
    ├── mavros_library/  # lib MAVROS: commands/ sequences/ utils/ (SUBMÓDULO)
    ├── icarus_tests/    # missão icarus (entry point `mission`)   (SUBMÓDULO)
    │
    ├── ardupilot/       # UPSTREAM — clonado por atlas-ws-init, não versionado
    ├── ardupilot_gazebo/# UPSTREAM — idem
    ├── ardupilot_gz/    # UPSTREAM — idem
    └── SITL_Models/     # UPSTREAM — idem
```

**Submódulos:** `mavros_library` e `icarus_tests` são repositórios próprios
(org `AeroTec-ATLAS`) referenciados como submódulos. Para atualizar para a versão mais
recente de cada um: `git submodule update --remote src/mavros_library`.

**Regra:** só editamos `atlas_mission`, `atlas_bringup`, `mavros_library` e `icarus_tests`.
Os quatro repos upstream são dependências — correções entram como configuração do nosso lado
ou como patch documentado em `docker/patches/`, nunca edição direta.
