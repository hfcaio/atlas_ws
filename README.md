# atlas_ws — ATLAS ROS 2 Workspace

This is the ROS 2 (**Humble**) workspace for **ATLAS**, IST's drone and avionics team. Everything lives here: mission code, the MAVROS library, competition and exercise missions, and the simulation environment.

All development runs **inside a Docker container** (`atlas-sim:humble`). The host only needs Docker — ROS, Gazebo and ArduPilot come with the image.

## How code reaches the drone

Our ROS 2 nodes never talk MAVLink directly. They use `mavros_library`, which wraps MAVROS services and topics, and MAVROS talks to ArduPilot:

```
mission node (atlas_mission / icarus_tests)
    → mavros_library (commands + sequences: arming, modes, missions, geofence)
        → MAVROS (ROS 2 ↔ MAVLink bridge)
            → ArduPilot (SITL on the PC, or the flight controller on the drone)
                ↔ Gazebo Harmonic (simulation only: physics and sensors)
```

Missions made in a Ground Station (QGroundControl / Mission Planner) go straight into ArduPilot, without going through ROS.

## Vehicles

| Vehicle | ArduPilot | Gazebo world | Used for |
|---------|-----------|--------------|----------|
| Multirotor | ArduCopter | `iris_runway.sdf` | Icarus mission, `atlas_mission`, multi-mission tests |
| Fixed wing | ArduPlane | `zephyr_runway.sdf` (or SITL without Gazebo) | fixed-wing missions |
| Quadplane VTOL | ArduPlane | `alti_transition_runway.sdf` | transition tests |

<!-- TODO: real hardware table (companion computer, flight controller, link, radio, cameras). -->

## Packages

| Package | What it does |
|---------|-------------|
| `src/mavros_library/` | ATLAS MAVROS library: `commands/` (arming, home, mission, geofence), `sequences/` (init, wait for connection), `utils/`. **Submodule.** |
| `src/icarus_tests/` | Icarus mission (`ros2 run icarus_tests mission`) and tests `test_0` … `test_8` (sensors, modes, fence, waypoints, RTL...). **Submodule.** |
| `src/atlas_mission/` | Mission nodes (`missao1/2/3`), `fake_target`, and the `mission.launch.py` / `mission_sim.launch.py` launch files. |
| `src/atlas_bringup/` | `sim.launch.py`: starts Gazebo + SITL + MAVROS from a ROS 2 launch file. |
| `src/ardupilot/`, `ardupilot_gazebo/`, `ardupilot_gz/`, `SITL_Models/` | **Upstream.** Cloned by `atlas-ws-init`, not versioned here. |
| `docker/` | The environment: `Dockerfile`, `docker-compose.yml` and the `ap-*` / `atlas-*` scripts. |
| `docs/` | HTML guides (`index.html` = Icarus) and `multi_missoes/`. |

## First-time setup

### Clone

```bash
git clone --recurse-submodules git@github.com:hfcaio/atlas_ws.git ~/git_projects/atlas_ws
```

(If you cloned without `--recurse-submodules`: `git submodule update --init --recursive`.)

### Configure and build the image

```bash
cd ~/git_projects/atlas_ws/docker
cp .env.example .env          # check UID/GID (id -u / id -g) and ATLAS_WS_HOST
docker compose build          # image atlas-sim:humble
```

### Fetch dependencies and build

```bash
docker compose run --rm dev bash -lc 'atlas-ws-init && atlas-deps && atlas-build'
```

- `atlas-ws-init` clones whatever is missing into `src/` (ArduPilot, ardupilot_gazebo, ardupilot_gz, SITL_Models). About 5 GB.
- `atlas-deps` runs `rosdep install`.
- `atlas-build` runs `colcon build` into `build_docker/` and `install_docker/`. The host's `build/` and `install/` are left untouched.

> After changing code, build **only the package you changed**. A bare `atlas-build` also rebuilds ArduPilot and the Gazebo plugin, which takes a long time.
> ```bash
> docker compose run --rm dev atlas-build --packages-select atlas_mission
> ```

### No `.bashrc` needed

Inside the container, every new shell already sources ROS, the workspace overlay and ArduPilot (the `atlas-env` script). All containers use `ROS_DOMAIN_ID=0` (set in `docker/.env`). If you change it, change it everywhere.

## Quick manual

```bash
ap-help              # inside the container
bash ~/git_projects/atlas_ws/docker/scripts/ap-help        # on the host
ap-help tudo         # the whole manual
```

(The `ap-help` manual itself is in Portuguese.)

## Running the simulation

All commands run from `~/git_projects/atlas_ws/docker`. Once per PC session, so windows can open:

```bash
xhost +local:docker
```

### Option A — simulation in the background + a shell for debugging

```bash
docker compose up -d                  # Gazebo + SITL + MAVROS, each in its own container
docker compose run --rm dev           # shell for your code (open as many as you want)
```

Inside, for example:

```bash
ros2 run icarus_tests mission
ros2 launch atlas_mission mission.launch.py
```

Logs: `docker compose logs -f sitl`. MAVProxy console: `docker attach atlas_sitl` (detach without killing it: `Ctrl-p Ctrl-q`).

Another vehicle: `VEHICLE=plane docker compose up -d` (`copter | plane | vtol`).

### Option B — everything in one terminal (tmux)

```bash
docker compose run --rm dev ap-sim copter
```

Three panes: Gazebo | SITL (MAVProxy) | MAVROS. `Ctrl-b` + arrow switches panes, `Ctrl-b d` detaches and leaves it running.

### Option C — simulation + mission in one command

```bash
docker compose run --rm dev ros2 launch atlas_mission mission_sim.launch.py
```

Starts Gazebo + SITL + MAVROS, waits ~30 s (`mission_start_delay`), then runs the Icarus mission with a fake target.

### Option D — several drones in the same world

```bash
docker compose run --rm dev ap-multi demo     # Gazebo + 2 SITL + 2 MAVROS + 2 agents
```

Generates a world with the iris drones already inside (no runtime spawn), starts one SITL and one MAVROS per drone (`/drone1/mavros/...`, `/drone2/mavros/...`) and, with `demo`, runs `docs/multi_missoes/drone_agent.py` on each: drone1 flies a square and tells drone2 (which is loitering) to go to a waypoint. Details in [`docs/multi_missoes/README.md`](docs/multi_missoes/README.md). `ap-multi down` stops everything.

## Ground Station missions

QGC runs on the **host**. Since everything uses host networking, it sees the SITL directly.

- SITL sends MAVLink to **UDP 14550 (MAVROS)** and **UDP 14551 (QGC)**.
- QGC listens on 14550 by default, which is already taken. Once: *Application Settings → Comm Links → Add → UDP, port 14551 → Connect*.
- Then the usual flow: *Plan → Open/draw → Upload*, then *Fly → arm → AUTO*.

> Don't upload missions through MAVROS and QGC/MAVProxy at the same time — you get `Mission upload timeout`. Use one at a time, or start with `NO_MAVPROXY=1`.

## Kill everything

```bash
docker compose down --remove-orphans                    # Option A containers
docker ps --filter name=atlas                           # make sure nothing is left
```

For Option B: `tmux kill-session -t sim` inside the container, or just close the terminal (`--rm` removes the container).

## Required parameters (Icarus)

For the Icarus tests and mission, in SITL or on the drone:

| Parameter | Value | Why |
|-----------|-------|-----|
| `AUTO_OPTIONS` | 3 | Allow arming and takeoff in AUTO |
| `FENCE_AUTOENABLE` | 1 | Enable geofences automatically on takeoff |
| `FENCE_MARGIN` | 1 | Minimum allowed breach distance |
| `FENCE_TYPE` | 1 | Other fences are defined manually |
| `MIS_OPTIONS` | 4 | Mission continues after a LAND command |
| `MIS_RESTART` | 0 | Mission doesn't restart when leaving AUTO |

## Verify everything is working

```bash
ap-check                                          # environment diagnostics (what's missing)
ros2 topic echo /mavros/state --once              # look for: connected: true
ros2 topic hz /mavros/local_position/pose         # should be a steady rate
ros2 topic list | grep mavros                     # MAVROS topics are there
```

## Submodule workflow

`mavros_library` and `icarus_tests` are separate repositories (org `AeroTec-ATLAS`). After pulling:

```bash
git submodule update --init --recursive
```

To bring in the latest version of one of them:

```bash
git submodule update --remote src/mavros_library
git add src/mavros_library && git commit -m "bump mavros_library"
```

To make changes to a submodule:

```bash
cd src/icarus_tests
git checkout main            # submodules sit in "detached HEAD": switch to a branch before editing
# ... make changes, commit, push ...
cd ../..
git add src/icarus_tests
git commit -m "bump icarus_tests"
```

## Repo rules

- We only edit `atlas_mission`, `atlas_bringup`, `mavros_library`, `icarus_tests` and `docker/`.
- Upstream repos (`ardupilot`, `ardupilot_gazebo`, `ardupilot_gz`, `SITL_Models`) are never edited. Fixes go in as our own configuration or as a patch in `docker/patches/`.
- Not in git: upstream repos, `build*/ install*/ log*/`, flight logs (`*.BIN`, `*.tlog`), core dumps and `docker/.env` (see [`.gitignore`](.gitignore)).
- The `docker-compose.yaml`, `Dockerfile` and `entrypoint.sh` in the **repo root** are the old environment (`caixinha` image). The one that matters is in `docker/`.

## More docs

- [`docker/README.md`](docker/README.md) — the environment in detail (image, ports, variables, common problems)
- [`docs/index.html`](docs/index.html) — Icarus mission guide
- [`docs/multi_missoes/`](docs/multi_missoes/) — two drones in the same Gazebo world (`ap-multi`)
- [`src/mavros_library/README.md`](src/mavros_library/README.md) — how the commands and sequences work
