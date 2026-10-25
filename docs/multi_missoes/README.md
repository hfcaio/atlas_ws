# Multi-missões: 2 drones, mesmo mundo Gazebo

Tarefa: (1) correr duas missões em simultâneo no mesmo mundo; (2) testar a comunicação entre os dois veículos (um recebe um waypoint enquanto o outro faz loiter até o receber).

> **Estado:** testado de ponta a ponta numa **réplica fora do teu Docker** (ROS 2 Humble + Gazebo Harmonic gz-sim 8 + ArduPilot `master` + MAVROS 2.16, numa VM de 2 CPUs). Resultado: 2 drones no mesmo mundo, cada um com a sua missão, e o drone 2 recebeu o waypoint do drone 1 e foi até lá. **Ainda não foi corrido no teu container `atlas-sim:humble`.** A secção "Se falhar" diz o que ver.

## Correr (o caminho curto)

No host, dentro de `~/git_projects/atlas_ws/docker`:

```bash
xhost +local:docker
docker compose run --rm dev bash /home/ros/atlas_ws/docker/scripts/ap-multi demo
```

(Depois de `docker compose build` passa a bastar `docker compose run --rm dev ap-multi demo`.)

Abre um tmux com 3 janelas (`Ctrl-b n/p` troca, `Ctrl-b d` sai): **sim** (Gazebo + 2 SITL), **mavros** (2 MAVROS) e **agents** (um `drone_agent.py` por drone). Pelo caminho:

1. gera `.multi_sim/worlds/multi_iris_runway.sdf` com 2 iris já dentro (drone1 em x=0, drone2 em x=3), cada um com o seu `fdm_port_in` (9002 / 9012);
2. arranca 2 `arducopter` (`--instance 0/1`, `--sysid 1/2`, portas TCP 5760 / 5770);
3. arranca 2 MAVROS em `/drone1/mavros/...` e `/drone2/mavros/...`;
4. o drone 1 descola a 5 m, voa o quadrado `(0,0) → (10,0) → (10,10) → (0,10)` e, ao chegar ao 2.º ponto, manda ao drone 2 o waypoint `(3,12,8)`; o drone 2 descola a 8 m, fica em loiter até o receber, vai até lá e fica em `ARRIVED`.

Outros comandos: `ap-multi` (só a simulação), `ap-multi N` (N drones), `ap-multi down` (desligar), `GUI=1 ap-multi demo` (janela do Gazebo), `ap-help multi`.

## Correr à mão (para debugar)

Num container, um terminal por peça (`docker exec -it <nome> bash`):

```bash
ap-multi gen 2                                   # só gera o mundo (.multi_sim/)
gz sim -s -r -v3 /home/ros/atlas_ws/.multi_sim/worlds/multi_iris_runway.sdf      # T1  (+ gz sim -g para a GUI)
# T2/T3: um SITL por drone (esperar o Gazebo)
cd /tmp && arducopter --model json --speedup 1 --slave 0 --sim-address=127.0.0.1 --instance 0 --sysid 1 \
  --defaults $ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm,$ARDUPILOT_HOME/Tools/autotest/default_params/gazebo-iris.parm
# (o segundo: --instance 1 --sysid 2, noutra pasta)
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760 tgt_system:=1 namespace:=drone1      # T4
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5770 tgt_system:=2 namespace:=drone2      # T5
# T6: drone 2 em loiter    T7: drone 1 com a missão
python3 docs/multi_missoes/drone_agent.py --ros-args -r __ns:=/drone2 -p takeoff_alt:=8.0 -p wait_for_cmd:=true
python3 docs/multi_missoes/drone_agent.py --ros-args -r __ns:=/drone1 -p takeoff_alt:=5.0 \
  -p waypoints:="0,0,5;10,0,5;10,10,5;0,10,5" -p peer_ns:=/drone2 -p notify_at_wp:=2 -p notify_wp:="3,12,8"
```

Teste só da comunicação (com o drone 2 em loiter): `ros2 topic pub -t 3 -r 1 /drone2/cmd/waypoint geometry_msgs/msg/PoseStamped "{pose: {position: {x: 8.0, y: 0.0, z: 8.0}}}"` e `ros2 topic echo /drone2/status`.

## O que a réplica mostrou (e porque o teu `iris.launch.py` falhava)

1. **`arducopter` saía a imprimir a ajuda.** O `ardupilot_sitl` passa sempre `--use_sim_time <bool>`, mas essa opção só existe no binário se o ArduPilot foi compilado com `--enable-DDS` (está dentro de `#if AP_DDS_ENABLED` em `SITL_cmdline.cpp`, embora a linha de ajuda apareça sempre). O teu binário em `install_docker/ardupilot_sitl/bin/arducopter` (deduzido do comportamento) não tem DDS, por isso qualquer valor (`True`, `true`) faz `exit(1)`. Reproduzido com um build normal. O `ap-multi` arranca o SITL à mão, sem essa opção.
2. **O MAVROS liga mas não publica `local_position/pose`** enquanto ninguém pedir *stream rates* ao ArduPilot. O `drone_agent.py` pede-os sozinho (serviço `mavros/set_stream_rate`) enquanto espera o FCU. Se escreveres outro nó, faz o mesmo ou usa `ros2 service call /drone1/mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 10, on_off: true}"`.
3. **Namespace do `apm.launch`:** `namespace:=drone1` dá `/drone1/mavros/...`. Com `namespace:=drone1/mavros` fica `/drone1/mavros/mavros/...`.
4. **Frame local = mundo.** Os dois SITL partem da mesma origem GPS, por isso o `mavros/local_position/pose` de cada drone já está no frame do mundo (o drone 2, spawnado em x=3, aparece em `pose_local=(3.0, 0.0)`). Por isso `origin_x/origin_y` ficam em `0.0`. *(A primeira versão deste README dizia o contrário: estava errada para este setup.)* Se usares `--home` diferentes por drone, aí sim: `origin = posição do home de cada um`.
5. **Altitudes diferentes.** O quadrado do drone 1 passa por cima do ponto onde o drone 2 faz loiter. À mesma altitude (5 m) os dois colidiram numa corrida. Agora o drone K voa a `5 + 3·(K-1)` m.
6. **Bug do `peer_ns` com `/`:** `peer_ns:=/drone2` gerava `//drone2/...` e o nó rebentava. Corrigido (aceita com ou sem `/`).
7. **O ArduCopter só segue setpoints** em GUIDED, armado e no ar; o agente trata disso (GUIDED → armar → descolar → missão). `qos_profile_sensor_data` para a pose (o MAVROS publica *best effort*).
8. **O MAVROS precisa do dataset `egm96-5.pgm`** (GeographicLib). Na tua imagem já deve estar; senão `Raster width is odd`/`File not readable` ao arrancar.

## Comunicação entre drones

- `/<drone>/cmd/waypoint` (PoseStamped, frame mundo): comando para um drone.
- `/<drone>/status` (String, 1 Hz): `LOITER`, `MISSION wp=2/4`, `GOTO_CMD`, `ARRIVED`, `LAND`...
- O drone 1 reenvia o waypoint a 1 Hz até ver `GOTO_CMD`/`ARRIVED` no status do drone 2 (handshake simples; não importa quem arranca primeiro).
- O "loiter" do drone 2 é GUIDED com o setpoint de posição constante.

## Checklist (✔ = visto na réplica; ☐ = confirmar no teu container)

- [x] `ros2 topic list | grep mavros` mostra `/drone1/...` e `/drone2/...`
- [x] `/droneK/mavros/state`: `connected: true`; sysid 1 e 2
- [x] Ambos descolam; drone 2 fica parado enquanto o drone 1 faz o 1.º WP
- [x] No WP 2 o drone 1 envia, o drone 2 passa a `GOTO_CMD` → `ARRIVED` em `(3, 12, 8)`
- [x] Drone 1 conclui a missão e aterra sem interferir no drone 2
- [ ] Tudo isto no `atlas-sim:humble` (`bash docker/scripts/ap-multi demo`)
- [ ] Ver os dois drones na GUI do Gazebo (`GUI=1`) — a réplica não tinha ecrã
- [ ] Missões de Ground Station em cada drone (`GCS=1` faz o MAVROS reencaminhar para a QGC em udp 14550; **não testado**)

## Se falhar

- `ap-multi` diz `'arducopter' nao esta no PATH`: `atlas-build --packages-select ardupilot_sitl` (ou `ap-check`).
- Os SITL ficam em `Waiting for connection`: normal até o MAVROS ligar (têm de ser arrancados com o Gazebo já a correr).
- Gazebo/SITL a perder sincronismo (`controller has reset`, `Time jump detected`): máquina sobrecarregada. Sobe `SETTLE=40`, fecha a GUI, ou `SPEEDUP=1`.
- Os logs, com `NO_TMUX=1`, ficam em `.multi_sim/logs/`.
- Não consegui reproduzir o `ros_gz_sim create` com timeout do teu `iris.launch.py`. O `ap-multi` evita o assunto (os drones já estão no mundo). Se mesmo assim `gz service -l | grep worlds` vier vazio com o Gazebo a correr no mesmo container, tenta `export GZ_IP=127.0.0.1` em todos os shells.
- Aviso: numa corrida a pose do drone 2 deu um salto estranho `(0.1,-1.7,0.3)` durante o loiter (provável colisão com o drone 1, na altura à mesma altitude). Não se repetiu com as altitudes separadas.
