# `artex27` — missão ARTEX 26: subida a 2000 m com gradiente ≤ 10 %

Pacote ROS 2 (`ament_python`) que constrói **em código** a missão, a geofence e a tabela de
parâmetros do exercício ARTEX 26, em Portugal. Substitui a pasta solta `artex_2000m/`.

> **Documentação completa (pt-PT):** [`docs/artex27.html`](../../docs/artex27.html) na raiz do
> workspace. Esta página é o resumo técnico; a HTML é a que se lê antes de voar.

| | |
|---|---|
| Aeronave | asa fixa, ArduPlane **4.8.0-dev** |
| Alvo | **2000 m** acima do ponto de descolagem |
| Gradiente | **≤ 10 %** sobre o solo (nominal 8 %, com 20 % de margem) |
| Estratégia | espiral de 9 × `LOITER_TO_ALT` empilhados no centro da área UAS |
| Descolagem | **manual** — o piloto sobe até ~100 m e passa para AUTO |
| Pouso nominal | manual (variante `manual_land:=true`) |
| Pouso em failsafe | automático: `DO_RETURN_PATH_START` → `DO_LAND_START` → `LAND` |
| Área | UAS TO/LD Area 11 TF — círculo de **997,25 m** de raio, teto 3000 m AMSL |

---

## 1. Estrutura

```
src/artex27/
  artex27/geo.py          constantes geográficas + geodesia (só stdlib)
  artex27/profile.py      fórmulas: TECS_CLMB_MAX, tempos, voltas, gradiente
  artex27/mission.py      build_mission() -> list[Waypoint]  + validação
  artex27/fence.py        build_fence()   -> list[Waypoint]  + validação
  artex27/params.py       tabela ARTEX_PARAMS (valor + unidade + motivo) e riscos abertos
  artex27/nodes/          artex_mission_node.py, artex_check_node.py
  launch/                 artex_mission.launch.py, artex_check.launch.py
  config/artex27.yaml     altitudes, raio, vento, rumo, pendências
  missions/               .waypoints / .plan / kml + README de como carregar nas GCS
  params/                 .param completo, delta de SITL e o diff justificado
  tools/                  extract_kml.py, build_mission.py, validate_mission.py
  tools/sim/              SITL: run_sitl.sh, testes de gradiente e de failsafe
  docs/                   BRIEF.md (contexto) e RECONCILIATION.md (decide números)
  test/                   linters + test_mission.py (geometria, sem ROS)
```

**Hierarquia dos documentos:** em caso de conflito de números manda
[`docs/RECONCILIATION.md`](docs/RECONCILIATION.md), depois
[`params/artex27_diff.md`](params/artex27_diff.md), depois os READMEs individuais.
Os tempos em `missions/README.md` foram calculados com `TECS_CLMB_MAX = 1.20` e estão
**desatualizados** — valem os de `docs/RECONCILIATION.md` §1.

---

## 2. Ambiente — tudo corre em Docker

Nada deste pacote se corre no host. O ambiente é o `docker/docker-compose.yml` deste
workspace (imagem `atlas-sim:humble`), **não** o `docker-compose.yaml` da raiz (serviço
`atlas`, imagem `caixinha`, monta em `/home/ros/atlas`), que é o ambiente antigo.

| | |
|---|---|
| Ficheiro compose | `docker/docker-compose.yml` |
| Imagem | `atlas-sim:humble` |
| Serviço para comandos avulsos | `dev` (tem `profiles: ["dev"]`, por isso não sobe com `docker compose up`) |
| Workspace dentro do container | `/home/ros/atlas_ws` (= raiz do repo, por bind mount de `..`) |
| Working dir | `/home/ros/atlas_ws` — os caminhos relativos deste README funcionam tal e qual |
| Rede | `network_mode: host` — `127.0.0.1` é o mesmo no host e em todos os containers |

**Todos os comandos abaixo pressupõem que estás em `docker/`:**

```bash
cd ~/git_projects/atlas_ws/docker
xhost +local:docker          # só na 1.ª vez da sessão, e só se quiseres GUI
```

### Duas formas de correr

```bash
# (a) um comando de cada vez — bom para scripts e CI
docker compose run --rm dev colcon build --packages-select artex27

# (b) um shell, e depois vários comandos lá dentro — é o que se faz na prática
docker compose run --rm dev bash
```

Dentro do shell (prompt `ros@...:/home/ros/atlas_ws$`):

```bash
colcon build --packages-select artex27
source install/setup.bash
ros2 launch artex27 artex_mission.launch.py
```

> O `source install/setup.bash` só vale **dentro de um shell**. Na forma (a), cada
> `docker compose run` arranca um container novo; o `atlas-env` da imagem já faz o
> `source` do `install/` do workspace no arranque do bash, por isso prefere
> `docker compose run --rm dev bash -lc '<comando>'` se precisares do overlay sem
> abrir shell interactivo.

### Compilar

```bash
docker compose run --rm dev colcon build --packages-select artex27
```

Dependências (uma vez, ou quando o `package.xml` muda):

```bash
docker compose run --rm dev atlas-deps
```

---

## 3. Usar sem ROS (só Python)

Os módulos de geometria e de perfil não dependem de ROS; `mission.py` e `fence.py` só precisam
de `mavros_msgs` para construir as mensagens `Waypoint`.

Com um shell aberto (`docker compose run --rm dev bash`):

```bash
# perfil vertical: tabela de vento, tempos, voltas, raio efetivo
python3 -m artex27.profile

# missão item a item + validação (21 itens, tudo dentro do círculo)
python3 -m artex27.mission

# variante de pouso manual, e gravar um .waypoints
python3 -m artex27.mission --manual-land --wpl /tmp/artex_manual.waypoints

# com o QFU real da pista, quando for conhecido
python3 -m artex27.mission --approach-brg 317.2

# geofence de inclusão
python3 -m artex27.fence

# tabela de parâmetros e riscos abertos
python3 -m artex27.params
```

Ou, um a um, a partir de `docker/`:

```bash
docker compose run --rm dev bash -lc 'python3 -m artex27.profile'
docker compose run --rm dev bash -lc 'python3 -m artex27.mission'
docker compose run --rm dev bash -lc 'python3 -m artex27.params'
```

> Estes módulos vêm do `install/` do workspace. Sem `colcon build` feito pelo menos uma
> vez, o `python3 -m artex27.…` não encontra o pacote.

---

## 4. Usar com ROS 2

Os dois nós assumem que o **MAVROS já está no ar**, ligado ao avião ou ao SITL.

### ⚠️ Em SITL é obrigatório `NO_MAVPROXY=1`

O `artex_mission` faz **push de missão e de geofence** via MAVROS. Se o SITL arrancar com
MAVProxy (o default do `ap-sim` e do `ap-sitl`), o MAVProxy e o MAVROS competem pelo mesmo
canal no protocolo de missão e o upload **falha**:

```
MISSION_ACK: INVALID_SEQUENCE
Mission upload timeout
```

Com `NO_MAVPROXY=1` o SITL expõe MAVLink cru em `tcp://127.0.0.1:5760` e o MAVROS liga lá
directamente, sem intermediário:

```bash
docker compose run --rm dev env NO_MAVPROXY=1 ap-sim plane
```

O `ap-sim` já detecta a variável e passa o `fcu_url` certo ao `ap-mavros`. O
`tools/sim/run_sitl.sh` tem a mesma opção com o nome `--no-mavproxy`.

### Carregar missão + geofence no FCU

A partir de `docker/`:

```bash
docker compose run --rm dev ros2 launch artex27 artex_mission.launch.py
docker compose run --rm dev ros2 launch artex27 artex_mission.launch.py manual_land:=true
docker compose run --rm dev ros2 launch artex27 artex_mission.launch.py approach_brg:=317.2
docker compose run --rm dev ros2 launch artex27 artex_mission.launch.py vehicle:=icarus

# ou directamente
docker compose run --rm dev bash -lc \
  'ros2 run artex27 artex_mission --ros-args \
     --params-file $(ros2 pkg prefix artex27)/share/artex27/config/artex27.yaml'
```

**Este nó não arma e não descola.** A descolagem do ARTEX é manual: o piloto descola, sobe até
~100 m sobre a pista e só então passa para AUTO. Por isso não se usa `run_mission()` da
`mavros_library`, que faria GUIDED → arm → AUTO → MISSION_START.

### Pré-voo automatizado

```bash
docker compose run --rm dev ros2 launch artex27 artex_check.launch.py
docker compose run --rm dev ros2 run artex27 artex_check --ros-args -p manual_land:=true
```

Lê do FCU a missão, a geofence e os parâmetros e reporta divergências face ao esperado.
Sai com código 0 se estiver tudo conforme. Não escreve nada.

### Correr os testes

```bash
docker compose run --rm dev colcon test --packages-select artex27
docker compose run --rm dev colcon test-result --verbose
```

---

## 5. Carregar manualmente nas GCS

Os ficheiros estáticos continuam em `missions/` e `params/`. Instruções completas em
[`missions/README.md`](missions/README.md) e [`params/README.md`](params/README.md). Em resumo:

| GCS | Missão | Geofence | KML das áreas |
|---|---|---|---|
| QGroundControl | `Plan` → `Open…` → `missions/artex27.plan` → `Upload` | vem no `.plan` | `Plan` → `Fence` → `Load KML/SHP File` → `missions/kml/uas_to_ld_area_11_tf.kml` |
| Mission Planner | `Plan` → `Load WP File` → `missions/artex27.waypoints` → `Write WPs` | `Geo-Fence` → `Load from file` → `missions/artex27_geofence.waypoints` | `Map Tool` → `KML Overlay` |
| MAVProxy / SITL | `wp load /home/ros/atlas_ws/src/artex27/missions/artex27.waypoints` | `fence load /home/ros/atlas_ws/src/artex27/missions/artex27_geofence.waypoints` | — |

Parâmetros: carregar `params/artex27.param` (ficheiro completo, 1134 parâmetros) no Mission
Planner ou no QGC. Em SITL usar o delta `params/artex27_sitl.param`.

> Os `wp load` / `fence load` são comandos da **consola do MAVProxy**, não da shell, e
> precisam de caminhos **absolutos**: o MAVProxy corre no diretório de trabalho do SITL
> (`tools/sim/run/0/`), não na raiz do workspace. Só se aplicam quando há MAVProxy — com
> `NO_MAVPROXY=1` não existe consola; aí a missão vai pelo `artex_mission` (§10) ou pelo
> próprio `run_sitl.sh`, que a carrega pelo SERIAL1.

> `RTL_AUTOLAND` tem de estar a **4** antes de carregar esta missão. Com um
> `DO_RETURN_PATH_START` na missão e `RTL_AUTOLAND = 0` o **arme falha**.

---

## 6. Estrutura da missão (21 itens)

| # | Comando | Posição | Alt rel | Nota |
|---|---|---|---|---|
| 0 | `WAYPOINT` (HOME) | pista | 0 (AMSL) | frame 0; o ArduPilot reescreve-o no arme |
| 1–9 | `LOITER_TO_ALT` | centro UAS | 300…2000 | espiral de subida, `param2 = 400` |
| 10–11 | `WAYPOINT` | 500 m do centro | 2000 | **PLACEHOLDER** da missão principal |
| 12 | `DO_RETURN_PATH_START` | — (0,0) | — | **188** |
| 13–16 | `LOITER_TO_ALT` | centro UAS | 1500…140 | espiral de descida; o último com `param4 = 1` |
| 17 | `DO_LAND_START` | — (0,0) | — | 189 |
| 18 | `WAYPOINT` | 1600 m antes do toque | 140 | porta de entrada da final |
| 19 | `WAYPOINT` | 700 m antes do toque | 60 | FAF |
| 20 | `LAND` | pista (= HOME) | 0 | |

Com `manual_land:=true` entram, entre o item 11 e o `DO_RETURN_PATH_START`, uma descida nominal
própria (4 × `LOITER_TO_ALT`) e um `LOITER_UNLIM` a 150 m, 300 m da pista para dentro do círculo
— o piloto assume aí. A sequência de retorno automático fica **intocada**, porque é ela que o
failsafe usa.

**Não reordenar os itens 12 em diante.** `AP_Mission::distance_to_mission_leg()` percorre a
missão para a frente a partir do `DO_RETURN_PATH_START` e **para** no primeiro `DO_LAND_START`;
sem ele nessa ordem, o retorno falha.

---

## 7. Lacuna conhecida na `mavros_library`

`MAV_CMD_DO_RETURN_PATH_START` (**188**) **não existe** no `commands.json` da `mavros_library` —
o ficheiro salta de 187 (`DO_SET_ACTUATOR`) para 189 (`DO_LAND_START`). A `mavros_library` é um
**submódulo git** (`git@github.com:AeroTec-ATLAS/mavros_library.git`) e não deve ser editada
aqui, por isso o 188 fica definido localmente em `artex27/mission.py`:

```python
CMD_DO_RETURN_PATH_START = COMMANDS.get('MAV_CMD_DO_RETURN_PATH_START', 188)
```

O `.get()` faz com que, assim que o upstream acrescentar a chave, o pacote passe a usá-la sem
alteração nenhuma.

**A fazer upstream:** abrir PR em `AeroTec-ATLAS/mavros_library` a acrescentar
`"MAV_CMD_DO_RETURN_PATH_START": 188` ao `mavros_library/config/commands.json`.

---

## 8. Parâmetros principais

Tabela completa e justificada em [`params/artex27_diff.md`](params/artex27_diff.md);
versão curta em `artex27/params.py` (`python3 -m artex27.params`).

| Parâmetro | Valor | Porquê |
|---|---|---|
| `TECS_CLMB_MAX` | **0.8** m/s | `0,08 × (15 − 5)`. **Valor de dia de voo** — recalcular com a previsão de vento |
| `TECS_SINK_MAX` | **1.0** m/s | 10 % também na descida (decisão do utilizador). ⚠️ ver riscos |
| `TECS_PITCH_MAX` | 10° | protecção secundária; só actua com acelerador automático |
| `PTCH_LIM_MAX_DEG` | 15° | **não** pôr 6–7°: cortaria o flare e a subida manual em FBWA |
| `TECS_TIME_CONST` | 8 s | compensa o ganho do laço de acelerador, que baixar `CLMB_MAX` multiplicou por 2,5 |
| `WP_LOITER_RAD` | 200 m | os `LOITER_TO_ALT` levam `param2 = 400` explícito |
| `NAVL1_LIM_BANK` | 0 | consequência aceite: raio × EAS2TAS² → 400 m viram ~487 m a 2000 m |
| `RTL_AUTOLAND` | **4** | obrigatório; com 0 o **arme falha** |
| `FENCE_TYPE` / `FENCE_ALT_MAX` | 5 / 2100 m | altitude máxima + polígono de inclusão |
| `BATT_LOW_MAH` / `BATT_CRT_MAH` | 8100 / 5400 | 30 % / 20 % de 27 000 mAh |

### Tabela de vento

| Vento [m/s] | 0 | 2 | **5** | 7 | 8 | 10 |
|---|---|---|---|---|---|---|
| `TECS_CLMB_MAX` | 1.20 | 1.04 | **0.80** | 0.64 | 0.56 | 0.40 |
| Subida 100 → 2000 m | 26,4′ | 30,4′ | **39,6′** | 49,5′ | 56,5′ | 79,2′ |

O gradiente resultante é sempre 8,0 %, porque a fórmula se auto-ajusta.

---

## 9. Riscos abertos

Todos listados em `artex27.params.OPEN_RISKS` e impressos pelo `artex_check`.

1. **`TECS_SINK_MIN = 2.0` > `TECS_SINK_MAX = 1.0`.** `SINK_MIN` é o afundamento ao ralenti;
   pedir 1,0 m/s obriga a descer **com motor** durante ~31 min, e os dois limites entram em
   contradição dentro do TECS (`AP_TECS.cpp:739` usa `−SINK_MIN·g` em `_STEdot_min`, `:1037` usa
   `−SINK_MAX·g` no laço de pitch). **Medir o afundamento real ao ralenti** antes de aceitar
   este par.
2. **Consumo médio nunca medido.** Subida (39,6′ com 5 m/s) + descida (31′) ≈ 70 min só para
   subir e descer. Com 18 900 mAh utilizáveis, a 20 A são 56,7 min: **não chega**.
3. **QFU da pista desconhecido.** O eixo 137,2°/317,2° é geométrico.
4. **Elevação do terreno desconhecida.** `FENCE_ALT_MAX_TP = 1` contorna o problema para a
   fence, mas não para a autorização de espaço aéreo.
5. **Coordenação de espaço aéreo** acima da Med-Hi Boundary (1371 m AMSL).
6. **`TECS_TIME_CONST = 8` por validar** — confiança baixa; verificar em SITL se o acelerador
   oscila.

---

## 10. SITL — sequência de ponta a ponta

O pacote compila, os módulos importam e a missão gerada valida. **Nada foi voado nem simulado
com este código.** Esta é a sequência mínima, do zero até à missão carregada no SITL.

Tudo a partir de `~/git_projects/atlas_ws/docker`. Precisas de **3 terminais** (ou de um `tmux`),
porque o SITL e o MAVROS ficam a correr em primeiro plano.

### Passo 0 — compilar (uma vez)

```bash
cd ~/git_projects/atlas_ws/docker
docker compose run --rm dev colcon build --packages-select artex27
```

### Passo 1 — SITL de asa fixa na localização do ARTEX, com os parâmetros do ARTEX

Terminal 1:

```bash
docker compose run --rm dev \
  src/artex27/tools/sim/run_sitl.sh --headless --no-mavproxy --no-mission --no-fence
```

O que o `run_sitl.sh` faz sozinho, e por isso não aparece na linha de comandos:

| | |
|---|---|
| veículo | `-v ArduPlane -f plane` — SITL nativo, **sem Gazebo** (ver §10.1) |
| localização | `-L ARTEX_RUNWAY`, via `export ARDUPILOT_LOCATIONS=tools/sim/locations.txt` (ver §10.2) |
| parâmetros | `--add-param-file=params/artex27_sitl.param` |
| eeprom | `-w` (apaga, para o `.param` pegar de forma determinística) |
| dir de trabalho | `tools/sim/run/0/` — `eeprom.bin` e `logs/` |

`--no-mission --no-fence` são de propósito: queremos que seja o **`artex_mission` via MAVROS**
a carregá-las, que é o que se quer exercitar. Para carregar missão e fence sem ROS, tira as duas
flags — o `run_sitl.sh` fá-lo pelo canal próprio `tcp:127.0.0.1:5762` (SERIAL1), onde não há
colisão com nada.

**`--no-mavproxy` não é opcional aqui** — ver o aviso da §4. Consequências:

- MAVLink cru em `tcp://127.0.0.1:5760`; **não** há `udp:14550`.
- Não há injecção de RC (é o MAVProxy que alimenta o RC do SITL). Os cenários de
  **failsafe de rádio deixam de ser representativos** — para esses, correr com MAVProxy e
  carregar a missão sem MAVROS.

### Passo 2 — MAVROS ligado ao MAVLink cru

Terminal 2:

```bash
docker compose run --rm dev ap-mavros tcp://127.0.0.1:5760
```

Confirmar que ligou (terminal 3):

```bash
docker compose run --rm dev bash -lc 'ros2 topic echo --once /mavros/state'
```

`connected: true` e `mode: MANUAL`.

### Passo 3 — carregar missão + geofence

Terminal 3:

```bash
docker compose run --rm dev ros2 launch artex27 artex_mission.launch.py
```

Variantes: `manual_land:=true`, `approach_brg:=317.2`, `vehicle:=icarus`.

### Passo 4 — verificar

```bash
docker compose run --rm dev ros2 launch artex27 artex_check.launch.py
```

Lê do FCU missão, geofence e parâmetros e compara com o esperado. **Código de saída 0** = tudo
conforme.

### Passo 5 — testes de perfil e de failsafe

```bash
docker compose run --rm dev python3 src/artex27/tools/sim/test_climb_gradient.py
docker compose run --rm dev python3 src/artex27/tools/sim/test_failsafes.py
```

Estes ligam a `tcp:127.0.0.1:5762` (SERIAL1), não ao MAVROS. Ver
[`tools/sim/README.md`](tools/sim/README.md).

### Tudo num só shell

```bash
cd ~/git_projects/atlas_ws/docker
docker compose run --rm dev bash
```

e lá dentro, com `tmux` ou com `&`:

```bash
colcon build --packages-select artex27
src/artex27/tools/sim/run_sitl.sh --headless --no-mavproxy --no-mission --no-fence &
sleep 25
ap-mavros tcp://127.0.0.1:5760 &
sleep 10
ros2 launch artex27 artex_mission.launch.py
ros2 launch artex27 artex_check.launch.py
```

---

> **O modelo de voo do SITL não é a aeronave do ARTEX** — nem o `plane` nativo que o
> `run_sitl.sh` usa, nem o `gazebo-zephyr` do `ap-sitl plane`. Do SITL transfere-se a
> **lógica**: geometria da missão, sequência de comandos, reengate do
> `DO_RETURN_PATH_START`, comportamento dos failsafes e contenção na fence. **Não**
> transferem os números absolutos de desempenho — razão de subida atingível a `THR_MAX`,
> consumo de corrente e, por consequência, a autonomia. Esses continuam a precisar de
> medição no avião real (§9, risco 2).

### 10.1 Variante com Gazebo

O `run_sitl.sh` corre SITL nativo, sem Gazebo: é determinístico, não precisa de GPU nem de
X11 e arranca em segundos. Para o visual, usa-se o atalho da imagem, em 2 terminais, com
`xhost +local:docker` feito antes:

```bash
# terminal 1 — mundo do Zephyr
docker compose run --rm dev ap-gazebo plane

# terminal 2 — SITL com a localização e os parâmetros do ARTEX
docker compose run --rm dev env \
  NO_MAVPROXY=1 WAIT_GAZEBO=10 \
  ARDUPILOT_LOCATIONS=/home/ros/atlas_ws/src/artex27/tools/sim/locations.txt \
  LOCATION=ARTEX_RUNWAY \
  ap-sitl plane -w --add-param-file=/home/ros/atlas_ws/src/artex27/params/artex27_sitl.param
```

Os argumentos extra do `ap-sitl` são reenviados tal e qual para o `sim_vehicle.py`. Daí para
a frente, os passos 2 a 4 são iguais.

> O `ap-sim plane` sobe Gazebo + SITL + MAVROS num só comando, em 3 painéis `tmux`
> (`Ctrl-b <seta>` muda de painel, `Ctrl-b d` destaca, `tmux attach -t sim` volta). Mas **não
> reenvia argumentos** ao `sim_vehicle.py`, por isso não dá para lhe passar o
> `--add-param-file` do ARTEX. Serve para um SITL genérico:
> `docker compose run --rm dev env NO_MAVPROXY=1 ap-sim plane`.

### 10.2 A `LOCATION` do ARTEX — como funciona de facto


`Tools/autotest/locations.txt` está dentro do submódulo `src/ardupilot` e **não se edita**.
Não é preciso: o `sim_vehicle.py` procura a localização em **três** sítios, por esta ordem
(`parse_locations()`, `Tools/autotest/sim_vehicle.py:739-741`):

1. o ficheiro apontado por **`$ARDUPILOT_LOCATIONS`**;
2. `$XDG_CONFIG_HOME/ardupilot/locations.txt` (ou `~/.config/ardupilot/locations.txt`);
3. `Tools/autotest/locations.txt` do submódulo.

É para isso que existe **`tools/sim/locations.txt`**. O `run_sitl.sh` exporta a variável
sozinho, por isso `-L ARTEX_RUNWAY` simplesmente funciona. Com o `ap-sitl`, exportar à mão:

```bash
docker compose run --rm dev env \
  ARDUPILOT_LOCATIONS=/home/ros/atlas_ws/src/artex27/tools/sim/locations.txt \
  LOCATION=ARTEX_RUNWAY NO_MAVPROXY=1 ap-sitl plane
```

Localizações definidas (`NOME=lat,lon,alt_AMSL,heading`):

| Nome | O que é |
|---|---|
| `ARTEX_RUNWAY` | marcador `Runway` do KML; é onde a missão põe o HOME. **Omissão do `run_sitl.sh`** |
| `ARTEX` | centro do círculo UAS TO/LD Area 11 TF; é onde fica a espiral |
| `ARTEX_RUNWAY_137` | a pista, com o avião já apontado no sentido da aproximação |

**Alternativa sem ficheiro nenhum:** o `sim_vehicle.py` tem `-l / --custom-location`, que
aceita `lat,lon,alt,heading` directamente:

```bash
docker compose run --rm dev env NO_MAVPROXY=1 ap-sitl plane \
  -l 39.3993570,-8.2864182,100,317
```

> **Pendência:** os `100` m de elevação em `locations.txt` são um valor de trabalho — a
> elevação real do terreno na pista não está confirmada. Só afecta a conversão
> relativa ↔ AMSL; o perfil de subida é relativo ao HOME e não muda.
