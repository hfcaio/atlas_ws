# `tools/sim/` — simulação SITL da missão ARTEX 2000 m

Scripts e procedimento para validar em **SITL** a subida a 2000 m com gradiente ≤ 10 % e
o comportamento dos **failsafes**, antes de voar a sério.

Tudo aqui é **pt-PT**, copiável, e corre **dentro do container `atlas-sim:humble`** que já
existe neste workspace (`docker/`). Não é preciso instalar nada no host além do Docker.

> Contexto: [`../../docs/BRIEF.md`](../../docs/BRIEF.md) · missão: [`../../missions/README.md`](../../missions/README.md)
> · parâmetros: [`../../params/artex27_diff.md`](../../params/artex27_diff.md)

---

## 1. Ficheiros

| Ficheiro | O que é |
|---|---|
| `run_sitl.sh` | Arranca o SITL do ArduPlane na área do ARTEX, aplica o `.param`, carrega a **geofence** e a **missão**. `--help` completo. |
| `artex_sitl.py` | Biblioteca partilhada (ligação, parâmetros, missão, geofence, amostragem) **e** CLI (`upload`, `fence`, `params`, `get`, `set`, `reset-sim`). |
| `test_climb_gradient.py` | Teste automático da subida: mede o gradiente sobre o solo e **falha** se passar de 10 % de forma sustentada. |
| `test_failsafes.py` | Bateria de failsafes (rádio, GCS, bateria) em duas fases de voo + duas verificações especiais. |
| `analyze_log.py` | Analisa um `.bin`/`.BIN`/`.tlog` (SITL ou voo real): razão de subida, gradiente real, mudanças de modo, eventos de failsafe. |
| `locations.txt` | Locations do ARTEX para o `sim_vehicle.py`, **sem tocar no submódulo** `src/ardupilot`. |
| `fixtures/` | Missão e `.param` mínimos, **só** para smoke-test dos scripts. Não são os ficheiros de voo. |
| `results/` | Saída dos testes (CSV + eventos). Ignorado pelo git excepto `results/ref_*`. |
| `run/` | Estado do SITL (`eeprom.bin`, `logs/*.BIN`, `mav.tlog`). Ignorado pelo git. |

---

## 2. Veredito: SITL nativo, não Gazebo

**Decisão: SITL nativo (`sim_vehicle.py -v ArduPlane -f plane`), sem Gazebo.**

O workspace tem a integração Gazebo completa (`src/ardupilot_gz/`, `src/ardupilot_gazebo/`,
`src/SITL_Models/`, `docker/scripts/ap-gazebo`), e ela é a escolha certa para a missão Icarus
— onde há um alvo visual, câmara e percepção. Para **esta** missão não é, por quatro razões
verificadas no repositório:

1. **Nenhum mundo de asa fixa serve.** Os mundos disponíveis são
   `src/ardupilot_gazebo/worlds/zephyr_runway.sdf` e
   `src/SITL_Models/Gazebo/worlds/{skywalker_x8,alti_transition,…}_runway.sdf`. Nenhum tem
   bloco `<spherical_coordinates>`, ou seja, a origem geográfica não é a área do ARTEX; e
   nenhum tem terreno com 2000 m de envergadura vertical útil. O que se ganharia era um
   avião a voar sobre uma pista plana infinita — exactamente o que o modelo nativo já dá.
2. **O que se está a medir não precisa de Gazebo.** O gradiente é
   `razão de subida ÷ groundspeed`; ambos saem do TECS e do EKF, que correm **dentro do
   ArduPilot**, não no simulador de física. O modelo `plane` nativo fecha a malha
   TECS → pitch/throttle → velocidade exactamente da mesma maneira.
3. **Velocidade.** Com Gazebo o SITL anda em lockstep com o motor de física e o
   *real-time factor* manda. O teste de subida completo precisa de **~41 minutos de
   simulação**. Em SITL nativo isso correu em **2 min de relógio a 20x** (medido — ver §6).
   Com Gazebo seriam 41 minutos reais, por corrida, e a bateria de failsafes multiplica isso.
4. **Determinismo.** Sem renderização nem lockstep, o resultado não depende da GPU da
   máquina. Os testes dão o mesmo em CI e no portátil.

**Quando valeria a pena o Gazebo:** validar a aterragem automática do failsafe com terreno
real (`FENCE_ALT_MAX_TP`, flare, `LAND_FLARE_ALT`) ou integrar com os nós ROS 2 de
`atlas_mission`. Nesse caso o caminho já documentado deste workspace é
`docker compose run --rm dev ap-sim plane` (ver [`docker/README.md`](../../../../docker/README.md)).

---

## 3. Pré-requisitos

### 3.1 O que é preciso ter

| | |
|---|---|
| Docker + `docker compose` | único requisito do host |
| Imagem `atlas-sim:humble` | `cd atlas_ws/docker && docker compose build` |
| Workspace montado | o `docker-compose.yml` já monta `atlas_ws` em `/home/ros/atlas_ws` |
| `src/ardupilot` | clonado por `atlas-ws-init` (ver [`docker/README.md`](../../../../docker/README.md)) |
| Binário SITL do ArduPlane | `src/ardupilot/build/sitl/bin/arduplane` |

A imagem já traz **pymavlink**, **pexpect** e **MAVProxy** — os scripts não instalam nada:

```bash
cd ~/git_projects/atlas_ws/docker
docker compose run --rm dev python3 -c "import pymavlink, pexpect; print('ok')"
```

### 3.2 Compilar o SITL (se o binário não existir)

```bash
cd ~/git_projects/atlas_ws/docker
docker compose run --rm dev bash -lc '
  cd $ARDUPILOT_HOME
  ./waf configure --board sitl
  ./waf plane
'
```

De raiz demora **10–20 min** (4 jobs). Incremental, segundos. Depois disto o `run_sitl.sh`
passa `-N` ao `sim_vehicle.py` e não volta a compilar.

### 3.3 Correr fora do container (não recomendado)

Se quiseres mesmo correr no host:

```bash
python3 -m pip install --user pymavlink pexpect MAVProxy
export PATH="$HOME/git_projects/atlas_ws/src/ardupilot/Tools/autotest:$PATH"
```

Nesta máquina o host **não** tem estes módulos (`ModuleNotFoundError: No module named
'pymavlink'` / `'pexpect'`), por isso todo o procedimento abaixo assume o container.

---

## 4. Procedimento, do princípio ao fim

Todos os comandos correm a partir de `~/git_projects/atlas_ws/docker`.

### 4.0 Abrir uma shell no container (recomendado)

Os testes precisam do SITL a correr **em simultâneo**. O mais simples é uma shell só:

```bash
cd ~/git_projects/atlas_ws/docker
docker compose run --rm dev bash
# daqui para baixo, já estás dentro do container:
cd $ATLAS_WS/src/artex27/tools/sim
```

### 4.1 Arrancar o SITL

```bash
# tudo por omissão: location ARTEX_RUNWAY + params + geofence + missão
./run_sitl.sh --headless --no-mission &
sleep 30
```

- `--headless` → MAVProxy sem mapa/consola e em `--daemon` (sem isto, lançado em segundo
  plano com o stdin fechado, o MAVProxy sai de imediato e leva o SITL com ele).
- `--no-mission` → **recomendado para os testes**: quem carrega a missão é o script de
  teste, e assim não há dois carregamentos. A **geofence continua a ser carregada** —
  é obrigatória (ver §5.2).
- Com GUI (precisa de `xhost +local:docker` no host): tirar o `--headless` e o `&`.

Opções úteis:

```bash
./run_sitl.sh --help                       # ajuda completa
./run_sitl.sh --headless -w 5,270          # 5 m/s de vento de oeste
./run_sitl.sh --headless --variant manual_land
./run_sitl.sh --dry-run                    # só imprime o comando
```

### 4.2 Teste de subida — gradiente ≤ 10 %

```bash
python3 ./test_climb_gradient.py --speedup 20 --timeout 900
```

Com vento:

```bash
python3 ./test_climb_gradient.py --speedup 20 --timeout 900 --wind 5,317 --tag vento5
```

**PASS** = as três linhas finais:

```
  PASS  atingiu 2000 m
  PASS  gradiente sustentado X.XX % <= 10.00 %
  PASS  tempo de subida dentro de 25 % do previsto
VEREDITO: PASS
```

Exit code `0` = PASS, `1` = FAIL, `2` = erro de ambiente.

### 4.3 Bateria de failsafes — **um SITL novo por cenário**

Um cenário de cada vez, contra um SITL acabado de arrancar:

```bash
python3 ./test_failsafes.py --scenario rc_nopulses --phase alt:1200 \
        --fast-climb 5 --climb-speedup 20 --inject-speedup 1 --observe 90
```

**Porque é que tem de ser um SITL novo:** depois de um teste em que o avião entra no
percurso de regresso, a flag `in_landing_sequence` do `AP_Mission` fica levantada e o
pre-arm seguinte rejeita com **`PreArm: In landing sequence`**
(`ArduPlane/AP_Arming_Plane.cpp:126-130`). A flag **sobrevive ao reboot do FCU** e
**`AP_Mission::clear()` não a limpa** — só `reset()` (`AP_Mission.cpp:265`) e
`set_current_cmd()` (`:607`) limpam, e `start_command()` (`:416-418`) volta a pô-la assim
que a missão reentra no `DO_LAND_START`. O `test_failsafes.py` tenta limpá-la
(`reset_mission_state()`: `MAV_CMD_DO_SET_MISSION_CURRENT` + `MISSION_SET_CURRENT` +
ciclo de modo), mas **não é fiável** quando o avião fica no ar depois do reboot — o
`PREFLIGHT_REBOOT_SHUTDOWN` reinicia o FCU **sem repor a posição do modelo de voo**, e o
hook que faz `mission.reset()` (`ArduPlane/mode.cpp:143-147`) exige `!is_flying()`.

Por isso o procedimento de referência é o laço abaixo — copiável como está:

```bash
run_one () {                     # $1 = cenário   $2 = fase   $3.. = extras
  local sc="$1" ph="$2"; shift 2
  pkill -f arduplane >/dev/null 2>&1; pkill -f mavproxy >/dev/null 2>&1; sleep 3
  nohup ./run_sitl.sh --headless --no-mission > /tmp/sitl_${sc}_${ph//:/}.log 2>&1 &
  sleep 30
  python3 ./test_failsafes.py --scenario "$sc" --phase "$ph" \
      --climb-speedup 20 --inject-speedup 1 --climb-timeout 600 \
      --out-dir results "$@"
  pkill -f arduplane >/dev/null 2>&1; pkill -f mavproxy >/dev/null 2>&1; sleep 2
}

for sc in rc_nopulses rc_throttle rc_protocol gcs batt_low batt_crit; do
  run_one "$sc" low                                  --observe 90
  run_one "$sc" high --fast-climb 5                  --observe 90
done
```

`--fast-climb 5` sobe o `TECS_CLMB_MAX` **só** enquanto ganha altitude (não se está a
testar gradiente aqui) e repõe-no antes da injecção. Sem isso, chegar a 2000 m a 0,8 m/s
demora 40 min de simulação por cenário.

**PASS** por teste = `STATUSTEXT` esperado + mudança de modo + salto para o
`DO_RETURN_PATH_START` + itens de missão a avançar de forma monótona.

### 4.4 Verificações especiais

Com a função `run_one` de §4.3 já definida:

```bash
# 1) reengate do DO_RETURN_PATH_START em função da altitude
for ph in alt:300 alt:700 alt:1200 alt:1800; do
  run_one rc_nopulses "$ph" --fast-climb 5 --observe 90
done

# 2) violação de fence durante o percurso de regresso
run_one fence_during_return alt:1500 --fast-climb 5 --observe 120
```

### 4.5 Analisar o log

```bash
ls run/0/logs/                                   # .BIN do dataflash
python3 ./analyze_log.py run/0/logs/00000001.BIN --target-alt 2000 --csv /tmp/voo.csv
python3 ./analyze_log.py run/0/mav.tlog          # ou o tlog do MAVProxy
```

O mesmo script serve para o `.bin` de um voo **real** — é a forma de confirmar no dia que
o gradiente sobre o solo ficou dentro dos 10 %.

### 4.6 Repor os parâmetros de simulação

Os testes **já repõem sozinhos** no bloco `finally` (`SIM_WIND_*`, `SIM_RC_FAIL`,
`SIM_BATT_*`, `SIM_SPEEDUP`, e os failsafes que relaxaram). Se um teste for interrompido a
meio (Ctrl-C, container morto), repor à mão:

```bash
python3 ./artex_sitl.py reset-sim
python3 ./artex_sitl.py get SIM_RC_FAIL SIM_WIND_SPD SIM_SPEEDUP THR_FAILSAFE FS_GCS_ENABL
```

E, por segurança, **nunca** levar um `eeprom.bin` de SITL para o avião real: o `run/` é
descartável, apaga-o (`rm -rf run/`) e o próximo `run_sitl.sh` recria-o do `.param`.

---

## 5. Decisões técnicas e armadilhas encontradas

Esta secção existe porque cada um destes pontos custou uma corrida falhada. São factos
observados neste workspace, não teoria.

### 5.1 Canal MAVLink dos testes: `tcp:127.0.0.1:5762`, não a saída do MAVProxy

O protocolo de missão do MAVLink tem **estado por canal**. Com o script a falar missão pela
mesma ligação do MAVProxy, o resultado foi `Mission upload timeout` no FCU e a missão a
ficar com **1 item** (só o HOME) — seguido de `Mission complete, changing mode to RTL`
assim que se entrava em AUTO. É o mesmo conflito que o [`docker/README.md`](../../../../docker/README.md)
documenta entre MAVProxy e MAVROS.

Solução: os scripts ligam-se ao **SERIAL1 do SITL**, que é um canal MAVLink independente.

```
libraries/AP_HAL_SITL/SITL_cmdline.cpp:253   const int BASE_PORT = 5760;
libraries/AP_HAL_SITL/SITL_State.h:31        _serial_path = {"tcp:0:wait", "tcp:2", "tcp:3", ...}
```

| Porta (instância 0) | Quem |
|---|---|
| `tcp:127.0.0.1:5760` | SERIAL0 — MAVProxy |
| **`tcp:127.0.0.1:5762`** | **SERIAL1 — os scripts de teste** (omissão de `--url`) |
| `tcp:127.0.0.1:5763` | SERIAL2 — livre |
| `udp:127.0.0.1:14550` | MAVROS / QGroundControl, via MAVProxy |

Instância *N* desloca tudo em +10N.

**Bug relacionado, corrigido:** o `MISSION_ACK` da `MISSION_CLEAR_ALL` ficava na fila e o
ciclo de upload confundia-o com o ACK final, dando "missão aceite" sem ter enviado item
nenhum. O `upload_mission()` agora drena esse ACK e **confirma a contagem** no FCU no fim.

### 5.2 A geofence tem de ser carregada, senão não arma

`artex27_sitl.param` tem `FENCE_TYPE=5` (bit0 altitude máx + bit2 inclusão/exclusão).
Com o bit2 e sem polígono carregado, o pre-arm falha com **"Polygon fence(s) invalid"**
(`libraries/AC_Fence/AC_Fence.cpp:428`).

O `run_sitl.sh` carrega `../../missions/artex27_geofence.waypoints` por omissão, pelo protocolo
de missão com `MAV_MISSION_TYPE_FENCE` e itens
`MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION` (5001). O ficheiro está no formato legado
`FENCE_POINT` (1.ª linha = *return point*, depois os vértices, fechando no primeiro); o
`read_fence()` descarta o return point e o vértice de fecho → 36 vértices.

Para desligar: `--no-fence` **e** `FENCE_TYPE=1` (só altitude).

### 5.3 `SIM_SPEEDUP` alto estraga os failsafes por tempo

O MAVProxy injecta o RC (via UDP `127.0.0.1:5501`, `sim_vehicle.py:1144`) e o heartbeat de
GCS em tempo de **relógio**; o FCU conta `RC_FS_TIMEOUT` e `FS_LONG_TIMEOUT` em tempo de
**simulação**. Com `SIM_SPEEDUP=20`, 1 s de `RC_FS_TIMEOUT` são 50 ms de relógio → o avião
declara perda de rádio sem haver falha nenhuma.

Pior: `ArduPlane/mode_takeoff.cpp:193` **adia** o failsafe longo enquanto o *flight stage* é
`TAKEOFF` e dispara-o assim que passa a `NORMAL`. Resultado observado: o avião chegava aos
100 m e saltava logo para RTL; a entrega para AUTO falhava sempre.

Como está resolvido:

- **`run_sitl.sh` arranca sempre a 1x** e avisa se lhe passarem `-s > 5`.
- Os testes mexem no `SIM_SPEEDUP` **por fase**: rápido para ganhar altitude (com os
  failsafes por tempo relaxados), e **1x na janela de injecção e observação**.
- `relax_failsafes()` põe `THR_FAILSAFE=0`, `FS_GCS_ENABL=0`, `FS_LONG_ACTN=0`,
  `FS_SHORT_ACTN=0` e `RC_FS_TIMEOUT=10` (máximo admitido, `RC_Channels_VarInfo.h:111`),
  e `restore_failsafes()` repõe os valores lidos antes. Não basta desligar o
  `THR_FAILSAFE`: `rc_failsafe_active()` (`ArduPlane/radio.cpp:348`) dispara por
  `RC_FS_TIMEOUT` mesmo com ele a 0.

**Consequência para quem lê os resultados:** o teste de subida corre com os failsafes por
tempo desligados. Quem os testa é o `test_failsafes.py`, a 1x. São testes diferentes de
propósito.

### 5.4 Base de tempo: segundos de simulação

O `Sampler` usa `GLOBAL_POSITION_INT.time_boot_ms` (relógio do FCU, que corre a
`SIM_SPEEDUP`) e **não** o relógio de parede. Assim as janelas (`--window`, `--settle`),
as durações e a comparação com a previsão analítica são independentes do speedup escolhido.
O speedup real medido é reportado (`speedup real medido`), porque o pedido nem sempre é o
obtido.

Nota: a **resolução** da amostragem degrada com o speedup. Com `--dt 0.5` (relógio) a 20x,
cada amostra cobre ~10 s de simulação. O teste imprime isso e reduz a janela
automaticamente se não houver amostras que cheguem.

### 5.5 A fase "manual" é simulada com o modo TAKEOFF

A descolagem real é **manual** (piloto sobe até ~100 m e passa a AUTO). Para ser
automatizável, o teste usa o modo `TAKEOFF` do ArduPlane com `TKOFF_ALT` = `--handover-alt`
(omissão 100 m) e repõe o `TKOFF_ALT` no fim. O gradiente **não é avaliado nesta fase** —
tal como na realidade, em que o piloto sobe à vontade. A avaliação começa na entrada em
AUTO, mais `--settle` segundos de simulação.

### 5.6 Location do ARTEX sem editar o submódulo

`src/ardupilot` é upstream e não se edita. O `sim_vehicle.py` lê primeiro o ficheiro
apontado por **`$ARDUPILOT_LOCATIONS`** e só depois o `Tools/autotest/locations.txt`
(`parse_locations()`, `Tools/autotest/sim_vehicle.py:735`). O `run_sitl.sh` exporta essa
variável a apontar para `sim/locations.txt`:

| Nome | Ponto | Nota |
|---|---|---|
| `ARTEX_RUNWAY` | `39.3993570, -8.2864182`, hdg 317 | **omissão**; é onde a missão põe o HOME |
| `ARTEX` | `39.4054602, -8.2937286`, hdg 0 | centro do círculo UAS (onde está a espiral) |
| `ARTEX_RUNWAY_137` | pista, hdg 137 | sentido de aproximação |

A altitude 100 m AMSL é um **valor de trabalho**: a elevação real do terreno na pista ainda
não está confirmada (pendência do `BRIEF.md` §8). Só afecta a conversão rel↔AMSL.

---

## 6. O que foi executado de verdade, e a saída obtida

Ambiente: container `atlas-sim:humble`, ArduPlane **V4.8.0-dev**, binário
`src/ardupilot/build/sitl/bin/arduplane` já compilado, modelo `plane`, sem Gazebo.

### 6.1 Ambiente — confirmado

```
$ docker compose run --rm dev bash -lc 'python3 -c "import pymavlink;print(pymavlink.__file__)"; which mavproxy.py sim_vehicle.py'
/usr/local/lib/python3.10/dist-packages/pymavlink/__init__.py
/usr/local/bin/mavproxy.py
/home/ros/atlas_ws/src/ardupilot/Tools/autotest/sim_vehicle.py
```

O binário SITL já existia (`src/ardupilot/build/sitl/bin/arduplane`), por isso **não foi
preciso compilar**. No host, `python3 -c "import pymavlink"` dá
`ModuleNotFoundError` — é mesmo só dentro do container.

### 6.2 Teste de subida com a missão e os parâmetros a sério — **PASS**

Comando:

```bash
./run_sitl.sh --headless --no-mission &          # carrega params + geofence
sleep 32
python3 ./test_climb_gradient.py --speedup 20 --timeout 900 --tag v1_sem_vento
```

Saída real (cortadas as linhas repetidas de `Loiter to alt complete` e do DDS):

```
[15:38:00] missao aceite pelo FCU (21 itens)
[15:38:01] parametros relevantes: AIRSPEED_CRUISE=15.0  TECS_CLMB_MAX=0.800000011920929  TECS_PITCH_MAX=10.0  PTCH_LIM_MAX_DEG=15.0
[15:38:01] previsao: gradiente teorico no pior caso = 5.3 % (TECS_CLMB_MAX 0.80 / (V 15.0 - W 0.0))
[15:38:01] previsao: tempo de subida = 2375 s (39.6 min) para 1900 m
[15:38:01] SIM_SPEEDUP 1.0 -> 20.0
[15:38:01] failsafes por tempo RELAXADOS (valores anteriores: THR_FAILSAFE=1, FS_GCS_ENABL=1, FS_LONG_ACTN=1, FS_SHORT_ACTN=0, RC_FS_TIMEOUT=1)
[15:38:01] --- FASE 1: descolagem (substituto automatico da fase manual) ---
[15:38:01] modo -> TAKEOFF
[15:38:01] armado
[15:38:01]     STATUSTEXT[6] Takeoff to 100m for 200.0m heading 317.2 deg
[15:38:04] entrega a 98.0 m — a passar para AUTO
[15:38:04] --- FASE 2: AUTO, subida a 2000 m ---
[15:38:04] modo -> AUTO
[15:38:04]     STATUSTEXT[6] Mission: 1 LoitAltitude
[15:39:02]   alt=  992.4 m  gs= 15.7 m/s  vz=+0.75 m/s  grad= 4.77 %
[15:40:02]   alt= 1948.0 m  gs= 16.6 m/s  vz=+0.84 m/s  grad= 5.10 %
[15:40:05] --- ANALISE ---

==================================================================
 RESULTADO — subida ARTEX 2000 m
==================================================================
  missao                  : /home/ros/atlas_ws/src/artex27/missions/artex27.plan
  vento simulado          : 0.0 m/s de 180 deg
  AIRSPEED_CRUISE         : 15.0 m/s
  TECS_CLMB_MAX           : 0.800000011920929 m/s
  --
  altitude inicial / final: 143 m -> 1996 m (rel. home)
  alvo atingido           : SIM
  tempo de subida (AUTO)  : 2453 s = 40.9 min de SIMULACAO   [previsto 39.6 min]
  speedup real medido     : 20.2x
  razao de subida media   : 0.77 m/s
  groundspeed media       : 15.8 m/s   (airspeed 14.9 m/s)
  --
  gradiente medio         : 4.89 %
  gradiente instantaneo   : 5.50 % (max)
  gradiente SUSTENTADO    : 5.21 % (max, janela 61 s sim, 6 amostras)
      pior ponto: t=1021 s  alt=834 m  gs=15.6 m/s  vz=+0.75 m/s
  limite                  : 10.00 %
==================================================================
  PASS  atingiu 2000 m
  PASS  gradiente sustentado 5.21 % <= 10.00 %
  PASS  tempo de subida dentro de 25 % do previsto

VEREDITO: PASS
```

Leitura:

- **`TECS_CLMB_MAX = 0.8` funciona.** Gradiente sustentado **5,21 %**, metade do limite.
  A folga é deliberada: 0,8 vem de `0.08 × (15 − 5)`, ou seja, já tem 5 m/s de vento de
  frente embutidos. **Sem vento** o avião gasta margem que não precisava.
- O avião subiu os 1900 m em **40,9 min de simulação**, contra os 39,6 previstos (+3 %).
  A diferença é o tempo que o TECS leva a estabilizar em cada transição de `LOITER_TO_ALT`.
- A groundspeed média (15,8 m/s) é ligeiramente superior à airspeed (14,9 m/s) porque a
  2000 m o ar é menos denso: a TAS sobe para a mesma EAS. Isto **baixa** o gradiente sobre
  o solo em altitude — a favor da segurança.
- **40 min de subida é um dado operacional, não só de simulação.** Com 27 000 mAh é preciso
  confirmar que a subida sozinha cabe no orçamento de energia.

CSV e eventos: `results/climb_v1_sem_vento.csv` / `.events.txt`.

### 6.3 Smoke-test anterior, com a missão de fixture — **PASS**

Antes de os ficheiros reais existirem, a mesma subida foi corrida com
`fixtures/fixture_espiral.waypoints` e `TECS_CLMB_MAX = 1.2`:

```
  tempo de subida (AUTO)  : 1645 s = 27.4 min de SIMULACAO   [previsto 26.4 min]
  speedup real medido     : 15.1x
  gradiente medio         : 7.32 %
  gradiente instantaneo   : 7.92 % (max)
  gradiente SUSTENTADO    : 7.67 % (max, janela 61 s sim, 8 amostras)
VEREDITO: PASS
```

Serve de ponto de calibração: com `TECS_CLMB_MAX` **1,2** o gradiente sustentado foi
**7,67 %**; com **0,8**, **5,21 %**. A razão entre os dois (1,47) acompanha a razão dos
`TECS_CLMB_MAX` (1,50) — ou seja, **o gradiente é governado pelo `TECS_CLMB_MAX`, como
previsto**, e a relação é linear. Isso permite converter directamente: para um limite duro
de 10 % sobre o solo, `TECS_CLMB_MAX` ≈ `0.10 × groundspeed`.

CSV: `results/climb_demo_sem_vento.csv`.

---

## 7. Parâmetros de simulação — valores confirmados no código

Tudo verificado em `src/ardupilot` desta cópia (ArduPlane 4.8.0-dev), não em documentação.

### 7.1 `SIM_RC_FAIL` — a nota do utilizador estava incompleta

O `BRIEF.md` §5 trazia «`SIM_RC_FAIL = 0`, mudar para 1 durante a simulação e repor a 0 no
fim (**não testado**)». Confirmado e corrigido:

```
libraries/SITL/SITL.h:161
    enum SITL_RCFail {
        SITL_RCFail_None                  = 0,
        SITL_RCFail_NoPulses              = 1,
        SITL_RCFail_Throttle950           = 2,
        SITL_RCFail_Protocol_Fail_Bit_Set = 3,
    };
libraries/SITL/SITL.cpp:143-148
    // @Values: 0:Disabled,1:No RC pusles,2:All Channels neutral except Throttle is 950us
    AP_GROUPINFO("RC_FAIL", 25, SIM, rc_fail, 0);
```

| Valor | Nome no código | O que faz | Como dispara o failsafe no ArduPlane |
|---|---|---|---|
| **0** | `None` | normal | — |
| **1** | `NoPulses` | o receptor deixa de entregar frames (`AP_RCProtocol_UDP.cpp:103` faz `return` antes de `add_input`) | `rc_failsafe_active()` por `RC_FS_TIMEOUT` (`ArduPlane/radio.cpp:353`) — **independente do `THR_FAILSAFE`** |
| **2** | `Throttle950` | todos os canais a 1500 µs e o throttle a **950 µs** (`AP_RCProtocol_UDP.cpp:170-176`) | `rc_throttle_value_ok()` (`radio.cpp:333-342`): OK só se `radio_in > THR_FS_VALUE`. Com `THR_FS_VALUE=950` e `radio_in=950`, **dispara** |
| **3** | `Protocol_Fail_Bit_Set` | canais a 1456 µs e bit de failsafe do protocolo activo | idem via flag de failsafe do frame |

**Pontos que a nota original não tinha:**

- **O valor 3 existe mas não está documentado** no `@Values` do parâmetro — só no `enum`.
  Os scripts cobrem-no (`--scenario rc_protocol`) exactamente por isso.
- **1 e 2 testam coisas diferentes.** O 1 testa `RC_FS_TIMEOUT`; o 2 testa
  `THR_FAILSAFE`/`THR_FS_VALUE`. Com `THR_FAILSAFE=0` o valor 2 não dispara, mas o 1
  continua a disparar. Para cobrir a configuração real (`THR_FAILSAFE=1`,
  `THR_FS_VALUE=950`) convém correr **os dois**.
- Repor a 0 no fim: os scripts fazem-no no `finally`; manualmente,
  `python3 ./artex_sitl.py set SIM_RC_FAIL=0`.

### 7.2 `SIM_WIND_*`

`libraries/SITL/SITL.cpp:90-116` e `:290-307`:

| Parâmetro | Omissão | Unidade | Significado |
|---|---|---|---|
| `SIM_WIND_SPD` | 0 | m/s | velocidade do vento |
| `SIM_WIND_DIR` | **180** | ° true | direcção **DE ONDE** sopra |
| `SIM_WIND_TURB` | 0 | m/s | variação aleatória |
| `SIM_WIND_TC` | 5 | s | constante de tempo das mudanças |
| `SIM_WIND_DIR_Z` | 0 | ° | componente vertical |
| `SIM_WIND_T` | 0 (`WIND_TYPE_SQRT`) | enum | perfil do vento com a altitude |
| `SIM_WIND_T_ALT` | 60 | m | altitude de referência do perfil |
| `SIM_WIND_T_COEF` | 0.01 | — | coeficiente do perfil |

Atenção ao `SIM_WIND_DIR` **= 180 por omissão**, não 0: um `SIM_WIND_SPD` posto sozinho dá
vento de sul. O `run_sitl.sh -w` e o `--wind` dos testes aceitam `SPD[,DIR[,TURB]]` e
escrevem os três.

**O gradiente medido já é sobre o solo.** `gradiente = climb ÷ groundspeed` e a groundspeed
do `VFR_HUD` inclui o vento. Com vento de frente a groundspeed cai e o gradiente sobe —
é precisamente o caso que o teste tem de apanhar, e por isso o vento entra como parâmetro
de teste e não como detalhe.

### 7.3 `SIM_BATT_*`

`libraries/SITL/SITL.cpp:124-140`:

| Parâmetro | Omissão | Nota |
|---|---|---|
| `SIM_BATT_VOLTAGE` | **12.6 V** | tensão em repouso. Alterá-la **reinicializa o estado de carga** |
| `SIM_BATT_CAP_AH` | **0** (= ilimitada) | capacidade. O `artex27_sitl.param` põe **27**, para que o failsafe por mAh possa disparar |
| `SIM_BATT_RES_OHM` | −1 (= usar a do modelo) | resistência interna; modela o *sag* sob carga. `RebootRequired` |

Do lado do FCU (`libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp`), confirmados os
nomes modernos da 4.8: `BATT_CAPACITY` (:76), `BATT_LOW_TIMER` (:102),
`BATT_LOW_VOLT` (:117), `BATT_LOW_MAH` (:125), `BATT_CRT_VOLT` (:133),
`BATT_CRT_MAH` (:141), `BATT_FS_LOW_ACT` (:153), `BATT_FS_CRT_ACT` (:165).

**Consequência para os testes:** o `artex27_sitl.param` tem `BATT_LOW_VOLT = 0` e
`BATT_CRT_VOLT = 0` — o failsafe de bateria é **por mAh** (`BATT_LOW_MAH = 8100`,
`BATT_CRT_MAH = 5400`). Baixar o `SIM_BATT_VOLTAGE` não dispara nada nesta configuração.
Por isso o `test_failsafes.py` escolhe o método conforme o que estiver configurado:

1. `BATT_*_VOLT > 0` → baixa o `SIM_BATT_VOLTAGE` para debaixo do limiar;
2. senão, `BATT_*_MAH > 0` → **baixa o limiar de mAh** para pouco acima do consumo actual
   (lido da `BATTERY_STATUS.current_consumed`), em vez de esperar 8,1 Ah de voo simulado;
3. senão → sobe o `BATT_*_VOLT` acima da tensão actual.

Em todos os casos o valor original é reposto no `finally`.

### 7.4 Outros parâmetros relevantes confirmados

| Parâmetro | Onde | Confirmação |
|---|---|---|
| `SIM_SPEEDUP` | `SITL.cpp:229` | existe e é escrevível em voo; `@Range 1 10` é só metadado — **20x foi usado e medido em 20,2x** |
| `RTL_AUTOLAND` | `ArduPlane/Parameters.cpp:720-725` | `4 = Go directly to landing sequence via DO_RETURN_PATH_START` |
| `MAV_GCS_SYSID` | `libraries/GCS_MAVLink/GCS.cpp:51` | omissão **255**; só o sysid aqui indicado conta para o failsafe de GCS |
| `RC_FS_TIMEOUT` | `RC_Channels_VarInfo.h:107-113` | omissão 1,0 s, `@Range 0.1 10.0` |
| `FS_GCS_ENABL` | `ArduPlane/Parameters.cpp:462-467` | `1=Heartbeat, 2=Heartbeat+REMRSSI, 3=Heartbeat só em AUTO` |
| `FS_LONG_ACTN` | `ArduPlane/Parameters.cpp:448-449` | `0=Continue, 1=RTL, 2=Glide, 3=Parachute, 4=Auto, 5=AUTOLAND-senão-RTL` |

**Mensagens de `STATUSTEXT` que os testes procuram** (texto exacto, do código):

| Mensagem | Origem |
|---|---|
| `RC Long Failsafe On: switched to <modo>` | `ArduPlane/events.cpp:240` |
| `GCS Failsafe On: switched to <modo>` | `ArduPlane/events.cpp:240` |
| `RC Short Failsafe On` | `ArduPlane/events.cpp:107` |
| `Battery 1 is low <V>V used <mAh> mAh` | `AP_BattMonitor.cpp:966` (`type_str` = `"low"`/`"critical"`, `:958`/`:962`) |
| `Return path started` | `AP_Mission.cpp:2566` |
| `Landing sequence start` | `AP_Mission.cpp:2512` |

---

## 8. Validação cruzada com `analyze_log.py`

O mesmo voo de §6.2, relido do **dataflash** (`.BIN`) em vez da telemetria ao vivo:

```
$ python3 ./analyze_log.py run/0/logs/00000009.BIN --target-alt 2000 --csv /tmp/voo9.csv
a ler run/0/logs/00000009.BIN (dataflash)...

======================================================================
 ANALISE DE LOG — 00000009.BIN
======================================================================
  duracao                 : 2515 s (41.9 min), 12574 amostras, dt~0.20 s
  altitude max (rel.)     : 1996 m
  tempo ate' 2000 m        : 2498 s (41.6 min)
  --
  gradiente medio         : 4.87 %
  gradiente instantaneo   : 39.77 % (max)
  gradiente SUSTENTADO    : 5.24 % (max, janela 20 s)
      pior ponto: t=109 s  alt=132 m  gs=15.1 m/s  vz=+0.80 m/s

  MUDANCAS DE MODO
         0.0 s   TAKEOFF
        51.6 s   AUTO
      2514.7 s   RTL
      2514.8 s   AUTO

  EVENTOS / FAILSAFES
        10.5 s   MSG  EKF2 IMU0 is using GPS
        10.5 s   MSG  EKF2 IMU1 is using GPS
      2514.7 s   MSG  GCS Failsafe On: switched to RTL
      2514.8 s   MSG  Return path started
======================================================================

VEREDITO: PASS — gradiente sustentado 5.24 % <= 10.00 %
```

Duas leituras independentes, dois valores quase iguais:

| | telemetria ao vivo (`test_climb_gradient.py`) | dataflash (`analyze_log.py`) |
|---|---|---|
| gradiente sustentado | 5,21 % | 5,24 % |
| tempo até 2000 m | 40,9 min | 41,6 min |
| altitude máxima | 1996 m | 1996 m |

Notas de interpretação:

- O **gradiente instantâneo de 39,77 %** é ruído do `GPS.VZ` a baixa altitude, não um pico
  real de subida — por isso o veredito usa o **sustentado**, e por isso o `--min-alt`
  (omissão 50 m) existe.
- O `GCS Failsafe On` no segundo 2514,7 é um **artefacto do próprio teste**: era o momento
  em que o script repunha `FS_GCS_ENABL=1` ainda com `SIM_SPEEDUP=20`. Foi exactamente este
  sintoma que levou a introduzir o `wait_stable()` (§5.3) nos testes de failsafe, que
  reactiva os failsafes já a 1x e só injecta a falha depois de o voo estabilizar.

O `.tlog` do MAVProxy também é lido (`python3 ./analyze_log.py run/0/mav.tlog`). Um `.tlog`
traz mensagens de **todos** os sistemas do link, incluindo os heartbeats da própria GCS
(`MAV_TYPE_GCS`, `custom_mode = 0` = MANUAL no Plane): sem filtrar por sysid a lista de
modos aparecia a saltar `MANUAL↔AUTO` dezenas de vezes. O `read_tlog()` fixa o sysid do
veículo no primeiro heartbeat que não seja de uma GCS e ignora o resto.

---

## 9. As duas verificações especiais

O agente de parâmetros levantou duas armadilhas no código da 4.8 que **só a simulação pode
confirmar** (`../../params/artex27_diff.md` §4.1, pontos 2 e 6). São estas.

Referência do percurso de regresso, em `artex27.waypoints`:

| # | cmd | Comando | Alt rel (m) | Papel |
|---|---|---|---|---|
| 9 | 31 | `LOITER_TO_ALT` | 2000 | topo da espiral de subida |
| 10, 11 | 16 | `WAYPOINT` | 2000 | missão principal |
| **12** | **188** | **`DO_RETURN_PATH_START`** | — | entrada do percurso de regresso |
| 13 | 31 | `LOITER_TO_ALT` | 1500 | perna 12→13 |
| 14 | 31 | `LOITER_TO_ALT` | 1000 | perna 13→14 |
| 15 | 31 | `LOITER_TO_ALT` | 500 | perna 14→15 |
| 16 | 31 | `LOITER_TO_ALT` | 140 | perna 15→16 |
| **17** | **189** | **`DO_LAND_START`** | — | início da sequência de aterragem |
| 18 | 16 | `WAYPOINT` | 140 | aproximação |
| 19 | 16 | `WAYPOINT` | 60 | final |
| 20 | 21 | `LAND` | 0 | cabeceira |

Os itens 13–16 estão **todos no mesmo ponto** (centro do círculo UAS), só mudam de
altitude. Logo cada perna do regresso é um **vector puramente vertical** — e
`jump_to_closest_mission_leg()` escolhe a perna mais próxima em **3D**. Previsão: o item
engatado depende só da **altitude** a que a falha acontece.


<!-- RESULTADOS_VERIFICACOES -->

---

## 10. Discrepâncias encontradas entre o `BRIEF.md`/`CLAUDE.md` e a 4.8

| Afirmação no contexto | O que o código diz |
|---|---|
| `FS_SHORT_TIMEOUT = 1.5` | **O parâmetro já não existe na 4.8.** `ArduPlane/Parameters.h:278` tem-no como `k_param_fs_timeout_short_unused`. O failsafe curto dispara **imediatamente** quando `failsafe.rc_failsafe` fica verdadeiro (`ArduPlane/system.cpp:409-421`), e esse flag é governado por `RC_FS_TIMEOUT` / `THR_FS_VALUE`. Escrever `FS_SHORT_TIMEOUT` não faz nada. |
| `SIM_RC_FAIL`: «0, mudar para 1, repor a 0 (não testado)» | Confirmado e completado: há **quatro** valores (0/1/2/3), e o 3 nem está no `@Values`. O 1 e o 2 testam mecanismos diferentes — ver §7.1. |
| `AIRSPEED_CRUISE = 14` (CLAUDE.md) | **15 m/s**, como o `BRIEF.md` corrige e o `.param` confirma. Os testes lêem o valor do FCU, não o assumem. |
| `TECS_CLMB_MAX = 0.1 × (V − vento)` (CLAUDE.md) | O ficheiro de parâmetros usa `0.08 × (V − vento)` = margem de 20 %. Medido: com 0,8 m/s o gradiente sustentado é **5,2 %**, não 8 % — porque a groundspeed real (15,8 m/s) é maior do que o pior caso assumido (10 m/s). |
| `RTL_AUTOLAND = 2` (CLAUDE.md) | A missão usa **4** (`DO_RETURN_PATH_START`). Com `RTL_AUTOLAND = 0` **e** um `DO_RETURN_PATH_START` na missão, **o arming falha** (`AP_Arming_Plane.cpp:425-436`) — detalhe que invalida qualquer teste feito com a configuração antiga. |

---

## 11. O que ficou por fazer, e porquê

| Item | Estado | Porquê |
|---|---|---|
| Subida com vento (`--wind`) | **não corrido** | O caminho está implementado e testado em seco (`SIM_WIND_*` são escritos e lidos de volta), mas cada corrida custa ~2,5 min e o tempo foi para as verificações especiais. É a corrida mais barata que falta: `python3 ./test_climb_gradient.py --speedup 20 --wind 5,317 --tag vento5`. **Previsão:** com 5 m/s de frente a groundspeed cai para ~10,8 m/s e o gradiente sobe de 5,2 % para ~7,4 % — ainda dentro do limite, que é exactamente a margem que o `0.08` do agente de parâmetros compra. |
| Aterragem automática completa (até ao `LAND`) | **não verificada** | Os testes observam 90–120 s depois do failsafe, o que chega para confirmar o salto e o início do percurso, mas não para percorrer os ~15 min de simulação até tocar o solo. Precisa de uma corrida dedicada com `--observe` grande. **É a verificação mais importante que falta**, porque o `BRIEF.md` exige «pouso em failsafe automático e funcional». |
| Gazebo | **não usado**, por decisão | Ver §2. |
| Elevação real do terreno na pista | **pendência externa** | `locations.txt` usa 100 m AMSL como valor de trabalho. Não afecta o perfil de subida (que é relativo ao home), afecta a conversão para AMSL e a leitura do `FENCE_ALT_MAX`. |
| QFU real da pista | **pendência externa** | O heading 317° de `ARTEX_RUNWAY` é o azimute pista→centro escolhido pelo agente de missão, não o eixo real. |
| Teste em hardware (HITL) | fora de âmbito | — |

---

## 12. Referência rápida das opções

### `run_sitl.sh`

| Opção | Omissão | Nota |
|---|---|---|
| `-m, --mission` | 1.ª de `../../missions/` (ignora ficheiros com `fence` no nome) | `.waypoints`, `.txt` ou `.plan` |
| `--variant NOME` | — | filtra a missão pelo nome (`--variant manual_land`) |
| `--no-mission` | — | recomendado quando quem carrega é o teste |
| `-F, --fence` | `../../missions/artex27_geofence.waypoints` | obrigatória com `FENCE_TYPE` bit2 |
| `--no-fence` | — | exige pôr `FENCE_TYPE=1` |
| `-p, --params` | `../../../params/artex27_sitl.param` | |
| `-L, --location` | `ARTEX_RUNWAY` | de `sim/locations.txt` |
| `-w, --wind SPD[,DIR[,TURB]]` | sem vento | `DIR` = de onde sopra |
| `-s, --speedup` | 1 | **não subir aqui** (§5.3) |
| `-f, --frame` | `plane` | modelo nativo do SITL |
| `-I, --instance` | 0 | desloca as portas +10N |
| `-d, --dir` | `sim/run/<instância>` | `eeprom.bin`, `logs/` |
| `--headless` | — | sem mapa/consola; implica `--daemon` |
| `--daemon` | — | MAVProxy sem shell interactiva |
| `--no-mavproxy` | — | **sem RC injectado** — ver aviso do script |
| `--no-wipe` | — | por omissão apaga a eeprom, para o `.param` pegar |
| `--dry-run` | — | só imprime o comando |

### `test_climb_gradient.py`

| Opção | Omissão |
|---|---|
| `--url` | `tcp:127.0.0.1:5762` |
| `--target-alt` | 2000 m |
| `--handover-alt` | 100 m |
| `--max-gradient` | 0.10 |
| `--window` / `--settle` | 60 s / 60 s **de simulação** |
| `--speedup` | 10 (`SIM_SPEEDUP` durante o teste; reposto no fim) |
| `--keep-failsafes` | desligado → os failsafes por tempo são relaxados (§5.3) |
| `--wind SPD[,DIR[,TURB]]` | — |
| `--mission` / `--variant` / `--no-mission` | 1.ª de `../../missions/` |
| `--params` | nenhum (usa o que o `run_sitl.sh` aplicou) |
| `--out-dir` / `--tag` | `sim/results` / timestamp |

### `test_failsafes.py`

| Opção | Omissão |
|---|---|
| `--scenario` | `all` = `rc_nopulses,rc_throttle,rc_protocol,gcs,batt_low,batt_crit`; além desses, por nome: `fence_during_return` |
| `--phase` | `all` = `low,high`; aceita `alt:<metros>` e listas |
| `--low-alt` / `--high-alt` | 150 m / 2000 m |
| `--fast-climb` | 0 (desligado) — sobe o `TECS_CLMB_MAX` só para ganhar altitude |
| `--climb-speedup` / `--inject-speedup` | 10 / **1** |
| `--observe` | 120 s |
| `--no-reboot` | — |

### `analyze_log.py`

| Opção | Omissão |
|---|---|
| `--target-alt` | 2000 m |
| `--max-gradient` | 0.10 |
| `--window` | 20 s |
| `--min-alt` / `--min-gs` | 50 m / 3 m/s (filtram ruído de GPS no solo) |
| `--csv` | — |
| `--all-messages` | só mensagens com palavras-chave de failsafe |
| `--use-repo-pymavlink` | força `src/ardupilot/modules/mavlink/pymavlink` |

### `artex_sitl.py` (CLI)

```bash
python3 ./artex_sitl.py wait                                  # espera o SITL
python3 ./artex_sitl.py upload --mission ../../missions/x.waypoints
python3 ./artex_sitl.py fence  --fence-file ../../missions/artex27_geofence.waypoints
python3 ./artex_sitl.py params --param-file ../../params/artex27_sitl.param
python3 ./artex_sitl.py get TECS_CLMB_MAX AIRSPEED_CRUISE RTL_AUTOLAND
python3 ./artex_sitl.py set SIM_WIND_SPD=5 SIM_WIND_DIR=270
python3 ./artex_sitl.py reset-sim
```
