> **Histórico — concluído.** Este é o briefing com que o pacote `artex27` foi construído.
> Os caminhos `artex_2000m/...` que aparecem abaixo já não existem: tudo foi migrado para
> `src/artex27/` (`missions/`, `params/`, `tools/`, `tools/sim/`, `docs/`). Guardado como
> registo das decisões; o documento vivo é o [`README.md`](../README.md) do pacote e a
> página [`docs/artex27.html`](../../../docs/artex27.html).

# HANDOFF — construir o pacote `artex27`

Tudo o que já foi apurado. **Não voltar a explorar o que está aqui** — é para poupar tokens.

## Objetivo

Levar a missão ARTEX (subida a 2000 m, gradiente ≤ 10 %) para dentro das convenções do repo:
um pacote ROS 2 `ament_python` em `src/artex27/`, que constrói a missão **em código** com a
`mavros_library`, em vez da pasta solta `artex_2000m/`.

## Documentos a ler primeiro (e só estes)

1. `artex_2000m/BRIEF.md` — contexto, geografia, decisões, pendências.
2. `artex_2000m/RECONCILIATION.md` — **manda sobre tudo o resto** em caso de conflito de números.
3. `artex_2000m/params/artex_2000m_diff.md` — justificação de cada parâmetro.
4. `artex_2000m/mission/README.md` — semântica dos comandos, como carregar nas GCS.

Não é preciso reler o código do ArduPilot: as conclusões relevantes estão nos documentos acima.

## Esqueleto já criado

```
src/artex27/
  artex27/__init__.py  artex27/nodes/__init__.py
  launch/  config/  resource/artex27  test/{test_flake8,test_pep257,test_copyright}.py
  missions/  params/  tools/
```

Falta: `package.xml`, `setup.py`, `setup.cfg`, os módulos e os nós.

## Convenções do repo (já verificadas — não reconfirmar)

**`src/atlas_mission/package.xml`** é o modelo. `<build_type>ament_python</build_type>`,
`exec_depend`: `rclpy`, `sensor_msgs`, `mavros_msgs`, `mavros_library`, `launch`, `launch_ros`,
`atlas_bringup`. `test_depend`: `ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest`.

**`setup.py`**: `data_files` com `resource/`, `package.xml` e `glob('launch/*.launch.py')` para
`share/<pkg>/launch`. `entry_points.console_scripts` aponta para `artex27.nodes.<x>:main`.
Para instalar `missions/`, `params/` e `config/`, acrescentar entradas em `data_files`.

**Idioma**: comentários e logs em português (pt-PT no `mission_sequence.py`, pt-BR nos nós do
`atlas_mission`). Indentação: 4 espaços nos nós, 2 na `mavros_library`. Usar **4 espaços**.

## API da `mavros_library` (já lida — usar, não reinventar)

```python
from mavros_library.commands import create_waypoint          # commands/messages/waypoint.py
from mavros_library.config import COMMANDS, FRAMES, TOPICS, SERVICES, QOS_PROFILE, TIME_LIMITS
from mavros_library.sequences import init_all, wait_for_connection, new_mission, set_geofence
```

`create_waypoint(frame_id, command_id, is_current, autocontinue, param1..param4, param5=lat,
param6=lon, param7=alt)` — devolve um `mavros_msgs.msg.Waypoint`. Note que **param5/6/7 são
`x_lat`/`y_long`/`z_alt`**.

`init_all(node, vehicles)` cria `node.<vehicle>` (um `SimpleNamespace`) com subscribers, clients e
estado: `.state`, `.global_position_global`, `.global_position_rel_alt`, `.home_position_home`,
`.mission_waypoints`, `.geofence_fences`, `.mission_reached`, `.terrain_report`.

`wait_for_connection(node, vehicles)` bloqueia até `state.connected`.
`new_mission(node, vehicle, waypoint_list)` = clear + push.
`set_geofence(node, vehicle, waypoint_list)` = clear + push da fence.
`run_mission(node, vehicle)` faz GUIDED → arm → AUTO → MISSION_START. **Não usar nesta missão**:
a decolagem é **manual** e o avião entra em AUTO pelo piloto. Explicar isso no código.

Prefixos de veículo válidos (`config/prefix.json`): `mavros`, `drone`, `vtol`, `icarus`, `prometeu`.
Default a usar: `mavros`.

`FRAMES['MAV_FRAME_GLOBAL_RELATIVE_ALT'] = 3`, `FRAMES['MAV_FRAME_GLOBAL'] = 0`.

`COMMANDS` tem `MAV_CMD_NAV_WAYPOINT` 16, `MAV_CMD_NAV_LOITER_UNLIM` 17,
`MAV_CMD_NAV_LOITER_TO_ALT` 31, `MAV_CMD_NAV_LAND` 21, `MAV_CMD_DO_LAND_START` 189.

> **Lacuna conhecida:** `MAV_CMD_DO_RETURN_PATH_START` (**188**) **não existe** no
> `commands.json` da `mavros_library` (salta de 187 para 189). A `mavros_library` é um
> **submódulo git** (`git@github.com:AeroTec-ATLAS/mavros_library.git`) — **não a editar**.
> Definir o 188 localmente em `artex27`, com comentário a dizer que devia ser enviado para
> upstream. Registar isto no README e na documentação.

## Geografia (do KML, já extraída — usar estes números)

| | |
|---|---|
| Área UAS TO/LD — centro | `39.4054608, -8.2937285` |
| Área UAS TO/LD — raio | **997,25 m** (círculo exato, resíduo 0,017 m) |
| Área UAS TO/LD — teto | 3000 m AMSL |
| Pista (placemark ponto) | `39.3993570, -8.2864182` |
| Pista → centro | 925,8 m, azimute 137,2° |
| LP-D25 teto | 4267 m AMSL |
| Med-Hi Boundary | 1371 m AMSL |
| Elevação do terreno | **DESCONHECIDA** — parametrizar, nunca inventar |

## Parâmetros decididos (estado final, já com a decisão do utilizador)

`TECS_CLMB_MAX=0.8`, `TECS_SINK_MAX=1.0`, `TECS_SINK_MIN=2`, `TECS_PITCH_MAX=10`,
`TECS_PITCH_MIN=-15`, `TECS_TIME_CONST=8`, `PTCH_LIM_MAX_DEG=15`, `AIRSPEED_CRUISE=15`,
`WP_LOITER_RAD=200`, `NAVL1_LIM_BANK=0`, `RTL_AUTOLAND=4`, `FENCE_ENABLE=1`, `FENCE_TYPE=5`,
`FENCE_ALT_MAX=2100`, `FENCE_AUTOENABLE=3`, `BATT_LOW_MAH=8100`, `BATT_CRT_MAH=5400`,
`BATT_FS_LOW_ACT=1`, `BATT_FS_CRT_ACT=2`, `FS_LONG_ACTN=1`, `FS_LONG_TIMEOUT=10`.

**O utilizador decidiu aplicar os 10 % também à descida** → `TECS_SINK_MAX` passou de 3.0 a
**1.0** (`0.10×(15−5)`), descida 2000→140 m em ~31 min.

> **Problema por resolver, herdado:** `TECS_SINK_MIN=2.0` ficou **maior** que
> `TECS_SINK_MAX=1.0`. No código, `AP_TECS.cpp:739` limita `STEdot_dem` entre `_STEdot_min`
> (= −SINK_MIN·g) e `_STEdot_max`, enquanto `:1037` usa `−SINK_MAX·g` no laço de pitch — ou seja,
> os dois limites ficam em contradição e a descida a 1,0 m/s exige voo com motor (o afundamento ao
> ralenti declarado é 2,0 m/s). **Investigar e propor uma saída** (medir o `SINK_MIN` real, ou
> aceitar descida com motor, ou rever o valor). Documentar como risco aberto — **não varrer para
> debaixo do tapete**.

## Estrutura da missão (21 itens, já validada)

`HOME` → 9× `LOITER_TO_ALT` (300/500/700/900/1100/1300/1500/1750/2000 m, `param2=400`, `param1=0`,
`param4=0`) → 2 WP placeholder da missão principal a 2000 m → `DO_RETURN_PATH_START` (188) →
4× `LOITER_TO_ALT` de descida (1500/1000/500/140 m, o último com `param4=1`) → `DO_LAND_START`
(189) → WP 140 m → WP 60 m → `LAND`.

Semântica confirmada do `LOITER_TO_ALT`: param1 ignorado; **param2 = raio, sinal = sentido, e 0
cai no `WP_LOITER_RAD`**; param4 = 0 crosstrack do centro, 1 pela tangente. Empilhar no mesmo
lat/lon dá espiral contínua. O raio efetivo cresce com a altitude (×EAS2TAS²): 400 m → ~492 m a
2000 m.

## O que construir

### Módulos em `artex27/artex27/`

- `geo.py` — constantes geográficas acima + utilitários (destino por azimute/distância, distância
  entre pontos, teste de contenção no círculo). Sem dependências fora da stdlib.
- `profile.py` — perfil de subida/descida: `tecs_clmb_max(airspeed, wind, margin=0.8)`, tempos,
  distâncias, nº de voltas, gradiente. É aqui que vivem as fórmulas, testáveis sem ROS.
- `mission.py` — `build_mission(...) -> list[Waypoint]` usando `create_waypoint`. Variante com
  `LOITER_UNLIM` (pouso manual nominal) por argumento.
- `fence.py` — `build_fence(...) -> list[Waypoint]` (fence de inclusão poligonal da área UAS).
- `params.py` — tabela `ARTEX_PARAMS` com valor + unidade + motivo, e um verificador que compara
  com o que está no FCU.

### Nós em `artex27/artex27/nodes/`

- `artex_mission_node.py` — liga, espera conexão, faz push da fence e da missão, confirma.
  **Não arma e não descola**: a decolagem é manual. Deixar isso explícito nos logs.
- `artex_check_node.py` — lê o que está no FCU (missão, fence, parâmetros) e reporta divergências
  face ao esperado. É o pré-voo automatizado.

### `launch/`, `config/`, `setup.py`, `package.xml`, `README.md`

`config/artex27.yaml` com altitudes, raio de loiter, vento previsto, airspeed, elevação do terreno
(placeholder explícito). Os nós lêem daqui.

### Migração

Mover de `artex_2000m/` para dentro do pacote: `mission/*.waypoints|.plan|kml/` → `missions/`,
`params/*.param|*.md` → `params/`, os scripts (`extract_kml.py`, `build_mission.py`,
`validate_mission.py`, e os de SITL em `artex_2000m/sim/`) → `tools/`.
Usar `git mv` quando o ficheiro já estiver versionado; caso contrário `mv`.
**Deixar `artex_2000m/` vazio e removido no fim**, com os documentos `BRIEF.md` e
`RECONCILIATION.md` movidos para `src/artex27/docs/` ou para o README do pacote — à sua escolha,
mas **nada pode ficar órfão**.

## Documentação — `docs/artex27.html`

Mesmo estilo de `docs/index.html` (ler só o `<style>` e a estrutura de `<section>`; não é preciso
ler as 28 000 linhas). Tema escuro, TOC lateral fixa, pt-PT, **self-contained** (sem CDN, sem
fontes externas), classes já existentes: `.layout .toc .hero .eyebrow .lede .label .card .grid2
.callout .pill .tablewrap`.

Secções a cobrir, no mínimo:
1. O que é a missão e o requisito dos 10 %
2. Geografia e espaço aéreo (incluindo o aviso da Med-Hi Boundary)
3. Perfil de subida e descida, com os números e a tabela de vento
4. Parâmetros, com o porquê dos principais
5. **Como correr** — o pacote ROS 2, os launch files, os comandos copiáveis
6. Como carregar missão/fence/KML no QGroundControl e no Mission Planner
   (a instrução já existe em `artex_2000m/mission/README.md` — reaproveitar)
7. Checklist de pré-voo — **ler as fotos em `images/requirements*.jpeg`**, que são a checklist de
   inspeção da ATLAS escrita à mão pela equipa, e transcrever o que for aplicável
8. Riscos e pendências em aberto, com destaque para: autonomia vs. tempo de subida,
   `SINK_MIN > SINK_MAX`, QFU da pista, elevação do terreno, coordenação de espaço aéreo

Acrescentar uma linha em `docs/README.md` a apontar para a página nova.

## Regras

- **Economia de tokens**: não ler ficheiros inteiros quando chega um `grep`/`sed -n`. Não reler o
  que está neste handoff. Não explorar `src/ardupilot/`.
- **Não editar submódulos**: `src/mavros_library`, `src/icarus_tests`, `src/ardupilot`,
  `src/ardupilot_gz`, `src/ardupilot_gazebo`, `src/SITL_Models`.
- **Pare quando só faltar testar.** O utilizador testa em SITL e em voo. Construa, verifique que
  `colcon build --packages-select artex27` passa e que os módulos importam e geram uma missão
  válida — e **não** tente correr SITL nem testes de voo.
- Onde faltar um dado, usar placeholder explícito e registar como pendência. Não inventar.

## Retorno

Em português, curto: ficheiros criados/movidos; como correr (comandos copiáveis); o que verificou
de facto; o que ficou por testar e porquê; e o que propõe para o problema do `SINK_MIN > SINK_MAX`.
