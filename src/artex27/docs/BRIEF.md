# BRIEF — Missão ARTEX 2000 m (ArduPlane asa fixa)

Contexto compartilhado entre os agentes. **Não re-derivar isto; usar como dado.**
Documento de contexto adicional: `/home/caio/git_projects/atlas_ws/CLAUDE.md`.

## 1. Decisões já tomadas pelo usuário

| Decisão | Valor |
|---|---|
| Estratégia de subida | **Espiral de LOITER_TO_ALT** (8–10 pontos, ~200–250 m cada) |
| Pouso nominal | **Manual** — piloto assume no fim da missão |
| Pouso em failsafe | **Automático e funcional**, via `RTL_AUTOLAND=4` + `DO_RETURN_PATH_START` + `DO_LAND_START` + `LAND` |
| Decolagem | **Manual**, piloto sobe até ~100 m sobre a pista e passa para AUTO |
| Altitude alvo | **2000 m relativa ao home** |
| Gradiente máximo | **10 %** sobre o solo |

## 2. Firmware

- Submódulo `src/ardupilot` → `ArduPlane V4.8.0-dev` (`src/ardupilot/ArduPlane/version.h`).
- Logo: nomes modernos de parâmetros (`AIRSPEED_CRUISE`, `PTCH_LIM_MAX_DEG`, …), **não** os legados
  (`TRIM_ARSPD_CM`, `LIM_PITCH_MAX`).
- `DO_RETURN_PATH_START` (MAV cmd **188**) é suportado. `RTL_AUTOLAND=4` existe.
- Fonte de verdade para qualquer parâmetro: ler o código em `src/ardupilot/` (`ArduPlane/Parameters.cpp`,
  `libraries/AP_TECS/AP_TECS.cpp`, `libraries/AP_Landing/`, `libraries/AP_Mission/`).
  **Não confiar em memória nem em docs de versões antigas.**

## 3. Geografia (extraída de `ARTEX 26 Geo v173.kml`)

Sistema: WGS84, lat/lon em graus decimais, altitudes do KML em **metros AMSL**.

| Feature | Geometria | Teto |
|---|---|---|
| **UAS TO/LD Area 11 TF** (= `UAS_TO/LD v2`) | **círculo**, centro `39.4054602, -8.2937286`, raio **996 m** | **3000 m AMSL** |
| Marcador `UAS TO/LD Area` | ponto `39.4054886, -8.2937569` (4 m do centro do círculo) | — |
| Marcador `Runway` | ponto `39.3993570, -8.2864182` — **925 m do centro**, dentro do círculo | — |
| GCS Container Yard | quadrilátero ~50 m, centro `39.3991284, -8.2843951` | — |
| Air Ops Control | quadrilátero ~27 m, centro `39.3982800, -8.2843593` | — |
| `LP-D25 segregated airspace` | quadrilátero grande (17,2 × 13,9 km) | **4267 m AMSL (14 000 ft)** |
| `LP-TRA56` | polígono a oeste | 3000 m AMSL |
| `Lo-Med Boundary` | mesma planta do LP-D25 | 610 m AMSL (2000 ft) |
| `Med-Hi Boundary` | mesma planta do LP-D25 | 1371 m AMSL (4500 ft) |
| `A98 v2 Certification Box` | pentágono ~1,4 km | 1000 m AMSL |

**Consequências:**
- A espiral de subida cabe dentro do círculo de 996 m de raio com margem confortável.
- 2000 m AGL a partir de uma pista a ~100 m AMSL ≈ **2100 m AMSL** → dentro do teto de 3000 m da
  área UAS e dentro do LP-D25 (4267 m), mas **acima da Med-Hi Boundary (1371 m)** → camada "Hi",
  exige coordenação com o Air Ops Control. Registrar como item de coordenação, não como bloqueio.
- A elevação real do terreno na pista ainda **não foi confirmada**; tratar como pendência e
  parametrizar qualquer cálculo AMSL por ela.

## 4. Aeronave e parâmetros de referência

Arquivo de referência de uma competição anterior: `Param ACC26 26-09-2026 2202.param` (1135 linhas).
Valores relevantes **já medidos no avião real**:

```
AIRSPEED_CRUISE 15      AIRSPEED_MIN 14       AIRSPEED_MAX 27
TECS_CLMB_MAX 5         TECS_SINK_MAX 5       TECS_SINK_MIN 2
TECS_PITCH_MAX 15       TECS_PITCH_MIN 0      TECS_TIME_CONST 5
PTCH_LIM_MAX_DEG 25     PTCH_LIM_MIN_DEG -25  PTCH_TRIM_DEG 0
THR_MAX 100             THR_MIN 0
RTL_ALTITUDE 70         RTL_AUTOLAND 0        RTL_CLIMB_MIN 0     RTL_RADIUS 60
WP_LOITER_RAD 60        WP_RADIUS 35          NAVL1_PERIOD 17
FENCE_ENABLE 0          FENCE_ALT_MAX 100     FENCE_RADIUS 300    FENCE_ACTION 1
BATT_CAPACITY 27000     BATT_LOW_MAH 500      BATT_FS_LOW_ACT 0   BATT_FS_CRT_ACT 0
FS_LONG_ACTN 1          FS_SHORT_ACTN 0       FS_LONG_TIMEOUT 5   FS_SHORT_TIMEOUT 1.5
THR_FAILSAFE 1          THR_FS_VALUE 950      FS_GCS_ENABL 1
LAND_FLARE_ALT 3        LAND_FLARE_SEC 2      LAND_PITCH_DEG 0    TECS_LAND_SINK 0.25
TECS_LAND_ARSPD -1      TECS_LAND_THR -1      LAND_DISARMDELAY 20
MIS_RESTART 0           Q_ENABLE 0
```

**Atenção:** `AIRSPEED_CRUISE` real é **15 m/s**, não 14 como está no CLAUDE.md. Usar 15.
`FENCE_ALT_MAX=100` e `FENCE_RADIUS=300` são incompatíveis com esta missão — precisam mudar ou
a fence fica desligada conscientemente.

## 5. Pesquisa do usuário (notas do Notion) — tratar como requisito

### Gradiente ≤ 10 %
- `gradiente = razão de subida ÷ groundspeed`
- **`TECS_CLMB_MAX = 0.08 × (V − W)`** onde V = `AIRSPEED_CRUISE`, W = vento.
  O 0.08 (em vez de 0.10) dá 20 % de margem; a fórmula usa o pior caso de groundspeed.
- `LIM_PITCH_MAX` / `PTCH_LIM_MAX_DEG`: **600–700 centigraus (6–7°)** como proteção secundária
  contra falha do TECS_CLMB_MAX — não é o controle principal.
- `TECS_PITCH_MAX`: pôr a **0**, o que em princípio reverte ao `PTCH_LIM_MAX_DEG` — **confirmar no código**.
- `TECS_TIME_CONST`: investigar.
- `FENCE_ALT_MAX`: limite de altitude dentro da fence, segurança (não obrigatório).
- `PTCH_TRIM_DEG`: valor correto do avião — é a base de que dependem os pontos acima.

### Subida
- **8–10 pontos `LOITER_TO_ALT`**, 200–250 m de ganho cada. Heading = 0? Raio a definir.

### RTL básico
`RTL_ALTITUDE=70`, `RTL_AUTOLAND=0`, `RTL_CLIMB_MIN=0`, `RTL_RADIUS=60`

### RTL personalizado (o que se quer)
Em failsafe **ou** no decorrer normal da missão:
1. `DO_RETURN_PATH_START` — início do percurso de aproximação à pista. Os WPs seguintes são a
   trajetória predefinida que o UAV segue depois deste comando, **independentemente da posição atual**.
2. WPs posicionados para aproximação **alinhada à pista**.
3. `DO_LAND_START` — inicia a sequência de aterragem até o `LAND`, que deve corresponder ao HOME.
4. `LAND`.

Parâmetro: **`RTL_AUTOLAND = 4`**.

### Simular failsafe de rádio no Mission Planner
```
SIM_RC_FAIL = 0   # mudar para 1 durante a simulação, repor a 0 no fim (não testado)
THR_FAILSAFE = 1
FS_LONG_ACTN = 1
```

### Parâmetros de aterragem
- Entrada e alinhamento na pista: `TECS_SINK_MIN`, `TECS_SINK_MAX`
- Aproximação final: `TECS_LAND_ARSPD`, `TECS_LAND_THR`
- Início do flare: `LAND_FLARE_ALT`, `LAND_FLARE_SEC`
- Flare: `TECS_LAND_SINK`, `LAND_PITCH_DEG`

## 6. Recursos do repositório

- `src/ardupilot/` — código-fonte, versão 4.8.0-dev. Fonte de verdade dos parâmetros.
- `src/ardupilot/Tools/autotest/sim_vehicle.py` — SITL.
- `src/SITL_Models/Gazebo/config/skywalker_x8.param` — modelo de asa fixa já no workspace.
- `src/ardupilot_gz`, `src/ardupilot_gazebo` — integração Gazebo.
- `src/atlas_mission/`, `src/mavros_library/` — nós ROS 2 da missão Icarus.
- `docker/`, `docker-compose.yaml`, `Dockerfile` — ambiente.
- `docs/index.html` — **referência de estilo** da documentação (missão Icarus, pt-PT, tema escuro).
- `images/requirements*.jpeg` — fotos da checklist de inspeção pré-voo da ATLAS, com anotações à mão.
- `ARTEX 26 Geo v173.kml` — geografia do exercício (440 placemarks).
- `Param ACC26 26-09-2026 2202.param` — parâmetros reais do avião.

## 7. Estrutura de entregáveis

> **Nota (migração):** este brief foi escrito quando os entregáveis viviam numa pasta solta
> `artex_2000m/`. Tudo foi movido para o pacote ROS 2 `src/artex27/`. A estrutura atual é:

```
src/artex27/
  artex27/              # módulos Python: geo, profile, mission, fence, params
  artex27/nodes/        # nós ROS 2: artex_mission_node, artex_check_node
  missions/             # .waypoints / .plan / kml, + README de como carregar nas GCS
  params/               # .param + tabela de diff justificada
  tools/                # extract_kml, build_mission, validate_mission
  tools/sim/            # scripts de SITL e de teste de failsafe
  docs/                 # BRIEF.md (este ficheiro) e RECONCILIATION.md
  launch/  config/
docs/artex27.html       # documentação publicada
```

## 8. Pendências abertas (registrar, não inventar)

- Elevação do terreno na pista (AMSL) e direção/cabeceiras da pista.
- Vento previsto em altitude (entra direto na fórmula do `TECS_CLMB_MAX`).
- Consumo médio em subida → dimensionar `BATT_LOW_MAH` / `BATT_CRT_MAH` para 27 000 mAh.
- Autorização de espaço aéreo acima da Med-Hi Boundary.
- Raio de loiter da espiral e `PTCH_TRIM_DEG` real do avião.
