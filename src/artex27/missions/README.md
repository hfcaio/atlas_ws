# `missions/` — ARTEX 2000 m: missão, geofence e KML

Subida de um ArduPlane (asa fixa, **ArduPlane 4.8.0-dev**) até **2000 m acima do HOME**,
com gradiente de subida **≤ 10 %**, inteiramente dentro da área **UAS TO/LD Area 11 TF**,
e com sequência de aterragem automática funcional para o caso de failsafe.

Tudo o que está aqui foi gerado a partir de `ARTEX 26 Geo v173.kml` e verificado contra o
código-fonte em `src/ardupilot/` (não contra documentação ou memória). As discrepâncias
encontradas estão na secção [Discrepâncias](#discrepâncias-brief--código).

---

## 1. Arquivos

| Arquivo | O que é |
|---|---|
| `artex27.waypoints` | Missão principal, **pouso automático**. QGC WPL 110, TAB. 21 itens. |
| `artex27.plan` | A mesma missão em JSON nativo do QGroundControl (inclui a geofence). |
| `artex27_manual_land.waypoints` | Variante: a parte nominal termina em `LOITER_UNLIM` sobre a pista, o piloto assume. 26 itens. |
| `artex27_manual_land.plan` | Idem, formato QGC. |
| `artex27_geofence.waypoints` | Geofence poligonal (36 vértices) no formato legado `FENCE_POINT`: 1ª linha = return point, depois os vértices, fechando no primeiro. |
| `artex27_geofence.plan` | Geofence nativa do QGC: círculo de inclusão, centro e raio da área UAS. Missão vazia. |
| `kml/` | KMLs reduzidos, **um polígono por arquivo**, prontos para `Load KML/SHP` do QGC. Mais `kml/artex_areas.kml` combinado para o Google Earth. |
| `extract_kml.py` | Extrator de features do KML (stdlib only). Re-executar quando sair a v174. |
| `build_mission.py` | Gerador de tudo o que está acima. É aqui que se muda raio de loiter, altitudes e eixo de aproximação. |
| `validate_mission.py` | Validador de gradiente / contenção / teto / sequência. Exit code ≠ 0 se reprovar. |

Regenerar tudo:

```bash
cd src/artex27/tools
./build_mission.py                                   # .waypoints + .plan + fences
./extract_kml.py "../../../ARTEX 26 Geo v173.kml" \
    --name 'UAS TO/LD Area 11 TF|^LP-D25|^LP-TRA56|Certification Box|^Runway$|Boundary 4500' \
    --kml-out kml/
./validate_mission.py artex27.waypoints artex27_manual_land.waypoints
```

---

## 2. Como carregar nas GCS

### QGroundControl

**Missão** — `Plan` → ícone de ficheiro (canto superior esquerdo) → `Open…` →
escolher `artex27.plan`. O `.plan` traz missão **e** geofence juntos.
Depois `Upload Required` → `Upload`.

> O `.waypoints` também abre no QGC (`Open…` aceita `.waypoints`/`.txt`), mas perde a
> geofence e a `plannedHomePosition`. Prefira o `.plan`.
> Por isso os `.waypoints` **não têm linhas de comentário**: o leitor WPL do QGC é
> estrito e rejeitaria linhas `#`.

**Geofence sozinha** — `Plan` → `Open…` → `artex27_geofence.plan`.

**Ver as áreas do KML (era a dúvida original)** — o `ARTEX 26 Geo v173.kml` tem 440
placemarks e o QGC **não abre um KML desses**: o importador de KML do QGC espera
*um polígono*. Por isso há a pasta `kml/`, com um arquivo por área:

- `Plan` → `Fence` → `Polygon Fence` → `Load KML/SHP File` → `kml/uas_to_ld_area_11_tf.kml`
- O mesmo para `kml/lp_d25_segregated_airspace.kml`, `kml/a98_v2_certification_box.kml`, etc.
- Para só *ver* tudo de uma vez sem carregar como fence, abra `kml/artex_areas.kml` no
  Google Earth.

### Mission Planner

**Missão** — `Plan` → `Load WP File` → `artex27.waypoints` → `Write WPs`.

**Geofence poligonal** — `Plan` → menu lateral `Geo-Fence` (ou botão direito no mapa →
`Draw Polygon` → `Fence` → `Load from file`) → `artex27_geofence.waypoints` → `Upload`.
Formato legado `FENCE_POINT`: a primeira linha é o *return point*.

**Ver o KML** — `Plan` → botão direito no mapa → `Map Tool` → `KML Overlay` e aponte
para `ARTEX 26 Geo v173.kml` (o MP aguenta o arquivo completo) ou para
`kml/artex_areas.kml` se quiser só o essencial.

### MAVProxy / SITL

```bash
wp load artex27.waypoints
fence load artex27_geofence.waypoints
```

---

## 3. Geografia usada

Extraída com `extract_kml.py` (ajuste de círculo por mínimos quadrados, resíduo máximo
**0,017 m** — é um círculo exato no KML):

| Feature | Valor |
|---|---|
| **UAS TO/LD Area 11 TF** — centro | `39.4054608, -8.2937285` |
| **UAS TO/LD Area 11 TF** — raio | **997,25 m** |
| **UAS TO/LD Area 11 TF** — teto | 3000 m AMSL |
| **Runway** (placemark ponto) | `39.3993570, -8.2864182` |
| Pista → centro do círculo | **925,8 m**, azimute **137,2°** |
| Folga da pista até a borda do círculo | **71,5 m** |

**Consequência operacional importante:** a pista está praticamente **colada à borda** do
círculo UAS. Qualquer aproximação final que venha "de fora" sai da área. Por isso o eixo
de aproximação escolhido é **137,2° / 317,2°**, ou seja, o avião entra na final vindo do
**centro do círculo** em direção à pista. Assim toda a aproximação fica dentro da área UAS.

> ⚠️ **Isto não é o QFU real da pista.** O KML só traz a pista como um **ponto**, sem
> eixo. 137,2° é a direção centro→pista, uma escolha geométrica para manter a aproximação
> contida. Quando souberem a direção real da pista e o vento, rodar:
> `./build_mission.py --approach-brg <QFU>`.

---

## 4. Estrutura da missão (`artex27.waypoints`)

| # | cmd | Comando | Posição | Alt rel (m) | Nota |
|---|---|---|---|---|---|
| 0 | 16 | HOME | pista | `0` **(placeholder AMSL)** | frame **0**, altitude absoluta |
| 1 | 31 | LOITER_TO_ALT | centro UAS | 300 | raio 400 m |
| 2 | 31 | LOITER_TO_ALT | centro UAS | 500 | |
| 3 | 31 | LOITER_TO_ALT | centro UAS | 700 | |
| 4 | 31 | LOITER_TO_ALT | centro UAS | 900 | |
| 5 | 31 | LOITER_TO_ALT | centro UAS | 1100 | |
| 6 | 31 | LOITER_TO_ALT | centro UAS | 1300 | |
| 7 | 31 | LOITER_TO_ALT | centro UAS | 1500 | |
| 8 | 31 | LOITER_TO_ALT | centro UAS | 1750 | |
| 9 | 31 | LOITER_TO_ALT | centro UAS | 2000 | topo da subida |
| 10 | 16 | WAYPOINT | 500 m @ 047° do centro | 2000 | **PLACEHOLDER missão principal** |
| 11 | 16 | WAYPOINT | 500 m @ 227° do centro | 2000 | **PLACEHOLDER missão principal** |
| 12 | 188 | **DO_RETURN_PATH_START** | — (0,0) | — | início do caminho de retorno |
| 13 | 31 | LOITER_TO_ALT | centro UAS | 1500 | descida |
| 14 | 31 | LOITER_TO_ALT | centro UAS | 1000 | |
| 15 | 31 | LOITER_TO_ALT | centro UAS | 500 | |
| 16 | 31 | LOITER_TO_ALT | centro UAS | 140 | `param4 = 1` (sai pela tangente) |
| 17 | 189 | **DO_LAND_START** | — (0,0) | — | |
| 18 | 16 | WAYPOINT | 1600 m antes do toque | 140 | porta de entrada da final |
| 19 | 16 | WAYPOINT | 700 m antes do toque | 60 | FAF |
| 20 | 21 | **LAND** | pista (= HOME) | 0 | |

Os itens **10 e 11 são placeholders**: a missão principal a 2000 m ainda não foi definida.
Substituir por waypoints reais mantendo-os a 2000 m e dentro do círculo; **não mexer na
ordem** dos itens 12→20.

`artex27_manual_land.waypoints` é idêntico, com uma diferença: entre os placeholders e
o `DO_RETURN_PATH_START` entra uma **descida nominal própria** (4 × `LOITER_TO_ALT`:
1500 / 1000 / 500 / 150 m) terminada num **`LOITER_UNLIM`** a 150 m, raio 200 m, 300 m à
frente da pista no eixo de aproximação. O piloto assume aí e pousa à mão. A descida
nominal é obrigatória: **não dá para ir de 2000 m direto para um loiter a 150 m** — o
validador reprova isso. A sequência `DO_RETURN_PATH_START → descida → DO_LAND_START →
LAND` continua lá, intocada, porque é ela que o failsafe usa.

> O `LOITER_UNLIM` fica **deslocado 300 m da pista, para dentro do círculo**, e não
> exatamente sobre ela: com raio de 200 m, um loiter centrado na pista sairia ~130 m
> para fora da área UAS.

---

## 5. Números da espiral

Premissas: `AIRSPEED_CRUISE = 15 m/s`, `TECS_CLMB_MAX = 0,08 × (V − vento)` (a fórmula do
BRIEF, com 20 % de margem sobre os 10 %), entrada em AUTO a **100 m** sobre a pista.

| | sem vento | vento de frente 5 m/s |
|---|---|---|
| `TECS_CLMB_MAX` | **1,20 m/s** | **0,80 m/s** |
| gradiente resultante | **8,0 %** | **8,0 %** |
| raio comandado (`param2`) | 400 m | 400 m |
| raio efetivo a 300 m / a 2000 m | 416 m / **492 m** | idem |
| tempo por volta (nível do mar) | 168 s | 168 s |
| ganho por volta | ~201 m | ~134 m |
| subida 100 → 2000 m | **1583 s ≈ 26,4 min** | **2375 s ≈ 39,6 min** |
| distância percorrida na subida | **23,8 km** | 35,6 km |
| voltas | **≈ 9,4** | ≈ 14,2 |

Número de `LOITER_TO_ALT` de subida: **9**, com ganhos de 200 m (×7) e 250 m (×2) —
dentro da faixa de 200–250 m pedida.

Descida (caminho de retorno), com `TECS_SINK_MAX = 2,0 m/s`: 2000 → 140 m em
**≈ 930 s (15,5 min)**, **13,9 km**, **≈ 5,0 voltas**, gradiente 13,3 %.

**Margem de contenção:** o raio efetivo máximo é 492 m (no topo) contra os 997 m da área —
sobram **~505 m** até a borda. O ponto mais exposto da missão é o próprio HOME/LAND
(925,8 m do centro, 71,5 m da borda).

### Por que o raio efetivo cresce com a altitude

`AP_L1_Control::loiter_radius()` (`libraries/AP_L1_Control/AP_L1_Control.cpp:148`): com
`NAVL1_LIM_BANK = 0` — que é o **default** (`AP_GROUPINFO("LIM_BANK", 3, …, 0.0f)`) — a
função devolve `radius * EAS2TAS²` incondicionalmente. A 2000 m isso é ×1,23.

Se quiserem o raio **fixo** em 400 m em toda a subida, pôr `NAVL1_LIM_BANK` ≠ 0 (ex.: 25):
aí a função devolve `MAX(V²/(tan(bank)·g) · EAS2TAS², radius)` = `MAX(≈60 m, 400 m)` = 400 m.
Item para o agente de parâmetros.

---

## 6. `LOITER_TO_ALT` — o que cada parâmetro faz de verdade (ArduPlane 4.8)

Verificado em `libraries/AP_Mission/AP_Mission.cpp:1178` (`mavlink_to_mission_cmd`),
`libraries/AP_Mission/AP_Mission.cpp:1712` (`mission_cmd_to_mavlink`),
`ArduPlane/commands_logic.cpp:539` (`do_loiter_to_alt`) e `:780` (`verify_loiter_to_alt`).

```c
case MAV_CMD_NAV_LOITER_TO_ALT:                     // MAV ID: 31
    cmd.p1 = fabsf(packet.param2);                  // param2 is radius in meters
    cmd.content.location.loiter_ccw = (packet.param2 < 0);
    cmd.content.location.loiter_xtrack = (packet.param4 > 0);
    break;
```

| Param | MAVLink diz | **ArduPlane 4.8 faz** |
|---|---|---|
| **param1** (*Heading Required*) | 0 = sai assim que atinge a altitude; 1 = só sai alinhado com o próximo WP | **IGNORADO.** O código nunca lê `packet.param1` para o cmd 31. O alinhamento de saída é **sempre** exigido: `verify_loiter_to_alt()` chama `verify_loiter_heading()` depois de atingir a altitude. **Pôr 0** (é o que os arquivos têm) — o valor é irrelevante. |
| **param2** (*Radius*) | raio em m | **Raio em metros, e o sinal define o sentido**: negativo = anti-horário. `param2 = 0` (ou \|param2\| ≤ 1) → cai no `WP_LOITER_RAD` (`ArduPlane/navigation.cpp:362-372`), que no avião está em **60 m** — pequeno demais. Por isso os arquivos escrevem **400** explicitamente. |
| **param3** | — | não é lido. |
| **param4** (*Xtrack*) | 0 / 1 | `loiter_xtrack = (param4 > 0)`. 0 = o crosstrack para a perna seguinte é medido **a partir do centro** do loiter; 1 = **a partir do ponto de saída pela tangente**. Os `LOITER_TO_ALT` empilhados usam **0**; o **último da descida** usa **1**, para interceptar a reta da aproximação em vez de apontar do centro. |

**Por que empilhar `LOITER_TO_ALT` no mesmo lat/lon funciona:** quando a altitude é
atingida, `verify_loiter_heading()` compara o rumo com o alvo do próximo nav cmd. Se o
próximo item está no **mesmo ponto** que o centro do loiter, `ModeLoiter::isHeadingLinedUp()`
(`ArduPlane/mode_loiter.cpp:61-64`) detecta `target_dist == 0` e devolve `true`
imediatamente — não há espera de alinhamento entre degraus. A subida é uma espiral contínua.

**Proteção anti-travamento:** `Plane::loiter_angle_update()` (`ArduPlane/navigation.cpp:70-78`)
marca `unable_to_achieve_target_alt` se, a cada **3 voltas**, a altitude variou menos de
**5 m**. Com 1,2 m/s e ~168 s por volta ganham-se ~600 m em 3 voltas — folga enorme.
A altitude é considerada atingida com **±5 m** (`labs(current_loc.alt - target) < 500` cm).

---

## 7. `DO_RETURN_PATH_START` em 4.8 — comportamento confirmado

Fonte: `libraries/AP_Mission/AP_Mission.cpp:2523` (`jump_to_closest_mission_leg`) e
`:2726` (`distance_to_mission_leg`); `ArduPlane/mode_rtl.cpp:126`;
`ArduPlane/Parameters.cpp:722-723`; `ArduPlane/AP_Arming_Plane.cpp:430`.

1. **`RTL_AUTOLAND = 4`** é o valor documentado no próprio código como
   *"Go directly to landing sequence via DO_RETURN_PATH_START mission item"*. Em `MODE RTL`,
   `ModeRTL::navigate()` chama `mission.jump_to_closest_mission_leg(current_loc)` e, se
   encontrar, faz `set_force_resume(true)` e troca para **AUTO**.
2. **Ordem exigida: `DO_RETURN_PATH_START` tem de vir ANTES do `DO_LAND_START`.**
   `distance_to_mission_leg()` percorre a missão **para a frente** a partir do
   `DO_RETURN_PATH_START` e **para** no primeiro `DO_LAND_START` ou comando de pouso
   (`goto reset_do_jump_tracking`). Se não encontrar nenhum antes do fim da missão,
   devolve `ret = false` e o retorno **falha**. Ou seja: sim, podem (e **têm de**) estar na
   mesma missão, nessa ordem, com os waypoints do percurso de aproximação entre eles.
3. **Como escolhe o ponto de reengate:** calcula, para cada perna entre dois itens
   posicionados do caminho de retorno, a distância **3D** (`get_distance_NED_alt_frame`)
   do avião à projeção na perna. Escolhe a perna mais próxima e salta para o índice do
   item final dela.
   É por isso que a espiral de descida é feita de `LOITER_TO_ALT` **empilhados no mesmo
   centro**: o vetor da perna fica puramente vertical, e a distância de reengate passa a ser
   «distância horizontal + excesso vertical fora da faixa de altitude». O avião reengata
   automaticamente na **faixa de altitude em que está**. Um failsafe a 600 m entra no
   degrau 1000→500; a 1800 m entra no degrau 2000→1500.
4. **Não reentra se já está a voltar:** `_flags.in_return_path` / `in_landing_sequence`
   bloqueiam um novo salto (`"Return path active"` / `"Landing sequence active"`).
5. **Arming check:** com `RTL_AUTOLAND = 0` e um `DO_RETURN_PATH_START` na missão, o
   arming **falha** com *"DO_RETURN_PATH_START set and RTL_AUTOLAND disabled"*
   (`AP_Arming_Plane.cpp:430`). O avião tem hoje `RTL_AUTOLAND = 0` → **tem de mudar para 4
   antes de carregar esta missão**, senão não arma.
6. Os itens `DO_RETURN_PATH_START` e `DO_LAND_START` são `stored_in_location`, ou seja,
   guardam lat/lon. Os arquivos escrevem **0,0** de propósito: assim `location.initialised()`
   é falso e eles não entram no cálculo de distância das pernas.

---

## 8. Validador

```
$ ./validate_mission.py artex27.waypoints
...
-- SEQUENCIA
  [ OK ] DO_RETURN_PATH_START (idx [12]) antes de DO_LAND_START (idx [17])
  [ OK ] caminho de retorno com 4 itens posicionados (idx 13..16)
  [ OK ] LAND e o ultimo item (idx 20)
  [ OK ] item imediatamente antes do LAND e um WAYPOINT (idx 19, 60 m)
...
RESULTADO: 0 erro(s), 1 aviso(s)
```

O único aviso é o teto, que fica em aberto enquanto não se souber a elevação da pista:

```
-- TETO
         altitude relativa maxima na missao: 2000 m
  [AVISO] --home-amsl nao informado: a elevacao do terreno na pista e PENDENCIA
          aberta no BRIEF; o teto nao pode ser fechado.
         para respeitar o teto de 3000 m AMSL da area UAS, a pista tem de estar
          abaixo de 1000 m AMSL.
         para o teto de 4267 m AMSL do LP-D25, abaixo de 2267 m AMSL.
```

Opções úteis:

```bash
./validate_mission.py artex27.waypoints --wind 5          # vento de frente
./validate_mission.py artex27.waypoints --home-amsl 120   # fecha o teto
./validate_mission.py artex27.waypoints --max-grad-descent 10   # 10 % também na descida
./validate_mission.py artex27.waypoints --clmb-max 1.4 --sink-max 2.5
```

Notas sobre o que o validador faz e não faz:

- O **gradiente de subida** usa o pior caso de groundspeed (`V − vento`), igual à fórmula
  do BRIEF. Default `--max-grad 10`.
- O **gradiente de descida** tem limite próprio (`--max-grad-descent`, default **25 %**).
  Os 10 % do exercício são uma regra de **subida**; na descida o limite default é só um
  teste de sanidade de afundamento. Para aplicar os 10 % também na descida, passar
  `--max-grad-descent 10` — nesse caso `TECS_SINK_MAX` tem de descer para ~1,5 m/s e a
  descida passa a demorar ~21 min.
- A perna **0 → 1** não é verificada: em AUTO a rampa parte da posição e altitude **atuais**
  (entrada combinada a ~100 m sobre a pista), não do HOME.
- A cadeia é **cortada** no `DO_RETURN_PATH_START` (percurso independente) e no
  `LOITER_UNLIM` (nunca termina), para não inventar pernas que ninguém voa.
- Numa perna cujo destino é um `LOITER_TO_ALT`, a rampa geométrica **não é vinculante** —
  a altitude é ganha/perdida dentro do próprio loiter, limitada pelo TECS. O validador
  trata esse caso pela taxa, não pela geometria.

---

## 9. Discrepâncias BRIEF ↔ código

Nenhuma contradição de fundo. Dois ajustes de precisão e um ponto que o BRIEF não cobria:

| Item | BRIEF | Código / KML | Usado |
|---|---|---|---|
| Raio da área UAS | 996 m | ajuste de círculo sobre os 73 vértices do KML: **997,25 m** (resíduo 0,017 m) | 997,25 m |
| Pista → centro | 925 m | **925,8 m**, azimute 137,2° | 925,8 m |
| `LOITER_TO_ALT` "Heading = 0?" | em aberto | `param1` **não é lido** pelo firmware; o alinhamento de saída é sempre exigido | `param1 = 0`, irrelevante |
| Raio do loiter com a altitude | não mencionado | `NAVL1_LIM_BANK = 0` (default) ⇒ raio escala com `EAS2TAS²`, +23 % a 2000 m | contabilizado (492 m no topo) |

---

## 10. Pendências que travam o fecho da missão

1. **Elevação do terreno na pista (AMSL).** Sem ela o `HOME` do arquivo fica com `0.0` como
   placeholder e o teto não fecha. Não foi inventado nenhum valor. A missão só é legal se a
   pista estiver **abaixo de 1000 m AMSL** (teto de 3000 m da área UAS).
2. **Direção real da pista (QFU) e cabeceiras.** O KML só traz um ponto. O eixo 137,2°/317,2°
   é geométrico, escolhido para manter a final dentro do círculo. Com o QFU real:
   `./build_mission.py --approach-brg <QFU>` — e verificar que a porta de entrada da final
   (1600 m antes do toque) continua a cair dentro do círculo. **Só há ~1900 m de reta
   utilizável dentro da área**, e numa direção que não a 137,2° pode ser bem menos.
3. **Vento previsto em altitude.** Entra direto em `TECS_CLMB_MAX = 0,08 × (15 − W)` e
   multiplica o tempo de subida (26 min sem vento → 40 min com 5 m/s de frente).
4. **`RTL_AUTOLAND = 4`.** Hoje o avião tem `0`. Com um `DO_RETURN_PATH_START` na missão e
   `RTL_AUTOLAND = 0`, **o arming falha**. Item para o agente de parâmetros.
5. **`TECS_CLMB_MAX` e `TECS_SINK_MAX`.** Hoje ambos a **5 m/s**. Com 5 m/s a subida seria a
   33 % de gradiente — mais de 3× o limite. Valores desta missão: `TECS_CLMB_MAX = 1,2`
   (sem vento) e `TECS_SINK_MAX = 2,0`.
6. **`WP_LOITER_RAD`** está em 60 m. Os `LOITER_TO_ALT` trazem o raio explícito (400 m),
   mas o `RTL_RADIUS` (60 m) e o loiter default continuam pequenos para esta missão.
7. **`FENCE_ALT_MAX = 100` e `FENCE_RADIUS = 300`** são incompatíveis com a missão.
   A fence poligonal/circular desta pasta substitui o `FENCE_RADIUS`; `FENCE_ALT_MAX` tem de
   subir para ≥ 2100 m ou a fence de altitude fica desligada conscientemente.
8. **Autonomia.** 26 min de subida + missão + 16 min de descida. Dimensionar
   `BATT_LOW_MAH` / `BATT_CRT_MAH` sobre os 27 000 mAh depende do consumo medido em subida —
   ainda não existe.
9. **Coordenação de espaço aéreo** acima da Med-Hi Boundary (1371 m AMSL) com o Air Ops Control.
10. **Missão principal a 2000 m** — itens 10 e 11 são placeholders.
