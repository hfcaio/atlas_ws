# `params/` — Parâmetros ArduPlane da missão ARTEX 2000 m

Firmware alvo: **ArduPlane V4.8.0-dev**. Qualquer outra versão invalida este conjunto
(nomes de parâmetros mudaram — ver secção "Migração" em `artex27_diff.md`).

## Ficheiros

| Ficheiro | O que é | Para quem |
|---|---|---|
| `artex27.param` | Conjunto **completo** (1134 parâmetros), pronto a carregar em Mission Planner / QGC. Derivado de `../../../../Param ACC26 26-09-2026 2202.param` com 23 alterações de missão + 7 correções de nomes 4.8 | voo real |
| `artex27_diff.md` | A tabela de diff justificada, parâmetro a parâmetro, com referência ao código-fonte. **É o documento que interessa ler** | revisão / documentação |
| `artex27_sitl.param` | **Delta**, não ficheiro completo. Só os valores de missão + os `SIM_*`. Sem calibrações, pinos, RC, servos ou ganhos de PID do avião real | agente de simulação |

O ficheiro original `Param ACC26 26-09-2026 2202.param` **não foi modificado**.

---

## Como carregar — voo real

**Ordem recomendada:**

1. **Confirmar a versão do firmware a bordo.** Se não for 4.8.x, parar aqui.
   Mission Planner → `Help` → o banner da versão, ou no MAVProxy: `version`.
2. **Guardar os parâmetros atuais** antes de qualquer coisa:
   Mission Planner → `CONFIG` → `Full Parameter List` → `Save to file`.
   É o único caminho de volta.
3. **Carregar `artex27.param`:** `Load from file` → `Write Params`.
4. **Reiniciar o autopiloto** e reler os parâmetros (`Refresh Params`) para confirmar que
   todos foram aceites. Qualquer linha rejeitada aparece no log da GCS.
5. **Carregar o ficheiro de geofence de inclusão** (entrega do agente de missão).
   **Sem ele o pre-arm falha** com `Polygon fence(s) invalid`, porque `FENCE_TYPE` = 5 inclui o
   bit 2 (inclusão/exclusão).
6. **Carregar a missão** (`.waypoints`). Tem de conter `DO_RETURN_PATH_START` (cmd 188) **e**
   `DO_LAND_START` (cmd 189) + `LAND`, senão o `RTL_AUTOLAND` = 4 não tem para onde saltar.
7. **Ajustar `TECS_CLMB_MAX` ao vento previsto** — ver a tabela abaixo. É o único parâmetro que
   muda de dia para dia.
8. Correr os pre-arm checks e resolver tudo o que aparecer.

---

## O que ajustar no campo, antes de armar

### `TECS_CLMB_MAX` — depende do vento, todos os dias

`TECS_CLMB_MAX = 0.08 × (15 − vento_de_frente)`

| Vento [m/s] | `TECS_CLMB_MAX` | Tempo de subida a 2000 m |
|---|---|---|
| 0 | 1.20 | 28 min |
| 2 | 1.04 | 32 min |
| 4 | 0.88 | 38 min |
| **5** | **0.80** ← valor no ficheiro | **42 min** |
| 6 | 0.72 | 46 min |
| 8 | 0.56 | 60 min |
| 10 | 0.40 | 83 min |

O valor gravado (0.80) respeita o limite duro de 10 % até 7 m/s de vento de frente.
**Acima de 7 m/s tem de ser baixado.** Acima de ~8 m/s a subida deixa de caber na autonomia —
nesse caso a decisão é adiar, não forçar o parâmetro.

### Verificações no chão

| O quê | Como | Critério |
|---|---|---|
| `THR_FS_VALUE` = 950 | Desligar o emissor com o avião desarmado e ligado | O PWM de throttle tem de cair abaixo de 950 e a GCS anunciar "Throttle failsafe on" |
| Fence carregada | `FENCE_ENABLE` = 1, `FENCE_AUTOENABLE` = 3 (ativa ao armar) | Sem pre-arm de fence |
| Missão | `DO_RETURN_PATH_START` + `DO_LAND_START` + `LAND` presentes | Sem pre-arm `DO_LAND_START set and RTL_AUTOLAND disabled` |
| Altitude da fence | `FENCE_ALT_MAX` = 2100 m **acima do home** (`FENCE_ALT_MAX_TP` = 1) | 2100 + elevação da pista < 3000 m AMSL |

### Verificações em voo (primeira saída)

| O quê | Como | O que fazer se falhar |
|---|---|---|
| Acelerador não satura na subida | Observar CTUN/ThrOut durante a subida em AUTO | Se ficar colado a 100 %, o avião não tem potência para a razão de subida pedida — baixar `TECS_CLMB_MAX` |
| Acelerador não oscila | Observar ThrOut e a velocidade | Subir `TECS_TIME_CONST` de 8 para 10, e/ou `TECS_THR_DAMP` de 0,5 para 0,7 |
| Pitch não satura na subida | Comparar o pitch com `TECS_PITCH_MAX` = 10 | Se saturar, medir `PTCH_TRIM_DEG` e subir `TECS_PITCH_MAX` para `PTCH_TRIM_DEG + 6` |
| Velocidade estável a 15 m/s na subida | Log ARSP | Se cair abaixo de 14, baixar `TECS_CLMB_MAX` |
| Descida controlada | Observar a velocidade na descida a `TECS_SINK_MAX` = 3 | Se a velocidade subir muito acima de 15, baixar `TECS_SINK_MAX` |

---

## Parâmetros que ainda estão "a definir"

Estes ficaram com o valor conservador ou desativados **de propósito**, por falta de dados.
A lista completa, com a origem de cada um, está na secção 9 de `artex27_diff.md`.

| Parâmetro | Estado | Dado que falta |
|---|---|---|
| `BATT_LOW_VOLT`, `BATT_CRT_VOLT` | 0 (desativados) | Número de células da bateria |
| `BATT_LOW_MAH`, `BATT_CRT_MAH` | 8100 / 5400 (premissa: 30 % / 20 %) | Consumo médio em A |
| `LAND_PITCH_DEG` | 0 | Tipo de trem de aterragem |
| `AIRSPEED_STALL` | 0 (assume `AIRSPEED_MIN` = 14) | Velocidade de perda real |
| `TECS_SINK_MIN` | 2 (default de fábrica) | Afundamento ao ralenti medido |
| `LAND_FLARE_ALT`, `LAND_FLARE_SEC`, `TECS_LAND_SINK` | defaults | Afinação após o primeiro pouso automático |

---

## O que reverter depois do voo

Nada tem de ser revertido para voo normal **exceto** se o avião for usado noutra missão. Nesse
caso os parâmetros abaixo são específicos do ARTEX 2000 m e sabotam um voo normal:

| Parâmetro | Valor ARTEX | Repor para |
|---|---|---|
| `TECS_CLMB_MAX` | 0.8 | 5 (ou a razão de subida real medida) |
| `TECS_SINK_MAX` | 3 | 5 |
| `TECS_PITCH_MAX` | 10 | 15 |
| `TECS_PITCH_MIN` | -15 | 0 |
| `TECS_TIME_CONST` | 8 | 5 |
| `PTCH_LIM_MAX_DEG` | 15 | 25 |
| `WP_LOITER_RAD` | 200 | 60 |
| `FENCE_ALT_MAX` | 2100 | o teto da nova missão |
| `FENCE_TYPE` | 5 | 4, se não houver fence de inclusão carregada |

A forma segura de reverter é recarregar o ficheiro guardado no passo 2.

**Em SITL:** repor `SIM_RC_FAIL` a 0 e `SIM_WIND_SPD` a 0 no fim de cada teste — ficam gravados
e contaminam o teste seguinte.

---

## Como carregar — SITL

```bash
# a partir de src/ardupilot/
Tools/autotest/sim_vehicle.py -v ArduPlane -f plane \
    --add-param-file=../../params/artex27_sitl.param \
    --console --map
```

ou, com o SITL já a correr, no MAVProxy:

```
param load ../../params/artex27_sitl.param
```

Para testar sem o ficheiro de geofence: `param set FENCE_TYPE 1` (só a fence de altitude).

Os `SIM_*` estão no ficheiro em estado de repouso (sem vento, sem falha de RC). Os testes de
failsafe são do agente de simulação; os botões são `SIM_RC_FAIL`, `SIM_BATT_CAP_AH`,
`SIM_BATT_VOLTAGE`, `SIM_WIND_SPD` e `SIM_WIND_DIR`.

---

## Atenção

- **`AIRSPEED_CRUISE` real é 15 m/s**, não 14 como está no `CLAUDE.md` da raiz. Todas as contas
  deste diretório usam 15.
- A nota do Notion que fala em `LIM_PITCH_MAX` a "600–700 centigraus" é de uma versão ≤ 4.4. Na
  4.8 o parâmetro chama-se `PTCH_LIM_MAX_DEG` e está em **graus**. O equivalente seria 6–7 — e
  **essa proposta foi rejeitada**; ver a secção 2.2 de `artex27_diff.md` para o porquê.
- Os valores TECS do ficheiro de referência não são medidas do avião: são os defaults de fábrica
  da ArduPlane. O TECS deste avião nunca foi afinado. Ver a secção 0 do diff.
