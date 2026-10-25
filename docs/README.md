# docs/

Documentação das missões do ATLAS. Páginas HTML autónomas: abrem no browser, sem servidor
e sem internet.

- **[`index.html`](index.html)** — missão **Icarus** (busca e órbita, sem drop): guia de uso
  completo, como correr, configurar e estender.
- **[`artex27.html`](artex27.html)** — missão **ARTEX 26** (asa fixa, subida a 2000 m com
  gradiente ≤ 10 %): requisito, geografia e espaço aéreo, perfil de subida e descida,
  parâmetros, como correr o pacote `src/artex27/`, como carregar nas GCS, checklist de pré-voo
  e riscos em aberto.

```bash
# no host, a partir da raiz do workspace
xdg-open docs/index.html        # Linux
xdg-open docs/artex27.html
```

## `index.html` — o que cobre

- Pré-requisitos e início rápido (SITL + `ros2 launch atlas_mission mission.launch.py`)
- Como correr no **drone real** (MAVROS no FCU físico, alvo real, checklist de segurança)
- Como o nó funciona, passo a passo
- Parâmetros (`auto_confirm`, `ALTITUDE_FINAL`, `SAFETY_RADIUS`, …)
- Contrato de tópicos e serviços (incl. a regra de QoS `BEST_EFFORT`)
- API da `mavros_library` (sequences, commands, construtores de waypoint)
- Abort e segurança
- Troubleshooting dos erros comuns (`Auto mode not armable`, push lento, frame `TERRAIN_ALT`)

## `artex27.html` — o que cobre

- A missão ARTEX e o requisito dos 10 %: porque é uma espiral de `LOITER_TO_ALT` e porque o
  gradiente é controlado por `TECS_CLMB_MAX = 0,08 × (V − vento)` e não pelo pitch
- Geografia e espaço aéreo (círculo UAS de 997 m, pista a 71,5 m da borda, Med-Hi Boundary)
- Perfil de subida e descida: tabela de vento, tempos, voltas, raio efetivo com a altitude
- Estrutura da missão (21 itens) e a ordem obrigatória `DO_RETURN_PATH_START` → `DO_LAND_START`
- Parâmetros, com o porquê dos principais e a migração de nomes para a 4.8
- Como correr: `colcon build`, os módulos sem ROS, os dois nós e os launch files
- Carregar missão/geofence/KML no QGroundControl, no Mission Planner e no MAVProxy
- Checklist de pré-voo (a *Calypso Pre-Flight Inspection* da ATLAS) e as 17 linhas que mudam
- Riscos em aberto: `SINK_MIN > SINK_MAX`, autonomia vs. tempo de subida, QFU, elevação do
  terreno, coordenação de espaço aéreo
