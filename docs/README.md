# docs/

Documentação da missão **Icarus** (busca e órbita, sem drop).

- **[`index.html`](index.html)** — guia de uso completo: como correr, configurar e estender
  a missão. Abre no browser (não precisa de servidor nem de internet).

```bash
# no host, a partir da raiz do workspace
xdg-open docs/index.html      # Linux
```

## O que cobre

- Pré-requisitos e início rápido (SITL + `ros2 launch atlas_mission mission.launch.py`)
- Como correr no **drone real** (MAVROS no FCU físico, alvo real, checklist de segurança)
- Como o nó funciona, passo a passo
- Parâmetros (`auto_confirm`, `ALTITUDE_FINAL`, `SAFETY_RADIUS`, …)
- Contrato de tópicos e serviços (incl. a regra de QoS `BEST_EFFORT`)
- API da `mavros_library` (sequences, commands, construtores de waypoint)
- Abort e segurança
- Troubleshooting dos erros comuns (`Auto mode not armable`, push lento, frame `TERRAIN_ALT`)
