# CLAUDE.md

Regras de uso de IA neste repositório. Ler antes de mexer em qualquer coisa.
Contexto do projeto (o quê, como rodar) está no `README.md` e em `docker/README.md`.

## Commits e ações

- Nunca criar commits, criar/apagar branches, fazer push, ou qualquer outra ação em
  `git` sem pedido explícito do usuário para aquela mudança específica. Um pedido
  genérico ("organiza isso") não autoriza decisões de conteúdo novas e não discutidas
  (ex.: reescrever um ficheiro inteiro) — confirmar antes de commitar essas.
- Nunca `--no-verify`, `--force`, `reset --hard`, `clean -f` ou outra operação
  destrutiva sem confirmação explícita, mesmo havendo backup.

## Escopo de edição

- Só editamos: `atlas_mission`, `atlas_bringup`, `mavros_library`, `icarus_tests`,
  `artex27` e `docker/`.
- Repos upstream (`ardupilot`, `ardupilot_gazebo`, `ardupilot_gz`, `SITL_Models`)
  nunca são editados diretamente. Correções entram como configuração do nosso lado
  ou como patch documentado em `docker/patches/`.

## Ambiente

- Todo o desenvolvimento e execução acontecem **dentro do container**
  `atlas-sim:humble`. Não assumir que o host tem ROS, Gazebo ou ArduPilot.
- O workspace do container compila em `build_docker/`/`install_docker/`, **nunca**
  em `build/`/`install/` (são do host; prefixos absolutos diferentes).
- Usar `atlas-build` (nunca `colcon build` direto — perde os paths certos).

## Convenções de código

- Python 3.10, `rclpy`. Pacotes nossos são `ament_python`.
- MAVROS: usar a `mavros_library` em vez de falar com tópicos/serviços crus. Se algo
  lá não existe, estender a lib — não duplicar lógica no nó de missão.
- Tópicos MAVROS de sensor/estado precisam QoS **BEST_EFFORT/VOLATILE**; com QoS
  default não chega nada.
- Nada de `time.sleep()` bloqueante dentro de callbacks; usar timers do rclpy.
- Comandos que armam ou movem o veículo ficam atrás de uma máquina de estados
  explícita, nunca disparados direto num callback de sensor.

## Antes de propor mudanças

- Correr `ap-check` primeiro e olhar o que já existe.
- Recompilar só o pacote alterado (`--packages-select`), não o workspace todo.
- Testar em SITL antes de qualquer coisa; `SPEEDUP=5` acelera.
- Não commitar `build*/`, `install*/`, `log*/`, `*.tlog`, `*.BIN`, `logs/`.

## Onde encontrar o resto

- Setup, comandos e regras gerais do repo: `README.md`.
- Ambiente docker em detalhe (imagem, portas, variáveis): `docker/README.md`.
- Missão Icarus: `docs/index.html`. Missão ARTEX: branch `comp/artex27`
  (`docs/artex27.html`, `src/artex27/README.md`).
