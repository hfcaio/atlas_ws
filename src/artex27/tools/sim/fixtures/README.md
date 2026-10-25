# fixtures/ — so para smoke-test

Estes ficheiros **nao sao** a missao nem os parametros de voo do ARTEX.
Servem apenas para validar que os scripts de `src/artex27/tools/sim/` correm de ponta
a ponta mesmo antes de os ficheiros reais existirem.

Os ficheiros de referencia sao:

- `src/artex27/missions/*.waypoints` (agente de missao)
- `src/artex27/params/artex27_sitl.param` (agente de parametros)

Quando esses existirem, corre os testes sem `--mission`/`--params` — os scripts
vao busca-los sozinhos.
