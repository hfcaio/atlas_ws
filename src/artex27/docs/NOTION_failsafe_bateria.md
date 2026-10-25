## Failsafe Bateria

Define o que o avião faz quando a bateria chega ao fim. São **dois níveis** — baixa e crítica —
e cada um dispara uma ação diferente. Nos parâmetros que vinham da ACC26 ambos estavam
**desligados** (`BATT_FS_LOW_ACT = 0`, `BATT_FS_CRT_ACT = 0`).

### Parâmetros

| Parâmetro | Valor ARTEX | O que faz |
|---|---|---|
| `BATT_MONITOR` | 4 | Tensão + corrente. Sem isto não há contagem de mAh |
| `BATT_CAPACITY` | 27000 | mAh da bateria. É a base de todos os cálculos |
| `BATT_LOW_MAH` | 8100 | Nível **baixo**: 30 % de reserva |
| `BATT_CRT_MAH` | 5400 | Nível **crítico**: 20 % de reserva |
| `BATT_FS_LOW_ACT` | 1 | Ação no nível baixo → **RTL** |
| `BATT_FS_CRT_ACT` | 2 | Ação no nível crítico → **Land** |
| `BATT_LOW_VOLT` | 0 | Desligado — falta saber o nº de células |
| `BATT_CRT_VOLT` | 0 | Idem |
| `BATT_LOW_TIMER` | 10 | Segundos de tensão baixa antes de disparar. **Só se aplica à tensão** |
| `BATT_FS_VOLTSRC` | 0 | Tensão crua (1 = compensada pela resistência interna) |

Valores de `BATT_FS_*_ACT` (confirmado em `ArduPlane/Plane.h:1245`):
`0` = nada · `1` = RTL · `2` = Land · `3` = Terminate

### O detalhe que engana toda a gente

**`BATT_LOW_MAH` e `BATT_CRT_MAH` são capacidade que SOBRA, não capacidade gasta.**
O código compara `pack_capacity − consumed` com o limiar
(`AP_BattMonitor_Backend.cpp:276`). Com 27 000 mAh e `BATT_LOW_MAH = 8100`, o failsafe
dispara depois de consumir **18 900 mAh**.

O valor que lá estava, `BATT_LOW_MAH = 500`, era 1,8 % da bateria — na prática, inútil:
disparava quando já não havia nada para fazer.

**Os limiares de mAh disparam sem temporizador.** O `BATT_LOW_TIMER` só filtra os limiares
de **tensão**, que servem para ignorar quedas momentâneas sob carga. Em mAh não há ruído,
logo não há atraso.

### O que cada nível faz, na prática

**Nível baixo → RTL (`BATT_FS_LOW_ACT = 1`)**
Com `RTL_AUTOLAND = 4`, o RTL salta para a perna mais próxima do `DO_RETURN_PATH_START` e
**segue o caminho de regresso planeado** até ao `DO_LAND_START` e ao `LAND`. É o
comportamento que se quer: a descida controlada, dentro da área, pela trajetória definida.

**Nível crítico → Land (`BATT_FS_CRT_ACT = 2`)**
Aqui o comportamento é **diferente e é preciso perceber porquê**. O código
(`ArduPlane/events.cpp:287`) faz, por esta ordem:

1. `is_best_land_sequence()` — se continuar a missão já chega a uma aterragem em menos
   distância, **continua a missão** e só levanta a flag de "em sequência de aterragem".
2. Senão, `jump_to_landing_sequence()` → salta **directamente para o `DO_LAND_START`** e
   mete AUTO.
3. Se não houver `DO_LAND_START`, cai para o comportamento de RTL.

Ou seja: **o nível crítico ignora o `DO_RETURN_PATH_START` e corta para a aterragem.**
Faz sentido — em bateria crítica quer-se o caminho mais curto para o chão — mas significa
que o avião **não** segue o percurso de regresso planeado. Convém saber isto antes de ver
acontecer no campo.

### Como testar em SITL

```
param set SIM_BATT_VOLTAGE 10.0     # força tensão baixa
param set SIM_BATT_CAP_AH 1.0       # ou reduz a capacidade para gastar depressa
```

Alternativa sem mexer na simulação: subir `BATT_LOW_VOLT` acima da tensão atual, que
dispara de imediato.

O que verificar:
- mensagem de failsafe no STATUSTEXT
- mudança de modo para RTL (nível baixo) ou AUTO+`DO_LAND_START` (nível crítico)
- em que item da missão o avião entra
- **testar a várias altitudes** — perto da pista e a 2000 m. O comportamento difere, porque
  a perna de regresso mais próxima depende da posição 3D.

### O problema que isto destapou

Esta é a parte que interessa mais do que os parâmetros.

Com `TECS_CLMB_MAX = 0.8` a subida de 100 a 2000 m leva **~40 min**. Com
`TECS_SINK_MAX = 1.0` (os 10 % aplicados também à descida) a descida leva **~31 min**.
São **~71 min só para subir e descer**, antes de qualquer missão a 2000 m.

Os 18 900 mAh utilizáveis antes do failsafe dão:

| Consumo médio | Janela até o RTL | Sobra para a missão |
|---|---|---|
| 15 A | 75,6 min | 4,6 min |
| 20 A | 56,7 min | **negativo** |
| 25 A | 45,4 min | **negativo** |

E com vento ≥ 10 m/s de frente a subida sozinha passa de 79 min, acima de qualquer janela.

**Conclusão: o consumo médio em subida deixou de ser um detalhe de dimensionamento e passou
a ser o dado que decide se a missão é viável.** Precisa de ser medido — bancada ou voo curto,
à potência de subida. Enquanto não houver esse número, não é possível afirmar que a missão
cabe na bateria.

Se o consumo real for alto, as saídas são: bateria maior, aceitar menos de 2000 m, ou subir
com gradiente acima de 10 % — que viola o requisito.

### Pendências

- **Número de células** da bateria, para definir `BATT_LOW_VOLT` / `BATT_CRT_VOLT`
  (regra prática: 3,5 V e 3,3 V por célula).
- **Consumo médio em subida**, em amperes. É o que bloqueia tudo o resto.
- Confirmar que a reserva de 30 %/20 % é adequada depois de haver consumo medido —
  os valores atuais são uma premissa, não uma conta fechada.
