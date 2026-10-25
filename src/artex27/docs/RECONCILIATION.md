# Reconciliação — divergências entre os agentes de missão e de parâmetros

Os dois agentes correram em paralelo e fecharam números incompatíveis. Este documento decide
qual vale e regista a consequência. **Em caso de dúvida, este ficheiro ganha sobre os READMEs
individuais de `missions/` e `params/`.**

## 1. `TECS_CLMB_MAX` — 1.20 vs 0.80

| | Missão | Parâmetros | **Decidido** |
|---|---|---|---|
| Valor | 1.20 | 0.80 | **0.80** |
| Premissa de vento | 0 m/s | 5 m/s | 5 m/s |
| Subida 100 → 2000 m | 26,4 min | 39,6 min | **39,6 min** |

**Decisão: 0.80.** A fórmula `0.08×(V−W)` é por definição dependente do vento; 1.20 só é válido
com vento rigorosamente nulo, e o vento a 2000 m é tipicamente mais forte que à superfície.
0.80 respeita o limite duro de 10 % até **W = 7 m/s**, o que dá margem operacional real.

**O que isto NÃO muda:** a geometria da missão. Numa espiral de `LOITER_TO_ALT` o gradiente é
controlado **só** pelo TECS — os waypoints não impõem rampa. Os ficheiros `.waypoints` e `.plan`
estão corretos para qualquer valor de `TECS_CLMB_MAX`.

**O que muda:** todos os tempos e distâncias documentados em `missions/README.md`, que foram
calculados com 1.20. Ler os números desta tabela em vez daqueles:

| W [m/s] | 0 | 2 | **5** | 7 | 8 | 10 |
|---|---|---|---|---|---|---|
| `TECS_CLMB_MAX` | 1.20 | 1.04 | **0.80** | 0.64 | 0.56 | 0.40 |
| Subida 100→2000 m | 26,4' | 30,4' | **39,6'** | 49,5' | 56,5' | 79,2' |

`TECS_CLMB_MAX` é um **valor de dia de voo**: definir a partir da previsão de vento em altitude,
não deixar fixo. O gradiente resultante é sempre 8,0 % porque a fórmula se auto-ajusta.

## 2. `TECS_SINK_MAX` — 2.0 vs 3.0

| | Missão | Parâmetros | **Decidido** |
|---|---|---|---|
| Valor | 2.0 | 3.0 | **3.0** |
| Gradiente de descida | 13,3 % | **20 %** | 20 % |
| Descida 2000 → 140 m | 15,5 min | **10,3 min** | 10,3 min |

**Decisão: 3.0**, pelo argumento de envelope do agente de parâmetros: `TECS_SINK_MIN=2` é o
afundamento ao ralenti, logo qualquer `SINK_MAX` acima disso faz o TECS converter altitude em
velocidade, empurrando para `AIRSPEED_MAX=27`. Com 5.0 seriam 6,7 min contínuos perto de 27 m/s.
3.0 encurta a exposição e ainda poupa 5 min de autonomia face a 2.0 — que, como se vê a seguir,
é decisivo.

**Ponto em aberto para si:** nenhum dos dois valores põe a **descida** abaixo de 10 %. O requisito
ARTEX de 10 % é, tanto quanto consta, só de **subida**. Se também valer para a descida, é preciso
`TECS_SINK_MAX = 1.5` — e aí a descida passa a 20,7 min, o que agrava o problema da secção 4.
**Confirme com a organização do exercício.**

## 3. `WP_LOITER_RAD` e o raio da espiral — não é conflito, mas tem uma ponta solta

`WP_LOITER_RAD` passou de 60 para **200**. Não colide com a missão, porque todos os
`LOITER_TO_ALT` escrevem **`param2 = 400` explícito**. O 200 só se aplica ao RTL e a loiters com
`param2 = 0`.

**Ponta solta real:** o agente de missão apontou que, com **`NAVL1_LIM_BANK = 0`** (valor atual),
`AP_L1_Control::loiter_radius()` devolve `raio × EAS2TAS²` — o raio cresce com a altitude, e os
400 m comandados tornam-se **~492 m a 2000 m**. Sinalizou isto para o agente de parâmetros, que
**não agiu**: `NAVL1_LIM_BANK` continua em 0.

**Decisão: aceitar e documentar.** 492 m contra 997 m de raio da área deixa ~505 m de folga —
confortável. Fixar o raio exigiria `NAVL1_LIM_BANK ≠ 0`, o que introduz um limite de inclinação
não testado neste avião. Não vale o risco por um ganho estético. **Verificar em SITL** que o raio
efetivo a 2000 m não passa dos ~500 m previstos.

## 4. Consequência não prevista: a autonomia deixa de fechar

Esta é a verdadeira saída da reconciliação, e **nenhum dos dois agentes a viu**, porque cada um
só tinha metade dos números.

Com `TECS_CLMB_MAX = 0.80` a subida passa de 26 para **39,6 min**. Somada à descida (10,3 min),
são **~50 min só para subir e descer**, antes de qualquer missão a 2000 m.

O failsafe de bateria baixa dispara a `BATT_LOW_MAH = 8100`, ou seja há **18 900 mAh** utilizáveis
antes do RTL. Quanto tempo isso dá depende do consumo médio — que **nunca foi medido**:

| Consumo | Janela até o RTL | Sem vento (sub 26,4' + des 10,3') | Vento 5 m/s (sub 39,6' + des 10,3') |
|---|---|---|---|
| 15 A | 75,6' | sobram 38,9' para a missão | sobram 25,7' |
| 20 A | 56,7' | sobram 20,0' | **sobram 6,8' — apertado** |
| 25 A | 45,4' | sobram 8,6' — apertado | **−4,6' — inviável** |
| 30 A | 37,8' | sobram 1,1' — apertado | **−12,1' — inviável** |

E com **vento ≥ 10 m/s a subida sozinha leva 79 min**, acima de qualquer uma destas janelas: a
missão fica impossível independentemente do consumo.

**Conclusões:**
1. **O consumo médio em subida é agora o dado que bloqueia a missão**, não um detalhe de
   dimensionamento de failsafe. Medir numa bancada ou num voo curto, à potência de subida.
2. Existe um **teto de vento operacional**. Com 20 A, acima de ~5 m/s a missão deixa de caber.
   Calcular o teto exato quando houver consumo medido e transformá-lo num critério de GO/NO-GO.
3. Se o consumo real for ≥ 25 A, as saídas são: bateria maior, aceitar menos de 2000 m, ou subir
   com `TECS_CLMB_MAX` mais alto e **gradiente acima de 10 %** — que viola o requisito.

## 5. Resumo do que fazer

- Usar os ficheiros de `missions/` como estão: a geometria é válida.
- Usar `params/artex_2000m.param` como está: os valores decididos já lá estão (`TECS_CLMB_MAX=0.8`,
  `TECS_SINK_MAX=3`, `WP_LOITER_RAD=200`, `NAVL1_LIM_BANK=0`).
- **Ignorar os tempos e distâncias de `missions/README.md`**, que assumem 1.20. Valem os da secção 1.
- Antes do voo: medir o consumo em subida, confirmar se o limite de 10 % também se aplica à
  descida, e obter o QFU real da pista e a elevação do terreno.
