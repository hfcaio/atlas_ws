# Diff de parâmetros — Missão ARTEX 2000 m

Firmware alvo: **ArduPlane V4.8.0-dev** (`src/ardupilot/ArduPlane/version.h`).
Base: `Param ACC26 26-09-2026 2202.param` (1135 linhas, não modificado).
Saída: `artex27.param` (1134 linhas).

Todos os nomes, unidades, ranges, defaults e semânticas abaixo foram confirmados **lendo o
código-fonte em `src/ardupilot/`**, com a referência de ficheiro:linha indicada.

Confiança:
- **Alta** — comportamento confirmado no código e a conta fecha com os dados disponíveis.
- **Média** — comportamento confirmado no código, mas o valor depende de uma premissa (vento,
  consumo, `PTCH_TRIM_DEG`, elevação do terreno) que ainda não está medida.
- **Baixa** — precisa de validação em SITL ou em voo antes de se confiar nele.

---

## 0. AVISO PRÉVIO — os "valores do avião real" são, na maioria, defaults de fábrica

O BRIEF descreve os valores TECS do `Param ACC26` como "já medidos no avião real". **Não são.**
Comparando com os defaults declarados no código 4.8:

| Parâmetro | Valor no ficheiro | Default no código 4.8 | Fonte |
|---|---|---|---|
| `TECS_CLMB_MAX` | 5 | **5.0** | `AP_TECS.cpp:27` |
| `TECS_SINK_MIN` | 2 | **2.0** | `AP_TECS.cpp:35` |
| `TECS_SINK_MAX` | 5 | **5.0** | `AP_TECS.cpp:115` |
| `TECS_TIME_CONST` | 5 | **5.0** | `AP_TECS.cpp:43` |
| `TECS_PITCH_MAX` | 15 | **15** | `AP_TECS.cpp:147` |
| `TECS_PITCH_MIN` | 0 | **0** | `AP_TECS.cpp:155` |
| `TECS_LAND_SINK` | 0.25 | **0.25** | `AP_TECS.cpp:164` |
| `TECS_LAND_PMAX` | 10 | **10** | `AP_TECS.cpp:187` |
| `TECS_LAND_ARSPD` | -1 | **-1** | `AP_TECS.cpp:123` |
| `TECS_LAND_THR` | -1 | **-1** | `AP_TECS.cpp:131` |
| `LAND_FLARE_ALT` | 3 | **3.0** | `AP_Landing.cpp:FLARE_ALT` |
| `LAND_FLARE_SEC` | 2 | **2.0** | `AP_Landing.cpp:FLARE_SEC` |
| `LAND_PITCH_DEG` | 0 | **0** | `AP_Landing.cpp:PITCH_DEG` |
| `PTCH_TRIM_DEG` | 0 | **0.0** | `Parameters.cpp:658` |

Só `PTCH_LIM_MAX_DEG` (25, default 20) e os ganhos de PID foram de facto mexidos.
**Conclusão operacional: o TECS deste avião nunca foi afinado.** A razão de subida real a
`THR_MAX`, a razão de afundamento ao ralenti e o pitch de trim são todos desconhecidos. Isto
limita a confiança de vários valores abaixo a Média/Baixa e torna o voo de verificação obrigatório.

---

## 1. Migração 4.6/4.7 → 4.8 (nomes que já não existem)

Estes não são decisões de missão — são correções sem as quais o ficheiro não carrega limpo na 4.8.

| Parâmetro antigo | Novo nome (4.8) | Valor | Fonte | Confiança |
|---|---|---|---|---|
| `GLIDE_SLOPE_MIN` | **`ALT_SLOPE_MIN`** | 15 | `Parameters.cpp:80` | Alta |
| `GLIDE_SLOPE_THR` | **`ALT_SLOPE_MAXHGT`** | 5 | `Parameters.cpp:89` | Alta |
| `SYSID_THISMAV` | **`MAV_SYSID`** | 1 | `Parameters.cpp:1390` | Alta |
| `SYSID_MYGCS` | **`MAV_GCS_SYSID`** | 255 | `Parameters.cpp:1391` | Alta |
| `TELEM_DELAY` | **`MAV_TELEM_DELAY`** | 0 | `Parameters.cpp:1393` | Alta |
| `SYSID_ENFORCE` | **removido** (passou a bit 0 de `MAV_OPTIONS`) | — | `Parameters.cpp:1392`, `GCS.cpp:66` | Alta |
| `FS_SHORT_TIMEOUT` | **removido** | — | `ReleaseNotes.txt:703` (PR 30350, 4.7.0-beta1) | Alta |

Notas:
- A conversão `SYSID_* → MAV_*` existe no firmware (`AP_Param::convert_old_parameters`), mas só
  atua sobre a EEPROM quando se faz upgrade de firmware. **Ao carregar um ficheiro `.param` por
  nome numa GCS, os nomes antigos são simplesmente rejeitados.** Daí a renomeação explícita.
- `FS_SHORT_TIMEOUT` desapareceu: na 4.8 o failsafe curto dispara imediatamente quando a flag de
  RC failsafe é levantada (`system.cpp:409-418`). O atraso de deteção passou para `RC_FS_TIMEOUT`
  (default 1.0 s, `RC_Channels_VarInfo.h:113`). O failsafe longo continua a contar
  `FS_LONG_TIMEOUT` a partir do último RC válido (`system.cpp:379-381`), por isso o tempo total
  até ao RTL **não** é a soma dos dois.
- `PTCH_LIM_MAX_DEG` está em **graus** na 4.8 (`Parameters.cpp:536`, `@Units: deg`), não em
  centigraus. A nota do Notion que fala em "600–700 centigraus" é de uma versão ≤ 4.4; na 4.8
  o equivalente seriam os valores **6–7** — ver o veredito na secção 2.

---

## 2. Gradiente / TECS

| Parâmetro | Atual | Novo | Unidade | O que faz | Por quê | Confiança |
|---|---|---|---|---|---|---|
| `TECS_CLMB_MAX` | 5 | **0.8** | m/s | Limite rígido da razão de subida pedida. Alimenta `_climb_rate_limit` (`AP_TECS.cpp:538`) que satura a taxa de variação da altitude pedida, e `_STEdot_max = CLMB_MAX·g` (`:1264`) que satura `SEBdot_dem` no laço de pitch (`:1038`) | Mecanismo **principal** do gradiente. `0.08 × (15 − 5)` = 0.8, para um vento de frente de 5 m/s. Mantém ≤ 8 % até 5 m/s e ≤ 10 % duro até 7 m/s de vento. Ver tabela 2.1 — **tem de ser reposto no dia com o vento previsto** | Média (depende do vento) |
| `TECS_PITCH_MAX` | 15 | **10** | deg | Limite de pitch **apenas em modos de acelerador automático**. Com 0 reverte a `PTCH_LIM_MAX_DEG` (`AP_TECS.cpp:1491-1495`) | Proteção secundária contra fuga do gradiente, sem tocar em FBWA/MANUAL. γ para 0.8 m/s a 15 m/s é só 3.1°, logo 10° deixa > 6° de folga para o AoA e para rajadas. **Regra:** manter `TECS_PITCH_MAX ≥ PTCH_TRIM_DEG + 6` | Média (depende de `PTCH_TRIM_DEG`) |
| `TECS_PITCH_MIN` | 0 | **-15** | deg | Idem, limite inferior. Com 0 reverte a `PTCH_LIM_MIN_DEG` (= -25) | -25° de nariz em baixo vindo de 2000 m é excessivo e constrói velocidade. -15° deixa o `TECS_SINK_MAX` ser o limite que morde primeiro (γ para 3 m/s a 15 m/s = -11.5°) e preserva autoridade de recuperação em underspeed | Média (depende de `PTCH_TRIM_DEG`) |
| `PTCH_LIM_MAX_DEG` | 25 | **15** | deg | Limite global de pitch. Aplicado **por cima** da saída do TECS em `calc_nav_pitch()` (`Attitude.cpp:638`) e também em FBWA (`mode_fbwa.cpp:11,16`) e TRAINING | Aperta o envelope sem comprometer a descolagem manual nem o flare. Tem de ficar **≥ `TECS_LAND_PMAX` (10)**, senão o flare é cortado — ver secção 2.2 | Alta |
| `TECS_TIME_CONST` | 5 | **8** | s | Constante de tempo dos laços de energia do TECS. É o inverso do ganho proporcional do acelerador (`K_STE2Thr = 1/(tc·K_thr2STE)`, `:757`) e do pitch (`SEB_error/tc`, `:1036`) | Compensa parcialmente o aumento de ganho do acelerador causado pela descida de `TECS_CLMB_MAX` — ver secção 2.3. Range válido 3–10 | Baixa (validar em SITL) |
| `TECS_SINK_MAX` | 5 | **3** | m/s | Limite rígido da razão de afundamento pedida (`:539`, `:1037`) | 5 m/s a 15 m/s dá 33 % de gradiente de descida; vindo de 2000 m obriga a 6,7 min em velocidade perto de `AIRSPEED_MAX` (27 m/s). 3.0 é 1,5 × o afundamento ao ralenti (`TECS_SINK_MIN` = 2) — atingível sem grande excursão de velocidade. Ver secção 2.4 | Média (depende do `TECS_SINK_MIN` real) |
| `WP_LOITER_RAD` | 60 | **200** | m | Raio de loiter usado por `LOITER_*` quando o item de missão não traz raio próprio | A espiral de subida com r = 60 m exige banco de **20,9°** (`atan(V²/rg)`) e fator de carga 1,07, com penalização de arrasto induzido durante ~40 min de subida. Com r = 200 m o banco cai para **6,5°**. 200 m cabe folgadamente no círculo UAS de 996 m. **Coordenar com o agente de missão** — se os `LOITER_TO_ALT` trouxerem raio no param 3, este valor fica inerte | Média |

### 2.1 Tabela `TECS_CLMB_MAX` vs. vento (V = `AIRSPEED_CRUISE` = 15 m/s)

`TECS_CLMB_MAX = 0.08 × (V − W)`. A coluna "gradiente duro" é `CLMB_MAX / (15 − W)` com o valor
adotado de 0.8, para mostrar até que vento o limite de 10 % continua a ser respeitado.

| Vento de frente W [m/s] | Groundspeed pior caso [m/s] | `TECS_CLMB_MAX` [m/s] | Gradiente resultante | Tempo de subida a 2000 m |
|---|---|---|---|---|
| 0 | 15 | **1.20** | 8,0 % | 27,8 min |
| 1 | 14 | **1.12** | 8,0 % | 29,8 min |
| 2 | 13 | **1.04** | 8,0 % | 32,1 min |
| 3 | 12 | **0.96** | 8,0 % | 34,7 min |
| 4 | 11 | **0.88** | 8,0 % | 37,9 min |
| **5** | **10** | **0.80** ← adotado | **8,0 %** | **41,7 min** |
| 6 | 9 | **0.72** | 8,0 % | 46,3 min |
| 7 | 8 | **0.64** | 8,0 % | 52,1 min |
| 8 | 7 | **0.56** | 8,0 % | 59,5 min |
| 9 | 6 | **0.48** | 8,0 % | 69,4 min |
| 10 | 5 | **0.40** | 8,0 % | 83,3 min |

**Porquê 0.8 como default no ficheiro:**
mantendo-o fixo em 0.8, o gradiente real é `0.8/(15−W)`: 5,3 % sem vento, 8,0 % com 5 m/s,
**10,0 % com 7 m/s** e > 10 % acima disso. Ou seja, 0.8 é o maior valor que ainda respeita o
limite duro de 10 % para ventos até 7 m/s — uma margem razoável sem dados meteorológicos.
Se a previsão em altitude der mais de 7 m/s, **tem de se baixar** segundo a tabela.

**Consequência operacional a registar:** a missão é limitada pelo vento, não pelo avião. Acima
de ~8 m/s em altitude a subida passa de 1 h e deixa de caber na autonomia; nesse caso a decisão
é adiar o voo ou renegociar o teto, não forçar o parâmetro.

**O avião aguenta 0.8 m/s com `THR_MAX = 100`?** Sim, com elevada probabilidade — 0.8 m/s a
15 m/s é γ = 3,1°, e o avião cruza a 50 % de acelerador (`TRIM_THROTTLE` = 50) com 100 %
disponível. Mas isto **não está medido** (ver secção 0). Verificação de campo obrigatória:
durante a subida, o acelerador não pode saturar em 100 % nem a velocidade pode cair abaixo de
`AIRSPEED_CRUISE`. Se saturar, o problema é potência, não parametrização.

### 2.2 VEREDITO — `PTCH_LIM_MAX_DEG` a 6–7° é **inseguro**. Não usar.

A nota do Notion propõe 6–7° como proteção secundária do gradiente. **Recomendação: rejeitar**, e
usar `TECS_PITCH_MAX = 10` em vez disso. Cinco argumentos, todos rastreáveis no código:

1. **Corta o flare da aterragem automática.** `calc_nav_pitch()` (`Attitude.cpp:638`) faz
   `nav_pitch_cd = constrain_int32(TECS_controller.get_pitch_demand(), pitch_limit_min*100, aparm.pitch_limit_max*100)`
   e é chamada também no ramo `MAV_CMD_NAV_LAND` do AUTO (`mode_auto.cpp:99-101`). Dentro do
   TECS, durante o flare, o limite superior é substituído por `TECS_LAND_PMAX` = 10°
   (`AP_TECS.cpp:1525-1528`, com o comentário explícito "this allows a flare pitch outside the
   normal TECS auto limits"). Com `PTCH_LIM_MAX_DEG` = 6–7, a camada do Plane volta a cortar
   para 6–7° e o `TECS_LAND_PMAX` fica inerte. **Degrada exatamente a aterragem de failsafe que
   é o objetivo do projeto.**
2. **Afeta a descolagem manual.** O mesmo parâmetro limita FBWA (`mode_fbwa.cpp:11,16`) e
   TRAINING (`mode_training.cpp:23-26`). Se o piloto subir os primeiros 100 m em FBWA, um teto de
   6–7° de pitch perto do solo é perigoso. (Em MANUAL não há limite, mas não se pode depender disso.)
3. **Argumento energético — é o modo de falha contra o qual o CLAUDE.md avisa.** Pitch = γ + AoA.
   Para γ = 5,7° (10 %) e um AoA de trim plausível de 3–5° a 15 m/s (perto do `AIRSPEED_MIN` = 14),
   o pitch necessário é 8,7–10,7°. Um teto de 6–7° fica **abaixo do pitch necessário para sequer
   manter a subida pedida**: o pitch satura, o erro de altitude persiste, e com `TECS_SPDWEIGHT` = 1
   a energia que o laço de acelerador continua a injetar vai para velocidade. Fica-se sem
   autoridade para travar um afundamento numa rajada ou corrente descendente.
4. **A referência está por medir.** `PTCH_TRIM_DEG` = 0 é o default, não uma medida. Fixar um teto
   de 6–7° é fixá-lo relativamente a um zero desconhecido: a margem real pode ser 4° ou negativa.
5. **O firmware não protege.** O único pre-arm é `PTCH_LIM_MAX_DEG < 3` → falha
   (`AP_Arming_Plane.cpp:83-86`). 6 ou 7 passa sem aviso nenhum.

**Em vez disso:**
- `TECS_PITCH_MAX = 10` — é o parâmetro certo para este fim. Só atua em modos de acelerador
  automático (`AP_TECS.cpp:1491-1494`), não toca em FBWA/MANUAL, e durante o flare é
  explicitamente substituído por `TECS_LAND_PMAX` dentro do TECS.
- `PTCH_LIM_MAX_DEG = 15` — redução moderada face a 25, mantém `> TECS_LAND_PMAX` (10) para não
  cortar o flare e preserva autoridade na subida manual.
- A barreira primária do gradiente continua a ser `TECS_CLMB_MAX`, que é um limite de **razão de
  subida** (a grandeza que define o gradiente), não de atitude.

### 2.3 `TECS_PITCH_MAX = 0` reverte mesmo para `PTCH_LIM_MAX_DEG`? **Confirmado.**

`AP_TECS.cpp:1488-1501`, em `AP_TECS::_update_pitch_limits()`:

```cpp
// If TECS_PITCH_{MAX,MIN} isn't set then use LIM_PITCH_{MAX,MIN}.
// Don't allow TECS_PITCH_{MAX,MIN} to be larger than LIM_PITCH_{MAX,MIN}.
if (_pitch_max == 0) {
    _PITCHmaxf = aparm.pitch_limit_max;
} else {
    _PITCHmaxf = _pitch_max;
}

if (_pitch_min == 0) {
    _PITCHminf = aparm.pitch_limit_min;
} else {
    _PITCHminf = _pitch_min;
}
```

`aparm.pitch_limit_max` é `PTCH_LIM_MAX_DEG` (`Parameters.cpp:536`). **A nota do utilizador está
correta.**

Duas ressalvas que valem mais do que a confirmação:
- **A segunda linha do comentário está desatualizada.** O código da 4.8 **não** limita
  `TECS_PITCH_MAX` a `PTCH_LIM_MAX_DEG` dentro do TECS. Pôr `TECS_PITCH_MAX` acima de
  `PTCH_LIM_MAX_DEG` não dá erro — simplesmente o corte efetivo acontece mais tarde, na camada do
  Plane (`Attitude.cpp:638`). Não confiar no comentário.
- **Não usar 0 nesta missão.** Com 0, o limite de pitch automático fica colado ao limite de FBWA,
  e perde-se a possibilidade de apertar um sem apertar o outro — que é precisamente o que o
  projeto precisa.

### 2.4 `TECS_TIME_CONST` — o que faz e porque muda aqui

`timeConstant()` (`AP_TECS.cpp:703-714`) devolve `TECS_TIME_CONST` em voo normal e
`TECS_LAND_TCONST` durante uma aterragem automática. É usado em:

| Local | Expressão | Efeito |
|---|---|---|
| `:735` | `_SPEdot_dem = (_SPE_dem − _SPE_est) / tc` | erro de altitude → taxa de energia pedida |
| `:1036` | `SEBdot_dem = _hgt_rate_dem·g·SPE_w + SEB_error/tc` | ganho P do laço de **pitch** = 1/tc |
| `:757` | `K_STE2Thr = 1 / (tc · K_thr2STE)` | ganho P do laço de **acelerador** = 1/tc |
| `:528`, `:1099` | filtros passa-baixo de `_TAS_rate_dem` e fade do integrador | suavização |

Ou seja, **tc é o inverso do ganho proporcional de ambos os laços de energia.** Menor = mais
rápido e agressivo; maior = mais suave, menos overshoot. Range 3–10, default 5.

**Porque é que vale mesmo mexer:**

```
K_thr2STE = (_STEdot_max − _STEdot_min) / (THRmax − THRmin)     (AP_TECS.cpp:756)
_STEdot_max = TECS_CLMB_MAX · g      _STEdot_min = −TECS_SINK_MIN · g
```

| Configuração | `K_thr2STE` | `K_STE2Thr` com tc | Ganho relativo |
|---|---|---|---|
| Antes: CLMB_MAX 5, SINK_MIN 2, tc 5 | (5+2)·9,81 = 68,7 | 1/(5·68,7) = 0,00291 | 1,00 × |
| Só baixando CLMB_MAX para 0.8, tc 5 | (0,8+2)·9,81 = 27,5 | 1/(5·27,5) = 0,00728 | **2,50 ×** |
| Adotado: CLMB_MAX 0.8, tc **8** | 27,5 | 1/(8·27,5) = 0,00455 | **1,56 ×** |

Baixar `TECS_CLMB_MAX` **sobe o ganho do laço de acelerador em 2,5 ×** — um efeito colateral que
não é óbvio e que pode dar oscilação (hunting) do acelerador. Subir tc para 8 recupera cerca de
metade; 12,5 seria a compensação exata mas está fora do range válido (3–10).

Como tc também desacelera a correção de erro de altitude, e nesta missão **queremos** que a
altitude seja perseguida devagar, o efeito secundário é desejável. A razão de subida continua a
ser limitada independentemente por `_climb_rate_limit`.

**Confiança Baixa no valor 8** — o mecanismo está confirmado no código, mas a resposta real
depende da planta. Teste SITL: subida completa de 100 m a 2000 m com `TECS_CLMB_MAX` = 0.8 e
observar o log TECS/CTUN; se o acelerador oscilar, subir tc para 10 e/ou `TECS_THR_DAMP` de 0,5
para 0,7. Se ficar lento de mais, descer para 6.

### 2.5 Descida de 2000 m — `TECS_SINK_MAX` = 5 é aceitável?

**Não, para esta missão.** 5 m/s com 15 m/s de velocidade dá γ = 19,5°, ou seja **33 % de
gradiente**. O problema não é o gradiente em si — é como o TECS o consegue:

- `TECS_SINK_MIN` = 2 m/s é, por definição, "minimum sink rate when at THR_MIN and
  AIRSPEED_CRUISE" (`AP_TECS.cpp:31-34`). Para afundar a 5 m/s o avião tem de converter altitude
  em velocidade.
- O código faz isso explicitamente: `_TAS_dem = _TAS_dem + (_TASmax − _TAS_dem) · _sink_fraction`
  (`AP_TECS.cpp:481`) — quando a descida pedida não é atingível, a velocidade pedida sobe na
  direção de `AIRSPEED_MAX` = 27 m/s.
- Vindo de 2000 m isso significa **6,7 minutos de descida contínua a velocidade perto de 27 m/s**,
  isto é, 80 % acima do cruzeiro e 3,2 × a pressão dinâmica. Para um avião cujo envelope real não
  foi explorado, é risco desnecessário.

**Proposta: `TECS_SINK_MAX = 3.0`** (γ = 11,5°, **20 %** de gradiente, 11,1 min de 2000 m).
É 1,5 × o afundamento ao ralenti declarado, logo atingível com um aumento modesto de velocidade.
Combina com `TECS_PITCH_MIN = -15`, que deixa o `SINK_MAX` morder primeiro.

A descida deve ser voada em **espiral** (LOITER_TO_ALT), não em perna reta: a 3 m/s e 15 m/s, uma
descida direta de 2000 m percorre 10 km, muito para lá do círculo UAS de 996 m. Isto é decisão do
agente de missão, mas o parâmetro assume-o.

**Interação a vigiar (ver secção 5):** uma descida mais lenta consome mais bateria. Com
`BATT_CRT_MAH` = 5400 e um consumo médio de 30 A, a reserva crítica dá 10,8 min — menos do que os
11,1 min de descida. Assim que o consumo real for medido, ou se sobe `BATT_CRT_MAH` ou se aceita
`TECS_SINK_MAX` = 4–5 **só** para o caso crítico.

---

## 3. Failsafes

| Parâmetro | Atual | Novo | Unidade | O que faz | Por quê | Confiança |
|---|---|---|---|---|---|---|
| `FS_LONG_TIMEOUT` | 5 | **10** | s | Tempo de falha de RC/GCS persistente antes do failsafe longo (`system.cpp:379-381`, contado desde o último RC válido) | Com o avião a até ~2,2 km de distância oblíqua (2000 m de altitude dentro de um círculo de 996 m), quedas breves de link são expectáveis e um RTL espúrio é indesejável. 5 s extra a 15 m/s são 75 m — irrelevante dentro da fence | Média |
| `FS_SHORT_TIMEOUT` | 1.5 | **removido** | — | — | Deixou de existir na 4.8 (ver secção 1) | Alta |

**Sem alteração, confirmado no código (manter):**

| Parâmetro | Valor | Confirmação |
|---|---|---|
| `FS_LONG_ACTN` | 1 (RTL) | `Parameters.cpp:448-451`. Em AUTO, valor 1 → `set_mode(mode_rtl)` (`events.cpp:214-216`). O default é **0 = Continue**, que faria o avião prosseguir a missão — por isso 1 é obrigatório |
| `FS_SHORT_ACTN` | 0 | `Parameters.cpp:441-444`. Em AUTO/GUIDED/LOITER, 0 = sem mudança de modo (`events.cpp:80-90`). A missão continua e o failsafe longo decide aos 10 s. Correto |
| `THR_FAILSAFE` | 1 | `Parameters.cpp:409-412`. 1 = failsafe ativo por perda de RC ou throttle < `THR_FS_VALUE` |
| `THR_FS_VALUE` | 950 | `Parameters.cpp:415-421`. **Verificação de campo obrigatória**: desligar o emissor no chão e confirmar que o PWM de throttle cai abaixo de 950 |
| `FS_GCS_ENABL` | 1 | `Parameters.cpp:462-467`. 1 = failsafe por perda de HEARTBEAT, dispara após `FS_LONG_TIMEOUT` e executa `FS_LONG_ACTN` |
| `FS_EKF_THRESH` | 0.8 | Default |

**Alternativa considerada e rejeitada:** a 4.8 tem `FS_LONG_ACTN = 5` (modo AUTOLAND) e
`FENCE_ACTION = 8` (AUTOLAND se possível, senão RTL). O modo AUTOLAND **estaria** disponível aqui
— exige `takeoff_state.initial_direction.initialized` (`mode_autoland.cpp:85-88`), que é capturado
do rumo GPS em MANUAL e FBWA (`mode.h:512,634`; `mode_autoland.cpp:273-279`). Rejeitado porque o
AUTOLAND constrói um circuito genérico a partir do rumo de descolagem, sem conhecer os limites do
espaço aéreo; o `DO_RETURN_PATH_START` dá uma trajetória explicitamente planeada dentro do
círculo UAS. Fica registado como plano B.

---

## 4. RTL / Aterragem

| Parâmetro | Atual | Novo | Unidade | O que faz | Por quê | Confiança |
|---|---|---|---|---|---|---|
| `RTL_AUTOLAND` | 0 | **4** | enum | 4 = "Go directly to landing sequence via DO_RETURN_PATH_START mission item" (`Parameters.cpp:723`; `RtlAutoland::DO_RETURN_PATH_START = 4` em `defines.h:68`) | Requisito do utilizador. Ver secção 4.1 | Alta |
| `TECS_LAND_ARSPD` | -1 | **14.5** | m/s | Velocidade-alvo na aproximação. Negativo = desativado, e o código usa `0.5·(AIRSPEED_CRUISE + AIRSPEED_MIN)` (`AP_Landing_Slope.cpp:354`) | 0,5·(15+14) = 14,5 — **o valor efetivo não muda**, mas passa a estar explícito e deixa de seguir silenciosamente alterações a `AIRSPEED_MIN`/`AIRSPEED_CRUISE` | Alta (mecanismo) / Média (valor) |

### 4.1 `RTL_AUTOLAND = 4` — o que faz exatamente na 4.8

É consumido num único sítio: `ModeRTL::navigate()` (`mode_rtl.cpp:126-137`).

```cpp
} else if (plane.g.rtl_autoland == RtlAutoland::DO_RETURN_PATH_START) {
    if (plane.have_position && plane.mission.jump_to_closest_mission_leg(plane.current_loc)) {
        plane.mission.set_force_resume(true);
        if (plane.set_mode(plane.mode_auto, ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND)) {
            return;
        }
        ...
    }
    plane.auto_state.checked_for_autoland = true;
}
```

Pontos que importam:

1. **Não espera por nada.** Ao contrário do valor 1 (que exige `reached_loiter_target()` e erro de
   altitude < 10 m) e do valor 2 (que vai direto ao `DO_LAND_START`), o valor 4 corre na **primeira**
   chamada a `navigate()` depois de entrar em RTL. Na prática o avião passa por RTL durante um ciclo
   de controlo e salta logo para AUTO.
2. **`jump_to_closest_mission_leg()`** (`AP_Mission.cpp:2523-2573`) percorre a missão à procura de
   **todos** os `MAV_CMD_DO_RETURN_PATH_START` (cmd **188**, confirmado em
   `modules/mavlink/.../common.xml:1471`), calcula para cada um a distância a cada **perna** do
   percurso de regresso e salta para o índice da perna mais próxima. **Pode entrar a meio do
   percurso de regresso, não obrigatoriamente no item 188.** Marca `_flags.in_return_path` e
   envia "Return path started".
3. **Se já estiver em sequência de aterragem ou de regresso**, devolve `true` sem saltar
   (mensagens "Landing sequence active" / "Return path active", `:2527-2534`). Protege contra
   reentradas.
4. **Relação com `DO_LAND_START`:** o valor 4 **não** usa o `DO_LAND_START` para o salto. Mas o
   `DO_LAND_START` continua a ser necessário, porque é o que levanta `in_landing_sequence`, a flag
   que `failsafe_in_landing_sequence()` (`events.cpp:5-19`) consulta para **inibir novos failsafes
   durante a aterragem**. A missão tem de ter os dois: `DO_RETURN_PATH_START` à entrada e, mais à
   frente, `DO_LAND_START` imediatamente antes da aproximação final e do `LAND`.
5. **Armação:** com `RTL_AUTOLAND = 0` e a missão a conter `DO_LAND_START` **ou**
   `DO_RETURN_PATH_START`, a armação **falha** (`AP_Arming_Plane.cpp:425-436`). Com 4 não há
   problema. Também há um pre-arm que proíbe `RTL_AUTOLAND != 0` em conjunto com `Q_RTL_MODE != 0`
   (`:223-227`) — irrelevante, `Q_ENABLE` = 0.
6. **Armadilha conhecida:** o guarda "já está a aterrar" do fence (`fence.cpp:126-129`) só
   reconhece `RTL_AUTOLAND` **1 e 2**, não o 4. Uma violação de fence durante o percurso de
   regresso volta a chamar `set_mode(RTL)`; o RTL por sua vez reencontra "Return path active" e
   volta a AUTO. Recupera, mas com um salto de modo. Registar e observar em SITL.

**E com `RTL_ALTITUDE` / `RTL_CLIMB_MIN` vindo de 2000 m?**

- `ModeRTL::_enter()` chama `do_RTL(get_RTL_altitude_cm())`, que com `RTL_ALTITUDE` = 70 devolve
  `home.alt + 7000 cm` (`altitude.cpp:99-105`). Vindo de 2000 m isso é um erro de altitude de
  1930 m. **Mas**, como o salto para AUTO acontece no ciclo seguinte, esse alvo vive um único
  ciclo de controlo; e mesmo que vivesse mais, a altitude pedida é limitada em taxa por
  `TECS_SINK_MAX`. **Inofensivo. Mantido em 70.**
- Manter 70 também é a melhor degradação: se o salto falhar (missão não carregada, sem
  `DO_RETURN_PATH_START` alcançável), o avião vai para casa e faz circuito a 70 m com
  `RTL_RADIUS` = 60 — o comportamento desejado.
- `RTL_CLIMB_MIN` = 0 mantido. Com `FLIGHT_OPTIONS` = 16 (bit 4 = `CLIMB_BEFORE_TURN`,
  `defines.h:154`) o RTL tenta subir até `RTL_ALTITUDE` antes de virar, limitando o banco a
  `LEVEL_ROLL_LIMIT` = 5° (`mode_rtl.cpp:49-74`). Vindo de 2000 m a condição
  `current_loc.alt > next_WP_loc.alt` é verdadeira de imediato → sem restrição de banco. A baixa
  altitude a restrição aplica-se e é desejável. **`FLIGHT_OPTIONS` mantido em 16.**

### 4.2 Parâmetros de aterragem, um a um

| Parâmetro | Atual | Proposto | O que faz | Justificação |
|---|---|---|---|---|
| `TECS_SINK_MIN` | 2 | **2 (manter, por medir)** | Razão de afundamento a `THR_MIN` e `AIRSPEED_CRUISE` (`AP_TECS.cpp:31-34`). Define `_STEdot_min = −SINK_MIN·g` (`:1265`), que entra no mapeamento acelerador↔energia (`K_thr2STE`) | É o default de fábrica, **nunca medido**. Se estiver errado, o feedforward de acelerador está errado. **Pendência:** medir em voo (ralenti a 15 m/s, ler a razão de afundamento estabilizada) e repor. Confiança **Baixa** |
| `TECS_SINK_MAX` | 5 | **3** | Limite rígido de afundamento pedido | Ver secção 2.5. Confiança **Média** |
| `TECS_LAND_ARSPD` | -1 | **14.5** | Velocidade-alvo na aproximação. -1 = desativado → o código usa `0.5·(AIRSPEED_CRUISE + AIRSPEED_MIN)` = 14,5 (`AP_Landing_Slope.cpp:351-360`) | Torna explícito o valor que já estava em vigor. Nota: `_TAS_dem` é saturado em `AIRSPEED_MIN` = 14, e com `AIRSPEED_STALL` = 0 a 4.8 assume que a velocidade de perda **é** `AIRSPEED_MIN` (`Parameters.cpp:309-313`) — ou seja, **não é possível aterrar abaixo de 14 m/s** sem primeiro medir e configurar `AIRSPEED_STALL`. Confiança **Alta** (mecanismo) |
| `TECS_LAND_THR` | -1 | **-1 (manter)** | Acelerador de cruzeiro na aproximação, **alternativa** ao `LAND_ARSPD` para aviões sem sensor de velocidade. Negativo = desativado (`AP_TECS.cpp:928`: `if (_flags.is_doing_auto_land && _landThrottle >= 0)`) | O avião tem sensor de velocidade (`ARSPD_TYPE` = 8, `ARSPD_USE` = 1). -1 é o correto. Confiança **Alta** |
| `LAND_FLARE_ALT` | 3 | **3 (manter)** | Altitude a que se tranca o rumo e se inicia o flare. "secondary to LAND_FLARE_SEC" | Default nunca afinado. Com o afundamento final previsto (~0,8–1 m/s), `FLARE_SEC` = 2 dispararia a 1,6–2 m, **abaixo** de `FLARE_ALT` = 3 — ou seja, na prática é o `FLARE_ALT` que dispara primeiro, ao contrário do que a documentação recomenda. Afinar no campo após o primeiro pouso automático. Confiança **Baixa** |
| `LAND_FLARE_SEC` | 2 | **2 (manter)** | Tempo vertical até ao ponto de toque a que se inicia o flare | Idem. Confiança **Baixa** |
| `TECS_LAND_SINK` | 0.25 | **0.25 (manter)** | Afundamento pedido durante o flare (m/s) | Default. Valor conservador e habitual. Afinar no campo: se o avião "flutuar" e passar o ponto, subir; se bater, descer. Confiança **Baixa** |
| `LAND_PITCH_DEG` | 0 | **a definir (manter 0)** | Pitch **mínimo** na fase final, depois do flare — define a atitude de toque | Depende do trem de aterragem, que não está documentado. 0 = toque nivelado. Para pouso de barriga/patim é comum 2–5° de nariz em cima. **Pendência:** tipo de trem / atitude de toque pretendida. Confiança **Baixa** |
| `TECS_LAND_PMAX` | 10 | **10 (manter)** | Pitch máximo na fase final. Substitui `_PITCHmaxf` durante o flare (`AP_TECS.cpp:1525-1528`) | Mantido — **e é a razão pela qual `PTCH_LIM_MAX_DEG` não pode descer abaixo de 10** (`Attitude.cpp:638`). Confiança **Alta** |
| `TECS_APPR_SMAX` | 0 | **0 (manter)** | Afundamento máximo na perna de aproximação. 0 = usa `TECS_SINK_MAX` (`AP_TECS.cpp:540-547`) | Com `SINK_MAX` agora a 3,0, a aproximação fica limitada a 3 m/s — muito acima do necessário para uma rampa de 3–5°. Confiança **Alta** |
| `LAND_WIND_COMP` | 50 | **50 (manter)** | % da componente de vento de frente somada à velocidade de aproximação, saturada em `AIRSPEED_CRUISE` sem o bit de `LAND_OPTIONS` (`AP_Landing_Slope.cpp:377-383`) | Default razoável. Confiança **Alta** |
| `LAND_DISARMDELAY` | 20 | **20 (manter)** | Segundos após o toque até desarmar | Confiança **Alta** |

---

## 5. Fence

| Parâmetro | Atual | Novo | Unidade | O que faz | Por quê | Confiança |
|---|---|---|---|---|---|---|
| `FENCE_ENABLE` | 0 | **1** | 0/1 | Ativa a funcionalidade de fence (`AC_Fence.cpp:57`) | O voo é a 2000 m dentro de espaço aéreo segregado com teto declarado. Voar sem fence não se justifica | Alta |
| `FENCE_TYPE` | 4 | **5** | bitmask | bit0 = altitude máxima, bit1 = círculo centrado no home, bit2 = inclusão/exclusão (polígonos+círculos), bit3 = altitude mínima (`AC_Fence.cpp:59-65`) | 5 = bit0 + bit2. Altitude máxima + a fence de inclusão do círculo UAS. **O bit1 é inútil aqui — ver 5.1** | Alta |
| `FENCE_ALT_MAX` | 100 | **2100** | m | Teto da fence. Frame dado por `FENCE_ALT_MAX_TP` | 100 m é incompatível com uma missão a 2000 m. 2100 dá 100 m de folga sobre o alvo, suficiente para overshoot do TECS (que a 0,8 m/s é de poucos metros) e para deriva barométrica numa hora de voo. Nota: `@Range 10 1000` é apenas metadados da GCS; **não há limite no código** — só `_alt_max_m < 0` falha o pre-arm (`AC_Fence.cpp:466-469`) | Média (depende da elevação da pista) |
| `FENCE_ALT_MAX_TP` | (ausente) | **1** | enum | Frame do teto: 0 = AMSL, 1 = acima do home, 2 = acima da origem, 3 = acima do terreno (`AC_Fence.cpp:167-173`) | É o default da 4.8, escrito explicitamente para não ficar ambíguo que `FENCE_ALT_MAX` é **relativo ao home** | Alta |
| `FENCE_AUTOENABLE` | 0 | **3** | enum | 0 = nunca, 1 = após descolagem automática, 2 = idem com floor na aterragem, 3 = ao armar (`AC_Fence.cpp:136-143`) | A descolagem é **manual**. As opções 1 e 2 só são acionadas por `auto_enable_fence_after_takeoff()`, chamado apenas dos caminhos de descolagem automática (`commands_logic.cpp:624`, `mode_takeoff.cpp:185`) — **nunca disparariam**. Além disso 1 e 2 estão depreciadas: o pre-arm avisa "FENCE_AUTOENABLE is %u, will be removed in 4.7, use 3" (`AC_Fence.cpp:494-499`). 3 ativa ao armar | Alta |

**Sem alteração (manter):** `FENCE_ACTION` = 1 (RTL). Valores válidos para Plane: 0 Report Only,
1 RTL, 6 Guided, 7 GuidedThrottlePass, 8 AUTOLAND-senão-RTL (`AC_Fence.cpp:72`, consumidos em
`fence.cpp:112-142`). Os valores 2–5 são explicitamente ignorados no Plane ("invalid enumeration
value for Plane", `fence.cpp:117-123`) — ou seja, pô-los equivale a não fazer nada.
`FENCE_ALT_MIN` = -10 e `FENCE_MARGIN` = 2 mantidos; `FENCE_OPTIONS` = 1 mantido (bit 0 = impede
mudança de modo até a violação ser resolvida).

### 5.1 Porque é que `FENCE_RADIUS` não serve — e fica onde está

O círculo da fence (`FENCE_TYPE` bit 1) é **sempre centrado no home**. A geometria do ARTEX não
permite usá-lo:

- Centro do círculo UAS: `39.4054602, -8.2937286`, raio **996 m**.
- Marcador `Runway` (≈ home): `39.3993570, -8.2864182`, a **925 m** do centro.

Um círculo centrado no home de raio `r` só fica inteiramente dentro do círculo UAS se
`925 + r ≤ 996`, ou seja **`r ≤ 71 m`** — inútil para voar. Já o `FENCE_RADIUS` = 300 do ficheiro
original sairia 229 m para fora da área UAS.

**Conclusão: a contenção horizontal tem de ser uma fence de inclusão** (bit 2), carregada como
ficheiro de geofence com um círculo de inclusão no centro UAS e raio 996 m menos margem.
`FENCE_RADIUS` fica em 300 por ser inerte com o bit 1 desligado — não foi tocado para não sugerir
que é utilizável.

**Dependência a registar:** com o bit 2 ativo e sem ficheiro de fence carregado, o **pre-arm
falha** com "Polygon fence(s) invalid" (`AC_Fence.cpp:428-438`). O ficheiro de geofence é entrega
do agente de missão. Para testar em SITL sem ele, usar `FENCE_TYPE` = 1 (só altitude).

**Dependência de teto:** 2100 m acima do home + elevação da pista tem de ficar abaixo dos 3000 m
AMSL do teto UAS, o que exige elevação ≤ 900 m. Se a pista estiver a ~100 m AMSL (a confirmar),
o teto da fence fica a ~2200 m AMSL, 800 m abaixo do limite. **A elevação da pista é pendência.**

---

## 6. Bateria

| Parâmetro | Atual | Novo | Unidade | O que faz | Por quê | Confiança |
|---|---|---|---|---|---|---|
| `BATT_LOW_MAH` | 500 | **8100** | mAh | Dispara o failsafe baixo quando a capacidade **restante** (`BATT_CAPACITY − consumido`) cai abaixo deste valor (`AP_BattMonitor_Backend.cpp:276-277`). Sem temporizador — dispara de imediato (`:189-191`) | 30 % de 27 000 mAh. 500 mAh (1,8 %) era inutilizável | Média (premissa de reserva) |
| `BATT_CRT_MAH` | 0 | **5400** | mAh | Idem, failsafe crítico (`:262-263`). 0 = desativado | 20 % de 27 000 mAh. Pre-arm exige `LOW_MAH > CRT_MAH` (`:220-222`) — 8100 > 5400 ✓ | Média (premissa de reserva) |
| `BATT_FS_LOW_ACT` | 0 | **1** | enum | Ação no failsafe baixo. Plane: 0 Warn, **1 RTL**, 2 Land, 3 Terminate, 4 QLand, 5 Parachute, 6 Loiter-to-QLand, 7 AUTOLAND-senão-RTL (`AP_BattMonitor_Params.cpp:147`) | 1 → `set_mode(mode_rtl)` (`events.cpp:308-336`) → com `RTL_AUTOLAND` = 4, entra no percurso de regresso planeado. Comportamento desejado | Alta |
| `BATT_FS_CRT_ACT` | 0 | **2** | enum | Ação no failsafe crítico | 2 (Land) chama `jump_to_landing_sequence()` e passa a AUTO (`events.cpp:299-310`), isto é, salta **direto ao `DO_LAND_START`** — o caminho mais curto ao solo — e, se falhar, cai por `FALLTHROUGH` para RTL. Dá uma escalada coerente: **baixo** = trajetória de regresso completa; **crítico** = aproximação direta | Alta |

**Mantidos:** `BATT_CAPACITY` = 27000, `BATT_LOW_TIMER` = 10, `BATT_FS_VOLTSRC` = 0.
`BATT2_*` (3300 mAh, provavelmente aviónica/payload) **não foi tocado** — os failsafes dessa
bateria continuam desligados, de propósito.

### 6.1 A conta, e a premissa que falta

O consumo médio é **pendência** — sem ele não é possível converter tempo em mAh. Por isso os
limiares foram definidos como **percentagem da capacidade**, premissa que se declara: reserva de
**30 % (baixo)** e **20 % (crítico)**, consistente com a prática de não descarregar LiPo abaixo de
~20 %.

O que a reserva tem de pagar, com os parâmetros desta proposta:

| Fase | Duração |
|---|---|
| Descida de 2000 m a `TECS_SINK_MAX` = 3 m/s | 11,1 min |
| Regresso ao campo (máx. ~2 km dentro do círculo UAS, a 15 m/s) | ~2,5 min |
| Circuito, aproximação e aterragem | ~5 min |
| **Total mínimo necessário** | **≈ 19 min** |

Quanto tempo compram os limiares, em função do consumo médio de retorno `I`:

| `I` médio [A] | 8100 mAh (baixo) | 5400 mAh (crítico) | Baixo cobre os 19 min? |
|---|---|---|---|
| 10 | 48,6 min | 32,4 min | sim, com larga folga |
| 15 | 32,4 min | 21,6 min | sim |
| 20 | 24,3 min | 16,2 min | sim |
| 25 | 19,4 min | 13,0 min | no limite |
| 30 | 16,2 min | 10,8 min | **não** |

**Leitura:** 30 % de reserva é suficiente enquanto o consumo médio de retorno não passar
~25 A. Assim que o consumo real for medido (voo de caracterização ou log de voo anterior), fechar
os valores por:

```
BATT_LOW_MAH = I_retorno[A] × (t_descida + t_regresso + t_circuito)[h] × 1000 × 1.3
BATT_CRT_MAH = I_retorno[A] × (t_descida_rapida + t_aproximacao)[h] × 1000 × 1.2
```

**Alerta já visível:** a 30 A, a reserva crítica (10,8 min) é menor do que a descida a 3 m/s
(11,1 min). Se o consumo medido ficar alto, ou se sobe `BATT_CRT_MAH`, ou se aceita um
`TECS_SINK_MAX` maior.

### 6.2 `BATT_LOW_VOLT` / `BATT_CRT_VOLT` — ficam a 0 (a definir)

Mantidos desativados **de propósito**. Para os definir é preciso o número de células em série da
bateria, que não está documentado (`BATT_VOLT_MULT` = 19,12 e `BATT_ARM_VOLT` = 7 não permitem
inferi-lo com segurança). A regra, assim que o dado existir:

```
BATT_LOW_VOLT = 3.5 V × n_células       BATT_CRT_VOLT = 3.3 V × n_células
```

Com `BATT_LOW_TIMER` = 10 s de persistência (`AP_BattMonitor_Backend.cpp:163-166`), que já está
correto. **Pendência: número de células e tensão em carga sob subida.**

---

## 7. Missão

**Sem alterações além de `WP_LOITER_RAD` (secção 2).** Confirmado:

| Parâmetro | Valor | Confirmação |
|---|---|---|
| `MIS_RESTART` | 0 | `AP_Mission.cpp:38`. 0 = retoma onde parou ao voltar a AUTO. Essencial para que o regresso por `DO_RETURN_PATH_START` não reinicie a missão |
| `MIS_OPTIONS` | 0 | `AP_Mission.cpp:40-47`. bit0 limpa missão no reboot, bit1 usa distância-à-aterragem no failsafe de bateria, bit2 continua após land, bit3 não zera contador. Nenhum desejado. (O bit 1 só teria efeito com `RTL_AUTOLAND` = 2, pelo caminho `is_best_land_sequence` em `events.cpp:329`) |
| `ALT_SLOPE_MIN` | 15 | `Parameters.cpp:80`. Diferenças de altitude < 15 m entre waypoints não geram rampa (`altitude.cpp:493-502`); acima disso o ArduPlane interpola linearmente. É o segundo mecanismo de gradiente descrito no CLAUDE.md |
| `ALT_SLOPE_MAXHGT` | 5 | `Parameters.cpp:89` |
| `FLIGHT_OPTIONS` | 16 | bit 4 = `CLIMB_BEFORE_TURN` (`defines.h:154`). Ver 4.1. **Importante:** o bit 14 (`IMMEDIATE_CLIMB_IN_AUTO`, `altitude.cpp:81`) está **desligado** e tem de continuar assim — ligá-lo faria o avião ignorar a rampa linear entre waypoints |
| `WP_RADIUS` | 35 | mantido |
| `NAVL1_PERIOD` | 17 | mantido (afinado no avião) |
| `Q_ENABLE` | 0 | obrigatório: `Q_RTL_MODE != 0` com `RTL_AUTOLAND != 0` falha o pre-arm (`AP_Arming_Plane.cpp:223-227`) |

---

## 8. Resumo das alterações

23 linhas alteradas num ficheiro de 1135. Nenhum ganho de PID, calibração de sensor, RC, servo ou
telemetria foi tocado.

| # | Parâmetro | Atual | Novo | Grupo | Confiança |
|---|---|---|---|---|---|
| 1 | `GLIDE_SLOPE_MIN` → `ALT_SLOPE_MIN` | 15 | 15 | Migração 4.8 | Alta |
| 2 | `GLIDE_SLOPE_THR` → `ALT_SLOPE_MAXHGT` | 5 | 5 | Migração 4.8 | Alta |
| 3 | `SYSID_THISMAV` → `MAV_SYSID` | 1 | 1 | Migração 4.8 | Alta |
| 4 | `SYSID_MYGCS` → `MAV_GCS_SYSID` | 255 | 255 | Migração 4.8 | Alta |
| 5 | `TELEM_DELAY` → `MAV_TELEM_DELAY` | 0 | 0 | Migração 4.8 | Alta |
| 6 | `SYSID_ENFORCE` | 0 | removido | Migração 4.8 | Alta |
| 7 | `FS_SHORT_TIMEOUT` | 1.5 | removido | Migração 4.8 | Alta |
| 8 | `TECS_CLMB_MAX` | 5 | **0.8** | Gradiente | Média |
| 9 | `TECS_PITCH_MAX` | 15 | **10** | Gradiente | Média |
| 10 | `TECS_PITCH_MIN` | 0 | **-15** | Gradiente | Média |
| 11 | `PTCH_LIM_MAX_DEG` | 25 | **15** | Gradiente | Alta |
| 12 | `TECS_TIME_CONST` | 5 | **8** | Gradiente | Baixa |
| 13 | `TECS_SINK_MAX` | 5 | **3** | Gradiente | Média |
| 14 | `WP_LOITER_RAD` | 60 | **200** | Gradiente | Média |
| 15 | `FS_LONG_TIMEOUT` | 5 | **10** | Failsafes | Média |
| 16 | `RTL_AUTOLAND` | 0 | **4** | RTL/Aterragem | Alta |
| 17 | `TECS_LAND_ARSPD` | -1 | **14.5** | RTL/Aterragem | Alta |
| 18 | `FENCE_ENABLE` | 0 | **1** | Fence | Alta |
| 19 | `FENCE_TYPE` | 4 | **5** | Fence | Alta |
| 20 | `FENCE_ALT_MAX` | 100 | **2100** | Fence | Média |
| 21 | `FENCE_ALT_MAX_TP` | (ausente) | **1** | Fence | Alta |
| 22 | `FENCE_AUTOENABLE` | 0 | **3** | Fence | Alta |
| 23 | `BATT_LOW_MAH` | 500 | **8100** | Bateria | Média |
| 24 | `BATT_CRT_MAH` | 0 | **5400** | Bateria | Média |
| 25 | `BATT_FS_LOW_ACT` | 0 | **1** | Bateria | Alta |
| 26 | `BATT_FS_CRT_ACT` | 0 | **2** | Bateria | Alta |

---

## 9. Pendências que impedem fechar valores

| # | Dado em falta | Parâmetros bloqueados | Como obter |
|---|---|---|---|
| 1 | **Vento previsto em altitude** | `TECS_CLMB_MAX` (tabela 2.1) | Previsão meteorológica em altitude no dia; validar com a estimativa de vento do EKF durante a subida |
| 2 | **`PTCH_TRIM_DEG` real / AoA de trim a 15 m/s** | `TECS_PITCH_MAX`, `TECS_PITCH_MIN`, `PTCH_LIM_MAX_DEG` | Voo nivelado estabilizado a 15 m/s; ler a atitude de pitch. Regra: `TECS_PITCH_MAX ≥ PTCH_TRIM_DEG + 6` |
| 3 | **Razão de subida real a `THR_MAX` e `AIRSPEED_CRUISE`** | validação de `TECS_CLMB_MAX`, `TECS_PITCH_MAX` | Subida a acelerador máximo, velocidade constante; é o valor que o `TECS_CLMB_MAX` **deveria** ter se não houvesse restrição de gradiente |
| 4 | **Razão de afundamento ao ralenti (`TECS_SINK_MIN`)** | `TECS_SINK_MIN`, e por consequência `TECS_SINK_MAX` e o ganho de acelerador | Planeio a `THR_MIN` a 15 m/s |
| 5 | **Consumo médio [A] em subida e em cruzeiro** | `BATT_LOW_MAH`, `BATT_CRT_MAH` | Log de voo com o monitor de corrente já instalado (`BATT_MONITOR` = 4) |
| 6 | **Número de células da bateria principal** | `BATT_LOW_VOLT`, `BATT_CRT_VOLT` (hoje a 0 = desativados) | Inspeção da bateria |
| 7 | **Elevação da pista (AMSL)** | verificação de `FENCE_ALT_MAX` contra o teto de 3000 m AMSL | Levantamento GPS / carta |
| 8 | **Ficheiro de geofence de inclusão (círculo UAS)** | `FENCE_TYPE` = 5 (sem ele o pre-arm falha) | Entrega do agente de missão |
| 9 | **Tipo de trem de aterragem / atitude de toque** | `LAND_PITCH_DEG` (hoje 0) | Documentação ou inspeção da aeronave |
| 10 | **PWM de failsafe do recetor no canal de acelerador** | validação de `THR_FS_VALUE` = 950 | Desligar o emissor no chão e ler o PWM |
| 11 | **Velocidade de perda real** | `AIRSPEED_STALL` (hoje 0 → assume `AIRSPEED_MIN` = 14), e com ela a possibilidade de aterrar abaixo de 14 m/s | Ensaio de perda em altitude segura |
| 12 | **Raio da espiral decidido pelo agente de missão** | `WP_LOITER_RAD` = 200 | Coordenação |
