#!/usr/bin/env python3
"""Parametros do ArduPlane para a missao ARTEX 2000 m.

Cada entrada traz **valor, unidade e motivo**. A justificacao longa de cada um
esta' em `params/artex27_diff.md`; aqui fica a versao curta, para o
verificador de pre-voo poder explicar o que esta' errado sem ninguem ter de
abrir um documento.

Firmware: ArduPlane 4.8.0-dev. Nomes modernos (`AIRSPEED_CRUISE`,
`PTCH_LIM_MAX_DEG`, `ALT_SLOPE_MIN`), nao os legados.
"""

from collections import namedtuple

Param = namedtuple('Param', 'value unit group reason tol')


def _p(value, unit, group, reason, tol=1e-6):
    return Param(value, unit, group, reason, tol)


# ---------------------------------------------------------------------------
# Tabela de parametros da missao
# ---------------------------------------------------------------------------
ARTEX_PARAMS = {
    # --- Gradiente / TECS --------------------------------------------------
    'TECS_CLMB_MAX': _p(
        0.8, 'm/s', 'gradiente',
        'Limite duro do gradiente de subida: 0,08 x (15 - 5 m/s de vento) = 8 %. '
        'VALOR DE DIA DE VOO: recalcular com a previsao de vento em altitude '
        '(ver profile.tecs_clmb_max).', tol=0.01),
    'TECS_SINK_MAX': _p(
        1.0, 'm/s', 'gradiente',
        'O utilizador decidiu aplicar os 10 % tambem a descida: 0,10 x (15 - 5) = 1,0. '
        'ATENCAO: fica abaixo de TECS_SINK_MIN = 2,0 (risco aberto).', tol=0.01),
    'TECS_SINK_MIN': _p(
        2.0, 'm/s', 'gradiente',
        'Afundamento ao ralenti a AIRSPEED_CRUISE. Valor DECLARADO, nunca medido. '
        'Entra no ganho do laco de acelerador (K_thr2STE).', tol=0.01),
    'TECS_PITCH_MAX': _p(
        10.0, 'deg', 'gradiente',
        'Protecao secundaria do gradiente, so ativa em modos de acelerador '
        'automatico. Durante o flare o TECS substitui-o por TECS_LAND_PMAX.', tol=0.01),
    'TECS_PITCH_MIN': _p(
        -15.0, 'deg', 'gradiente',
        'Deixa o TECS_SINK_MAX morder antes do limite de atitude na descida.', tol=0.01),
    'TECS_TIME_CONST': _p(
        8.0, 's', 'gradiente',
        'Baixar TECS_CLMB_MAX para 0,8 multiplica o ganho do laco de acelerador '
        'por 2,5; subir tc para 8 recupera cerca de metade. Confianca BAIXA: '
        'se o acelerador oscilar em SITL, subir para 10.', tol=0.01),
    'PTCH_LIM_MAX_DEG': _p(
        15.0, 'deg', 'gradiente',
        'Reducao moderada face a 25. NAO por 6-7 graus: cortaria o flare da '
        'aterragem automatica e o pitch da subida manual em FBWA.', tol=0.01),
    'AIRSPEED_CRUISE': _p(
        15.0, 'm/s', 'gradiente',
        'Velocidade de cruzeiro real do aviao (nao 14 como no CLAUDE.md). '
        'Base de toda a conta de gradiente.', tol=0.01),
    'WP_LOITER_RAD': _p(
        200.0, 'm', 'navegacao',
        'Raio de loiter por omissao. Os LOITER_TO_ALT da espiral levam param2 = 400 '
        'explicito; os 200 valem para o RTL e para loiters com param2 = 0.', tol=0.01),
    'NAVL1_LIM_BANK': _p(
        0.0, 'deg', 'navegacao',
        'Mantido a 0 (default). Consequencia aceite: loiter_radius() devolve '
        'raio x EAS2TAS^2, logo os 400 m viram ~487 m a 2000 m. Ha ~505 m de '
        'folga ate a borda da area UAS.', tol=0.01),
    'ALT_SLOPE_MIN': _p(
        15.0, 'm', 'navegacao',
        'Nome 4.8 do antigo GLIDE_SLOPE_MIN. Nao afeta a espiral (LOITER_TO_ALT '
        'nao faz rampa), afeta as pernas de aproximacao.', tol=0.01),
    'ALT_SLOPE_MAXHGT': _p(
        5.0, 'm', 'navegacao',
        'Nome 4.8 do antigo GLIDE_SLOPE_THR.', tol=0.01),

    # --- RTL / aterragem ---------------------------------------------------
    'RTL_AUTOLAND': _p(
        4.0, '-', 'rtl',
        'Vai direto a sequencia de aterragem pelo DO_RETURN_PATH_START. '
        'OBRIGATORIO: com 0 e um DO_RETURN_PATH_START na missao, o ARME FALHA.', tol=0.01),
    'TECS_LAND_ARSPD': _p(
        14.5, 'm/s', 'rtl',
        'Velocidade da aproximacao final. -1 (herdar o cruzeiro) deixaria a '
        'final a 15 m/s.', tol=0.01),
    'MIS_RESTART': _p(
        0.0, '-', 'rtl',
        'Retoma a missao de onde parou ao voltar para AUTO.', tol=0.01),

    # --- Failsafes ---------------------------------------------------------
    'FS_LONG_ACTN': _p(
        1.0, '-', 'failsafe',
        'RTL. O default 0 faria o aviao CONTINUAR a missao em AUTO.', tol=0.01),
    'FS_LONG_TIMEOUT': _p(
        10.0, 's', 'failsafe',
        'Subido de 5 para 10: a 2000 m ha tempo para a ligacao recuperar antes '
        'de abortar uma subida de 40 minutos.', tol=0.01),
    'FS_SHORT_ACTN': _p(
        0.0, '-', 'failsafe',
        'Continua em AUTO na falha curta. A espiral ja e a trajetoria segura.', tol=0.01),
    'THR_FAILSAFE': _p(
        1.0, '-', 'failsafe',
        'Deteta perda de RC pelo PWM do canal de acelerador.', tol=0.01),
    'THR_FS_VALUE': _p(
        950.0, 'PWM', 'failsafe',
        'PENDENTE DE VERIFICACAO: desligar o emissor no chao e ler o PWM real '
        'do recetor.', tol=0.01),
    'FS_GCS_ENABL': _p(
        1.0, '-', 'failsafe',
        'Failsafe por perda de telemetria.', tol=0.01),

    # --- Fence -------------------------------------------------------------
    'FENCE_ENABLE': _p(
        1.0, '-', 'fence',
        'Estava a 0. A missao exige contencao na area UAS.', tol=0.01),
    'FENCE_TYPE': _p(
        5.0, 'bitmask', 'fence',
        '1 (altitude maxima) + 4 (poligono de inclusao). O bit 2 (FENCE_RADIUS '
        'circular) fica DESLIGADO: 300 m nao chega para a espiral de 400 m.', tol=0.01),
    'FENCE_ALT_MAX': _p(
        2100.0, 'm', 'fence',
        'Acima dos 2000 m alvo, com folga para o overshoot do TECS. '
        'Estava a 100 m, incompativel com a missao.', tol=0.01),
    'FENCE_ALT_MAX_TP': _p(
        1.0, '-', 'fence',
        'FENCE_ALT_MAX medido a partir do ponto de descolagem, nao AMSL. '
        'Torna o teto independente da elevacao do terreno (que e pendencia).', tol=0.01),
    'FENCE_AUTOENABLE': _p(
        3.0, '-', 'fence',
        'Liga a fence no arme e desliga-a na aterragem automatica.', tol=0.01),
    'FENCE_ACTION': _p(
        1.0, '-', 'fence',
        'RTL ao violar a fence.', tol=0.01),

    # --- Bateria -----------------------------------------------------------
    'BATT_CAPACITY': _p(
        27000.0, 'mAh', 'bateria',
        'Capacidade declarada da bateria principal.', tol=1.0),
    'BATT_LOW_MAH': _p(
        8100.0, 'mAh', 'bateria',
        '30 % de reserva. Deixa 18 900 mAh uteis. Quanto tempo isso da depende '
        'do consumo medio, que NUNCA FOI MEDIDO.', tol=1.0),
    'BATT_CRT_MAH': _p(
        5400.0, 'mAh', 'bateria',
        '20 % de reserva. A 30 A da 10,8 min -- menos do que a descida de '
        '31 min com TECS_SINK_MAX = 1,0. Ver riscos.', tol=1.0),
    'BATT_FS_LOW_ACT': _p(
        1.0, '-', 'bateria',
        'RTL na bateria baixa.', tol=0.01),
    'BATT_FS_CRT_ACT': _p(
        2.0, '-', 'bateria',
        'Aterrar na bateria critica.', tol=0.01),
}


# ---------------------------------------------------------------------------
# Riscos abertos. Nao sao parametros -- sao coisas que ficam por fechar.
# ---------------------------------------------------------------------------
OPEN_RISKS = [
    ('TECS_SINK_MIN > TECS_SINK_MAX',
     'TECS_SINK_MIN = 2,0 e maior que TECS_SINK_MAX = 1,0. SINK_MIN e o '
     'afundamento ao ralenti: pedir 1,0 m/s obriga a descer COM MOTOR durante '
     'os ~31 min da descida, e os dois limites entram em contradicao dentro do '
     'TECS (AP_TECS.cpp:739 usa -SINK_MIN*g em _STEdot_min, :1037 usa '
     '-SINK_MAX*g no laco de pitch). Medir o afundamento real ao ralenti antes '
     'de aceitar este par de valores.'),
    ('Consumo medio nunca medido',
     'Subida (39,6 min a 5 m/s de vento) + descida (31 min) = ~70 min so para '
     'subir e descer. Com 18 900 mAh uteis, a 20 A sao 56,7 min: NAO CHEGA. '
     'Medir o consumo em subida e transformar o teto de vento num criterio de '
     'GO/NO-GO.'),
    ('QFU da pista desconhecido',
     'O KML so traz a pista como ponto. O eixo 137,2/317,2 graus e geometrico, '
     'escolhido para manter a final dentro do circulo. Com o QFU real, '
     'regenerar a missao com --approach-brg.'),
    ('Elevacao do terreno desconhecida',
     'Sem ela nao se verifica o teto de 3000 m AMSL da area UAS nem se fecha a '
     'altitude absoluta do HOME. FENCE_ALT_MAX_TP = 1 contorna o problema para '
     'a fence, mas nao para a autorizacao de espaco aereo.'),
    ('Coordenacao de espaco aereo',
     '2000 m acima da pista fica acima da Med-Hi Boundary (1371 m AMSL): '
     'camada "Hi", exige coordenacao com o Air Ops Control.'),
    ('TECS_TIME_CONST = 8 por validar',
     'Confianca baixa. Verificar em SITL se o acelerador oscila durante a '
     'subida; se sim, subir para 10 e/ou TECS_THR_DAMP de 0,5 para 0,7.'),
]


def expected_value(name):
    """Valor esperado de um parametro, ou `None` se nao fizer parte da tabela."""
    p = ARTEX_PARAMS.get(name)
    return None if p is None else p.value


def check_params(fcu_values, params=None):
    """Compara os valores lidos do FCU com a tabela.

    `fcu_values` e' um dict `{nome: valor}`; nomes em falta sao reportados como
    'nao lido'. Devolve (ok, divergencias, nao_lidos).
    """
    table = params or ARTEX_PARAMS
    divergences = []
    missing = []
    for name, p in sorted(table.items()):
        if name not in fcu_values or fcu_values[name] is None:
            missing.append(name)
            continue
        got = float(fcu_values[name])
        if abs(got - p.value) > p.tol:
            divergences.append((name, got, p.value, p.reason))
    return (not divergences and not missing), divergences, missing


def to_param_file(params=None):
    """Serializa no formato `.param` (NOME,VALOR) do Mission Planner."""
    table = params or ARTEX_PARAMS
    return ''.join('%s,%g\n' % (n, p.value) for n, p in sorted(table.items()))


def main():
    """Imprime a tabela e os riscos abertos."""
    print('Parametros ARTEX 2000 m (ArduPlane 4.8)\n')
    group = None
    for name, p in sorted(ARTEX_PARAMS.items(), key=lambda kv: (kv[1].group, kv[0])):
        if p.group != group:
            group = p.group
            print('[%s]' % group)
        print('  %-18s %-10g %-7s %s' % (name, p.value, p.unit, p.reason.split('.')[0] + '.'))
    print('\nRISCOS ABERTOS')
    for title, text in OPEN_RISKS:
        print('  * %s\n      %s' % (title, text))


if __name__ == '__main__':
    main()
