#!/usr/bin/env python3
"""Perfil vertical da missao ARTEX: subida, descida, espiral e gradiente.

E' aqui que vivem as formulas. Nenhuma depende de ROS, para poderem ser
verificadas com `python3 -m artex27.profile` ou importadas num teste.

Regra que manda em tudo: **o gradiente sobre o solo nunca passa de 10 %**.
Numa espiral de `LOITER_TO_ALT` os waypoints nao impoem rampa nenhuma -- o
gradiente e' controlado so' pelo TECS. Logo:

    TECS_CLMB_MAX = margem x limite x (AIRSPEED_CRUISE - vento)

O vento entra porque o gradiente e' razao de subida a dividir pela velocidade
**sobre o solo**, e com vento de frente a velocidade sobre o solo cai.
`TECS_CLMB_MAX` e' por isso um valor de dia de voo: define-se a partir da
previsao de vento em altitude, nao fica fixo no ficheiro.
"""

import math

# ---------------------------------------------------------------------------
# Constantes da aeronave (medidas, ver `params/artex27_diff.md`)
# ---------------------------------------------------------------------------
AIRSPEED_CRUISE = 15.0     # m/s EAS
AIRSPEED_MIN = 14.0        # m/s
AIRSPEED_MAX = 27.0        # m/s

GRADIENT_LIMIT = 0.10      # requisito ARTEX: 10 %
CLIMB_MARGIN = 0.8         # 20 % de folga -> fator efetivo 0.08
DESCENT_MARGIN = 1.0       # na descida usa-se o limite cheio (decisao do utilizador)

# Vento de projeto assumido para fechar os valores do ficheiro .param.
DESIGN_WIND_MS = 5.0

# ---------------------------------------------------------------------------
# Geometria da espiral
# ---------------------------------------------------------------------------
LOITER_RADIUS_M = 400.0
CLIMB_STEPS_M = [300, 500, 700, 900, 1100, 1300, 1500, 1750, 2000]
DESCENT_STEPS_M = [1500, 1000, 500, 140]
CRUISE_ALT_M = 2000.0

# Atmosfera padrao ISA
RHO0 = 1.225
_ISA_K = 2.25577e-5
_ISA_N = 4.25588


def tecs_clmb_max(airspeed=AIRSPEED_CRUISE, wind=0.0,
                  gradient_limit=GRADIENT_LIMIT, margin=CLIMB_MARGIN):
    """Razao de subida maxima [m/s] que respeita o gradiente sobre o solo.

    `margin` < 1 reserva folga: 0.8 sobre um limite de 0.10 da' o fator 0.08 do
    BRIEF, isto e', 8 % de gradiente nominal com 2 pontos percentuais de reserva
    para erro de estimativa de vento e para rajadas.
    """
    ground_speed = airspeed - wind
    if ground_speed <= 0.0:
        raise ValueError('Vento maior ou igual que a velocidade de cruzeiro.')
    return gradient_limit * margin * ground_speed


def tecs_sink_max(airspeed=AIRSPEED_CRUISE, wind=DESIGN_WIND_MS,
                  gradient_limit=GRADIENT_LIMIT, margin=DESCENT_MARGIN):
    """Razao de afundamento maxima [m/s] com o mesmo criterio de gradiente.

    O utilizador decidiu aplicar os 10 % tambem a' descida. Com V = 15 e
    W = 5 da' 1,0 m/s. Ver `params.py`: este valor entra em conflito com
    `TECS_SINK_MIN = 2.0` e isso esta' registado como risco aberto.
    """
    ground_speed = airspeed - wind
    if ground_speed <= 0.0:
        raise ValueError('Vento maior ou igual que a velocidade de cruzeiro.')
    return gradient_limit * margin * ground_speed


def gradient(vertical_rate, airspeed=AIRSPEED_CRUISE, wind=0.0):
    """Gradiente sobre o solo (fracao, nao percentagem) para uma razao vertical."""
    return abs(vertical_rate) / (airspeed - wind)


def eas2tas(alt_amsl_m):
    """Fator EAS->TAS pela atmosfera ISA. Ao nivel do mar vale 1,0."""
    sigma = (1.0 - _ISA_K * alt_amsl_m) ** _ISA_N   # rho/rho0
    return math.sqrt(1.0 / sigma)


def loiter_radius_eff(radius_m, alt_amsl_m):
    """Raio de loiter efetivo do ArduPlane a uma dada altitude.

    Com `NAVL1_LIM_BANK = 0` (o valor desta missao),
    `AP_L1_Control::loiter_radius()` devolve `raio x EAS2TAS^2`
    incondicionalmente: o circulo cresce com a altitude. A 2000 m sao ~1,22x,
    ou seja os 400 m comandados viram ~487 m reais.
    """
    return radius_m * eas2tas(alt_amsl_m) ** 2


def turn_period_s(radius_m, alt_amsl_m, airspeed=AIRSPEED_CRUISE):
    """Tempo de uma volta completa [s].

    `2*pi*r_eff / TAS` = `2*pi*r*EAS2TAS / EAS`.
    """
    return 2.0 * math.pi * radius_m * eas2tas(alt_amsl_m) / airspeed


def _segment(alt_from, alt_to, rate, radius_m, airspeed, terrain_elev_m):
    """Numeros de um degrau da espiral entre duas altitudes relativas."""
    dh = abs(alt_to - alt_from)
    t = dh / rate if rate > 0 else float('inf')
    alt_mid_amsl = 0.5 * (alt_from + alt_to) + terrain_elev_m
    period = turn_period_s(radius_m, alt_mid_amsl, airspeed)
    tas = airspeed * eas2tas(alt_mid_amsl)
    return {
        'alt_from': alt_from,
        'alt_to': alt_to,
        'delta_m': dh,
        'time_s': t,
        'turns': t / period if period > 0 else 0.0,
        'turn_period_s': period,
        'radius_eff_m': loiter_radius_eff(radius_m, alt_mid_amsl),
        'air_distance_m': tas * t,
    }


def climb_profile(wind=DESIGN_WIND_MS, airspeed=AIRSPEED_CRUISE,
                  entry_alt_m=100.0, steps_m=None, radius_m=LOITER_RADIUS_M,
                  terrain_elev_m=0.0, margin=CLIMB_MARGIN):
    """Perfil completo da subida, degrau a degrau.

    `terrain_elev_m` so' entra no calculo de densidade do ar (raio e TAS). Com 0
    o resultado e' ligeiramente conservador: o ar la' em cima e' menos denso que
    o assumido, logo o raio e o TAS reais sao um pouco maiores.
    """
    steps = list(steps_m or CLIMB_STEPS_M)
    rate = tecs_clmb_max(airspeed, wind, margin=margin)
    segs = []
    prev = entry_alt_m
    for alt in steps:
        segs.append(_segment(prev, alt, rate, radius_m, airspeed, terrain_elev_m))
        prev = alt
    return _summarize(segs, rate, wind, airspeed, 'subida')


def descent_profile(wind=DESIGN_WIND_MS, airspeed=AIRSPEED_CRUISE,
                    top_alt_m=CRUISE_ALT_M, steps_m=None,
                    radius_m=LOITER_RADIUS_M, terrain_elev_m=0.0,
                    margin=DESCENT_MARGIN, sink_rate=None):
    """Perfil completo da descida em espiral, degrau a degrau."""
    steps = list(steps_m or DESCENT_STEPS_M)
    rate = sink_rate if sink_rate else tecs_sink_max(airspeed, wind, margin=margin)
    segs = []
    prev = top_alt_m
    for alt in steps:
        segs.append(_segment(prev, alt, rate, radius_m, airspeed, terrain_elev_m))
        prev = alt
    return _summarize(segs, rate, wind, airspeed, 'descida')


def _summarize(segs, rate, wind, airspeed, kind):
    total_t = sum(s['time_s'] for s in segs)
    return {
        'tipo': kind,
        'vento_ms': wind,
        'airspeed_ms': airspeed,
        'razao_ms': rate,
        'gradiente': gradient(rate, airspeed, wind),
        'degraus': segs,
        'tempo_s': total_t,
        'tempo_min': total_t / 60.0,
        'voltas': sum(s['turns'] for s in segs),
        'distancia_m': sum(s['air_distance_m'] for s in segs),
        'raio_efetivo_max_m': max(s['radius_eff_m'] for s in segs),
    }


def wind_table(winds=(0, 2, 5, 7, 8, 10), airspeed=AIRSPEED_CRUISE,
               entry_alt_m=100.0, top_alt_m=CRUISE_ALT_M):
    """Tabela `TECS_CLMB_MAX` x vento, com o tempo de subida correspondente.

    Reproduz a tabela da seccao 1 do RECONCILIATION.md.
    """
    rows = []
    for w in winds:
        if w >= airspeed:
            continue
        clmb = tecs_clmb_max(airspeed, w)
        t = (top_alt_m - entry_alt_m) / clmb
        rows.append({
            'vento_ms': w,
            'clmb_max_ms': clmb,
            'gradiente': gradient(clmb, airspeed, w),
            'subida_s': t,
            'subida_min': t / 60.0,
        })
    return rows


def endurance_window(battery_mah=27000.0, low_mah=8100.0, avg_current_a=None):
    """Janela de voo ate' ao failsafe de bateria baixa, em minutos.

    Devolve `None` se o consumo medio nao for conhecido -- e e' esse o caso:
    o consumo em subida **nunca foi medido** e e' hoje o dado que bloqueia a
    missao (ver RECONCILIATION.md, seccao 4). Nao inventar um valor.
    """
    if not avg_current_a:
        return None
    usable_mah = battery_mah - low_mah
    return (usable_mah / 1000.0) / avg_current_a * 60.0


def _fmt(p):
    return (f"{p['tipo']:8s} vento={p['vento_ms']:.0f} m/s  "
            f"razao={p['razao_ms']:.2f} m/s  "
            f"gradiente={p['gradiente'] * 100:.1f} %  "
            f"tempo={p['tempo_min']:.1f} min  "
            f"voltas={p['voltas']:.1f}  "
            f"distancia={p['distancia_m'] / 1000.0:.1f} km  "
            f"raio_ef_max={p['raio_efetivo_max_m']:.0f} m")


def main():
    """Imprime os numeros do perfil para conferencia rapida."""
    print('ARTEX 2000 m -- perfil vertical\n')
    print('Tabela TECS_CLMB_MAX x vento (V = %.0f m/s):' % AIRSPEED_CRUISE)
    print('  %-8s %-14s %-11s %s' % ('vento', 'TECS_CLMB_MAX', 'gradiente', 'subida 100->2000'))
    for r in wind_table():
        print('  %-8.0f %-14.2f %-11.1f %.1f min'
              % (r['vento_ms'], r['clmb_max_ms'], r['gradiente'] * 100, r['subida_min']))
    print()
    for w in (0.0, DESIGN_WIND_MS):
        print(_fmt(climb_profile(wind=w)))
        print(_fmt(descent_profile(wind=w, sink_rate=tecs_sink_max())))
    print('\nConsumo medio em subida: PENDENTE -- sem ele nao se fecha a autonomia.')


if __name__ == '__main__':
    main()
