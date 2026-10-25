#!/usr/bin/env python3
"""Construcao da missao ARTEX 2000 m em codigo, com a `mavros_library`.

Substitui o ficheiro `.waypoints` escrito a mao: a missao passa a ser gerada a
partir das constantes de `geo.py` e `profile.py`, o que permite mudar o rumo de
aproximacao, as altitudes ou o raio sem reescrever nada.

Estrutura (21 itens na variante automatica):

    0   HOME                      (frame 0, altitude ABSOLUTA -- placeholder)
    1-9 LOITER_TO_ALT             espiral de subida, 300 -> 2000 m
    10-11 WAYPOINT                PLACEHOLDER da missao principal a 2000 m
    12  DO_RETURN_PATH_START      inicio do caminho de retorno
    13-16 LOITER_TO_ALT           espiral de descida, 1500 -> 140 m
    17  DO_LAND_START             inicio da sequencia de aterragem
    18  WAYPOINT                  porta de entrada da final, 140 m
    19  WAYPOINT                  FAF, 60 m
    20  LAND                      cabeceira (= HOME)

A ordem `DO_RETURN_PATH_START` -> ... -> `DO_LAND_START` -> ... -> `LAND` e'
**obrigatoria**: `AP_Mission::distance_to_mission_leg()` percorre a missao para
a frente a partir do `DO_RETURN_PATH_START` e para no primeiro `DO_LAND_START`;
sem ele o retorno falha. Nao reordenar os itens 12 em diante.

A descolagem e' MANUAL. Esta missao nao tem `NAV_TAKEOFF`: o piloto descola,
sobe ate' ~100 m sobre a pista e passa para AUTO. Por isso nao se usa
`run_mission()` da `mavros_library` (que faz GUIDED -> arm -> AUTO).
"""

from mavros_library.commands import create_waypoint
from mavros_library.config import COMMANDS, FRAMES

from . import geo
from . import profile

# ---------------------------------------------------------------------------
# Comandos
# ---------------------------------------------------------------------------
CMD_WAYPOINT = COMMANDS['MAV_CMD_NAV_WAYPOINT']              # 16
CMD_LOITER_UNLIM = COMMANDS['MAV_CMD_NAV_LOITER_UNLIM']      # 17
CMD_LAND = COMMANDS['MAV_CMD_NAV_LAND']                      # 21
CMD_LOITER_TO_ALT = COMMANDS['MAV_CMD_NAV_LOITER_TO_ALT']    # 31
CMD_DO_LAND_START = COMMANDS['MAV_CMD_DO_LAND_START']        # 189

# LACUNA DA mavros_library: `MAV_CMD_DO_RETURN_PATH_START` (188) nao existe no
# `commands.json` da biblioteca -- o ficheiro salta de 187 (DO_SET_ACTUATOR)
# para 189 (DO_LAND_START). A `mavros_library` e' um submodulo git e nao deve
# ser editada aqui, por isso o 188 fica definido localmente.
# TODO(upstream): abrir PR em AeroTec-ATLAS/mavros_library a acrescentar
#   "MAV_CMD_DO_RETURN_PATH_START": 188
# ao `mavros_library/config/commands.json`, e passar a usar COMMANDS[...].
CMD_DO_RETURN_PATH_START = COMMANDS.get('MAV_CMD_DO_RETURN_PATH_START', 188)

FRAME_GLOBAL = FRAMES['MAV_FRAME_GLOBAL']                        # 0, AMSL
FRAME_REL = FRAMES['MAV_FRAME_GLOBAL_RELATIVE_ALT']              # 3, rel. ao home

# ---------------------------------------------------------------------------
# Geometria da aproximacao final
# ---------------------------------------------------------------------------
APP_GATE_DIST_M = 1600.0    # porta de entrada da final, antes do ponto de toque
APP_GATE_ALT_M = 140.0      # igual ao ultimo LOITER_TO_ALT: perna de ligacao nivelada
APP_FAF_DIST_M = 700.0      # fix de aproximacao final
APP_FAF_ALT_M = 60.0        # rampas de 8,9 % e 8,6 %: dentro dos 10 %

# Variante de pouso manual
MANUAL_LOITER_DIST_M = 300.0   # da pista, no eixo de aproximacao, para dentro do circulo
MANUAL_LOITER_ALT_M = 150.0
MANUAL_LOITER_RADIUS_M = 200.0

# Fim do caminho de regresso em failsafe quando `return_loiter=True`: em vez de
# aterrar, o aviao espera pelo piloto. O DO_LAND_START CONTINUA a ser preciso --
# `distance_to_mission_leg()` (AP_Mission.cpp:2792) varre para a frente a partir
# do DO_RETURN_PATH_START e so' devolve sucesso se encontrar um DO_LAND_START ou
# um comando de aterragem; um LOITER_UNLIM sozinho faria o regresso FALHAR.
# Nada obriga a um LAND depois do DO_LAND_START (AP_Arming_Plane.cpp:425-433).
#
# AVISO: num failsafe de BATERIA isto circula ate' a bateria acabar. So' usar
# quando houver piloto a acompanhar.
RETURN_LOITER_DIST_M = 300.0
RETURN_LOITER_ALT_M = 150.0
RETURN_LOITER_RADIUS_M = 200.0

# Placeholders da missao principal a 2000 m
PLACEHOLDER_DIST_M = 500.0
PLACEHOLDER_BRGS_DEG = (47.0, 227.0)

# Altitude ABSOLUTA do HOME. PENDENCIA: a elevacao do terreno na pista nao foi
# medida. 0.0 e' um placeholder honesto -- o ArduPilot reescreve o item 0 com o
# home real no momento do arme, por isso nao e' critico, mas nao serve para
# verificar tetos de espaco aereo.
HOME_AMSL_PLACEHOLDER = 0.0


def _wp(command, lat, lon, alt, frame=FRAME_REL,
        p1=0.0, p2=0.0, p3=0.0, p4=0.0, is_current=False):
    """Atalho sobre `create_waypoint` (param5/6/7 = lat/lon/alt)."""
    return create_waypoint(frame, command, is_current, True,
                           p1, p2, p3, p4, lat, lon, alt)


def build_mission(manual_land=False,
                  approach_brg=None,
                  climb_steps_m=None,
                  descent_steps_m=None,
                  loiter_radius_m=profile.LOITER_RADIUS_M,
                  home_amsl_m=HOME_AMSL_PLACEHOLDER,
                  main_mission=None,
                  return_loiter=False,
                  avoid_waypoints=None):
    """Monta a missao completa e devolve uma `list[mavros_msgs.msg.Waypoint]`.

    `manual_land=True` acrescenta, antes do `DO_RETURN_PATH_START`, uma descida
    nominal propria terminada em `LOITER_UNLIM` a 150 m: o piloto assume ai' e
    pousa a' mao. A sequencia de retorno automatico fica intocada, porque e' ela
    que o failsafe usa.

    `main_mission` substitui os dois waypoints placeholder a 2000 m. Deve ser
    uma lista de `(lat, lon, alt_rel)`; todos os pontos tem de ficar dentro do
    circulo UAS (verificado por `validate_mission`).

    `avoid_waypoints` sao pontos `(lat, lon, alt_rel)` inseridos no caminho de
    regresso, entre a espiral de descida e o DO_LAND_START. E' assim que se
    contorna uma zona proibida: o ArduPlane NAO tem planeador de desvio (o
    BendyRuler/Dijkstra existe so' para Copter e Rover), e a geofence e' uma
    restricao que dispara uma reaccao, nao um encaminhador. Quem evita a zona e'
    a rota; a fence e' a rede de seguranca.

    `return_loiter=True` termina o caminho de regresso em LOITER_UNLIM em vez de
    aterrar -- ver o aviso em RETURN_LOITER_ALT_M.
    """
    if approach_brg is None or approach_brg < 0.0:
        approach_brg = geo.DEFAULT_APPROACH_BRG_DEG
    climb = list(climb_steps_m or profile.CLIMB_STEPS_M)
    descent = list(descent_steps_m or profile.DESCENT_STEPS_M)
    back = (approach_brg + 180.0) % 360.0   # do ponto de toque para tras

    items = []

    # 0 -- HOME. Frame 0 (AMSL). O ArduPilot reescreve-o no arme.
    items.append(_wp(CMD_WAYPOINT, geo.RUNWAY[0], geo.RUNWAY[1], home_amsl_m,
                     frame=FRAME_GLOBAL, is_current=True))

    # 1..9 -- espiral de subida, LOITER_TO_ALT empilhados no centro do circulo.
    #   param1 = ignorado pelo ArduPlane
    #   param2 = raio em metros; o sinal da' o sentido; 0 cairia no WP_LOITER_RAD
    #   param3 = nao usado
    #   param4 = 0 -> crosstrack a partir do centro; 1 -> a partir da tangente
    for alt in climb:
        items.append(_wp(CMD_LOITER_TO_ALT, geo.UAS_CENTER[0], geo.UAS_CENTER[1],
                         float(alt), p2=loiter_radius_m, p4=0.0))

    # 10..11 -- missao principal a 2000 m (placeholders por omissao).
    for lat, lon, alt in _main_mission_points(main_mission):
        items.append(_wp(CMD_WAYPOINT, lat, lon, alt))

    if manual_land:
        # Fim nominal: desce em espiral ate' a altitude de circuito e espera o
        # piloto. A descida TEM de estar aqui -- nao se vai de 2000 m direto
        # para um loiter a 150 m.
        for alt in descent[:-1] + [MANUAL_LOITER_ALT_M]:
            items.append(_wp(CMD_LOITER_TO_ALT, geo.UAS_CENTER[0], geo.UAS_CENTER[1],
                             float(alt), p2=loiter_radius_m, p4=0.0))
        # LOITER_UNLIM deslocado 300 m da pista para dentro do circulo: com raio
        # de 200 m, um loiter centrado na pista sairia ~130 m da area UAS.
        # Em LOITER_UNLIM o raio vai em param3, nao em param2.
        lat, lon = geo.offset_latlon(geo.RUNWAY, back, MANUAL_LOITER_DIST_M)
        items.append(_wp(CMD_LOITER_UNLIM, lat, lon, MANUAL_LOITER_ALT_M,
                         p3=MANUAL_LOITER_RADIUS_M))

    # DO_RETURN_PATH_START -- lat/lon a 0,0 de proposito: assim
    # `location.initialised()` e' falso e o item nao entra no calculo de
    # distancia das pernas de retorno.
    items.append(_wp(CMD_DO_RETURN_PATH_START, 0.0, 0.0, 0.0))

    # Espiral de descida, mesmo centro. As pernas entre estes itens sao
    # puramente verticais, o que faz `jump_to_closest_mission_leg()` reengatar
    # na faixa de altitude em que o aviao esta' quando o failsafe dispara.
    for i, alt in enumerate(descent):
        last = (i == len(descent) - 1)
        items.append(_wp(CMD_LOITER_TO_ALT, geo.UAS_CENTER[0], geo.UAS_CENTER[1],
                         float(alt), p2=loiter_radius_m,
                         p4=1.0 if last else 0.0))

    # Contorno de zona proibida, se definido. Fica DEPOIS da espiral de descida
    # e ANTES do DO_LAND_START, para entrar na varredura das pernas de regresso.
    for lat, lon, alt in (avoid_waypoints or []):
        items.append(_wp(CMD_WAYPOINT, float(lat), float(lon), float(alt)))

    # DO_LAND_START
    items.append(_wp(CMD_DO_LAND_START, 0.0, 0.0, 0.0))

    if return_loiter:
        # Espera pelo piloto em vez de aterrar.
        lat, lon = geo.offset_latlon(geo.RUNWAY, back, RETURN_LOITER_DIST_M)
        items.append(_wp(CMD_LOITER_UNLIM, lat, lon, RETURN_LOITER_ALT_M,
                         p3=RETURN_LOITER_RADIUS_M))
        return items

    # Sequencia de aterragem, alinhada ao eixo de aproximacao
    lat, lon = geo.offset_latlon(geo.RUNWAY, back, APP_GATE_DIST_M)
    items.append(_wp(CMD_WAYPOINT, lat, lon, APP_GATE_ALT_M))
    lat, lon = geo.offset_latlon(geo.RUNWAY, back, APP_FAF_DIST_M)
    items.append(_wp(CMD_WAYPOINT, lat, lon, APP_FAF_ALT_M))
    items.append(_wp(CMD_LAND, geo.RUNWAY[0], geo.RUNWAY[1], 0.0))

    return items


def _main_mission_points(main_mission):
    """Pontos da missao principal a 2000 m, ou os dois placeholders."""
    if main_mission:
        return [(float(a), float(b), float(c)) for a, b, c in main_mission]
    pts = []
    for brg in PLACEHOLDER_BRGS_DEG:
        lat, lon = geo.offset_latlon(geo.UAS_CENTER, brg, PLACEHOLDER_DIST_M)
        pts.append((lat, lon, profile.CRUISE_ALT_M))
    return pts


# ---------------------------------------------------------------------------
# Validacao
# ---------------------------------------------------------------------------
# Comandos que nao carregam posicao: ficam fora da verificacao de contencao.
_NON_POSITIONAL = (CMD_DO_RETURN_PATH_START, CMD_DO_LAND_START)


def validate_mission(items, manual_land=False, gradient_limit=profile.GRADIENT_LIMIT,
                     return_loiter=False, n_avoid=0):
    """Verifica a missao e devolve uma lista de problemas (vazia = tudo bem).

    Confere:
      1. contagem e sequencia de comandos esperada;
      2. `DO_RETURN_PATH_START` antes do `DO_LAND_START`, com `LAND` depois;
      3. todos os pontos com posicao dentro do circulo UAS;
      4. frames (0 so' no HOME, 3 nos restantes);
      5. gradiente de cada perna entre waypoints de aproximacao.
    """
    errors = []
    expected = expected_sequence(manual_land, return_loiter, n_avoid)

    if len(items) != len(expected):
        errors.append(f'Numero de itens = {len(items)}, esperado {len(expected)}.')

    for i, (item, cmd) in enumerate(zip(items, expected)):
        if item.command != cmd:
            errors.append(f'Item {i}: comando {item.command}, esperado {cmd}.')

    cmds = [it.command for it in items]
    if CMD_DO_RETURN_PATH_START not in cmds:
        errors.append('Falta DO_RETURN_PATH_START (188).')
    elif CMD_DO_LAND_START not in cmds:
        errors.append('Falta DO_LAND_START (189).')
    elif cmds.index(CMD_DO_RETURN_PATH_START) > cmds.index(CMD_DO_LAND_START):
        errors.append('DO_RETURN_PATH_START tem de vir ANTES do DO_LAND_START.')
    elif return_loiter:
        # Variante de espera: o DO_LAND_START e' preciso so' para
        # `distance_to_mission_leg()` (AP_Mission.cpp:2792) terminar a varredura
        # com sucesso. Nada obriga a um LAND a seguir
        # (AP_Arming_Plane.cpp:425-433), e aqui nao se quer um.
        if CMD_LOITER_UNLIM not in cmds[cmds.index(CMD_DO_LAND_START):]:
            errors.append('return_loiter: falta LOITER_UNLIM depois do DO_LAND_START.')
    elif CMD_LAND not in cmds[cmds.index(CMD_DO_LAND_START):]:
        errors.append('Nao ha LAND depois do DO_LAND_START.')

    for i, item in enumerate(items):
        if item.command in _NON_POSITIONAL:
            continue
        if item.x_lat == 0.0 and item.y_long == 0.0:
            errors.append(f'Item {i}: posicao 0,0 num comando posicional.')
            continue
        margin = geo.uas_margin_m((item.x_lat, item.y_long))
        if margin < 0.0:
            errors.append(f'Item {i}: {-margin:.0f} m FORA do circulo UAS.')

    if items and items[0].frame != FRAME_GLOBAL:
        errors.append('Item 0 (HOME) tem de usar o frame 0 (altitude AMSL).')
    for i, item in enumerate(items[1:], start=1):
        if item.frame != FRAME_REL:
            errors.append(f'Item {i}: frame {item.frame}, esperado {FRAME_REL}.')

    errors.extend(_approach_gradient_errors(items, gradient_limit))
    return errors


def _approach_gradient_errors(items, gradient_limit):
    """Gradiente das pernas da aproximacao final (as unicas com rampa real)."""
    out = []
    try:
        start = [it.command for it in items].index(CMD_DO_LAND_START)
    except ValueError:
        return out
    legs = items[start + 1:]
    for a, b in zip(legs, legs[1:]):
        dist = geo.distance_m((a.x_lat, a.y_long), (b.x_lat, b.y_long))
        if dist <= 0.0:
            continue
        grad = abs(a.z_alt - b.z_alt) / dist
        if grad > gradient_limit:
            out.append(f'Perna de aproximacao {a.z_alt:.0f} -> {b.z_alt:.0f} m '
                       f'em {dist:.0f} m: gradiente {grad * 100:.1f} % '
                       f'> {gradient_limit * 100:.0f} %.')
    return out


def expected_sequence(manual_land=False, return_loiter=False, n_avoid=0):
    """Sequencia de comandos esperada, para a validacao e para os testes."""
    seq = [CMD_WAYPOINT]
    seq += [CMD_LOITER_TO_ALT] * len(profile.CLIMB_STEPS_M)
    seq += [CMD_WAYPOINT] * 2
    if manual_land:
        seq += [CMD_LOITER_TO_ALT] * len(profile.DESCENT_STEPS_M)
        seq += [CMD_LOITER_UNLIM]
    seq += [CMD_DO_RETURN_PATH_START]
    seq += [CMD_LOITER_TO_ALT] * len(profile.DESCENT_STEPS_M)
    seq += [CMD_WAYPOINT] * n_avoid
    if return_loiter:
        seq += [CMD_DO_LAND_START, CMD_LOITER_UNLIM]
    else:
        seq += [CMD_DO_LAND_START, CMD_WAYPOINT, CMD_WAYPOINT, CMD_LAND]
    return seq


def describe(items):
    """Linhas legiveis da missao, para log e para o no de verificacao."""
    names = {
        CMD_WAYPOINT: 'WAYPOINT',
        CMD_LOITER_UNLIM: 'LOITER_UNLIM',
        CMD_LAND: 'LAND',
        CMD_LOITER_TO_ALT: 'LOITER_TO_ALT',
        CMD_DO_RETURN_PATH_START: 'DO_RETURN_PATH_START',
        CMD_DO_LAND_START: 'DO_LAND_START',
    }
    out = []
    for i, it in enumerate(items):
        name = names.get(it.command, str(it.command))
        if it.x_lat == 0.0 and it.y_long == 0.0:
            where = '-'
        else:
            where = (f'{it.x_lat:.7f},{it.y_long:.7f} '
                     f'(r={geo.distance_m(geo.UAS_CENTER, (it.x_lat, it.y_long)):.0f} m)')
        out.append(f'{i:3d}  {name:<21s} alt={it.z_alt:7.1f}  frame={it.frame}  {where}')
    return out


def to_wpl(items):
    """Serializa no formato QGC WPL 110 (TAB-separado), para gravar um .waypoints."""
    lines = ['QGC WPL 110']
    for i, it in enumerate(items):
        lines.append('\t'.join([
            str(i), '1' if it.is_current else '0', str(it.frame), str(it.command),
            f'{it.param1:.8f}', f'{it.param2:.8f}',
            f'{it.param3:.8f}', f'{it.param4:.8f}',
            f'{it.x_lat:.8f}', f'{it.y_long:.8f}', f'{it.z_alt:.6f}',
            '1' if it.autocontinue else '0',
        ]))
    return '\n'.join(lines) + '\n'


def main():
    """Gera a missao e imprime a descricao e o resultado da validacao."""
    import argparse
    ap = argparse.ArgumentParser(description='Gera a missao ARTEX 2000 m.')
    ap.add_argument('--manual-land', action='store_true',
                    help='variante com LOITER_UNLIM para pouso manual')
    ap.add_argument('--approach-brg', type=float, default=None,
                    help='rumo de aproximacao em graus; omissao = %.1f (centro -> pista)'
                         % geo.DEFAULT_APPROACH_BRG_DEG)
    ap.add_argument('--return-loiter', action='store_true',
                    help='o caminho de regresso termina em LOITER_UNLIM a espera do '
                         'piloto, em vez de aterrar (cuidado: em failsafe de bateria '
                         'circula ate a bateria acabar)')
    ap.add_argument('--avoid', metavar='LAT,LON,ALT', action='append', default=[],
                    help='ponto de contorno de zona proibida no caminho de regresso; '
                         'repetir para varios, pela ordem de passagem')
    ap.add_argument('--wpl', metavar='FICHEIRO',
                    help='grava tambem em formato QGC WPL 110')
    args = ap.parse_args()

    avoid = []
    for spec in args.avoid:
        parts = spec.split(',')
        if len(parts) != 3:
            ap.error('--avoid espera LAT,LON,ALT (recebido: %r)' % spec)
        avoid.append(tuple(float(x) for x in parts))

    items = build_mission(manual_land=args.manual_land, approach_brg=args.approach_brg,
                          return_loiter=args.return_loiter, avoid_waypoints=avoid)
    for line in describe(items):
        print(line)
    errors = validate_mission(items, manual_land=args.manual_land,
                              return_loiter=args.return_loiter, n_avoid=len(avoid))
    print()
    if errors:
        print('PROBLEMAS:')
        for e in errors:
            print('  - ' + e)
    else:
        print('Missao valida: %d itens, todos dentro do circulo UAS.' % len(items))
    if args.wpl:
        with open(args.wpl, 'w', encoding='utf-8') as fh:
            fh.write(to_wpl(items))
        print('Gravado em ' + args.wpl)
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
