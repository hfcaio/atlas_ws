#!/usr/bin/env python3
"""Geofence de inclusao da area UAS TO/LD Area 11 TF.

O aviao tem `FENCE_RADIUS = 300` e `FENCE_ALT_MAX = 100`, que sao incompativeis
com esta missao. A fence desta missao e':

  * **inclusao poligonal** inscrita no circulo UAS (`FENCE_TYPE` com o bit 4),
    com um recuo de seguranca para dentro da borda;
  * **teto** `FENCE_ALT_MAX = 2100` m (bit 1), acima dos 2000 m alvo.

`FENCE_TYPE = 5` = 1 (altitude maxima) + 4 (poligono). O `FENCE_RADIUS`
circular (bit 2) fica desligado de proposito: 300 m nao chega para a espiral.

Formato: o plugin de geofence do MAVROS aceita uma lista de `Waypoint` em que
cada vertice usa `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION` (5001) com
`param1` = numero total de vertices do poligono.
"""

from mavros_library.commands import create_waypoint
from mavros_library.config import COMMANDS, FRAMES

from . import geo

CMD_FENCE_RETURN_POINT = COMMANDS['MAV_CMD_NAV_FENCE_RETURN_POINT']                 # 5000
CMD_FENCE_POLY_INCLUSION = COMMANDS['MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION']   # 5001
CMD_FENCE_CIRCLE_INCLUSION = COMMANDS['MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION']         # 5003

FRAME_REL = FRAMES['MAV_FRAME_GLOBAL_RELATIVE_ALT']

# Recuo para dentro da borda do circulo UAS. Um poligono inscrito ja' fica por
# dentro do circulo; o recuo cobre erro de GPS e o `FENCE_MARGIN` do firmware.
FENCE_INSET_M = 20.0
FENCE_VERTICES = 36

# Teto da fence. Acima do alvo de 2000 m, com folga para o overshoot do TECS.
FENCE_ALT_MAX_M = 2100.0


def fence_polygon(inset_m=FENCE_INSET_M, vertices=FENCE_VERTICES):
    """Vertices (lat, lon) do poligono de inclusao."""
    return geo.inscribed_polygon(geo.UAS_CENTER, geo.UAS_RADIUS_M - inset_m, vertices)


def build_fence(inset_m=FENCE_INSET_M, vertices=FENCE_VERTICES, circle=False):
    """Devolve a fence como `list[mavros_msgs.msg.Waypoint]`.

    `circle=True` usa um unico item `FENCE_CIRCLE_INCLUSION` em vez do poligono.
    E' mais fiel a' geometria real da area, mas nem todas as GCS o desenham bem;
    o poligono de 36 vertices e' o que os ficheiros `.waypoints`/`.plan` ja'
    usavam, por isso e' o default.
    """
    if circle:
        return [create_waypoint(
            FRAME_REL, CMD_FENCE_CIRCLE_INCLUSION, False, True,
            geo.UAS_RADIUS_M - inset_m, 0.0, 0.0, 0.0,
            geo.UAS_CENTER[0], geo.UAS_CENTER[1], 0.0)]

    poly = fence_polygon(inset_m, vertices)
    return [create_waypoint(
        FRAME_REL, CMD_FENCE_POLY_INCLUSION, False, True,
        float(len(poly)), 0.0, 0.0, 0.0, lat, lon, 0.0)
        for lat, lon in poly]


def validate_fence(items, mission_items=None):
    """Verifica a fence e, opcionalmente, que a missao cabe la' dentro."""
    errors = []
    if not items:
        errors.append('Fence vazia.')
        return errors

    for i, it in enumerate(items):
        if it.command not in (CMD_FENCE_POLY_INCLUSION, CMD_FENCE_CIRCLE_INCLUSION):
            errors.append(f'Item {i} da fence: comando {it.command} inesperado.')
        if geo.uas_margin_m((it.x_lat, it.y_long)) < 0.0:
            errors.append(f'Item {i} da fence fora do circulo UAS.')

    if items[0].command == CMD_FENCE_POLY_INCLUSION:
        n = int(items[0].param1)
        if n != len(items):
            errors.append(f'param1 = {n} mas a fence tem {len(items)} vertices.')
        if n < 3:
            errors.append('Um poligono de inclusao precisa de pelo menos 3 vertices.')

    if mission_items:
        radius = geo.UAS_RADIUS_M - FENCE_INSET_M
        for i, it in enumerate(mission_items):
            if it.x_lat == 0.0 and it.y_long == 0.0:
                continue
            d = geo.distance_m(geo.UAS_CENTER, (it.x_lat, it.y_long))
            if d > radius:
                errors.append(f'Item {i} da missao esta a {d:.0f} m do centro, '
                              f'fora da fence ({radius:.0f} m).')
    return errors


def to_wpl(items):
    """Formato legado FENCE_POINT: 1a linha = return point, vertices, e repete o 1o.

    E' o que o Mission Planner e o `fence load` do MAVProxy leem.
    """
    pts = [(it.x_lat, it.y_long) for it in items]
    lines = ['%.8f\t%.8f' % geo.UAS_CENTER]
    lines += ['%.8f\t%.8f' % p for p in pts]
    lines.append('%.8f\t%.8f' % pts[0])
    return '\n'.join(lines) + '\n'


def main():
    """Gera a fence e imprime o resultado da validacao."""
    from .mission import build_mission
    items = build_fence()
    print('Fence de inclusao: %d vertices, raio %.1f m (recuo %.0f m).'
          % (len(items), geo.UAS_RADIUS_M - FENCE_INSET_M, FENCE_INSET_M))
    errors = validate_fence(items, build_mission())
    if errors:
        print('PROBLEMAS:')
        for e in errors:
            print('  - ' + e)
        return 1
    print('Fence valida e a missao cabe toda la dentro.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
