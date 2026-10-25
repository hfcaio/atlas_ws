#!/usr/bin/env python3
"""Geografia do exercicio ARTEX 26 e utilitarios geodesicos.

Todas as constantes vieram do ficheiro `ARTEX 26 Geo v173.kml`, extraidas com
`tools/extract_kml.py` (ajuste de circulo por minimos quadrados, residuo maximo
0,017 m -- o circulo da area UAS e' exato no KML).

Este modulo nao depende de ROS nem de nada fora da stdlib: serve para poder
validar a geometria da missao sem levantar um no.

Nota sobre a projecao: a 39 deg de latitude e com distancias abaixo de ~25 km, a
projecao plana local (equirretangular centrada na origem) erra menos de 1 m.
Toda a missao cabe num circulo de 1 km de raio, logo a aproximacao e' folgada.
"""

import math

# Semi-eixo maior do WGS84, em metros. O mesmo valor que `tools/extract_kml.py`
# e `tools/build_mission.py` usam, para que a missao gerada aqui coincida com os
# ficheiros .waypoints ja' existentes ate' ao centimetro.
R_EARTH = 6378137.0

# ---------------------------------------------------------------------------
# Area UAS TO/LD Area 11 TF -- o circulo onde a missao toda tem de caber
# ---------------------------------------------------------------------------
UAS_CENTER = (39.4054608, -8.2937285)
UAS_RADIUS_M = 997.25
UAS_CEIL_AMSL_M = 3000.0

# ---------------------------------------------------------------------------
# Pista. O KML so' traz um placemark de ponto, sem eixo: o QFU real e' pendencia.
# ---------------------------------------------------------------------------
RUNWAY = (39.3993570, -8.2864182)
RUNWAY_TO_CENTER_M = 925.8
RUNWAY_TO_CENTER_BRG_DEG = 137.2

# DEFAULT_APPROACH_BRG_DEG e' calculado mais abaixo, a seguir a `bearing_deg`.

# ---------------------------------------------------------------------------
# Espaco aereo em redor (metros AMSL)
# ---------------------------------------------------------------------------
LP_D25_CEIL_AMSL_M = 4267.0      # 14 000 ft -- espaco segregado
MED_HI_BOUNDARY_AMSL_M = 1371.0  # 4 500 ft -- acima disto e' camada "Hi"
LO_MED_BOUNDARY_AMSL_M = 610.0   # 2 000 ft
A98_CERT_BOX_CEIL_AMSL_M = 1000.0

# ---------------------------------------------------------------------------
# PENDENCIA: elevacao do terreno na pista. NAO INVENTAR.
# Sem este numero nao se fecha nenhuma conta AMSL (teto da area, FENCE_ALT_MAX,
# altitude absoluta do HOME no ficheiro de missao).
# ---------------------------------------------------------------------------
TERRAIN_ELEV_AMSL_M = None

# Altitude relativa a que o piloto passa para AUTO depois da descolagem manual.
AUTO_ENTRY_ALT_M = 100.0


def local_proj(lat0, lon0):
    """Devolve (fwd, inv): projecao plana local centrada em (lat0, lon0).

    `fwd(lat, lon) -> (este, norte)` em metros.
    `inv(este, norte) -> (lat, lon)` em graus.
    """
    coslat = math.cos(math.radians(lat0))

    def fwd(lat, lon):
        return (math.radians(lon - lon0) * R_EARTH * coslat,
                math.radians(lat - lat0) * R_EARTH)

    def inv(e, n):
        return (lat0 + math.degrees(n / R_EARTH),
                lon0 + math.degrees(e / (R_EARTH * coslat)))

    return fwd, inv


def distance_m(a, b):
    """Distancia horizontal em metros entre dois pares (lat, lon)."""
    fwd, _ = local_proj(a[0], a[1])
    e, n = fwd(b[0], b[1])
    return math.hypot(e, n)


def bearing_deg(a, b):
    """Azimute verdadeiro de `a` para `b`, em graus [0, 360)."""
    la1, lo1 = math.radians(a[0]), math.radians(a[1])
    la2, lo2 = math.radians(b[0]), math.radians(b[1])
    y = math.sin(lo2 - lo1) * math.cos(la2)
    x = (math.cos(la1) * math.sin(la2)
         - math.sin(la1) * math.cos(la2) * math.cos(lo2 - lo1))
    return math.degrees(math.atan2(y, x)) % 360.0


# Rumo de aproximacao por omissao: o aviao entra na final vindo do CENTRO do
# circulo em direcao a' pista (137,2 deg). NAO e' o QFU real da pista -- o KML
# so' traz um ponto, sem eixo. E' a escolha geometrica que mantem toda a
# aproximacao dentro da area UAS, porque a pista esta' a apenas 71,5 m da borda.
# Calculado, nao arredondado, para a missao gerada bater com os .waypoints.
DEFAULT_APPROACH_BRG_DEG = bearing_deg(UAS_CENTER, RUNWAY)


def offset_latlon(origin, bearing, dist_m):
    """Ponto a `dist_m` metros de `origin` no azimute `bearing` (graus)."""
    _, inv = local_proj(origin[0], origin[1])
    br = math.radians(bearing)
    return inv(math.sin(br) * dist_m, math.cos(br) * dist_m)


def inside_uas_circle(point, margin_m=0.0):
    """True se `point` estiver dentro do circulo da area UAS.

    `margin_m` encolhe o circulo: util para exigir folga, nao so' contencao.
    """
    return distance_m(UAS_CENTER, point) <= (UAS_RADIUS_M - margin_m)


def uas_margin_m(point):
    """Folga em metros entre `point` e a borda do circulo UAS.

    Negativo significa que o ponto esta' fora da area.
    """
    return UAS_RADIUS_M - distance_m(UAS_CENTER, point)


def inscribed_polygon(center, radius_m, vertices):
    """Vertices de um poligono regular inscrito num circulo, do norte no sentido horario."""
    return [offset_latlon(center, 360.0 * k / vertices, radius_m)
            for k in range(vertices)]


def amsl(rel_alt_m, terrain_elev_m=None):
    """Converte altitude relativa ao home em altitude AMSL.

    Levanta `ValueError` se a elevacao do terreno nao for conhecida -- e' de
    proposito: a elevacao da pista e' uma pendencia aberta e inventa'-la
    falsificaria a verificacao de tetos de espaco aereo.
    """
    elev = TERRAIN_ELEV_AMSL_M if terrain_elev_m is None else terrain_elev_m
    if elev is None:
        raise ValueError(
            'Elevacao do terreno na pista desconhecida (PENDENCIA). '
            'Passe terrain_elev_m=<AMSL da pista> ou preencha '
            'terrain_elev_amsl_m no config/artex27.yaml.')
    return rel_alt_m + float(elev)


def airspace_check(top_rel_alt_m, terrain_elev_m=None):
    """Verifica o topo da missao contra os tetos de espaco aereo conhecidos.

    Devolve uma lista de (nivel, mensagem) com nivel em {'ok', 'aviso', 'erro'}.
    """
    out = []
    try:
        top = amsl(top_rel_alt_m, terrain_elev_m)
    except ValueError as exc:
        out.append(('aviso', str(exc)))
        return out

    out.append(('ok', f'Topo da missao = {top:.0f} m AMSL '
                      f'({top_rel_alt_m:.0f} m acima da pista).'))

    if top > UAS_CEIL_AMSL_M:
        out.append(('erro', f'Acima do teto da area UAS '
                            f'({UAS_CEIL_AMSL_M:.0f} m AMSL).'))
    else:
        out.append(('ok', f'Abaixo do teto da area UAS '
                          f'(folga {UAS_CEIL_AMSL_M - top:.0f} m).'))

    if top > LP_D25_CEIL_AMSL_M:
        out.append(('erro', f'Acima do teto do LP-D25 '
                            f'({LP_D25_CEIL_AMSL_M:.0f} m AMSL).'))

    if top > MED_HI_BOUNDARY_AMSL_M:
        out.append(('aviso', f'Acima da Med-Hi Boundary '
                             f'({MED_HI_BOUNDARY_AMSL_M:.0f} m AMSL): camada "Hi", '
                             f'exige coordenacao com o Air Ops Control.'))
    return out
