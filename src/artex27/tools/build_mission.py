#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build_mission.py - gera os arquivos de missao do ARTEX 2000 m.

Produz, em QGC WPL 110 (.waypoints) e em JSON nativo do QGroundControl (.plan):

  artex27.waypoints / .plan              pouso automatico no fim da missao
  artex27_manual_land.waypoints / .plan  termina em LOITER_UNLIM (piloto assume)
  artex27_geofence.waypoints                   fence poligonal legada (lat/lon)
  artex27_geofence.plan                        fence nativa do QGC (circulo + poligono)

Toda a geometria vem do KML (ver extract_kml.py). Nada aqui e chutado:
a elevacao do terreno na pista NAO e conhecida e por isso TODAS as altitudes
dos itens de missao estao em frame 3 (MAV_FRAME_GLOBAL_RELATIVE_ALT), relativas
ao HOME. A unica altitude absoluta do arquivo e a da linha 0 (HOME), que fica
com o placeholder HOME_AMSL_PLACEHOLDER.

Rode:  ./build_mission.py            (usa os defaults abaixo)
       ./build_mission.py --help     (para ver o que da pra mudar)
"""

import argparse
import json
import math
import os
import sys

from extract_kml import offset_latlon, distance_m, bearing_deg  # noqa: E402

# --------------------------------------------------------------------------
# Geografia (extraida de "ARTEX 26 Geo v173.kml" com extract_kml.py)
# --------------------------------------------------------------------------
UAS_CENTER = (39.4054608, -8.2937285)   # ajuste de circulo, residuo 0.017 m
UAS_RADIUS = 997.25                      # m
UAS_CEIL_AMSL = 3000.0                   # m AMSL
RUNWAY = (39.3993570, -8.2864182)        # placemark "Runway" (ponto, sem eixo)

# Elevacao do terreno na pista: PENDENCIA. Nao inventar.
HOME_AMSL_PLACEHOLDER = 0.0

# --------------------------------------------------------------------------
# Aeronave (Param ACC26, confirmado no BRIEF)
# --------------------------------------------------------------------------
AIRSPEED_CRUISE = 15.0                   # m/s
GRADIENT_LIMIT = 0.10                    # 10 %
CLMB_FACTOR = 0.08                       # TECS_CLMB_MAX = 0.08 * (V - W), 20 % de margem

# --------------------------------------------------------------------------
# Comandos MAVLink
# --------------------------------------------------------------------------
CMD_WAYPOINT = 16
CMD_LOITER_UNLIM = 17
CMD_LAND = 21
CMD_LOITER_TO_ALT = 31
CMD_DO_RETURN_PATH_START = 188
CMD_DO_LAND_START = 189

FRAME_GLOBAL = 0        # altitude absoluta AMSL  (so o HOME)
FRAME_REL = 3           # MAV_FRAME_GLOBAL_RELATIVE_ALT


# --------------------------------------------------------------------------
# Perfil vertical
# --------------------------------------------------------------------------
AUTO_ENTRY_ALT = 100.0   # altitude relativa em que o piloto passa para AUTO

# 9 LOITER_TO_ALT de subida. Ganhos: 7 x 200 m + 2 x 250 m = 1900 m.
CLIMB_STEPS = [300, 500, 700, 900, 1100, 1300, 1500, 1750, 2000]

# 4 LOITER_TO_ALT de descida no caminho de retorno.
DESCENT_STEPS = [1500, 1000, 500, 140]

LOITER_RADIUS = 400.0    # m, comandado em param2 do LOITER_TO_ALT

# Aproximacao final (gradiente < 9 % em ambos os trechos)
APP_GATE_DIST = 1600.0   # m antes do ponto de toque
APP_GATE_ALT = 140.0     # igual a altitude do ultimo LOITER_TO_ALT: perna de ligacao nivelada
APP_FAF_DIST = 700.0
APP_FAF_ALT = 60.0       # rampas: 80/900 = 8.9 % e 60/700 = 8.6 %

# Variante manual: LOITER_UNLIM deslocado para dentro do circulo UAS
MANUAL_LOITER_DIST = 300.0   # m da pista, no eixo de aproximacao
MANUAL_LOITER_ALT = 150.0
MANUAL_LOITER_RADIUS = 200.0

# Placeholder da missao principal a 2000 m
PLACEHOLDER_DIST = 500.0     # m do centro do circulo
PLACEHOLDER_BRGS = (47.0, 227.0)
PLACEHOLDER_ALT = 2000.0

# Fence
FENCE_INSET = 20.0           # m para dentro da borda do circulo UAS
FENCE_VERTICES = 36


# --------------------------------------------------------------------------
class Item(object):
    """Um item de missao."""

    __slots__ = ("cmd", "frame", "p1", "p2", "p3", "p4", "lat", "lon", "alt",
                 "autocont", "label")

    def __init__(self, cmd, frame=FRAME_REL, p1=0.0, p2=0.0, p3=0.0, p4=0.0,
                 lat=0.0, lon=0.0, alt=0.0, autocont=1, label=""):
        self.cmd = cmd
        self.frame = frame
        self.p1, self.p2, self.p3, self.p4 = float(p1), float(p2), float(p3), float(p4)
        self.lat, self.lon, self.alt = float(lat), float(lon), float(alt)
        self.autocont = autocont
        self.label = label


def wpl_line(idx, it, current=0):
    return "%d\t%d\t%d\t%d\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.6f\t%d" % (
        idx, current, it.frame, it.cmd,
        it.p1, it.p2, it.p3, it.p4,
        it.lat, it.lon, it.alt, it.autocont)


def write_wpl(path, items):
    lines = ["QGC WPL 110"]
    for i, it in enumerate(items):
        lines.append(wpl_line(i, it, current=1 if i == 0 else 0))
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")


def write_plan(path, items):
    """items[0] e o HOME; o resto vira mission.items."""
    home = items[0]
    mission_items = []
    for n, it in enumerate(items[1:], start=1):
        obj = {
            "autoContinue": bool(it.autocont),
            "command": it.cmd,
            "doJumpId": n,
            "frame": it.frame,
            "params": [it.p1, it.p2, it.p3, it.p4, it.lat, it.lon, it.alt],
            "type": "SimpleItem",
        }
        if it.cmd in (CMD_WAYPOINT, CMD_LOITER_UNLIM, CMD_LOITER_TO_ALT, CMD_LAND):
            obj["Altitude"] = it.alt
            obj["AltitudeMode"] = 1       # 1 = relativo ao home
            obj["AMSLAltAboveTerrain"] = None
        mission_items.append(obj)

    plan = {
        "fileType": "Plan",
        "geoFence": fence_geofence_block(),
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": AIRSPEED_CRUISE,
            "firmwareType": 3,            # ArduPilot
            "hoverSpeed": 5,
            "items": mission_items,
            "plannedHomePosition": [home.lat, home.lon, home.alt],
            "vehicleType": 1,             # 1 = fixed wing
            "version": 2,
        },
        "rallyPoints": {"points": [], "version": 2},
        "version": 1,
    }
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(plan, fh, indent=4)
        fh.write("\n")


# --------------------------------------------------------------------------
def fence_polygon():
    """Poligono de 36 vertices inscrito no circulo UAS, com recuo de seguranca."""
    r = UAS_RADIUS - FENCE_INSET
    return [offset_latlon(UAS_CENTER, 360.0 * k / FENCE_VERTICES, r)
            for k in range(FENCE_VERTICES)]


def fence_geofence_block():
    return {
        "circles": [{
            "circle": {"center": [UAS_CENTER[0], UAS_CENTER[1]],
                       "radius": round(UAS_RADIUS - FENCE_INSET, 1)},
            "inclusion": True,
            "version": 1,
        }],
        # So o circulo. Um poligono inscrito quase identico seria redundante e,
        # com FENCE_OPTIONS na opcao "intersection" (default), apertaria o
        # limite sem ganho nenhum. O poligono vai so no arquivo legado.
        "polygons": [],
        "version": 2,
    }


def write_fence_legacy(path):
    """Formato legado FENCE_POINT: 1a linha = return point, depois os vertices,
    repetindo o primeiro no fim. Lido por MAVProxy (`fence load`) e pelo
    Mission Planner."""
    poly = fence_polygon()
    pts = [UAS_CENTER] + poly + [poly[0]]
    with open(path, "w", encoding="utf-8") as fh:
        for la, lo in pts:
            fh.write("%.8f\t%.8f\n" % (la, lo))


def write_fence_plan(path):
    plan = {
        "fileType": "Plan",
        "geoFence": fence_geofence_block(),
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": AIRSPEED_CRUISE,
            "firmwareType": 3,
            "hoverSpeed": 5,
            "items": [],
            "plannedHomePosition": [RUNWAY[0], RUNWAY[1], HOME_AMSL_PLACEHOLDER],
            "vehicleType": 1,
            "version": 2,
        },
        "rallyPoints": {"points": [], "version": 2},
        "version": 1,
    }
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(plan, fh, indent=4)
        fh.write("\n")


# --------------------------------------------------------------------------
def build(manual_land, approach_brg, return_loiter=False, avoid=None):
    """Monta a lista de itens. approach_brg = rumo de aproximacao (graus),
    isto e, a direcao em que o aviao voa na final.

    avoid: pontos (lat, lon, alt) inseridos no caminho de regresso, entre a
    espiral de descida e o DO_LAND_START, para contornar uma zona proibida.
    O ArduPlane NAO tem planeador de desvio (BendyRuler/Dijkstra so' existe em
    Copter e Rover) e a geofence e' uma restricao, nao um encaminhador: quem
    evita a zona e' a rota.

    return_loiter: termina o caminho de regresso em LOITER_UNLIM em vez de
    aterrar. O DO_LAND_START CONTINUA a ser preciso -- distance_to_mission_leg()
    (AP_Mission.cpp:2792) varre para a frente a partir do DO_RETURN_PATH_START e
    so' devolve sucesso se encontrar um DO_LAND_START ou comando de aterragem.
    AVISO: num failsafe de BATERIA isto circula ate' a bateria acabar."""
    back = (approach_brg + 180.0) % 360.0   # do ponto de toque para tras

    items = []

    # 0 - HOME (frame 0, altitude ABSOLUTA; placeholder ate medir o terreno)
    items.append(Item(CMD_WAYPOINT, frame=FRAME_GLOBAL,
                      lat=RUNWAY[0], lon=RUNWAY[1], alt=HOME_AMSL_PLACEHOLDER,
                      label="HOME (pista) - alt AMSL = PLACEHOLDER"))

    # 1..9 - espiral de subida: LOITER_TO_ALT empilhados no centro do circulo UAS
    #   param1 = ignorado pelo ArduPlane (ver README)
    #   param2 = raio em m, sinal negativo = sentido anti-horario
    #   param3 = nao usado
    #   param4 = 0 -> crosstrack a partir do centro; 1 -> a partir da tangente de saida
    for alt in CLIMB_STEPS:
        items.append(Item(CMD_LOITER_TO_ALT, p1=0, p2=LOITER_RADIUS, p3=0, p4=0,
                          lat=UAS_CENTER[0], lon=UAS_CENTER[1], alt=alt,
                          label="SUBIDA  LOITER_TO_ALT %d m" % alt))

    # 10..11 - PLACEHOLDER da missao principal a 2000 m
    for brg in PLACEHOLDER_BRGS:
        la, lo = offset_latlon(UAS_CENTER, brg, PLACEHOLDER_DIST)
        items.append(Item(CMD_WAYPOINT, p1=0, p2=0, p3=0, p4=0,
                          lat=la, lon=lo, alt=PLACEHOLDER_ALT,
                          label="*** PLACEHOLDER missao principal (2000 m) ***"))

    if manual_land:
        # Fim nominal: desce em espiral ate a altitude de circuito e fica em
        # LOITER_UNLIM a espera do piloto. A descida TEM de estar aqui: nao da
        # para ir de 2000 m direto para o LOITER_UNLIM.
        for alt in DESCENT_STEPS[:-1] + [MANUAL_LOITER_ALT]:
            items.append(Item(CMD_LOITER_TO_ALT, p1=0, p2=LOITER_RADIUS, p3=0, p4=0,
                              lat=UAS_CENTER[0], lon=UAS_CENTER[1], alt=alt,
                              label="DESCIDA NOMINAL LOITER_TO_ALT %d m" % alt))
        # param3 = raio (sinal = sentido).
        la, lo = offset_latlon(RUNWAY, back, MANUAL_LOITER_DIST)
        items.append(Item(CMD_LOITER_UNLIM, p1=0, p2=0,
                          p3=MANUAL_LOITER_RADIUS, p4=0,
                          lat=la, lon=lo, alt=MANUAL_LOITER_ALT,
                          label="FIM NOMINAL  LOITER_UNLIM (piloto assume)"))

    # DO_RETURN_PATH_START - inicio do caminho de retorno predefinido.
    # Sem lat/lon (0,0) para nao entrar no calculo de distancia das pernas.
    items.append(Item(CMD_DO_RETURN_PATH_START, p1=0, p2=0, p3=0, p4=0,
                      lat=0.0, lon=0.0, alt=0.0,
                      label="DO_RETURN_PATH_START"))

    # Espiral de descida, mesmo centro. As pernas entre estes itens sao
    # puramente verticais, o que faz o jump_to_closest_mission_leg reengatar
    # na faixa de altitude correta.
    for i, alt in enumerate(DESCENT_STEPS):
        last = (i == len(DESCENT_STEPS) - 1)
        items.append(Item(CMD_LOITER_TO_ALT, p1=0, p2=LOITER_RADIUS, p3=0,
                          p4=1 if last else 0,
                          lat=UAS_CENTER[0], lon=UAS_CENTER[1], alt=alt,
                          label="DESCIDA LOITER_TO_ALT %d m%s"
                                % (alt, "  (p4=1: sai pela tangente)" if last else "")))

    # Contorno de zona proibida (entra nas pernas de regresso)
    for i, (la, lo, al) in enumerate(avoid or [], start=1):
        items.append(Item(CMD_WAYPOINT, lat=float(la), lon=float(lo), alt=float(al),
                          label="CONTORNO %d - desvio de zona proibida" % i))

    # DO_LAND_START
    items.append(Item(CMD_DO_LAND_START, p1=0, p2=0, p3=0, p4=0,
                      lat=0.0, lon=0.0, alt=0.0, label="DO_LAND_START"))

    if return_loiter:
        la, lo = offset_latlon(RUNWAY, back, MANUAL_LOITER_DIST)
        items.append(Item(CMD_LOITER_UNLIM, p1=0, p2=0,
                          p3=MANUAL_LOITER_RADIUS, p4=0,
                          lat=la, lon=lo, alt=MANUAL_LOITER_ALT,
                          label="ESPERA  LOITER_UNLIM (piloto assume; NAO aterra)"))
        return items

    # Sequencia de aterragem, alinhada ao eixo de aproximacao
    la, lo = offset_latlon(RUNWAY, back, APP_GATE_DIST)
    items.append(Item(CMD_WAYPOINT, lat=la, lon=lo, alt=APP_GATE_ALT,
                      label="APROX 1 - porta de entrada da final"))
    la, lo = offset_latlon(RUNWAY, back, APP_FAF_DIST)
    items.append(Item(CMD_WAYPOINT, lat=la, lon=lo, alt=APP_FAF_ALT,
                      label="APROX 2 - FAF (fix de aproximacao final)"))
    items.append(Item(CMD_LAND, p1=0, p2=0, p3=0, p4=0,
                      lat=RUNWAY[0], lon=RUNWAY[1], alt=0.0,
                      label="LAND (= HOME)"))

    return items


# --------------------------------------------------------------------------
def spiral_numbers(wind):
    """Numeros da espiral, para o README e para o log."""
    v = AIRSPEED_CRUISE
    clmb = CLMB_FACTOR * (v - wind)
    gs = v - wind                       # pior caso: tudo contra o vento
    climb_total = CLIMB_STEPS[-1] - AUTO_ENTRY_ALT
    t = climb_total / clmb
    air_dist = v * t
    lap_len = 2.0 * math.pi * LOITER_RADIUS
    laps = air_dist / lap_len
    lap_time = lap_len / v
    return {
        "wind": wind,
        "tecs_clmb_max": clmb,
        "gradient_pct": 100.0 * clmb / gs,
        "climb_total_m": climb_total,
        "climb_time_s": t,
        "air_distance_m": air_dist,
        "lap_len_m": lap_len,
        "laps": laps,
        "lap_time_s": lap_time,
        "alt_per_lap_m": clmb * lap_time,
    }


def main(argv=None):
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--outdir", default=here)
    ap.add_argument("--approach-brg", type=float, default=None,
                    help="rumo de aproximacao em graus. Default: o azimute "
                         "centro-do-circulo -> pista (mantem toda a final "
                         "dentro da area UAS). SUBSTITUIR pelo QFU real.")
    ap.add_argument("--wind", type=float, default=0.0,
                    help="vento de frente em m/s para os numeros da espiral")
    ap.add_argument("--avoid", metavar="LAT,LON,ALT", action="append", default=[],
                    help="ponto de contorno de zona proibida no caminho de "
                         "regresso; repetir para varios, pela ordem de passagem")
    args = ap.parse_args(argv)

    avoid = []
    for spec in args.avoid:
        parts = spec.split(",")
        if len(parts) != 3:
            ap.error("--avoid espera LAT,LON,ALT (recebido: %r)" % spec)
        avoid.append(tuple(float(x) for x in parts))

    brg = args.approach_brg
    if brg is None:
        brg = bearing_deg(UAS_CENTER, RUNWAY)

    os.makedirs(args.outdir, exist_ok=True)
    out = lambda n: os.path.join(args.outdir, n)   # noqa: E731

    auto = build(manual_land=False, approach_brg=brg, avoid=avoid)
    man = build(manual_land=True, approach_brg=brg, avoid=avoid)
    fsl = build(manual_land=False, approach_brg=brg, avoid=avoid, return_loiter=True)

    write_wpl(out("artex27.waypoints"), auto)
    write_plan(out("artex27.plan"), auto)
    write_wpl(out("artex27_manual_land.waypoints"), man)
    write_plan(out("artex27_manual_land.plan"), man)
    write_wpl(out("artex27_fs_loiter.waypoints"), fsl)
    write_plan(out("artex27_fs_loiter.plan"), fsl)
    write_fence_legacy(out("artex27_geofence.waypoints"))
    write_fence_plan(out("artex27_geofence.plan"))

    print("eixo de aproximacao: %.1f deg (final), reciproco %.1f deg"
          % (brg, (brg + 180) % 360))
    print("pista a %.1f m do centro do circulo UAS (raio %.1f m) -> folga %.1f m"
          % (distance_m(UAS_CENTER, RUNWAY), UAS_RADIUS,
             UAS_RADIUS - distance_m(UAS_CENTER, RUNWAY)))
    print()
    n = spiral_numbers(args.wind)
    print("ESPIRAL DE SUBIDA (vento de frente = %.1f m/s)" % n["wind"])
    print("  raio comandado        : %.0f m" % LOITER_RADIUS)
    print("  TECS_CLMB_MAX         : %.2f m/s" % n["tecs_clmb_max"])
    print("  gradiente resultante  : %.1f %%" % n["gradient_pct"])
    print("  ganho total           : %.0f m (%.0f -> %.0f)"
          % (n["climb_total_m"], AUTO_ENTRY_ALT, CLIMB_STEPS[-1]))
    print("  tempo de subida       : %.0f s = %.1f min" % (n["climb_time_s"], n["climb_time_s"] / 60))
    print("  distancia percorrida  : %.0f m = %.1f km" % (n["air_distance_m"], n["air_distance_m"] / 1000))
    print("  comprimento da volta  : %.0f m   tempo/volta %.0f s" % (n["lap_len_m"], n["lap_time_s"]))
    print("  numero de voltas      : %.1f" % n["laps"])
    print("  ganho por volta       : %.0f m" % n["alt_per_lap_m"])
    print("  LOITER_TO_ALT         : %d itens, ganhos %s"
          % (len(CLIMB_STEPS),
             [CLIMB_STEPS[0] - int(AUTO_ENTRY_ALT)]
             + [CLIMB_STEPS[i + 1] - CLIMB_STEPS[i] for i in range(len(CLIMB_STEPS) - 1)]))
    print()
    for name in ("artex27.waypoints", "artex27.plan",
                 "artex27_manual_land.waypoints", "artex27_manual_land.plan",
                 "artex27_geofence.waypoints", "artex27_geofence.plan"):
        print("  escrito: %s" % name)
    print()
    print("ANOTACAO DOS ITENS (artex27.waypoints)")
    for i, it in enumerate(auto):
        print("  %2d  cmd %-3d frame %d alt %7.1f  %s" % (i, it.cmd, it.frame, it.alt, it.label))
    return 0


if __name__ == "__main__":
    sys.exit(main())
