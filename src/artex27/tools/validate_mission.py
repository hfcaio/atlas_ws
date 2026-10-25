#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
validate_mission.py - valida um arquivo de missao QGC WPL 110 do ARTEX 2000 m.

O que e verificado
------------------
 1. FORMATO    cabecalho "QGC WPL 110", 12 campos separados por TAB, indices
               em sequencia.
 2. FRAMES     item 0 (HOME) em frame 0 (AMSL absoluto); todos os outros em
               frame 3 (MAV_FRAME_GLOBAL_RELATIVE_ALT).
 3. SEQUENCIA  DO_RETURN_PATH_START (188) antes de DO_LAND_START (189);
               LAND (21) como ultimo item; um WAYPOINT imediatamente antes do
               LAND.
 4. GRADIENTE  entre itens consecutivos. Subida: <= --max-grad (default 10 %,
               a regra do ARTEX). Descida: <= --max-grad-descent (default 25 %,
               so teste de sanidade de afundamento).
               Para LOITER_TO_ALT (31) o gradiente NAO e geometrico: e
               TECS_CLMB_MAX / groundspeed (subida) ou TECS_SINK_MAX / groundspeed
               (descida). O script calcula voltas, tempo e distancia percorrida
               na espiral. Subida e descida tem limites separados
               (--max-grad / --max-grad-descent).
 5. CONTENCAO  todo item (mais o raio de loiter corrigido pela altitude) tem de
               caber dentro do circulo da area UAS TO/LD.
 6. TETO       altitude AMSL maxima vs 3000 m (area UAS) e 4267 m (LP-D25),
               parametrizada por --home-amsl.

Exemplos
--------
  ./validate_mission.py artex27.waypoints
  ./validate_mission.py artex27.waypoints --wind 5
  ./validate_mission.py artex27.waypoints --home-amsl 120
  ./validate_mission.py *.waypoints --wind 5 --airspeed 15

Exit code 0 = tudo passou (avisos nao reprovam), != 0 = pelo menos um ERRO.
"""

import argparse
import math
import os
import sys

# ------------------------------------------------------------------ geografia
UAS_CENTER = (39.4054608, -8.2937285)
UAS_RADIUS = 997.25
UAS_CEIL_AMSL = 3000.0
LPD25_CEIL_AMSL = 4267.0
MEDHI_BOUNDARY_AMSL = 1371.0

R_EARTH = 6378137.0

CMD_NAMES = {
    16: "WAYPOINT", 17: "LOITER_UNLIM", 18: "LOITER_TURNS", 19: "LOITER_TIME",
    20: "RTL", 21: "LAND", 22: "TAKEOFF", 31: "LOITER_TO_ALT",
    112: "COND_DELAY", 177: "DO_JUMP", 178: "DO_CHANGE_SPEED",
    188: "DO_RETURN_PATH_START", 189: "DO_LAND_START",
}
# comandos que carregam posicao util para navegacao
NAV_CMDS = {16, 17, 18, 19, 21, 22, 31}
LOITER_CMDS = {17, 18, 19, 31}


def dist_m(a, b):
    la0 = math.radians((a[0] + b[0]) / 2.0)
    dx = math.radians(b[1] - a[1]) * R_EARTH * math.cos(la0)
    dy = math.radians(b[0] - a[0]) * R_EARTH
    return math.hypot(dx, dy)


def eas2tas(alt_amsl_m):
    """Razao TAS/EAS pela atmosfera padrao (troposfera)."""
    h = max(0.0, alt_amsl_m)
    sigma = (1.0 - 2.25577e-5 * h) ** 4.2559
    return 1.0 / math.sqrt(max(sigma, 1e-6))


# ------------------------------------------------------------------ parsing
class Wp(object):
    def __init__(self, idx, cur, frame, cmd, p, lat, lon, alt, autocont, line):
        self.idx, self.cur, self.frame, self.cmd = idx, cur, frame, cmd
        self.p1, self.p2, self.p3, self.p4 = p
        self.lat, self.lon, self.alt = lat, lon, alt
        self.autocont, self.line = autocont, line

    @property
    def name(self):
        return CMD_NAMES.get(self.cmd, "cmd%d" % self.cmd)

    @property
    def has_pos(self):
        return self.cmd in NAV_CMDS and not (self.lat == 0.0 and self.lon == 0.0)

    @property
    def pos(self):
        return (self.lat, self.lon)

    def loiter_radius(self):
        """Raio comandado em metros, 0 se nao for loiter ou se herdar WP_LOITER_RAD."""
        if self.cmd == 31:
            return abs(self.p2)
        if self.cmd in (17, 18):
            return abs(self.p3)
        return 0.0


class Report(object):
    def __init__(self):
        self.errors = 0
        self.warns = 0

    def ok(self, msg):
        print("  [ OK ] %s" % msg)

    def warn(self, msg):
        self.warns += 1
        print("  [AVISO] %s" % msg)

    def err(self, msg):
        self.errors += 1
        print("  [ERRO] %s" % msg)

    def info(self, msg):
        print("         %s" % msg)


def parse(path, rep):
    with open(path, "r", encoding="utf-8") as fh:
        raw = fh.read().splitlines()
    if not raw:
        rep.err("arquivo vazio")
        return []
    if raw[0].strip() != "QGC WPL 110":
        rep.err("cabecalho invalido: %r (esperado 'QGC WPL 110')" % raw[0])
        return []
    wps = []
    for ln, line in enumerate(raw[1:], start=2):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if "\t" not in line:
            rep.err("linha %d nao usa TAB como separador" % ln)
            continue
        f = line.split("\t")
        if len(f) != 12:
            rep.err("linha %d tem %d campos (esperado 12)" % (ln, len(f)))
            continue
        try:
            wps.append(Wp(int(f[0]), int(f[1]), int(f[2]), int(f[3]),
                          [float(x) for x in f[4:8]],
                          float(f[8]), float(f[9]), float(f[10]), int(f[11]), ln))
        except ValueError as e:
            rep.err("linha %d nao converte: %s" % (ln, e))
    for i, w in enumerate(wps):
        if w.idx != i:
            rep.err("indice fora de sequencia na linha %d: %d (esperado %d)"
                    % (w.line, w.idx, i))
    return wps


# ------------------------------------------------------------------ checks
def check_frames(wps, rep):
    print("\n-- FRAMES")
    if not wps:
        return
    if wps[0].frame != 0:
        rep.err("item 0 (HOME) deve estar em frame 0 (AMSL absoluto), esta em %d"
                % wps[0].frame)
    else:
        rep.ok("item 0 (HOME) em frame 0, alt %.1f m AMSL" % wps[0].alt)
    bad = [w for w in wps[1:] if w.frame != 3]
    if bad:
        for w in bad:
            rep.err("item %d (%s) em frame %d; esperado 3 (GLOBAL_RELATIVE_ALT)"
                    % (w.idx, w.name, w.frame))
    else:
        rep.ok("itens 1..%d todos em frame 3 (GLOBAL_RELATIVE_ALT)" % wps[-1].idx)


def check_sequence(wps, rep):
    print("\n-- SEQUENCIA")
    idx_rp = [w.idx for w in wps if w.cmd == 188]
    idx_ls = [w.idx for w in wps if w.cmd == 189]
    idx_land = [w.idx for w in wps if w.cmd == 21]

    if not idx_rp:
        rep.err("falta DO_RETURN_PATH_START (188)")
    elif len(idx_rp) > 1:
        rep.warn("ha %d DO_RETURN_PATH_START; o firmware escolhe o mais proximo" % len(idx_rp))
    if not idx_ls:
        rep.err("falta DO_LAND_START (189)")
    if not idx_land:
        rep.err("falta LAND (21)")

    if idx_rp and idx_ls:
        if max(idx_rp) < min(idx_ls):
            rep.ok("DO_RETURN_PATH_START (idx %s) antes de DO_LAND_START (idx %s)"
                   % (idx_rp, idx_ls))
        else:
            rep.err("DO_RETURN_PATH_START tem de vir ANTES de DO_LAND_START "
                    "(AP_Mission::distance_to_mission_leg para a busca no "
                    "DO_LAND_START ou num comando de pouso)")
        between = [w for w in wps if idx_rp[0] < w.idx < idx_ls[0] and w.has_pos]
        if not between:
            rep.err("nao ha nenhum item com posicao entre DO_RETURN_PATH_START e "
                    "DO_LAND_START: o caminho de retorno fica vazio")
        else:
            rep.ok("caminho de retorno com %d itens posicionados (idx %d..%d)"
                   % (len(between), between[0].idx, between[-1].idx))

    if idx_land:
        if idx_land[-1] != wps[-1].idx:
            rep.err("LAND nao e o ultimo item (LAND em %d, ultimo e %d)"
                    % (idx_land[-1], wps[-1].idx))
        else:
            rep.ok("LAND e o ultimo item (idx %d)" % idx_land[-1])
        prev = [w for w in wps if w.idx < idx_land[-1] and w.has_pos]
        if prev and prev[-1].cmd == 16:
            rep.ok("item imediatamente antes do LAND e um WAYPOINT (idx %d, %.0f m)"
                   % (prev[-1].idx, prev[-1].alt))
        elif prev:
            rep.warn("item posicionado antes do LAND e %s, nao WAYPOINT; o "
                     "ArduPlane espera um WAYPOINT para definir a rampa final"
                     % prev[-1].name)

    n_unlim = [w.idx for w in wps if w.cmd == 17]
    if n_unlim:
        rep.info("LOITER_UNLIM em idx %s -> variante de pouso MANUAL "
                 "(a missao nominal nunca chega sozinha ao DO_RETURN_PATH_START; "
                 "esse trecho so e usado por RTL/failsafe com RTL_AUTOLAND=4)"
                 % n_unlim)


def check_gradient(wps, rep, airspeed, wind, clmb_max, sink_max,
                   max_grad, max_grad_desc):
    print("\n-- GRADIENTE  (airspeed %.1f m/s, vento de frente %.1f m/s, "
          "groundspeed pior caso %.1f m/s)" % (airspeed, wind, airspeed - wind))
    gs = max(airspeed - wind, 0.1)
    print("   TECS_CLMB_MAX assumido %.2f m/s -> %.1f %% | "
          "TECS_SINK_MAX assumido %.2f m/s -> %.1f %%"
          % (clmb_max, 100 * clmb_max / gs, sink_max, 100 * sink_max / gs))

    # A missao nao e uma cadeia unica: o DO_RETURN_PATH_START abre um percurso
    # independente (o aviao entra nele vindo de qualquer lugar) e o LOITER_UNLIM
    # nunca termina, entao nada e voado depois dele em sequencia.
    segments = []
    cur = []
    for w in wps:
        if w.idx == 0:
            continue                       # HOME: ver nota abaixo
        if w.cmd == 188:                   # DO_RETURN_PATH_START
            if len(cur) > 1:
                segments.append(cur)
            cur = []
            continue
        if not w.has_pos:
            continue
        cur.append(w)
        if w.cmd == 17:                    # LOITER_UNLIM: fim do trecho nominal
            segments.append(cur)
            cur = []
    if len(cur) > 1:
        segments.append(cur)

    rep.info("item 0 e o HOME: em AUTO a primeira rampa parte da posicao e "
             "altitude ATUAIS (entrada combinada a ~100 m sobre a pista), "
             "nao do HOME. A perna 0->1 nao e verificada.")
    if len(segments) > 1:
        rep.info("a missao foi partida em %d trechos independentes "
                 "(DO_RETURN_PATH_START / LOITER_UNLIM cortam a cadeia)."
                 % len(segments))
    nav = [w for seg in segments for w in seg]
    if not segments:
        rep.warn("menos de 2 itens posicionados, nada a verificar")
        return

    tol = 1e-4                       # 0.01 ponto percentual
    acc = {"subida": [0.0, 0.0, 0.0], "descida": [0.0, 0.0, 0.0]}  # laps, s, m

    def spiral(tag, a, b, dz, note):
        """Trecho em que a altitude e ganha/perdida dentro de um LOITER_TO_ALT:
        o gradiente nao e geometrico, e TECS_(CLMB|SINK)_MAX / groundspeed."""
        rate = clmb_max if dz > 0 else sink_max
        limit = max_grad if dz > 0 else max_grad_desc
        grad = rate / gs
        radius = b.loiter_radius() or a.loiter_radius()
        r_eff = radius * (eas2tas((a.alt + b.alt) / 2.0) ** 2)   # NAVL1_LIM_BANK = 0
        lap = 2 * math.pi * r_eff
        t = abs(dz) / rate
        d = airspeed * t
        laps = d / lap if lap > 0 else 0.0
        k = "subida" if dz > 0 else "descida"
        acc[k][0] += laps
        acc[k][1] += t
        acc[k][2] += d
        msg = ("%s  ESPIRAL dz %+7.1f m  raio_eff %4.0f m  %4.1f voltas  "
               "%5.0f s  %5.2f km  grad %4.1f %% (lim %.0f %%)%s"
               % (tag, dz, r_eff, laps, t, d / 1000.0, 100 * grad, 100 * limit, note))
        if grad > limit + tol:
            rep.err(msg)
        else:
            rep.ok(msg)

    pairs = [(a, b) for seg in segments for a, b in zip(seg, seg[1:])]
    for a, b in pairs:
        dz = b.alt - a.alt
        d_center = dist_m(a.pos, b.pos)
        ra, rb = a.loiter_radius(), b.loiter_radius()
        tag = "%2d %-14s -> %2d %-14s" % (a.idx, a.name, b.idx, b.name)

        # --- dois loiters no mesmo centro: espiral pura ---
        if a.cmd in LOITER_CMDS and b.cmd in LOITER_CMDS and d_center < 1.0:
            if abs(dz) < 1e-6:
                rep.info("%s  mesmo centro, mesma altitude" % tag)
            else:
                spiral(tag, a, b, dz, "")
            continue

        # --- destino e LOITER_TO_ALT: a altitude e atingida dentro do loiter ---
        if b.cmd == 31 and abs(dz) > 1e-6:
            spiral(tag, a, b, dz,
                   "  [transito de %.0f m ate o loiter]" % max(d_center - ra - rb, 0.0))
            continue

        # --- perna reta (pior caso: sai/entra pela borda do loiter) ---
        d_h = max(d_center - ra - rb, 1.0)
        grad = abs(dz) / d_h
        rate_req = abs(dz) / (d_h / gs)
        limit = max_grad if dz > 0 else max_grad_desc
        msg = ("%s  dz %+7.1f m  dist_min %6.0f m  grad %4.1f %% (lim %.0f %%)  "
               "taxa exigida %.2f m/s" % (tag, dz, d_h, 100 * grad, 100 * limit, rate_req))
        if grad > limit + tol:
            rep.err(msg)
        else:
            rep.ok(msg)
        rate_limit = clmb_max if dz > 0 else sink_max
        if rate_req > rate_limit + 1e-6:
            rep.err("    ^ exige %.2f m/s, acima do limite assumido %.2f m/s"
                    % (rate_req, rate_limit))

    for k in ("subida", "descida"):
        laps, t, d = acc[k]
        if laps:
            rep.info("espiral de %s: %.1f voltas, %.0f s (%.1f min), %.1f km"
                     % (k, laps, t, t / 60.0, d / 1000.0))


def check_containment(wps, rep, margin):
    print("\n-- CONTENCAO na area UAS TO/LD (centro %.7f, %.7f  raio %.1f m, "
          "margem %.0f m)" % (UAS_CENTER[0], UAS_CENTER[1], UAS_RADIUS, margin))
    worst = None
    out = []
    for w in wps:
        if not w.has_pos:
            continue
        d = dist_m(UAS_CENTER, w.pos)
        r = w.loiter_radius()
        r_eff = r * (eas2tas(w.alt) ** 2) if r else 0.0
        reach = d + r_eff
        if worst is None or reach > worst[0]:
            worst = (reach, w, d, r_eff)
        if reach > UAS_RADIUS - margin:
            out.append((w, d, r_eff, reach))
    if out:
        for w, d, r_eff, reach in out:
            rep.err("item %d (%s, %.0f m): centro a %.0f m + raio_eff %.0f m = "
                    "%.0f m > %.0f m" % (w.idx, w.name, w.alt, d, r_eff, reach,
                                         UAS_RADIUS - margin))
    else:
        rep.ok("todos os %d itens posicionados cabem no circulo"
               % len([w for w in wps if w.has_pos]))
    if worst:
        reach, w, d, r_eff = worst
        rep.info("pior caso: item %d (%s) alcanca %.0f m do centro "
                 "(folga %.0f m ate a borda)" % (w.idx, w.name, reach, UAS_RADIUS - reach))


def check_ceiling(wps, rep, home_amsl):
    print("\n-- TETO")
    max_rel = max((w.alt for w in wps[1:]), default=0.0)
    rep.info("altitude relativa maxima na missao: %.0f m" % max_rel)
    if home_amsl is None:
        max_home = UAS_CEIL_AMSL - max_rel
        rep.warn("--home-amsl nao informado: a elevacao do terreno na pista e "
                 "PENDENCIA aberta no BRIEF; o teto nao pode ser fechado.")
        rep.info("para respeitar o teto de %.0f m AMSL da area UAS, a pista tem "
                 "de estar abaixo de %.0f m AMSL." % (UAS_CEIL_AMSL, max_home))
        rep.info("para o teto de %.0f m AMSL do LP-D25, abaixo de %.0f m AMSL."
                 % (LPD25_CEIL_AMSL, LPD25_CEIL_AMSL - max_rel))
        return
    top = home_amsl + max_rel
    rep.info("home %.0f m AMSL + %.0f m = topo da missao %.0f m AMSL"
             % (home_amsl, max_rel, top))
    for name, ceil in (("area UAS TO/LD", UAS_CEIL_AMSL), ("LP-D25", LPD25_CEIL_AMSL)):
        if top > ceil:
            rep.err("topo %.0f m AMSL acima do teto de %s (%.0f m AMSL)"
                    % (top, name, ceil))
        else:
            rep.ok("topo %.0f m AMSL dentro do teto de %s (%.0f m, folga %.0f m)"
                   % (top, name, ceil, ceil - top))
    if top > MEDHI_BOUNDARY_AMSL:
        rep.warn("topo %.0f m AMSL acima da Med-Hi Boundary (%.0f m AMSL): camada "
                 "'Hi', exige coordenacao com o Air Ops Control."
                 % (top, MEDHI_BOUNDARY_AMSL))


# ------------------------------------------------------------------ main
def validate(path, args):
    print("=" * 78)
    print("ARQUIVO: %s" % path)
    print("=" * 78)
    rep = Report()
    print("\n-- FORMATO")
    wps = parse(path, rep)
    if not wps:
        print("\nRESULTADO: %d erro(s)" % rep.errors)
        return rep
    rep.ok("QGC WPL 110, %d itens, 12 campos TAB em todas as linhas" % len(wps))
    for w in wps:
        print("         %2d  %-22s frame %d  alt %8.1f  p1..p4 %g/%g/%g/%g"
              % (w.idx, w.name, w.frame, w.alt, w.p1, w.p2, w.p3, w.p4))

    gs = max(args.airspeed - args.wind, 0.1)
    clmb = args.clmb_max if args.clmb_max is not None else 0.08 * gs
    sink = args.sink_max

    check_frames(wps, rep)
    check_sequence(wps, rep)
    check_gradient(wps, rep, args.airspeed, args.wind, clmb, sink,
                   args.max_grad / 100.0, args.max_grad_descent / 100.0)
    check_containment(wps, rep, args.margin)
    check_ceiling(wps, rep, args.home_amsl)

    print("\nRESULTADO: %d erro(s), %d aviso(s)\n" % (rep.errors, rep.warns))
    return rep


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+", help="arquivos .waypoints")
    ap.add_argument("--airspeed", type=float, default=15.0,
                    help="AIRSPEED_CRUISE em m/s (default 15)")
    ap.add_argument("--wind", type=float, default=0.0,
                    help="vento de frente em m/s (default 0)")
    ap.add_argument("--clmb-max", type=float, default=None,
                    help="TECS_CLMB_MAX em m/s. Default: 0.08*(airspeed-wind)")
    ap.add_argument("--sink-max", type=float, default=2.0,
                    help="TECS_SINK_MAX em m/s (default 2.0)")
    ap.add_argument("--max-grad", type=float, default=10.0,
                    help="gradiente maximo de SUBIDA em %% (default 10, regra ARTEX)")
    ap.add_argument("--max-grad-descent", type=float, default=25.0,
                    help="gradiente maximo de DESCIDA em %% (default 25). NAO e a "
                         "regra dos 10 %% do exercicio (essa vale para a subida): "
                         "e um teste de sanidade de afundamento. 25 %% a 15 m/s = "
                         "3.75 m/s, acima do que o TECS sustenta sem acelerar. "
                         "Use --max-grad-descent 10 para exigir os mesmos 10 %% "
                         "tambem na descida.")
    ap.add_argument("--margin", type=float, default=50.0,
                    help="margem de navegacao ate a borda do circulo UAS, em m "
                         "(default 50)")
    ap.add_argument("--home-amsl", type=float, default=None,
                    help="elevacao do terreno na pista, em m AMSL. SEM DEFAULT: "
                         "e pendencia aberta. Sem este valor o teto fica em aviso.")
    args = ap.parse_args(argv)

    total_err = 0
    for p in args.files:
        if not os.path.exists(p):
            print("ERRO: nao encontrei %s" % p, file=sys.stderr)
            total_err += 1
            continue
        total_err += validate(p, args).errors
    return 1 if total_err else 0


if __name__ == "__main__":
    sys.exit(main())
