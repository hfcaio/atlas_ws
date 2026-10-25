#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_log.py — analisa um log de voo (SITL ou real) para a missao ARTEX 2000 m.

Extrai:
  * perfil de altitude e razao de subida;
  * groundspeed e GRADIENTE REAL SOBRE O SOLO (razao de subida / groundspeed);
  * mudancas de modo de voo;
  * eventos de failsafe (mensagens ERR do dataflash + STATUSTEXT);
  * tempo de subida ate' uma altitude alvo.

PARSER
  Nao escreve parser nenhum: usa o do proprio repositorio —
  `pymavlink.mavutil.mavlink_connection()`, que encaminha
    .bin / .BIN / .log  -> pymavlink.DFReader  (dataflash)
    .tlog               -> leitor de telemetria MAVLink
  O pymavlink usado e' o de src/ardupilot/modules/mavlink/pymavlink (o mesmo que
  alimenta o mavlogdump.py do repo). Para forcar esse, ver --use-repo-pymavlink.

USO
  python3 analyze_log.py voo.BIN
  python3 analyze_log.py voo.BIN --target-alt 2000 --max-gradient 0.10 --csv saida.csv
  python3 analyze_log.py voo.tlog --window 20

CODIGOS DE SAIDA
  0 = analise concluida e (se --max-gradient) gradiente dentro do limite
  1 = gradiente sustentado acima do limite
  2 = erro (ficheiro ilegivel, sem dados)
"""

from __future__ import annotations

import argparse
import math
import os
import sys


def _maybe_use_repo_pymavlink(force: bool) -> None:
    """Poe o pymavlink do submodulo src/ardupilot a' frente no sys.path."""
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.abspath(os.path.join(here, "..", "..",
                                        "src", "ardupilot", "modules", "mavlink"))
    if os.path.isdir(os.path.join(repo, "pymavlink")):
        if force:
            sys.path.insert(0, repo)
        else:
            sys.path.append(repo)
    elif force:
        sys.stderr.write("AVISO: %s nao existe; uso o pymavlink instalado.\n" % repo)


# o parse do argumento tem de acontecer antes do import do pymavlink
_FORCE_REPO = "--use-repo-pymavlink" in sys.argv
_maybe_use_repo_pymavlink(_FORCE_REPO)

try:
    from pymavlink import mavutil
except ImportError:
    sys.stderr.write(
        "ERRO: pymavlink nao encontrado.\n"
        "  Dentro do container atlas-sim ja esta instalado.\n"
        "  Fora:  python3 -m pip install --user pymavlink\n")
    raise SystemExit(2)


# --------------------------------------------------------------------------- #
# Tabelas do ArduPilot (libraries/AP_Logger/AP_Logger.h:114, ArduPlane/mode.h:38)
# --------------------------------------------------------------------------- #

ERR_SUBSYS = {
    1: "MAIN", 2: "RADIO", 3: "COMPASS", 4: "OPTFLOW",
    5: "FAILSAFE_RADIO", 6: "FAILSAFE_BATT", 7: "FAILSAFE_GPS",
    8: "FAILSAFE_GCS", 9: "FAILSAFE_FENCE", 10: "FLIGHT_MODE", 11: "GPS",
    12: "CRASH_CHECK", 13: "FLIP", 14: "AUTOTUNE", 15: "PARACHUTES",
    16: "EKFCHECK", 17: "FAILSAFE_EKFINAV", 18: "BARO", 19: "CPU",
    20: "FAILSAFE_ADSB", 21: "TERRAIN", 22: "NAVIGATION",
    23: "FAILSAFE_TERRAIN", 24: "EKF_PRIMARY", 25: "THRUST_LOSS_CHECK",
    26: "FAILSAFE_SENSORS", 27: "FAILSAFE_LEAK", 28: "PILOT_INPUT",
    29: "FAILSAFE_VIBE", 30: "INTERNAL_ERROR", 31: "FAILSAFE_DEADRECKON",
}
ERR_CODE = {0: "RESOLVED/OFF", 1: "FAILED_TO_INITIALISE", 2: "OCCURRED/ON",
            4: "UNHEALTHY"}

PLANE_MODE_NAME = {
    0: "MANUAL", 1: "CIRCLE", 2: "STABILIZE", 3: "TRAINING", 4: "ACRO",
    5: "FBWA", 6: "FBWB", 7: "CRUISE", 8: "AUTOTUNE", 10: "AUTO", 11: "RTL",
    12: "LOITER", 13: "TAKEOFF", 14: "AVOID_ADSB", 15: "GUIDED",
    16: "INITIALISING", 17: "QSTABILIZE", 18: "QHOVER", 19: "QLOITER",
    20: "QLAND", 21: "QRTL", 22: "QAUTOTUNE", 23: "QACRO", 24: "THERMAL",
    25: "LOITER_ALT_QLAND", 26: "AUTOLAND",
}

FS_KEYWORDS = ("failsafe", "battery", "rtl", "return path", "landing sequence",
               "crash", "terminat", "parachute", "ekf", "glide")


# --------------------------------------------------------------------------- #

class Track:
    """Serie temporal unificada, independente do formato do log."""

    def __init__(self):
        self.t = []          # s desde o inicio do log
        self.alt = []        # m (relativa ao primeiro ponto com a aeronave no chao)
        self.alt_raw = []    # m (como vem do log)
        self.gs = []         # m/s groundspeed
        self.vz = []         # m/s razao de subida (do log ou derivada)
        self.modes = []      # (t, nome)
        self.events = []     # (t, "tipo", "detalhe")


def read_bin(path: str, verbose: bool) -> Track:
    tr = Track()
    mlog = mavutil.mavlink_connection(path, dialect="ardupilotmega")
    t0 = None
    last_mode = None
    while True:
        m = mlog.recv_match(type=["GPS", "POS", "CTUN", "MODE", "ERR", "MSG",
                                  "BARO", "TECS"])
        if m is None:
            break
        typ = m.get_type()
        ts = getattr(m, "TimeUS", None)
        t = (ts * 1e-6) if ts is not None else None
        if t is not None and t0 is None:
            t0 = t
        tt = (t - t0) if (t is not None and t0 is not None) else None

        if typ == "GPS" and tt is not None:
            tr.t.append(tt)
            tr.alt_raw.append(float(getattr(m, "Alt", 0.0)))
            tr.gs.append(float(getattr(m, "Spd", 0.0)))
            tr.vz.append(float(getattr(m, "VZ", 0.0)) * -1.0
                         if hasattr(m, "VZ") else float("nan"))
        elif typ == "MODE" and tt is not None:
            num = int(getattr(m, "ModeNum", getattr(m, "Mode", -1)))
            name = PLANE_MODE_NAME.get(num, "MODE_%d" % num)
            if name != last_mode:
                tr.modes.append((tt, name))
                last_mode = name
        elif typ == "ERR" and tt is not None:
            sub = int(getattr(m, "Subsys", -1))
            cod = int(getattr(m, "ECode", -1))
            tr.events.append((tt, "ERR", "%s / %s"
                              % (ERR_SUBSYS.get(sub, "SUBSYS_%d" % sub),
                                 ERR_CODE.get(cod, "code_%d" % cod))))
        elif typ == "MSG" and tt is not None:
            txt = getattr(m, "Message", "")
            if isinstance(txt, bytes):
                txt = txt.decode("utf-8", "replace")
            if verbose or any(k in txt.lower() for k in FS_KEYWORDS):
                tr.events.append((tt, "MSG", txt))
    return tr


def read_tlog(path: str, verbose: bool) -> Track:
    """Um .tlog traz mensagens de TODOS os sistemas do link — inclusive os
    heartbeats da propria GCS (MAV_TYPE_GCS, custom_mode 0 = MANUAL no Plane).
    Sem filtrar por sysid, a lista de modos fica a saltar MANUAL<->AUTO."""
    tr = Track()
    mlog = mavutil.mavlink_connection(path)
    t0 = None
    last_mode = None
    vehicle_sys = None
    pend = {"gs": None, "vz": None}
    while True:
        m = mlog.recv_match(type=["GLOBAL_POSITION_INT", "VFR_HUD",
                                  "HEARTBEAT", "STATUSTEXT"])
        if m is None:
            break
        t = getattr(m, "_timestamp", None)
        if t is None:
            continue
        # fixa o sysid do veiculo no 1o heartbeat que nao seja de uma GCS
        if vehicle_sys is None and m.get_type() == "HEARTBEAT":
            if (m.type != mavutil.mavlink.MAV_TYPE_GCS and
                    m.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID):
                vehicle_sys = m.get_srcSystem()
        if vehicle_sys is not None and m.get_srcSystem() != vehicle_sys:
            continue
        if t0 is None:
            t0 = t
        tt = t - t0
        typ = m.get_type()
        if typ == "VFR_HUD":
            pend["gs"] = m.groundspeed
            pend["vz"] = m.climb
        elif typ == "GLOBAL_POSITION_INT":
            tr.t.append(tt)
            tr.alt_raw.append(m.relative_alt * 1e-3)
            tr.gs.append(pend["gs"] if pend["gs"] is not None else float("nan"))
            tr.vz.append(pend["vz"] if pend["vz"] is not None else float("nan"))
        elif typ == "HEARTBEAT":
            name = PLANE_MODE_NAME.get(m.custom_mode, "MODE_%d" % m.custom_mode)
            if name != last_mode:
                tr.modes.append((tt, name))
                last_mode = name
        elif typ == "STATUSTEXT":
            txt = m.text
            if isinstance(txt, bytes):
                txt = txt.decode("utf-8", "replace")
            if verbose or any(k in txt.lower() for k in FS_KEYWORDS):
                tr.events.append((tt, "MSG", txt))
    return tr


def finalize(tr: Track) -> None:
    """Altitude relativa ao arranque e razao de subida derivada se faltar."""
    if not tr.alt_raw:
        return
    base = min(tr.alt_raw[:max(1, len(tr.alt_raw) // 50)])
    tr.alt = [a - base for a in tr.alt_raw]
    have_vz = any(not math.isnan(v) for v in tr.vz)
    if not have_vz:
        tr.vz = [float("nan")] * len(tr.t)
    # suaviza/deriva: usa sempre a derivada da altitude numa janela de ~2 s
    # quando o log nao traz razao de subida
    for i in range(len(tr.t)):
        if math.isnan(tr.vz[i]):
            j = max(0, i - 4)
            dt = tr.t[i] - tr.t[j]
            tr.vz[i] = ((tr.alt[i] - tr.alt[j]) / dt) if dt > 0.2 else float("nan")


def rolling(vals, n):
    out, buf, acc = [], [], 0.0
    for v in vals:
        if v is None or math.isnan(v):
            buf, acc = [], 0.0
            out.append(float("nan"))
            continue
        buf.append(v)
        acc += v
        if len(buf) > n:
            acc -= buf.pop(0)
        out.append(acc / len(buf) if len(buf) == n else float("nan"))
    return out


# --------------------------------------------------------------------------- #

def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log", help="ficheiro .bin/.BIN/.log (dataflash) ou .tlog")
    p.add_argument("--target-alt", type=float, default=2000.0,
                   help="altitude alvo para medir o tempo de subida, m")
    p.add_argument("--max-gradient", type=float, default=0.10,
                   help="limite de gradiente para o veredito (omissao: %(default)s)")
    p.add_argument("--window", type=float, default=20.0,
                   help="janela do gradiente sustentado, s (omissao: %(default)s)")
    p.add_argument("--min-alt", type=float, default=50.0,
                   help="ignora amostras abaixo desta altitude (chao/descolagem)")
    p.add_argument("--min-gs", type=float, default=3.0,
                   help="ignora amostras com groundspeed abaixo deste valor")
    p.add_argument("--csv", default=None, help="grava a serie temporal neste CSV")
    p.add_argument("--all-messages", action="store_true",
                   help="lista todos os STATUSTEXT/MSG, nao so' os de failsafe")
    p.add_argument("--use-repo-pymavlink", action="store_true",
                   help="forca o pymavlink de src/ardupilot/modules/mavlink")
    a = p.parse_args(argv)

    if not os.path.isfile(a.log):
        sys.stderr.write("ERRO: ficheiro nao encontrado: %s\n" % a.log)
        return 2

    ext = os.path.splitext(a.log)[1].lower()
    print("a ler %s (%s)..." % (a.log, "tlog" if ext == ".tlog" else "dataflash"))
    tr = read_tlog(a.log, a.all_messages) if ext == ".tlog" \
        else read_bin(a.log, a.all_messages)
    finalize(tr)

    if not tr.t:
        sys.stderr.write("ERRO: sem amostras de posicao no log.\n")
        return 2

    dt = (tr.t[-1] - tr.t[0]) / max(1, len(tr.t) - 1)
    n_win = max(2, int(round(a.window / max(dt, 1e-3))))

    grad = []
    for i in range(len(tr.t)):
        gs, vz = tr.gs[i], tr.vz[i]
        if (gs is None or math.isnan(gs) or gs < a.min_gs
                or vz is None or math.isnan(vz) or tr.alt[i] < a.min_alt):
            grad.append(float("nan"))
        else:
            grad.append(vz / gs)
    sust = rolling(grad, n_win)

    valid_i = [g for g in grad if not math.isnan(g)]
    valid_s = [(g, i) for i, g in enumerate(sust) if not math.isnan(g)]

    # tempo de subida ate' ao alvo
    t_start = t_reach = None
    for i, alt in enumerate(tr.alt):
        if t_start is None and alt >= a.min_alt:
            t_start = tr.t[i]
        if t_start is not None and alt >= a.target_alt * 0.995:
            t_reach = tr.t[i]
            break

    print("")
    print("=" * 70)
    print(" ANALISE DE LOG — %s" % os.path.basename(a.log))
    print("=" * 70)
    print("  duracao                 : %.0f s (%.1f min), %d amostras, dt~%.2f s"
          % (tr.t[-1] - tr.t[0], (tr.t[-1] - tr.t[0]) / 60.0, len(tr.t), dt))
    print("  altitude max (rel.)     : %.0f m" % max(tr.alt))
    if t_reach is not None:
        print("  tempo ate' %.0f m        : %.0f s (%.1f min)"
              % (a.target_alt, t_reach - t_start, (t_reach - t_start) / 60.0))
    else:
        print("  tempo ate' %.0f m        : nao atingida" % a.target_alt)
    if valid_i:
        gmax = max(valid_i)
        gmean = sum(valid_i) / len(valid_i)
        print("  --")
        print("  gradiente medio         : %.2f %%" % (100 * gmean))
        print("  gradiente instantaneo   : %.2f %% (max)" % (100 * gmax))
    if valid_s:
        gs_max, idx = max(valid_s, key=lambda x: x[0])
        print("  gradiente SUSTENTADO    : %.2f %% (max, janela %.0f s)"
              % (100 * gs_max, a.window))
        print("      pior ponto: t=%.0f s  alt=%.0f m  gs=%.1f m/s  vz=%+.2f m/s"
              % (tr.t[idx], tr.alt[idx], tr.gs[idx], tr.vz[idx]))
    else:
        gs_max = float("nan")

    print("")
    print("  MUDANCAS DE MODO")
    if tr.modes:
        for t, name in tr.modes:
            print("    %8.1f s   %s" % (t, name))
    else:
        print("    (nenhuma registada)")

    print("")
    print("  EVENTOS / FAILSAFES")
    if tr.events:
        for t, kind, detail in tr.events:
            print("    %8.1f s   %-4s %s" % (t, kind, detail))
    else:
        print("    (nenhum)")
    print("=" * 70)

    if a.csv:
        import csv
        with open(a.csv, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["t_s", "alt_rel_m", "gs_ms", "vz_ms", "grad", "grad_sust"])
            for i in range(len(tr.t)):
                w.writerow(["%.3f" % tr.t[i], "%.2f" % tr.alt[i],
                            "%.2f" % tr.gs[i], "%.3f" % tr.vz[i],
                            "%.5f" % grad[i], "%.5f" % sust[i]])
        print("CSV gravado: %s" % a.csv)

    if math.isnan(gs_max):
        print("\nVEREDITO: INCONCLUSIVO (sem amostras validas de gradiente)")
        return 2
    if gs_max > a.max_gradient:
        print("\nVEREDITO: FAIL — gradiente sustentado %.2f %% > %.2f %%"
              % (100 * gs_max, 100 * a.max_gradient))
        return 1
    print("\nVEREDITO: PASS — gradiente sustentado %.2f %% <= %.2f %%"
          % (100 * gs_max, 100 * a.max_gradient))
    return 0


if __name__ == "__main__":
    sys.exit(main())
