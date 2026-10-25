#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
artex_sitl.py — biblioteca partilhada dos testes de SITL da missao ARTEX 2000 m.

Tambem e' utilizavel como CLI:

    python3 artex_sitl.py wait      --url tcp:127.0.0.1:5762
    python3 artex_sitl.py upload    --url tcp:127.0.0.1:5762 --mission ../../missions/x.waypoints
    python3 artex_sitl.py params    --url tcp:127.0.0.1:5762 --param-file ../../params/x.param
    python3 artex_sitl.py get       --url tcp:127.0.0.1:5762 TECS_CLMB_MAX AIRSPEED_CRUISE
    python3 artex_sitl.py set       --url tcp:127.0.0.1:5762 SIM_WIND_SPD=5 SIM_WIND_DIR=270
    python3 artex_sitl.py reset-sim --url tcp:127.0.0.1:5762

Nada aqui depende de ROS nem de Gazebo: so' pymavlink.

Referencias verificadas no codigo deste workspace (src/ardupilot, ArduPlane V4.8.0-dev):
  libraries/SITL/SITL.cpp      -> SIM_WIND_SPD/DIR/TURB/TC, SIM_BATT_VOLTAGE/CAP_AH/RES_OHM, SIM_RC_FAIL
  libraries/SITL/SITL.h:161    -> enum SITL_RCFail {None=0, NoPulses=1, Throttle950=2, ProtocolFailBit=3}
  ArduPlane/mode.h:38          -> numeros de modo (AUTO=10, RTL=11, TAKEOFF=13, AUTOLAND=26)
  ArduPlane/Parameters.cpp:720 -> RTL_AUTOLAND 0..4 (4 = DO_RETURN_PATH_START)
  libraries/AP_Mission/AP_Mission.cpp:2512/2566 -> "Landing sequence start" / "Return path started"
  ArduPlane/events.cpp:240     -> "%s Failsafe On: switched to %s" ("GCS" | "RC Long")
  libraries/AP_BattMonitor/AP_BattMonitor.cpp:966 -> "Battery %d is %s %.2fV used %.0f mAh"
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:  # pragma: no cover
    sys.stderr.write(
        "ERRO: pymavlink nao encontrado.\n"
        "  Dentro do container atlas-sim ja esta instalado.\n"
        "  Fora dele:  python3 -m pip install --user pymavlink pexpect MAVProxy\n"
    )
    raise SystemExit(2)


# --------------------------------------------------------------------------- #
# Constantes do ArduPlane 4.8                                                  #
# --------------------------------------------------------------------------- #

PLANE_MODE = {
    "MANUAL": 0, "CIRCLE": 1, "STABILIZE": 2, "TRAINING": 3, "ACRO": 4,
    "FBWA": 5, "FBWB": 6, "CRUISE": 7, "AUTOTUNE": 8, "AUTO": 10,
    "RTL": 11, "LOITER": 12, "TAKEOFF": 13, "AVOID_ADSB": 14, "GUIDED": 15,
    "INITIALISING": 16, "THERMAL": 24, "AUTOLAND": 26,
}
MODE_NAME = {v: k for k, v in PLANE_MODE.items()}

# libraries/SITL/SITL.h:161 — enum SITL_RCFail
SIM_RC_FAIL_NONE = 0           # sem falha
SIM_RC_FAIL_NO_PULSES = 1      # receptor deixa de emitir frames  -> RC_FS_TIMEOUT
SIM_RC_FAIL_THROTTLE_950 = 2   # todos os canais a 1500, throttle a 950 us -> THR_FS_VALUE
SIM_RC_FAIL_PROTOCOL_BIT = 3   # bit de failsafe do protocolo (nao documentado no @Values)

# Canal MAVLink dos testes: SERIAL1 do SITL = TCP base_port+2 = 5762
# (libraries/AP_HAL_SITL/SITL_State.h:31 -> _serial_path = {"tcp:0:wait","tcp:2",...};
#  libraries/AP_HAL_SITL/SITL_cmdline.cpp:253 -> BASE_PORT = 5760).
#
# NAO usar a porta do MAVProxy (5760 / --out udp:14550): o protocolo de missao
# tem estado POR CANAL, e o MAVProxy responde/interfere no mesmo canal — o
# resultado observado foi "Mission upload timeout" e a missao a ficar com 1 item
# (so' o HOME), seguido de "Mission complete, changing mode to RTL" ao entrar em
# AUTO.  E' o mesmo conflito que docker/README.md descreve entre MAVProxy e MAVROS.
DEFAULT_URL = "tcp:127.0.0.1:5762"

# Diretorio deste ficheiro; usado para resolver os caminhos por omissao.
SIM_DIR = os.path.dirname(os.path.abspath(__file__))
# sim/ vive em src/artex27/tools/sim -> a raiz do pacote esta dois niveis acima.
ARTEX_DIR = os.path.dirname(os.path.dirname(SIM_DIR))
DEFAULT_PARAM_FILE = os.path.join(ARTEX_DIR, "params", "artex27_sitl.param")
DEFAULT_MISSION_DIR = os.path.join(ARTEX_DIR, "missions")


# --------------------------------------------------------------------------- #
# Utilidades                                                                   #
# --------------------------------------------------------------------------- #

class TestFailure(Exception):
    """Falha de teste (criterio nao cumprido) — distinta de um erro de ambiente."""


def log(msg: str) -> None:
    sys.stdout.write("%s %s\n" % (time.strftime("[%H:%M:%S]"), msg))
    sys.stdout.flush()


def ok(msg: str) -> None:
    log("  PASS  " + msg)


def fail(msg: str) -> None:
    log("  FAIL  " + msg)


def require_file(path: str, what: str) -> str:
    """Falha com mensagem clara se o ficheiro nao existir (os agentes irmaos
    podem ainda nao o ter gerado)."""
    if not path:
        raise SystemExit("ERRO: caminho vazio para %s" % what)
    if not os.path.isfile(path):
        raise SystemExit(
            "ERRO: %s nao encontrado:\n"
            "    %s\n"
            "  Este ficheiro e' produzido por outro agente do projeto ARTEX.\n"
            "  Gera-o (ou passa --%s <caminho>) antes de correr este teste."
            % (what, path, what)
        )
    return path


def find_default_mission(variant: str | None = None) -> str:
    """Procura uma missao em src/artex27/missions/."""
    if not os.path.isdir(DEFAULT_MISSION_DIR):
        raise SystemExit(
            "ERRO: %s nao existe. A missao e' produzida por outro agente.\n"
            "  Passa --mission <caminho> explicitamente." % DEFAULT_MISSION_DIR
        )
    cands = sorted(
        os.path.join(DEFAULT_MISSION_DIR, f)
        for f in os.listdir(DEFAULT_MISSION_DIR)
        if f.endswith((".waypoints", ".txt", ".plan")) and "fence" not in f.lower()
    )
    if variant:
        cands = [c for c in cands if variant in os.path.basename(c)]
    if not cands:
        raise SystemExit(
            "ERRO: sem ficheiros de missao em %s%s.\n"
            "  Passa --mission <caminho> explicitamente."
            % (DEFAULT_MISSION_DIR, (" para a variante '%s'" % variant) if variant else "")
        )
    return cands[0]


# --------------------------------------------------------------------------- #
# Ligacao                                                                      #
# --------------------------------------------------------------------------- #

def connect(url: str = DEFAULT_URL, timeout: float = 60.0, rate_hz: int = 5):
    """Liga ao SITL e espera o primeiro heartbeat. Devolve a conexao mavutil."""
    log("a ligar a %s ..." % url)
    m = mavutil.mavlink_connection(url, source_system=250, source_component=190)
    t0 = time.time()
    hb = None
    while time.time() - t0 < timeout:
        hb = m.wait_heartbeat(timeout=5)
        if hb is not None:
            break
        log("  ... sem heartbeat, a insistir (%.0fs)" % (time.time() - t0))
    if hb is None:
        raise SystemExit(
            "ERRO: sem heartbeat em %s apos %.0fs.\n"
            "  O SITL esta no ar? Arranca-o com:  ./run_sitl.sh\n"
            "  Canal dos testes: tcp:127.0.0.1:5762 (SERIAL1 do SITL, +10 por instancia)."
            % (url, timeout)
        )
    log("ligado: sysid=%d compid=%d autopiloto=%d tipo=%d"
        % (m.target_system, m.target_component, hb.autopilot, hb.type))
    # taxas de telemetria: o canal dos testes e' independente do MAVProxy
    m.mav.request_data_stream_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, rate_hz, 1)
    return m


def wait_ready(m, timeout: float = 120.0) -> None:
    """Espera posicao valida (EKF com GPS) — pre-requisito para armar."""
    log("a esperar posicao valida (GPS/EKF)...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg and msg.lat != 0 and msg.lon != 0:
            log("posicao valida: %.7f %.7f  alt_amsl=%.1f m"
                % (msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3))
            return
    raise SystemExit("ERRO: o SITL nao obteve posicao valida em %.0fs" % timeout)


# --------------------------------------------------------------------------- #
# Parametros                                                                   #
# --------------------------------------------------------------------------- #

def param_get(m, name: str, timeout: float = 5.0, retries: int = 3):
    """Le um parametro. Devolve float ou None."""
    for _ in range(retries):
        m.mav.param_request_read_send(
            m.target_system, m.target_component, name.encode("ascii"), -1)
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
            if msg is None:
                break
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode("ascii", "ignore")
            if pid.strip("\x00") == name:
                return float(msg.param_value)
    return None


def param_set(m, name: str, value: float, timeout: float = 5.0, retries: int = 5,
              verify: bool = True) -> float:
    """Escreve um parametro e confirma o valor. Levanta TestFailure se nao pegar."""
    for _ in range(retries):
        m.mav.param_set_send(
            m.target_system, m.target_component, name.encode("ascii"),
            float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
            if msg is None:
                break
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode("ascii", "ignore")
            if pid.strip("\x00") != name:
                continue
            got = float(msg.param_value)
            if (not verify) or abs(got - float(value)) < max(1e-4, abs(value) * 1e-4):
                return got
            break
    raise TestFailure("nao consegui por %s=%s (ultimo valor lido: %s)"
                      % (name, value, param_get(m, name)))


def parse_param_file(path: str) -> "list[tuple[str, float]]":
    """Le um .param do Mission Planner / QGC.  Formatos aceites:
         NAME<espacos ou virgula ou tab>VALUE
       Comentarios com '#'.  Linhas de cabecalho do QGC ignoradas."""
    out = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            line = raw.split("#")[0].strip()
            if not line:
                continue
            parts = line.replace(",", " ").replace("\t", " ").split()
            if len(parts) < 2:
                continue
            name = parts[0].upper()
            # formato QGC "1 1 NAME VALUE TYPE"
            if len(parts) >= 4 and parts[0].isdigit() and parts[1].isdigit():
                name, val = parts[2].upper(), parts[3]
            else:
                val = parts[1]
            try:
                out.append((name, float(val)))
            except ValueError:
                continue
    return out


def load_param_file(m, path: str, skip_prefixes=("SIM_",)) -> int:
    """Envia todos os parametros de um .param. Devolve quantos foram aplicados."""
    require_file(path, "param-file")
    n = 0
    for name, val in parse_param_file(path):
        if any(name.startswith(p) for p in skip_prefixes):
            continue
        try:
            param_set(m, name, val)
            n += 1
        except TestFailure as e:
            log("  aviso: %s" % e)
    log("parametros aplicados: %d (de %s)" % (n, path))
    return n


# --------------------------------------------------------------------------- #
# Missao                                                                       #
# --------------------------------------------------------------------------- #

class MissionItem:
    __slots__ = ("seq", "frame", "command", "current", "autocontinue",
                 "p1", "p2", "p3", "p4", "x", "y", "z")

    def __init__(self, seq, frame, command, current, autocontinue,
                 p1, p2, p3, p4, x, y, z):
        self.seq, self.frame, self.command = seq, frame, command
        self.current, self.autocontinue = current, autocontinue
        self.p1, self.p2, self.p3, self.p4 = p1, p2, p3, p4
        self.x, self.y, self.z = x, y, z   # x=lat deg, y=lon deg, z=alt m


def read_mission(path: str) -> "list[MissionItem]":
    """Le QGC WPL 110 (.waypoints/.txt) ou QGroundControl .plan (JSON)."""
    require_file(path, "mission")
    if path.endswith(".plan"):
        return _read_plan(path)
    return _read_wpl(path)


def _read_wpl(path: str) -> "list[MissionItem]":
    items = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        first = fh.readline()
        if "QGC WPL" not in first:
            fh.seek(0)
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            f = line.split("\t") if "\t" in line else line.split()
            if len(f) < 12:
                raise SystemExit("ERRO: linha de missao com %d campos (esperados 12):\n  %s"
                                 % (len(f), line))
            items.append(MissionItem(
                seq=int(f[0]), current=int(f[1]), frame=int(f[2]), command=int(f[3]),
                p1=float(f[4]), p2=float(f[5]), p3=float(f[6]), p4=float(f[7]),
                x=float(f[8]), y=float(f[9]), z=float(f[10]),
                autocontinue=int(float(f[11]))))
    return items


def _read_plan(path: str) -> "list[MissionItem]":
    with open(path, "r", encoding="utf-8") as fh:
        plan = json.load(fh)
    mission = plan["mission"]
    items = []
    home = mission.get("plannedHomePosition")
    seq = 0
    if home:
        items.append(MissionItem(0, 0, 16, 1, 1, 0, 0, 0, 0,
                                 float(home[0]), float(home[1]), float(home[2])))
        seq = 1
    for it in mission["items"]:
        if it.get("type") != "SimpleItem":
            raise SystemExit("ERRO: .plan com item complexo (%s) — exporta como .waypoints"
                             % it.get("type"))
        p = list(it["params"]) + [0] * 7
        items.append(MissionItem(
            seq=seq, frame=int(it["frame"]), command=int(it["command"]),
            current=0, autocontinue=int(it.get("autoContinue", True)),
            p1=float(p[0] or 0), p2=float(p[1] or 0), p3=float(p[2] or 0),
            p4=float(p[3] or 0), x=float(p[4] or 0), y=float(p[5] or 0),
            z=float(p[6] or 0)))
        seq += 1
    return items


def upload_mission(m, path: str, timeout: float = 60.0) -> "list[MissionItem]":
    """Carrega a missao no FCU pelo protocolo MISSION_ITEM_INT."""
    items = read_mission(path)
    log("a carregar missao: %s (%d itens)" % (path, len(items)))

    m.mav.mission_clear_all_send(m.target_system, m.target_component,
                                 mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    # drenar o MISSION_ACK do clear_all: se ficar na fila, o ciclo abaixo
    # confunde-o com o ACK final e devolve "aceite" sem enviar item nenhum
    # (sintoma: o FCU fica com 1 item, so' o HOME).
    t0 = time.time()
    while time.time() - t0 < 3:
        if m.recv_match(type="MISSION_ACK", blocking=True, timeout=1) is not None:
            break
    while m.recv_match(type=["MISSION_ACK", "MISSION_REQUEST",
                             "MISSION_REQUEST_INT"], blocking=False) is not None:
        pass

    m.mav.mission_count_send(m.target_system, m.target_component, len(items),
                             mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    t0 = time.time()
    sent = set()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT",
                                 "MISSION_ACK"], blocking=True, timeout=5)
        if msg is None:
            continue
        if msg.get_type() == "MISSION_ACK":
            if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise SystemExit("ERRO: o FCU rejeitou a missao (MISSION_ACK type=%d)" % msg.type)
            log("missao aceite pelo FCU (%d itens)" % len(items))
            time.sleep(1.0)
            n = mission_count(m)
            if n is not None and n != len(items):
                raise SystemExit(
                    "ERRO: o FCU ficou com %s itens em vez de %d.\n"
                    "  Causa tipica: duas entidades a falar o protocolo de missao\n"
                    "  no MESMO canal (MAVProxy + script). Usa o canal SERIAL1\n"
                    "  (tcp:127.0.0.1:5762) e nao a saida do MAVProxy."
                    % (n, len(items)))
            if n is None:
                log("  aviso: nao consegui confirmar a contagem de itens no FCU")
            return items
        i = items[msg.seq]
        m.mav.mission_item_int_send(
            m.target_system, m.target_component, i.seq, i.frame, i.command,
            1 if i.seq == 0 else 0, i.autocontinue,
            i.p1, i.p2, i.p3, i.p4,
            int(round(i.x * 1e7)), int(round(i.y * 1e7)), float(i.z),
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        sent.add(msg.seq)
    raise SystemExit("ERRO: timeout a carregar a missao (enviados %d/%d itens)"
                     % (len(sent), len(items)))


def mission_count(m, timeout: float = 10.0, retries: int = 3):
    """Numero de itens de missao que o FCU tem realmente guardados."""
    for _ in range(retries):
        m.mav.mission_request_list_send(m.target_system, m.target_component,
                                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = m.recv_match(type="MISSION_COUNT", blocking=True, timeout=timeout)
            if msg is None:
                break
            if msg.get_srcSystem() != m.target_system:
                continue
            # fecha a transaccao para o FCU nao ficar a espera de pedidos
            m.mav.mission_ack_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_MISSION_ACCEPTED,
                                   mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            return msg.count
        time.sleep(0.5)
    return None


def battery_consumed_mah(m, timeout: float = 5.0):
    """mAh ja' consumidos, da BATTERY_STATUS (campo current_consumed)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="BATTERY_STATUS", blocking=True, timeout=timeout)
        if msg is None:
            return None
        if msg.get_srcSystem() != m.target_system:
            continue
        v = msg.current_consumed
        return float(v) if v is not None and v >= 0 else 0.0
    return None


# --------------------------------------------------------------------------- #
# Geofence                                                                     #
# --------------------------------------------------------------------------- #

MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001


def read_fence(path: str):
    """Le o geofence no formato legado FENCE_POINT (o que o MAVProxy aceita em
    `fence load`): uma linha por ponto, "lat lon".

      linha 0      -> return point
      linhas 1..N  -> vertices do poligono, com a ultima igual a' primeira

    Devolve a lista de vertices UNICOS (sem o return point e sem o fecho)."""
    require_file(path, "fence")
    pts = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            line = raw.split("#")[0].strip()
            if not line:
                continue
            f = line.replace(",", " ").split()
            if len(f) < 2:
                continue
            try:
                pts.append((float(f[0]), float(f[1])))
            except ValueError:
                continue
    if len(pts) < 4:
        raise SystemExit("ERRO: ficheiro de fence com %d pontos; esperava "
                         "return point + poligono" % len(pts))
    verts = pts[1:]
    if verts[0] == verts[-1]:          # fecho duplicado
        verts = verts[:-1]
    return verts


def upload_fence(m, path: str, timeout: float = 60.0):
    """Carrega a fence de inclusao pelo protocolo de missao, tipo FENCE.

    Necessario quando FENCE_TYPE tem o bit2 (inclusao/exclusao): sem poligono
    carregado o pre-arm falha com "Polygon fence(s) invalid"
    (libraries/AC_Fence/AC_Fence.cpp:428)."""
    verts = read_fence(path)
    log("a carregar geofence: %s (%d vertices de inclusao)" % (path, len(verts)))

    m.mav.mission_clear_all_send(m.target_system, m.target_component,
                                 mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
    t0 = time.time()
    while time.time() - t0 < 3:
        if m.recv_match(type="MISSION_ACK", blocking=True, timeout=1) is not None:
            break
    while m.recv_match(type=["MISSION_ACK", "MISSION_REQUEST",
                             "MISSION_REQUEST_INT"], blocking=False) is not None:
        pass

    m.mav.mission_count_send(m.target_system, m.target_component, len(verts),
                             mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT",
                                 "MISSION_ACK"], blocking=True, timeout=5)
        if msg is None:
            continue
        if msg.get_type() == "MISSION_ACK":
            if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise SystemExit("ERRO: o FCU rejeitou a fence "
                                 "(MISSION_ACK type=%d)" % msg.type)
            log("geofence aceite pelo FCU (%d vertices)" % len(verts))
            return verts
        lat, lon = verts[msg.seq]
        m.mav.mission_item_int_send(
            m.target_system, m.target_component, msg.seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            0, 1, float(len(verts)), 0, 0, 0,
            int(round(lat * 1e7)), int(round(lon * 1e7)), 0.0,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
    raise SystemExit("ERRO: timeout a carregar a geofence")


def find_default_fence():
    """src/artex27/missions/artex27_geofence.waypoints, se existir."""
    p = os.path.join(DEFAULT_MISSION_DIR, "artex27_geofence.waypoints")
    return p if os.path.isfile(p) else None


def mission_index(items, command: int):
    """Indice (seq) do primeiro item com este comando MAV_CMD, ou None."""
    for i in items:
        if i.command == command:
            return i.seq
    return None


MAV_CMD_DO_LAND_START = 189
MAV_CMD_DO_RETURN_PATH_START = 188
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_LOITER_TO_ALT = 31


# --------------------------------------------------------------------------- #
# Modos, armar                                                                 #
# --------------------------------------------------------------------------- #

def set_mode(m, mode, timeout: float = 15.0) -> None:
    num = PLANE_MODE[mode] if isinstance(mode, str) else int(mode)
    name = MODE_NAME.get(num, str(num))
    m.mav.set_mode_send(m.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, num)
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
        if hb and hb.custom_mode == num:
            log("modo -> %s" % name)
            return
        m.mav.set_mode_send(m.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, num)
    cur = current_mode(m)
    raise TestFailure(
        "nao consegui entrar no modo %s (continua em %s). Causas tipicas: "
        "pre-arm/arming check, failsafe activo a forcar RTL, ou missao vazia."
        % (name, MODE_NAME.get(cur, cur)))


def current_mode(m, timeout: float = 5.0):
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
    return hb.custom_mode if hb else None


def arm(m, timeout: float = 30.0, force: bool = False) -> None:
    log("a armar...")
    t0 = time.time()
    prearm = None
    while time.time() - t0 < timeout:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 21196 if force else 0, 0, 0, 0, 0, 0)
        msg = m.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=3)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            txt = msg.text
            if isinstance(txt, bytes):
                txt = txt.decode("utf-8", "replace")
            if "prearm" in txt.lower() or "arm" in txt.lower():
                prearm = txt
            continue
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            log("armado")
            return
    extra = ""
    if prearm and "landing sequence" in prearm.lower():
        extra = ("\n  'In landing sequence' = a flag in_landing_sequence do AP_Mission"
                 "\n  ficou do teste anterior (ArduPlane/AP_Arming_Plane.cpp:126)."
                 "\n  AP_Mission::clear() NAO limpa esta flag; so' reset()/set_current_cmd()"
                 "\n  a limpam, e AP_Mission::start_command() volta a po-la se a missao"
                 "\n  reentrar no DO_LAND_START (AP_Mission.cpp:416-418)."
                 "\n  SOLUCAO FIAVEL: um SITL novo por cenario — ver README secao 4.3.")
    raise TestFailure("nao consegui armar em %.0fs (ultimo pre-arm: %s)%s"
                      % (timeout, prearm or "nenhum", extra))


def is_armed(m) -> bool:
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
    return bool(hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))


def set_current_mission_item(m, seq: int) -> None:
    m.mav.mission_set_current_send(m.target_system, m.target_component, seq)
    log("item de missao corrente -> %d" % seq)


def reset_mission_state(m, first_item: int = 1) -> None:
    """Limpa as flags in_landing_sequence / in_return_path do AP_Mission.

    Sem isto, depois de um teste em que o aviao entrou no percurso de regresso,
    o pre-arm seguinte falha com "PreArm: In landing sequence"
    (ArduPlane/AP_Arming_Plane.cpp:126-130) — e a flag SOBREVIVE ao reboot,
    porque com MIS_RESTART=0 o indice de missao e' retomado de onde parou.

    Dois mecanismos, aplicados os dois:
      1. mudar para um modo sem navegacao automatica com o aviao desarmado faz
         o proprio ArduPlane chamar mission.reset() (ArduPlane/mode.cpp:143-147,
         mensagem "In landing sequence: mission reset");
      2. MISSION_SET_CURRENT 0 -> AP_Mission::set_current_cmd() limpa as duas
         flags explicitamente (AP_Mission.cpp:604-608).
    """
    # 1) MAV_CMD_DO_SET_MISSION_CURRENT (224) — caminho moderno, com ACK
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT, 0,
        float(first_item), 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    acked = None
    while time.time() - t0 < 5:
        msg = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        if msg is None:
            break
        if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT:
            acked = msg.result
            break
    # 2) MISSION_SET_CURRENT (mensagem legada, sem ACK) como rede de seguranca
    m.mav.mission_set_current_send(m.target_system, m.target_component, first_item)
    time.sleep(0.5)
    # 3) ciclo de modo: com o aviao desarmado e no chao, ArduPlane chama
    #    mission.reset() ao entrar num modo sem navegacao automatica. Tem de ser
    #    uma MUDANCA real de modo — set_mode para o modo corrente nao dispara o
    #    hook (ArduPlane/mode.cpp:143-147).
    try:
        set_mode(m, "FBWA")
        set_mode(m, "MANUAL")
    except TestFailure as e:
        log("  aviso no ciclo de modo: %s" % e)
    log("estado da missao reposto (item corrente -> %d, DO_SET_MISSION_CURRENT ack=%s)"
        % (first_item, acked))


def wait_stable(m, s, dt: float = 0.5, quiet: float = 15.0,
                timeout: float = 120.0, expect_mode: str = "AUTO") -> bool:
    """Espera que o voo estabilize: sem STATUSTEXT de failsafe durante `quiet`
    segundos de SIMULACAO e no modo esperado.

    Necessario depois de repor os failsafes por tempo: ao voltar de SIM_SPEEDUP
    alto para 1x, o `tnow - gcs_last_seen_ms` do FCU pode estar momentaneamente
    acima do FS_LONG_TIMEOUT e disparar um "GCS Failsafe On" que nao tem nada a
    ver com o teste."""
    noisy = ("failsafe", "return path", "landing sequence")
    t_start = s.t_sim
    last_event = s.t_sim
    t0 = time.time()
    while time.time() - t0 < timeout:
        s.pump(dt)
        for t, _sev, txt in s.texts:
            if t > last_event and any(n in txt.lower() for n in noisy):
                last_event = t
        mode_ok = True
        if expect_mode:
            cur = s.modes[-1][1] if s.modes else None
            mode_ok = (cur == PLANE_MODE.get(expect_mode))
        if mode_ok and (s.t_sim - last_event) >= quiet:
            log("voo estavel ha' %.0f s de simulacao (modo %s)"
                % (s.t_sim - last_event, expect_mode))
            return True
    log("AVISO: nao estabilizou em %.0f s de relogio (ultimo evento a t=%.0f s sim)"
        % (timeout, last_event))
    return False


def reboot_autopilot(m, url: str, settle: float = 6.0):
    """Reinicia o FCU (SITL) para repor o veiculo no chao, desarmado, entre
    cenarios de teste. Os parametros guardados sobrevivem ao reboot.
    Devolve uma NOVA conexao (a antiga fica com sequencias invalidas)."""
    log("a reiniciar o autopiloto (PREFLIGHT_REBOOT_SHUTDOWN)...")
    try:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
            1, 0, 0, 0, 0, 0, 0)
    except Exception as e:
        log("  aviso: %s" % e)
    try:
        m.close()
    except Exception:
        pass
    time.sleep(settle)
    m2 = connect(url, timeout=90)
    wait_ready(m2)
    return m2


class HeartbeatSender:
    """Envia HEARTBEAT de GCS a 1 Hz num thread. Usado no teste de failsafe de
    telemetria: para-se o envio e o FCU deixa de ver a GCS.

    Nota: o ArduPilot so' conta como GCS o sysid em MAV_GCS_SYSID (omissao 255);
    o teste poe MAV_GCS_SYSID igual ao sysid deste script."""

    def __init__(self, m, period: float = 1.0):
        import threading
        self.m = m
        self.period = period
        self._stop = threading.Event()
        self._thread = None
        self._threading = threading

    def start(self):
        if self._thread:
            return self
        self._stop.clear()
        self._thread = self._threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        log("heartbeats de GCS: ON (sysid %d)" % self.m.source_system)
        return self

    def _run(self):
        while not self._stop.is_set():
            try:
                self.m.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception:
                pass
            self._stop.wait(self.period)

    def stop(self):
        if self._thread:
            self._stop.set()
            self._thread.join(timeout=3)
            self._thread = None
            log("heartbeats de GCS: OFF")
        return self


# --------------------------------------------------------------------------- #
# Amostragem de telemetria                                                     #
# --------------------------------------------------------------------------- #

class Sampler:
    """Agrega GLOBAL_POSITION_INT + VFR_HUD + STATUSTEXT + HEARTBEAT numa
    linha de amostra por iteracao, e guarda tudo para analise posterior.

    BASE DE TEMPO: segundos de SIMULACAO, nao de relogio. Vem do
    GLOBAL_POSITION_INT.time_boot_ms (tempo do FCU, que corre a SIM_SPEEDUP).
    Assim todas as janelas, duracoes e comparacoes com a previsao sao
    independentes do speedup escolhido."""

    def __init__(self, m, home_alt_amsl: float | None = None):
        self.m = m
        self.rows = []          # dicts
        self.texts = []         # (t_sim, severity, text)
        self.modes = []         # (t_sim, mode_num)
        self.t0 = time.time()   # relogio, para timeouts
        self._boot0 = None      # time_boot_ms da 1a amostra
        self._tsim = 0.0        # tempo de simulacao corrente
        self.home_alt = home_alt_amsl
        self._last_mode = None
        self._st = {"alt_rel": None, "gs": None, "climb": None,
                    "airspeed": None, "hdg": None, "lat": None, "lon": None,
                    "throttle": None}

    @property
    def t_sim(self) -> float:
        """Segundos de simulacao desde a primeira amostra."""
        return self._tsim

    @property
    def speedup_measured(self):
        """Razao real entre tempo de simulacao e tempo de relogio."""
        wall = time.time() - self.t0
        return (self._tsim / wall) if wall > 1 else None

    # -- recolha -----------------------------------------------------------
    def pump(self, duration: float = 0.5) -> None:
        """Consome mensagens durante 'duration' segundos e grava uma amostra."""
        end = time.time() + duration
        while time.time() < end:
            msg = self.m.recv_match(blocking=True, timeout=max(0.05, end - time.time()))
            if msg is None:
                break
            self._absorb(msg)
        self._record()

    def _absorb(self, msg) -> None:
        # so' o veiculo alvo: num link partilhado chegam heartbeats de outros
        # componentes (MAVProxy, AP_DDS, camaras) e o modo lido saltaria.
        try:
            if msg.get_srcSystem() != self.m.target_system:
                return
        except AttributeError:
            pass
        t = msg.get_type()
        if t == "HEARTBEAT" and msg.type == mavutil.mavlink.MAV_TYPE_GCS:
            return
        if t == "GLOBAL_POSITION_INT":
            if self._boot0 is None:
                self._boot0 = msg.time_boot_ms * 1e-3
            self._tsim = msg.time_boot_ms * 1e-3 - self._boot0
            self._st["alt_rel"] = msg.relative_alt * 1e-3
            self._st["lat"] = msg.lat * 1e-7
            self._st["lon"] = msg.lon * 1e-7
            self._st["hdg"] = msg.hdg * 1e-2
        elif t == "VFR_HUD":
            self._st["gs"] = msg.groundspeed
            self._st["climb"] = msg.climb
            self._st["airspeed"] = msg.airspeed
            self._st["throttle"] = msg.throttle
        elif t == "STATUSTEXT":
            txt = msg.text
            if isinstance(txt, bytes):
                txt = txt.decode("utf-8", "replace")
            self.texts.append((self._tsim, msg.severity, txt))
            log("    STATUSTEXT[%d] %s" % (msg.severity, txt))
        elif t == "HEARTBEAT":
            if msg.custom_mode != self._last_mode:
                self._last_mode = msg.custom_mode
                self.modes.append((self._tsim, msg.custom_mode))
                log("    MODO -> %s" % MODE_NAME.get(msg.custom_mode, msg.custom_mode))
        elif t == "MISSION_CURRENT":
            self._st["wp"] = msg.seq

    def _record(self) -> None:
        if self._st["alt_rel"] is None or self._st["gs"] is None:
            return
        gs = self._st["gs"]
        climb = self._st["climb"] if self._st["climb"] is not None else 0.0
        grad = (climb / gs) if gs > 0.5 else float("nan")
        self.rows.append({
            "t": self._tsim,
            "t_wall": time.time() - self.t0,
            "alt_rel": self._st["alt_rel"],
            "lat": self._st["lat"], "lon": self._st["lon"],
            "gs": gs, "climb": climb, "airspeed": self._st["airspeed"],
            "throttle": self._st["throttle"], "hdg": self._st["hdg"],
            "wp": self._st.get("wp"),
            "grad": grad,
        })

    # -- consultas ---------------------------------------------------------
    @property
    def alt(self):
        return self.rows[-1]["alt_rel"] if self.rows else None

    def saw_text(self, needle: str, since: float = 0.0) -> bool:
        n = needle.lower()
        return any(n in txt.lower() for t, _s, txt in self.texts if t >= since)

    def texts_since(self, since: float):
        return [(t, s, x) for t, s, x in self.texts if t >= since]

    def saw_mode(self, mode, since: float = 0.0) -> bool:
        num = PLANE_MODE[mode] if isinstance(mode, str) else int(mode)
        return any(mm == num for t, mm in self.modes if t >= since)

    def modes_since(self, since: float):
        return [(t, MODE_NAME.get(mm, mm)) for t, mm in self.modes if t >= since]

    def write_csv(self, path: str) -> None:
        import csv
        if not self.rows:
            return
        cols = ["t", "t_wall", "alt_rel", "lat", "lon", "gs", "airspeed",
                "climb", "grad", "throttle", "hdg", "wp"]
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
            w.writeheader()
            for r in self.rows:
                w.writerow(r)
        log("CSV gravado: %s (%d amostras)" % (path, len(self.rows)))

    def write_events(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as fh:
            fh.write("# t_sim_s\ttipo\tdetalhe\n")
            for t, mm in self.modes:
                fh.write("%.2f\tMODE\t%s\n" % (t, MODE_NAME.get(mm, mm)))
            for t, s, x in self.texts:
                fh.write("%.2f\tTEXT(sev=%d)\t%s\n" % (t, s, x))
        log("eventos gravados: %s" % path)


def rolling_mean(values, n: int):
    """Media movel simples; devolve lista do mesmo tamanho (primeiros = NaN)."""
    out, acc, buf = [], 0.0, []
    for v in values:
        if v is None or (isinstance(v, float) and math.isnan(v)):
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
# Ambiente de simulacao: vento e bateria                                       #
# --------------------------------------------------------------------------- #

SIM_DEFAULTS = {
    # valores de fabrica lidos em libraries/SITL/SITL.cpp
    "SIM_SPEEDUP": 1.0,
    "SIM_WIND_SPD": 0.0,
    "SIM_WIND_DIR": 180.0,
    "SIM_WIND_TURB": 0.0,
    "SIM_RC_FAIL": 0.0,
    "SIM_BATT_VOLTAGE": 12.6,
    "SIM_BATT_CAP_AH": 0.0,
}


def set_wind(m, speed: float, direction: float, turbulence: float = 0.0) -> None:
    """SIM_WIND_DIR e' a direccao DE ONDE vem o vento (graus true)."""
    param_set(m, "SIM_WIND_SPD", speed)
    param_set(m, "SIM_WIND_DIR", direction)
    param_set(m, "SIM_WIND_TURB", turbulence)
    log("vento simulado: %.1f m/s de %.0f deg (turb %.1f)" % (speed, direction, turbulence))


def set_speedup(m, speedup: float) -> float:
    """SIM_SPEEDUP (libraries/SITL/SITL.cpp:229). Devolve o valor anterior.

    ATENCAO: acima de ~5x os failsafes por TEMPO deixam de ser representativos.
    O MAVProxy injecta o RC e o heartbeat de GCS em tempo de RELOGIO; o FCU
    conta RC_FS_TIMEOUT / FS_LONG_TIMEOUT em tempo de SIMULACAO. Com speedup 20,
    5 s de simulacao sao 0,25 s de relogio e o aviao entra em failsafe sozinho.
    Por isso: subida/posicionamento com speedup alto e failsafes relaxados;
    injeccao de falhas com speedup baixo e failsafes activos."""
    before = param_get(m, "SIM_SPEEDUP")
    param_set(m, "SIM_SPEEDUP", speedup)
    log("SIM_SPEEDUP %s -> %s" % (before, speedup))
    return before if before is not None else 1.0


# Parametros de failsafe por TEMPO, relaxados enquanto se ganha altitude com
# SIM_SPEEDUP alto.  Valor = o que se poe durante a fase relaxada.
#
# Porque e' preciso (verificado neste workspace, nao e' teoria):
#   * o MAVProxy injecta o RC (modulo rcin) e o heartbeat de GCS em tempo de
#     RELOGIO; o FCU conta RC_FS_TIMEOUT / FS_LONG_TIMEOUT em tempo de SIMULACAO;
#   * com SIM_SPEEDUP=10, 1 s de RC_FS_TIMEOUT sao 0,1 s de relogio -> o FCU
#     declara perda de RC sem haver falha nenhuma;
#   * pior: ArduPlane/mode_takeoff.cpp:193 ADIA o failsafe longo enquanto o
#     flight stage e' TAKEOFF e dispara-o assim que passa a NORMAL — ou seja,
#     o aviao chega aos 100 m e salta logo para RTL, e a entrega para AUTO falha.
# Nota: rc_failsafe_active() (ArduPlane/radio.cpp:348) dispara por RC_FS_TIMEOUT
# mesmo com THR_FAILSAFE=0; por isso nao basta desligar o THR_FAILSAFE.
RELAXED_FS = {
    "THR_FAILSAFE": 0,     # ignora o THR_FS_VALUE
    "FS_GCS_ENABL": 0,     # ignora a perda de heartbeat da GCS
    "FS_LONG_ACTN": 0,     # "Continue" — em AUTO nao muda de modo
    "FS_SHORT_ACTN": 0,    # "Continue"
    "RC_FS_TIMEOUT": 10,   # maximo admitido (RC_Channels_VarInfo.h:111, 0.1..10)
}


def relax_failsafes(m) -> dict:
    """Relaxa temporariamente os failsafes por tempo. Devolve os valores
    anteriores, para `restore_failsafes`."""
    before = {}
    for name, target in RELAXED_FS.items():
        v = param_get(m, name)
        if v is None:
            continue
        before[name] = v
        if v != target:
            try:
                param_set(m, name, target)
            except TestFailure as e:
                log("  aviso ao relaxar %s: %s" % (name, e))
    if before:
        log("failsafes por tempo RELAXADOS (valores anteriores: %s)"
            % ", ".join("%s=%g" % (k, v) for k, v in before.items()))
    return before


def restore_failsafes(m, before: dict) -> None:
    for name, v in (before or {}).items():
        try:
            param_set(m, name, v)
        except TestFailure as e:
            log("  aviso ao repor %s: %s" % (name, e))
    if before:
        log("failsafes por tempo repostos: %s"
            % ", ".join("%s=%g" % (k, v) for k, v in before.items()))


def reset_sim_params(m) -> None:
    """Repoe os SIM_* de simulacao aos valores de fabrica. Correr SEMPRE no fim."""
    for name, val in SIM_DEFAULTS.items():
        try:
            param_set(m, name, val)
        except TestFailure as e:
            log("  aviso ao repor: %s" % e)
    log("SIM_* repostos aos valores de fabrica")


# --------------------------------------------------------------------------- #
# CLI                                                                          #
# --------------------------------------------------------------------------- #

def _main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["wait", "upload", "fence", "params",
                                    "get", "set", "reset-sim"])
    ap.add_argument("rest", nargs="*")
    ap.add_argument("--url", default=DEFAULT_URL)
    ap.add_argument("--mission", default=None)
    ap.add_argument("--fence-file", default=None)
    ap.add_argument("--param-file", default=None)
    ap.add_argument("--timeout", type=float, default=90.0)
    a = ap.parse_args(argv)

    m = connect(a.url, timeout=a.timeout)

    if a.cmd == "wait":
        wait_ready(m)
    elif a.cmd == "upload":
        upload_mission(m, a.mission or find_default_mission())
    elif a.cmd == "fence":
        upload_fence(m, a.fence_file or find_default_fence())
    elif a.cmd == "params":
        load_param_file(m, a.param_file or DEFAULT_PARAM_FILE)
    elif a.cmd == "get":
        for name in a.rest:
            print("%-20s = %s" % (name, param_get(m, name)))
    elif a.cmd == "set":
        for kv in a.rest:
            name, _, val = kv.partition("=")
            print("%-20s = %s" % (name, param_set(m, name, float(val))))
    elif a.cmd == "reset-sim":
        reset_sim_params(m)
    return 0


if __name__ == "__main__":
    sys.exit(_main())
