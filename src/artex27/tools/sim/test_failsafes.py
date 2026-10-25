#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_failsafes.py — bateria de testes de failsafe em SITL para a missao ARTEX.

CENARIOS
  rc_nopulses  SIM_RC_FAIL=1  o receptor deixa de emitir frames.
                              Dispara por RC_FS_TIMEOUT (omissao 1.0 s) e depois
                              FS_LONG_TIMEOUT. E' o caso "perdi o link do radio".
  rc_throttle  SIM_RC_FAIL=2  todos os canais a 1500 us e o throttle a 950 us.
                              Dispara por THR_FAILSAFE/THR_FS_VALUE
                              (ArduPlane/radio.cpp:333 -> ok sse radio_in > THR_FS_VALUE;
                              com THR_FS_VALUE=950 e radio_in=950 dispara).
  rc_protocol  SIM_RC_FAIL=3  bit de failsafe do protocolo. NAO esta documentado
                              no @Values do parametro, so' existe no enum
                              (libraries/SITL/SITL.h:161). Incluido por completude.
  gcs          perda de telemetria: o script deixa de enviar heartbeats de GCS.
                              Exige FS_GCS_ENABL != 0 e MAV_GCS_SYSID = sysid do script.
  batt_low     bateria baixa  (BATT_LOW_VOLT / BATT_LOW_MAH, accao BATT_FS_LOW_ACT)
  batt_crit    bateria critica (BATT_CRT_VOLT / BATT_CRT_MAH, accao BATT_FS_CRT_ACT)

FASES (cada cenario corre em ambas por omissao)
  low   perto da pista e baixo (--low-alt, omissao 150 m) — fase inicial
  high  no fim da espiral     (--high-alt, omissao 2000 m)

O QUE E' VERIFICADO, por teste
  1. aparece o STATUSTEXT esperado;
  2. o modo muda para RTL (ou para a accao configurada);
  3. com RTL_AUTOLAND=4, o FCU salta para o DO_RETURN_PATH_START
     ("Return path started", AP_Mission.cpp:2566) e volta a AUTO;
     com RTL_AUTOLAND=1/2, salta para o DO_LAND_START
     ("Landing sequence start", AP_Mission.cpp:2512);
  4. o item de missao corrente passa a estar dentro do troco de regresso e
     avanca monotonicamente -> a aproximacao segue a trajectoria predefinida.

CODIGOS DE SAIDA
  0 = todos PASS    1 = pelo menos um FAIL    2 = erro de ambiente
"""

from __future__ import annotations

import argparse
import os
import sys
import time

from artex_sitl import (  # noqa: E402
    DEFAULT_URL, MAV_CMD_DO_LAND_START, MAV_CMD_DO_RETURN_PATH_START,
    SIM_DIR, SIM_RC_FAIL_NONE, SIM_RC_FAIL_NO_PULSES,
    SIM_RC_FAIL_PROTOCOL_BIT, SIM_RC_FAIL_THROTTLE_950, Sampler, TestFailure,
    arm, connect, fail, find_default_mission, HeartbeatSender, load_param_file,
    battery_consumed_mah, log, mission_index, param_get, param_set,
    read_mission, reboot_autopilot,
    relax_failsafes, require_file, reset_mission_state, reset_sim_params,
    restore_failsafes, set_mode, set_speedup, set_wind, upload_mission,
    wait_ready, wait_stable,
)

# cenarios corridos por "--scenario all"
SCENARIOS = ["rc_nopulses", "rc_throttle", "rc_protocol", "gcs",
             "batt_low", "batt_crit"]
# verificacoes especiais: so' quando pedidas pelo nome
EXTRA_SCENARIOS = ["fence_during_return"]
ALL_SCENARIOS = SCENARIOS + EXTRA_SCENARIOS
PHASES = ["low", "high"]


def phase_alt(phase, a):
    """'low' -> --low-alt, 'high' -> --high-alt, 'alt:NNN' -> NNN metros."""
    if phase == "low":
        return a.low_alt
    if phase == "high":
        return a.high_alt
    if phase.startswith("alt:"):
        return float(phase[4:])
    raise SystemExit("ERRO: fase invalida '%s'" % phase)

# accoes de FS_LONG_ACTN (ArduPlane/Parameters.cpp:449)
FS_LONG_ACTN_NAME = {0: "Continue", 1: "RTL", 2: "Glide(FBWA)",
                     3: "Parachute", 4: "Auto", 5: "AUTOLAND_or_RTL"}
# accoes de BATT_FS_*_ACT (libraries/AP_BattMonitor + ArduPlane/events.cpp)
BATT_ACT_NAME = {0: "None", 1: "RTL", 2: "Land", 3: "Terminate", 4: "QLand",
                 5: "Loiter_to_QLand", 6: "Loiter_to_QLand", 7: "AUTOLAND_or_RTL"}


# --------------------------------------------------------------------------- #

def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--url", default=DEFAULT_URL)
    p.add_argument("--scenario", default="all",
                   help="lista separada por virgulas, ou 'all' (= %s). "
                        "Fora do 'all', por nome: %s"
                        % (",".join(SCENARIOS), ",".join(EXTRA_SCENARIOS)))
    p.add_argument("--phase", default="all",
                   help="low, high, alt:<metros>, lista separada por virgulas, "
                        "ou 'all' (omissao). Ex.: --phase alt:300,alt:600,alt:1100")
    p.add_argument("--mission", default=None)
    p.add_argument("--variant", default=None)
    p.add_argument("--no-mission", action="store_true",
                   help="usa a missao ja' carregada no FCU")
    p.add_argument("--params", default=None, help="aplica este .param antes de cada teste")
    p.add_argument("--low-alt", type=float, default=150.0)
    p.add_argument("--high-alt", type=float, default=2000.0)
    p.add_argument("--fast-climb", type=float, default=0.0, metavar="MS",
                   help="sobe TECS_CLMB_MAX para este valor APENAS enquanto ganha "
                        "altitude na fase 'high' (nao se esta a testar gradiente "
                        "aqui). Ex.: --fast-climb 5. 0 = desligado.")
    p.add_argument("--observe", type=float, default=120.0,
                   help="segundos de observacao apos a injeccao (omissao: %(default)s)")
    p.add_argument("--climb-timeout", type=float, default=900.0,
                   help="tempo de relogio maximo para chegar a altitude, s")
    p.add_argument("--wind", default=None, metavar="SPD[,DIR[,TURB]]")
    p.add_argument("--no-reboot", action="store_true",
                   help="nao reiniciar o FCU entre cenarios (so' faz sentido com 1 cenario)")
    p.add_argument("--dt", type=float, default=0.5)
    p.add_argument("--climb-speedup", type=float, default=10.0,
                   help="SIM_SPEEDUP enquanto ganha altitude (omissao: %(default)s). "
                        "Nesta janela os failsafes por tempo estao DESLIGADOS.")
    p.add_argument("--inject-speedup", type=float, default=1.0,
                   help="SIM_SPEEDUP na janela de injeccao e observacao "
                        "(omissao: %(default)s). NAO subir: o MAVProxy alimenta o "
                        "RC e o heartbeat de GCS em tempo de relogio, mas o FCU "
                        "conta RC_FS_TIMEOUT/FS_LONG_TIMEOUT em tempo de simulacao; "
                        "acima de ~3x os failsafes por tempo disparam sozinhos.")
    p.add_argument("--out-dir", default=os.path.join(SIM_DIR, "results"))
    p.add_argument("--keep-sim-params", action="store_true")
    return p.parse_args(argv)


# --------------------------------------------------------------------------- #
# Preparacao de voo                                                            #
# --------------------------------------------------------------------------- #

def fly_to(m, s, alt, dt, timeout, fast_climb=0.0,
           climb_speedup=10.0, inject_speedup=1.0):
    """Descola (TAKEOFF ate' 100 m), passa a AUTO e sobe ate' `alt`.

    Durante esta fase o SIM_SPEEDUP sobe e os failsafes por tempo sao
    desligados; no fim repoe-se tudo, para que a injeccao seguinte aconteca
    num ambiente representativo."""
    fs_before = relax_failsafes(m)
    set_speedup(m, climb_speedup)
    # limpa in_landing_sequence/in_return_path de um teste anterior, senao o
    # pre-arm rejeita com "In landing sequence"
    reset_mission_state(m)
    handover = min(100.0, alt * 0.6)
    tk_before = param_get(m, "TKOFF_ALT")
    param_set(m, "TKOFF_ALT", handover)
    set_mode(m, "TAKEOFF")
    arm(m)

    t0 = time.time()
    while time.time() - t0 < timeout:
        s.pump(dt)
        if s.alt is not None and s.alt >= handover * 0.97:
            break
    else:
        raise TestFailure("nao chegou aos %.0f m de entrega" % handover)
    if tk_before is not None:
        param_set(m, "TKOFF_ALT", tk_before)

    set_mode(m, "AUTO")

    clmb_before = None
    if fast_climb > 0 and alt > handover * 1.5:
        clmb_before = param_get(m, "TECS_CLMB_MAX")
        param_set(m, "TECS_CLMB_MAX", fast_climb)
        log("subida acelerada: TECS_CLMB_MAX %.2f -> %.2f (so' para ganhar "
            "altitude; o gradiente e' testado no test_climb_gradient.py)"
            % (clmb_before or 0, fast_climb))

    t0 = time.time()
    last = 0.0
    while time.time() - t0 < timeout:
        s.pump(dt)
        if s.alt is None:
            continue
        if s.alt >= alt * 0.98:
            break
        if time.time() - last > 60:
            last = time.time()
            log("  a subir: %.0f / %.0f m" % (s.alt, alt))
    else:
        raise TestFailure("nao chegou a %.0f m em %.0f s de relogio "
                          "(sobe o --speedup do run_sitl.sh ou usa --fast-climb)" % (alt, timeout))

    if clmb_before is not None:
        param_set(m, "TECS_CLMB_MAX", clmb_before)
        log("TECS_CLMB_MAX reposto a %.2f" % clmb_before)

    # ambiente representativo para a injeccao:
    #   1) voltar a 1x ANTES de reactivar os failsafes por tempo;
    #   2) deixar o FCU ver heartbeats/RC frescos a esse ritmo;
    #   3) so' entao repor os failsafes e esperar que nada dispare sozinho.
    set_speedup(m, inject_speedup)
    for _ in range(int(10 / dt)):
        s.pump(dt)
    restore_failsafes(m, fs_before)
    wait_stable(m, s, dt=dt, quiet=15.0, timeout=180.0, expect_mode="AUTO")
    log("em posicao: alt=%.0f m" % s.alt)


# --------------------------------------------------------------------------- #
# Injeccao de falhas                                                           #
# --------------------------------------------------------------------------- #

def inject(m, scenario, hb: HeartbeatSender | None):
    """Injeta a falha. Devolve (descricao, funcao_de_reposicao, textos_esperados)."""
    if scenario.startswith("rc_"):
        val = {"rc_nopulses": SIM_RC_FAIL_NO_PULSES,
               "rc_throttle": SIM_RC_FAIL_THROTTLE_950,
               "rc_protocol": SIM_RC_FAIL_PROTOCOL_BIT}[scenario]
        thr_fs = param_get(m, "THR_FAILSAFE")
        thr_fs_val = param_get(m, "THR_FS_VALUE")
        rc_fs_to = param_get(m, "RC_FS_TIMEOUT")
        fs_long = param_get(m, "FS_LONG_TIMEOUT")
        log("contexto RC: THR_FAILSAFE=%s THR_FS_VALUE=%s RC_FS_TIMEOUT=%s "
            "FS_LONG_TIMEOUT=%s" % (thr_fs, thr_fs_val, rc_fs_to, fs_long))
        if scenario == "rc_throttle" and (thr_fs or 0) < 1:
            log("  AVISO: THR_FAILSAFE=0 -> este cenario nao pode disparar")
        param_set(m, "SIM_RC_FAIL", val)
        return ("SIM_RC_FAIL=%d" % val,
                lambda: param_set(m, "SIM_RC_FAIL", SIM_RC_FAIL_NONE),
                ["RC Long Failsafe On", "RC Short Failsafe"])

    if scenario == "gcs":
        if (param_get(m, "FS_GCS_ENABL") or 0) < 1:
            raise TestFailure("FS_GCS_ENABL=0 — o failsafe de telemetria esta desligado")
        if hb is None:
            raise TestFailure("cenario 'gcs' precisa do HeartbeatSender")
        hb.stop()
        return ("heartbeats de GCS parados", lambda: hb.start(), ["GCS Failsafe On"])

    if scenario in ("batt_low", "batt_crit"):
        crit = scenario == "batt_crit"
        act_p = "BATT_FS_CRT_ACT" if crit else "BATT_FS_LOW_ACT"
        volt_p = "BATT_CRT_VOLT" if crit else "BATT_LOW_VOLT"
        mah_p = "BATT_CRT_MAH" if crit else "BATT_LOW_MAH"
        act = param_get(m, act_p) or 0
        thr = param_get(m, volt_p) or 0.0
        mah = param_get(m, mah_p) or 0.0
        log("contexto bateria: %s=%s (%s)  %s=%.2f V  %s=%.0f mAh  BATT_LOW_TIMER=%s"
            % (act_p, act, BATT_ACT_NAME.get(int(act), "?"), volt_p, thr,
               mah_p, mah, param_get(m, "BATT_LOW_TIMER")))
        if act < 1:
            raise TestFailure("%s=0 — nenhuma accao configurada para este failsafe" % act_p)

        sim_v_before = param_get(m, "SIM_BATT_VOLTAGE")
        if sim_v_before is None:
            sim_v_before = 12.6
        txt = "is critical" if crit else "is low"
        expected = ["Battery 1 %s" % txt, "Battery %s" % txt]

        if thr > 0:
            # limiar de TENSAO activo: baixar a tensao simulada abaixo dele
            new_v = max(1.0, thr - 0.5)
            param_set(m, "SIM_BATT_VOLTAGE", new_v)
            desc = "SIM_BATT_VOLTAGE=%.2f (< %s=%.2f)" % (new_v, volt_p, thr)

            def restore_fn(_v=float(sim_v_before)):
                param_set(m, "SIM_BATT_VOLTAGE", _v)
        elif mah > 0:
            # failsafe por mAh (e o caso do artex27_sitl.param: BATT_LOW_VOLT=0,
            # BATT_LOW_MAH=8100). Em vez de esperar 8,1 Ah de voo simulado, sobe-se
            # o limiar para pouco acima da capacidade que RESTA.
            #
            # ATENCAO: BATT_LOW_MAH/BATT_CRT_MAH sao capacidade RESTANTE, nao
            # consumida. AP_BattMonitor_Backend.cpp:277 compara
            #     (BATT_CAPACITY - consumed_mah) < BATT_LOW_MAH
            # Logo o gatilho e' `limiar > restante`, nao `limiar < consumido`.
            used = battery_consumed_mah(m) or 0.0
            pack = param_get(m, "BATT_CAPACITY") or 0.0
            remaining = pack - used
            new_mah = remaining + 1.0
            param_set(m, mah_p, new_mah)
            desc = ("%s=%.0f mAh (restante %.0f = %.0f capacidade - %.0f consumido)"
                    % (mah_p, new_mah, remaining, pack, used))

            def restore_fn(_v=float(mah)):
                param_set(m, mah_p, _v)
        else:
            # ambos os limiares desligados: subir o de tensao acima da actual
            v_now = float(sim_v_before)
            param_set(m, volt_p, v_now + 1.0)
            desc = "%s=%.2f (> tensao actual %.2f)" % (volt_p, v_now + 1.0, v_now)

            def restore_fn(_v=float(thr)):
                param_set(m, volt_p, _v)

        return (desc, restore_fn, expected)

    raise TestFailure("cenario desconhecido: %s" % scenario)


# --------------------------------------------------------------------------- #
# Verificacao                                                                  #
# --------------------------------------------------------------------------- #

def check(s, t_inject, expected_texts, rtl_autoland, ret_idx, land_idx,
          fs_action=None):
    """Devolve (passou: bool, linhas de relatorio).

    fs_action: valor de BATT_FS_*_ACT quando o cenario e' de bateria.
    A accao Land (2) NAO se comporta como RTL: handle_battery_failsafe()
    (ArduPlane/events.cpp:287) chama jump_to_landing_sequence(), que salta
    directamente para o DO_LAND_START e ignora o DO_RETURN_PATH_START. E se o
    aviao ja' estiver em AUTO, nao ha' mudanca de modo nenhuma.
    """
    land_action = fs_action is not None and int(fs_action) == 2
    rep, passed = [], True

    hit = next((t for t in expected_texts if s.saw_text(t, since=t_inject)), None)
    if hit:
        rep.append(("PASS", "STATUSTEXT de failsafe visto: '%s'" % hit))
    else:
        passed = False
        rep.append(("FAIL", "nenhum dos STATUSTEXT esperados %s; vistos: %s"
                    % (expected_texts,
                       [x for *_, x in s.texts_since(t_inject)] or "nenhum")))

    modes = s.modes_since(t_inject)
    if any(name == "RTL" for _, name in modes):
        rep.append(("PASS", "modo passou por RTL (sequencia: %s)"
                    % " -> ".join(n for _, n in modes)))
    elif any(name in ("AUTOLAND", "AUTO") for _, name in modes):
        rep.append(("PASS", "modo mudou para %s (sequencia: %s)"
                    % (modes[-1][1], " -> ".join(n for _, n in modes))))
    elif land_action and s.saw_text("Landing sequence start", since=t_inject):
        rep.append(("PASS", "accao Land: ja' estava em AUTO, nao ha' mudanca de modo "
                            "(salto directo para a sequencia de aterragem)"))
    else:
        passed = False
        rep.append(("FAIL", "modo nao mudou apos a falha (sequencia: %s)"
                    % (" -> ".join(n for _, n in modes) or "sem mudancas")))

    # salto para o troco de regresso
    if land_action:
        # Land corta para a aterragem; o RTL_AUTOLAND nao se aplica.
        marker, idx, pname = "Landing sequence start", land_idx, "DO_LAND_START"
    elif int(rtl_autoland) == 4:
        marker, idx, pname = "Return path started", ret_idx, "DO_RETURN_PATH_START"
    elif int(rtl_autoland) in (1, 2):
        marker, idx, pname = "Landing sequence start", land_idx, "DO_LAND_START"
    else:
        marker, idx, pname = None, None, None

    if marker is None:
        rep.append(("WARN", "RTL_AUTOLAND=%s -> sem salto automatico para troco "
                            "de regresso; nada a verificar aqui" % rtl_autoland))
    elif idx is None:
        passed = False
        rep.append(("FAIL", "RTL_AUTOLAND=%s mas a missao nao tem %s"
                    % (rtl_autoland, pname)))
    elif s.saw_text(marker, since=t_inject):
        rep.append(("PASS", "'%s' -> saltou para o %s (item %d)" % (marker, pname, idx)))
        if any(name == "AUTO" for _, name in modes):
            rep.append(("PASS", "voltou a AUTO para seguir a trajectoria predefinida"))
        elif land_action:
            rep.append(("PASS", "manteve-se em AUTO a seguir a sequencia de aterragem"))
        else:
            passed = False
            rep.append(("FAIL", "nao voltou a AUTO depois do salto"))
    else:
        passed = False
        rep.append(("FAIL", "nao apareceu '%s' — o salto para o %s nao aconteceu"
                    % (marker, pname)))

    # a aproximacao segue a trajectoria predefinida: MISSION_CURRENT dentro do
    # troco de regresso e a avancar
    wps = [r["wp"] for r in s.rows if r["t"] >= t_inject and r["wp"] is not None]
    if idx is not None and wps:
        inside = [w for w in wps if w >= idx]
        if inside:
            mono = all(b >= a for a, b in zip(inside, inside[1:]))
            rep.append((("PASS" if mono else "FAIL"),
                        "itens de missao percorridos apos a falha: %s%s"
                        % (sorted(set(inside)), "" if mono else "  (NAO monotonico)")))
            # VERIFICACAO ESPECIAL 2: jump_to_closest_mission_leg() escolhe a
            # perna mais proxima em 3D. Como os LOITER_TO_ALT da descida estao
            # empilhados no mesmo centro, cada perna e' um vector vertical puro
            # -> o item engatado deve depender da ALTITUDE da falha.
            rep.append(("INFO", "REENGATE: primeiro item do percurso de regresso "
                                "engatado = %d (item 188 esta' no %d)"
                        % (inside[0], idx)))
            passed = passed and mono
        else:
            passed = False
            rep.append(("FAIL", "o item de missao corrente nunca entrou no troco "
                                "de regresso (>= %d); ficou em %s" % (idx, sorted(set(wps)))))
    elif idx is not None:
        rep.append(("WARN", "sem MISSION_CURRENT recebido; nao consigo confirmar "
                            "o percurso de aproximacao"))

    return passed, rep


# --------------------------------------------------------------------------- #

def run_fence_during_return(m, a, phase, mission_items, tag):
    """VERIFICACAO ESPECIAL 1 (params/artex27_diff.md secao 4.1, ponto 6).

    O guarda "ja' esta a aterrar" do fence (ArduPlane/fence.cpp:126-129) so'
    reconhece RTL_AUTOLAND 1 e 2, nao o 4. Previsao do agente de parametros:
    uma violacao de fence DURANTE o percurso de regresso volta a chamar
    set_mode(RTL); o RTL encontra "Return path active" e volta a AUTO —
    recupera, mas com um salto de modo.

    Procedimento: subir, disparar perda de RC para entrar no percurso de
    regresso, e so' depois baixar o FENCE_ALT_MAX para debaixo da altitude
    actual (viola a fence de altitude sem mexer na geometria lateral).
    """
    log("")
    log("=" * 66)
    log(" VERIFICACAO ESPECIAL: violacao de fence durante o regresso | FASE %s" % phase)
    log("=" * 66)

    ret_idx = mission_index(mission_items, MAV_CMD_DO_RETURN_PATH_START)
    rtl_autoland = param_get(m, "RTL_AUTOLAND") or 0
    fence_alt_before = param_get(m, "FENCE_ALT_MAX")
    fence_act = param_get(m, "FENCE_ACTION")
    log("RTL_AUTOLAND=%s  FENCE_ENABLE=%s  FENCE_TYPE=%s  FENCE_ACTION=%s  "
        "FENCE_ALT_MAX=%s" % (rtl_autoland, param_get(m, "FENCE_ENABLE"),
                              param_get(m, "FENCE_TYPE"), fence_act, fence_alt_before))

    s = Sampler(m)
    hb = HeartbeatSender(m).start()
    try:
        alt = phase_alt(phase, a)
        fly_to(m, s, alt, a.dt, a.climb_timeout, fast_climb=a.fast_climb,
               climb_speedup=a.climb_speedup, inject_speedup=a.inject_speedup)
        for _ in range(int(10 / a.dt)):
            s.pump(a.dt)

        # 1) entrar no percurso de regresso por failsafe de RC
        t_fs = s.t_sim
        param_set(m, "SIM_RC_FAIL", SIM_RC_FAIL_NO_PULSES)
        log(">>> perda de RC injectada (alt=%.0f m) — a esperar o regresso" % (s.alt or -1))
        t0 = time.time()
        while time.time() - t0 < a.observe:
            s.pump(a.dt)
            if s.saw_text("Return path started", since=t_fs):
                break
        in_return = s.saw_text("Return path started", since=t_fs)
        modes_fs = s.modes_since(t_fs)
        log("depois da perda de RC: %s | 'Return path started'=%s"
            % (" -> ".join(n for _, n in modes_fs) or "(sem mudancas)", in_return))

        # estabilizar dentro do percurso de regresso
        for _ in range(int(20 / a.dt)):
            s.pump(a.dt)

        # 2) violar a fence de ALTITUDE, sem mexer na geometria lateral
        t_fence = s.t_sim
        alt_now = s.alt or alt
        new_max = max(20.0, alt_now - 150.0)
        param_set(m, "FENCE_ALT_MAX", new_max)
        log(">>> FENCE_ALT_MAX %s -> %.0f m (altitude actual %.0f m) = violacao"
            % (fence_alt_before, new_max, alt_now))

        t0 = time.time()
        while time.time() - t0 < a.observe:
            s.pump(a.dt)

        modes_after = s.modes_since(t_fence)
        seq = " -> ".join(n for _, n in modes_after) or "(sem mudancas de modo)"
        wps = sorted({r["wp"] for r in s.rows if r["t"] >= t_fence and r["wp"] is not None})
        breached = s.saw_text("fence", since=t_fence) or s.saw_text("Fence", since=t_fence)
        bounce = any(n == "RTL" for _, n in modes_after) and \
                 any(n == "AUTO" for _, n in modes_after)

        print("")
        print("-" * 66)
        print(" VERIFICACAO 1 — fence durante o regresso (alt %.0f m)" % alt)
        print("-" * 66)
        print("  entrou no percurso de regresso        : %s" % ("SIM" if in_return else "NAO"))
        print("  mensagens de fence apos a violacao    : %s" % ("SIM" if breached else "NAO"))
        print("  sequencia de modos apos a violacao    : %s" % seq)
        print("  itens de missao durante/depois        : %s" % (wps or "n/d"))
        print("  salto RTL->AUTO previsto pelo diff    : %s"
              % ("CONFIRMADO" if bounce else "NAO observado"))
        for t, _sev, txt in s.texts_since(t_fence):
            print("      t=%7.1f s  %s" % (t, txt))
        print("-" * 66)

        base = os.path.join(a.out_dir, "fence_return_%s_%s" % (phase, tag))
        s.write_csv(base + ".csv")
        s.write_events(base + ".events.txt")
        # criterio de PASS: o aviao tem de acabar a seguir a missao (AUTO),
        # nao em RTL "burro" nem em modo nenhum
        final = modes_after[-1][1] if modes_after else None
        passed = in_return and (final in (None, "AUTO"))
        print("  ==> %s" % ("PASS" if passed else "FAIL"))
        return passed
    except TestFailure as e:
        fail("fence_during_return/%s: %s" % (phase, e))
        return False
    finally:
        hb.stop()
        try:
            param_set(m, "SIM_RC_FAIL", SIM_RC_FAIL_NONE)
            if fence_alt_before is not None:
                param_set(m, "FENCE_ALT_MAX", fence_alt_before)
        except Exception as e:
            log("aviso ao reverter: %s" % e)


def run_one(m, a, scenario, phase, mission_items, tag):
    log("")
    log("=" * 66)
    log(" CENARIO %s | FASE %s" % (scenario.upper(), phase))
    log("=" * 66)

    rtl_autoland = param_get(m, "RTL_AUTOLAND") or 0
    fs_long = param_get(m, "FS_LONG_ACTN") or 0
    log("RTL_AUTOLAND=%s  FS_LONG_ACTN=%s (%s)  FS_SHORT_ACTN=%s  "
        "FS_LONG_TIMEOUT=%s  FS_SHORT_TIMEOUT=%s"
        % (rtl_autoland, fs_long, FS_LONG_ACTN_NAME.get(int(fs_long), "?"),
           param_get(m, "FS_SHORT_ACTN"), param_get(m, "FS_LONG_TIMEOUT"),
           param_get(m, "FS_SHORT_TIMEOUT")))

    ret_idx = mission_index(mission_items, MAV_CMD_DO_RETURN_PATH_START)
    land_idx = mission_index(mission_items, MAV_CMD_DO_LAND_START)
    log("missao: %d itens, DO_RETURN_PATH_START=%s, DO_LAND_START=%s"
        % (len(mission_items), ret_idx, land_idx))

    s = Sampler(m)
    hb = HeartbeatSender(m).start()
    # para o failsafe de GCS o FCU tem de reconhecer este script como GCS
    gcs_sysid_before = param_get(m, "MAV_GCS_SYSID")
    if scenario == "gcs":
        param_set(m, "MAV_GCS_SYSID", m.source_system)
        log("MAV_GCS_SYSID %s -> %d (este script passa a ser a GCS)"
            % (gcs_sysid_before, m.source_system))
        time.sleep(3)   # garantir que o FCU ja' viu heartbeats nossos

    restore = None
    try:
        alt = phase_alt(phase, a)
        fly_to(m, s, alt, a.dt, a.climb_timeout,
               fast_climb=(a.fast_climb if alt > a.low_alt * 1.5 else 0.0),
               climb_speedup=a.climb_speedup, inject_speedup=a.inject_speedup)

        t_inject = s.t_sim
        desc, restore, expected = inject(m, scenario, hb)
        log(">>> FALHA INJECTADA: %s   (alt=%.0f m)" % (desc, s.alt or -1))

        t0 = time.time()
        while time.time() - t0 < a.observe:
            s.pump(a.dt)

        # nos cenarios de bateria a accao configurada muda o que se espera
        fs_action = None
        if scenario in ("batt_low", "batt_crit"):
            fs_action = param_get(
                m, "BATT_FS_CRT_ACT" if scenario == "batt_crit" else "BATT_FS_LOW_ACT")

        passed, rep = check(s, t_inject, expected, rtl_autoland,
                            ret_idx, land_idx, fs_action=fs_action)

        print("")
        print("-" * 66)
        print(" %s / %s  (alt de injeccao %.0f m)" % (scenario, phase, alt))
        print("-" * 66)
        for verdict, line in rep:
            print("  %-4s %s" % (verdict, line))
        print("  ==> %s" % ("PASS" if passed else "FAIL"))
        print("-" * 66)

        base = os.path.join(a.out_dir, "fs_%s_%s_%s" % (scenario, phase, tag))
        s.write_csv(base + ".csv")
        s.write_events(base + ".events.txt")
        return passed

    except TestFailure as e:
        fail("%s/%s: %s" % (scenario, phase, e))
        base = os.path.join(a.out_dir, "fs_%s_%s_%s" % (scenario, phase, tag))
        s.write_events(base + ".events.txt")
        return False
    finally:
        hb.stop()
        if restore:
            try:
                restore()
                log("falha revertida")
            except Exception as e:
                log("aviso ao reverter: %s" % e)
        if scenario == "gcs" and gcs_sysid_before is not None:
            try:
                param_set(m, "MAV_GCS_SYSID", gcs_sysid_before)
            except Exception:
                pass


def main(argv=None) -> int:
    a = parse_args(argv)
    os.makedirs(a.out_dir, exist_ok=True)
    tag = time.strftime("%Y%m%d_%H%M%S")

    scenarios = SCENARIOS if a.scenario == "all" else [x.strip() for x in a.scenario.split(",")]
    phases = PHASES if a.phase == "all" else [x.strip() for x in a.phase.split(",")]
    for sc in scenarios:
        if sc not in ALL_SCENARIOS:
            raise SystemExit("ERRO: cenario desconhecido '%s' (opcoes: %s)"
                             % (sc, ",".join(ALL_SCENARIOS)))
    for ph in phases:
        if ph not in PHASES and not ph.startswith("alt:"):
            raise SystemExit("ERRO: fase desconhecida '%s' (opcoes: %s, ou alt:<metros>)"
                             % (ph, ",".join(PHASES)))

    m = connect(a.url)
    wait_ready(m)

    if a.params:
        load_param_file(m, require_file(a.params, "params"))
    if a.wind:
        parts = [float(x) for x in a.wind.split(",")]
        set_wind(m, parts[0], parts[1] if len(parts) > 1 else 180.0,
                 parts[2] if len(parts) > 2 else 0.0)

    # O ficheiro de missao e' sempre necessario: e' dele que saem os indices de
    # DO_RETURN_PATH_START / DO_LAND_START usados na verificacao.
    mission_path = a.mission or find_default_mission(a.variant)
    if a.no_mission:
        items = read_mission(mission_path)
        log("missao NAO carregada (--no-mission); uso %s so' como referencia"
            % mission_path)
    else:
        items = upload_mission(m, mission_path)

    results = {}
    first = True
    try:
        for sc in scenarios:
            for ph in phases:
                if not first and not a.no_reboot:
                    m = reboot_autopilot(m, a.url)
                    if a.params:
                        load_param_file(m, a.params)
                    if a.wind:
                        parts = [float(x) for x in a.wind.split(",")]
                        set_wind(m, parts[0], parts[1] if len(parts) > 1 else 180.0,
                                 parts[2] if len(parts) > 2 else 0.0)
                    items = upload_mission(m, mission_path)
                first = False
                if sc == "fence_during_return":
                    results[(sc, ph)] = run_fence_during_return(m, a, ph, items, tag)
                else:
                    results[(sc, ph)] = run_one(m, a, sc, ph, items, tag)
    finally:
        if not a.keep_sim_params:
            try:
                reset_sim_params(m)
            except Exception as e:
                log("aviso: nao consegui repor os SIM_*: %s" % e)

    print("")
    print("=" * 66)
    print(" RESUMO — failsafes ARTEX 2000 m")
    print("=" * 66)
    for (sc, ph), v in results.items():
        print("  %-12s %-5s  %s" % (sc, ph, "PASS" if v else "FAIL"))
    n_fail = sum(1 for v in results.values() if not v)
    print("=" * 66)
    print(" %d/%d PASS" % (len(results) - n_fail, len(results)))
    print(" resultados em: %s" % a.out_dir)
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
