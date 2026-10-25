#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_climb_gradient.py — valida em SITL a subida a 2000 m com gradiente <= 10 %.

O que faz, por ordem:
  1. liga ao SITL (porta de teste udp:127.0.0.1:14560, criada pelo run_sitl.sh);
  2. aplica (opcionalmente) o .param da missao e o vento pedido;
  3. carrega a missao;
  4. simula a FASE MANUAL: descola em modo TAKEOFF ate' --handover-alt (~100 m).
     O TAKEOFF e' um substituto automatizavel da descolagem a' mao; o gradiente
     NAO e' avaliado nesta fase, tal como na realidade (o piloto sobe a subir);
  5. passa a AUTO nessa altitude — o ponto de entrega combinado;
  6. amostra altitude, groundspeed, razao de subida e gradiente instantaneo
     a 2 Hz ate' atingir --target-alt;
  7. FALHA se o gradiente SUSTENTADO (media movel de --window segundos)
     passar de --max-gradient em qualquer ponto;
  8. compara o tempo de subida com o previsto e grava CSV + eventos.

GRADIENTE SOBRE O SOLO
  gradiente = razao_de_subida / groundspeed
  O groundspeed do VFR_HUD ja' inclui o vento, por isso este quociente E' o
  gradiente sobre o solo. Com vento de frente o groundspeed cai e o gradiente
  sobe — e' exactamente o caso que este teste tem de apanhar.

CODIGOS DE SAIDA
  0 = PASS    1 = FAIL (criterio nao cumprido)    2 = erro de ambiente
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

from artex_sitl import (  # noqa: E402
    DEFAULT_URL, DEFAULT_PARAM_FILE, SIM_DIR, TestFailure,
    arm, connect, fail, find_default_mission, load_param_file, log, ok,
    param_get, param_set, relax_failsafes, reset_sim_params, require_file,
    restore_failsafes, set_mode, set_speedup, set_wind, upload_mission,
    wait_ready, Sampler, rolling_mean,
)


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--url", default=DEFAULT_URL,
                   help="endpoint MAVLink do SITL (omissao: %(default)s)")
    p.add_argument("--mission", default=None,
                   help="missao a carregar; omissao: a 1a de src/artex27/missions/")
    p.add_argument("--variant", default=None, help="filtra a missao por substring")
    p.add_argument("--no-mission", action="store_true",
                   help="usa a missao ja' carregada no FCU")
    p.add_argument("--params", default=None,
                   help="aplica este .param antes do teste (omissao: nenhum; "
                        "usa --params %s para o da missao)" % DEFAULT_PARAM_FILE)
    p.add_argument("--target-alt", type=float, default=2000.0,
                   help="altitude relativa alvo, m (omissao: %(default)s)")
    p.add_argument("--handover-alt", type=float, default=100.0,
                   help="altitude de passagem para AUTO, m (omissao: %(default)s)")
    p.add_argument("--max-gradient", type=float, default=0.10,
                   help="gradiente maximo admissivel (omissao: %(default)s = 10 %%)")
    p.add_argument("--window", type=float, default=60.0,
                   help="janela de media movel para 'sustentado', em segundos de "
                        "SIMULACAO (omissao: %(default)s)")
    p.add_argument("--settle", type=float, default=60.0,
                   help="segundos de SIMULACAO apos a entrada em AUTO que sao "
                        "ignorados (transitorio) (omissao: %(default)s)")
    p.add_argument("--wind", default=None, metavar="SPD[,DIR[,TURB]]",
                   help="vento simulado; DIR = de onde vem, graus true")
    p.add_argument("--timeout", type=float, default=3600.0,
                   help="tempo maximo de simulacao, s de relogio (omissao: %(default)s)")
    p.add_argument("--dt", type=float, default=0.5, help="periodo de amostragem, s")
    p.add_argument("--speedup", type=float, default=10.0,
                   help="SIM_SPEEDUP durante o teste (omissao: %(default)s). "
                        "A subida nominal demora ~24 min de simulacao; a 10x sao "
                        "~2,5 min de relogio. Reposto no fim.")
    p.add_argument("--keep-failsafes", action="store_true",
                   help="NAO desligar THR_FAILSAFE/FS_GCS_ENABL durante a subida. "
                        "Por omissao sao desligados: com SIM_SPEEDUP > ~5 o MAVProxy "
                        "nao alimenta o RC nem o heartbeat de GCS depressa o "
                        "suficiente em tempo de simulacao e o aviao entra em "
                        "failsafe sozinho. Os failsafes sao testados no "
                        "test_failsafes.py, nao aqui.")
    p.add_argument("--out-dir", default=os.path.join(SIM_DIR, "results"),
                   help="onde gravar CSV e eventos")
    p.add_argument("--tag", default=None, help="sufixo dos ficheiros de saida")
    p.add_argument("--keep-sim-params", action="store_true",
                   help="nao repor os SIM_* no fim (por omissao repoe)")
    return p.parse_args(argv)


def main(argv=None) -> int:
    a = parse_args(argv)
    os.makedirs(a.out_dir, exist_ok=True)
    tag = a.tag or time.strftime("%Y%m%d_%H%M%S")

    m = connect(a.url)
    wait_ready(m)

    # ------------------------------------------------------------ setup --
    if a.params:
        load_param_file(m, require_file(a.params, "params"))

    if a.wind:
        parts = [float(x) for x in a.wind.split(",")]
        set_wind(m, parts[0], parts[1] if len(parts) > 1 else 180.0,
                 parts[2] if len(parts) > 2 else 0.0)
    wind_spd = param_get(m, "SIM_WIND_SPD") or 0.0
    wind_dir = param_get(m, "SIM_WIND_DIR") or 0.0

    if not a.no_mission:
        mission = a.mission or find_default_mission(a.variant)
        upload_mission(m, mission)
    else:
        mission = "(ja' carregada no FCU)"

    v_cruise = param_get(m, "AIRSPEED_CRUISE")
    clmb_max = param_get(m, "TECS_CLMB_MAX")
    pitch_max = param_get(m, "PTCH_LIM_MAX_DEG")
    tecs_pitch_max = param_get(m, "TECS_PITCH_MAX")
    log("parametros relevantes: AIRSPEED_CRUISE=%s  TECS_CLMB_MAX=%s  "
        "TECS_PITCH_MAX=%s  PTCH_LIM_MAX_DEG=%s"
        % (v_cruise, clmb_max, tecs_pitch_max, pitch_max))

    # previsao analitica (pior caso de groundspeed: vento todo de frente)
    gs_worst = max(1.0, (v_cruise or 15.0) - wind_spd)
    grad_pred = (clmb_max or 0.0) / gs_worst
    t_pred = (a.target_alt - a.handover_alt) / (clmb_max or 1e-9)
    log("previsao: gradiente teorico no pior caso = %.1f %% "
        "(TECS_CLMB_MAX %.2f / (V %.1f - W %.1f))"
        % (100 * grad_pred, clmb_max or 0, v_cruise or 0, wind_spd))
    log("previsao: tempo de subida = %.0f s (%.1f min) para %.0f m"
        % (t_pred, t_pred / 60.0, a.target_alt - a.handover_alt))

    s = Sampler(m)
    tkoff_alt_before = None
    speedup_before = None
    fs_before = {}

    try:
        if a.speedup and a.speedup != 1:
            speedup_before = set_speedup(m, a.speedup)
        if not a.keep_failsafes:
            fs_before = relax_failsafes(m)
        # -------------------------------------------- fase "manual" (TAKEOFF) --
        log("--- FASE 1: descolagem (substituto automatico da fase manual) ---")
        tkoff_alt_before = param_get(m, "TKOFF_ALT")
        param_set(m, "TKOFF_ALT", a.handover_alt)
        set_mode(m, "TAKEOFF")
        arm(m)

        t0 = time.time()
        while time.time() - t0 < a.timeout:
            s.pump(a.dt)
            if s.alt is not None and s.alt >= a.handover_alt * 0.97:
                break
            if time.time() - t0 > 600:
                raise TestFailure("nao chegou a %.0f m em 600 s de relogio "
                                  "(sobe o --speedup do run_sitl.sh)" % a.handover_alt)
        log("entrega a %.1f m — a passar para AUTO" % (s.alt or -1))

        # ------------------------------------------------------- fase AUTO --
        log("--- FASE 2: AUTO, subida a %.0f m ---" % a.target_alt)
        set_mode(m, "AUTO")
        s.pump(a.dt)
        t_auto = s.t_sim          # tempo de SIMULACAO da entrega
        i_auto = len(s.rows)

        t0 = time.time()
        reached = False
        while time.time() - t0 < a.timeout:
            s.pump(a.dt)
            if s.alt is None:
                continue
            if s.alt >= a.target_alt * 0.995:
                reached = True
                break
            # progresso a cada ~60 s de relogio
            if s.rows and len(s.rows) % max(1, int(60 / a.dt)) == 0:
                r = s.rows[-1]
                log("  alt=%7.1f m  gs=%5.1f m/s  vz=%+5.2f m/s  grad=%5.2f %%"
                    % (r["alt_rel"], r["gs"], r["climb"], 100 * r["grad"]))
        t_climb = s.t_sim - t_auto          # segundos de SIMULACAO
        sp = s.speedup_measured

        # ---------------------------------------------------------- analise --
        log("--- ANALISE ---")
        rows = [r for r in s.rows[i_auto:]
                if r["t"] >= t_auto + a.settle and r["alt_rel"] >= a.handover_alt]
        if len(rows) < 10:
            raise TestFailure("amostras insuficientes depois do transitorio (%d)" % len(rows))

        grads = [r["grad"] for r in rows]
        dt_sim = ((rows[-1]["t"] - rows[0]["t"]) / max(1, len(rows) - 1)) or a.dt
        n_win = max(2, int(round(a.window / dt_sim)))
        if n_win > len(rows) // 2:
            n_win = max(2, len(rows) // 2)
            log("  NOTA  janela reduzida para %.0f s de simulacao (%d amostras): "
                "a subida foi curta demais para a janela pedida"
                % (n_win * dt_sim, n_win))
        sust = rolling_mean(grads, n_win)
        valid_inst = [g for g in grads if not math.isnan(g)]
        valid_sust = [(g, r) for g, r in zip(sust, rows) if not math.isnan(g)]

        g_inst_max = max(valid_inst) if valid_inst else float("nan")
        g_sust_max, r_worst = max(valid_sust, key=lambda x: x[0]) if valid_sust else (float("nan"), None)
        g_mean = sum(valid_inst) / len(valid_inst) if valid_inst else float("nan")
        vz_mean = sum(r["climb"] for r in rows) / len(rows)
        gs_mean = sum(r["gs"] for r in rows) / len(rows)
        as_mean = sum(r["airspeed"] or 0 for r in rows) / len(rows)
        alt0, alt1 = rows[0]["alt_rel"], rows[-1]["alt_rel"]

        print("")
        print("=" * 66)
        print(" RESULTADO — subida ARTEX 2000 m")
        print("=" * 66)
        print("  missao                  : %s" % mission)
        print("  vento simulado          : %.1f m/s de %.0f deg" % (wind_spd, wind_dir))
        print("  AIRSPEED_CRUISE         : %s m/s" % v_cruise)
        print("  TECS_CLMB_MAX           : %s m/s" % clmb_max)
        print("  --")
        print("  altitude inicial / final: %.0f m -> %.0f m (rel. home)" % (alt0, alt1))
        print("  alvo atingido           : %s" % ("SIM" if reached else "NAO"))
        print("  tempo de subida (AUTO)  : %.0f s = %.1f min de SIMULACAO"
              "   [previsto %.1f min]" % (t_climb, t_climb / 60.0, t_pred / 60.0))
        print("  speedup real medido     : %s"
              % ("%.1fx" % sp if sp else "n/d"))
        print("  razao de subida media   : %.2f m/s" % vz_mean)
        print("  groundspeed media       : %.1f m/s   (airspeed %.1f m/s)" % (gs_mean, as_mean))
        print("  --")
        print("  gradiente medio         : %.2f %%" % (100 * g_mean))
        print("  gradiente instantaneo   : %.2f %% (max)" % (100 * g_inst_max))
        print("  gradiente SUSTENTADO    : %.2f %% (max, janela %.0f s sim, "
              "%d amostras)" % (100 * g_sust_max, n_win * dt_sim, n_win))
        print("  resolucao da amostragem : 1 amostra / %.1f s de simulacao "
              "(dt=%.2f s de relogio x speedup)" % (dt_sim, a.dt))
        if r_worst:
            print("      pior ponto: t=%.0f s  alt=%.0f m  gs=%.1f m/s  vz=%+.2f m/s"
                  % (r_worst["t"], r_worst["alt_rel"], r_worst["gs"], r_worst["climb"]))
        print("  limite                  : %.2f %%" % (100 * a.max_gradient))
        print("=" * 66)

        csv_path = os.path.join(a.out_dir, "climb_%s.csv" % tag)
        evt_path = os.path.join(a.out_dir, "climb_%s.events.txt" % tag)
        s.write_csv(csv_path)
        s.write_events(evt_path)

        # --------------------------------------------------------- vereditos --
        passed = True
        if not reached:
            fail("nao atingiu %.0f m dentro do timeout" % a.target_alt)
            passed = False
        else:
            ok("atingiu %.0f m" % a.target_alt)

        if math.isnan(g_sust_max) or g_sust_max > a.max_gradient:
            if math.isnan(g_sust_max):
                fail("sem janela sustentada valida: a subida produziu %d amostras "
                     "uteis, menos do que a janela de %.0f s pedida. Baixa o "
                     "--window ou sobe o --target-alt." % (len(rows), a.window))
            else:
                fail("gradiente sustentado %.2f %% > limite %.2f %%"
                     % (100 * g_sust_max, 100 * a.max_gradient))
            passed = False
        else:
            ok("gradiente sustentado %.2f %% <= %.2f %%"
               % (100 * g_sust_max, 100 * a.max_gradient))

        # informativo: picos instantaneos sao normais em transicoes de WP
        if g_inst_max > a.max_gradient:
            log("  NOTA  houve picos instantaneos ate' %.2f %% (nao sustentados); "
                "normal em transicoes de waypoint." % (100 * g_inst_max))

        if reached and t_pred > 0:
            dev = abs(t_climb - t_pred) / t_pred
            if dev > 0.25:
                log("  NOTA  tempo de subida desviou %.0f %% do previsto "
                    "(%.1f vs %.1f min) — rever TECS_CLMB_MAX ou THR_MAX."
                    % (100 * dev, t_climb / 60.0, t_pred / 60.0))
            else:
                ok("tempo de subida dentro de 25 % do previsto")

        print("")
        print("VEREDITO: %s" % ("PASS" if passed else "FAIL"))
        return 0 if passed else 1

    except TestFailure as e:
        fail(str(e))
        s.write_csv(os.path.join(a.out_dir, "climb_%s.csv" % tag))
        s.write_events(os.path.join(a.out_dir, "climb_%s.events.txt" % tag))
        print("\nVEREDITO: FAIL")
        return 1
    finally:
        try:
            if tkoff_alt_before is not None:
                param_set(m, "TKOFF_ALT", tkoff_alt_before)
        except Exception:
            pass
        try:
            restore_failsafes(m, fs_before)
        except Exception as e:
            log("aviso: nao consegui repor os failsafes: %s" % e)
        try:
            if speedup_before is not None:
                set_speedup(m, speedup_before)
        except Exception as e:
            log("aviso: nao consegui repor o SIM_SPEEDUP: %s" % e)
        if not a.keep_sim_params:
            try:
                reset_sim_params(m)
            except Exception as e:
                log("aviso: nao consegui repor os SIM_*: %s" % e)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except KeyboardInterrupt:
        sys.exit(130)
