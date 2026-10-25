#!/usr/bin/env bash
# run_sitl.sh — arranca o ArduPlane SITL para a missao ARTEX 2000 m.
#
# Corre DENTRO do container atlas-sim:humble (e la' que vivem o sim_vehicle.py,
# o MAVProxy e o pymavlink). O compose e' o docker/docker-compose.yml deste
# workspace (imagem atlas-sim:humble, working_dir /home/ros/atlas_ws), NAO o
# docker-compose.yaml da raiz (servico 'atlas', imagem 'caixinha').
#
#   cd ~/git_projects/atlas_ws/docker
#   docker compose run --rm dev src/artex27/tools/sim/run_sitl.sh --headless
#
# Ou, com um shell ja aberto (docker compose run --rm dev bash):
#
#   src/artex27/tools/sim/run_sitl.sh --headless
#
# SE O MAVROS FOR CARREGAR A MISSAO (ros2 launch artex27 artex_mission.launch.py)
# E' OBRIGATORIO --no-mavproxy: o MAVProxy e o MAVROS colidem no protocolo de
# missao (MISSION_ACK: INVALID_SEQUENCE / Mission upload timeout). Sequencia:
#
#   ./run_sitl.sh --headless --no-mavproxy --no-mission --no-fence   # term. 1
#   ap-mavros tcp://127.0.0.1:5760                                   # term. 2
#   ros2 launch artex27 artex_mission.launch.py                      # term. 3
#
# Sem MAVROS, este script carrega missao e geofence sozinho pelo SERIAL1
# (tcp:127.0.0.1:5762), canal proprio onde nao ha colisao com o MAVProxy.
#
# Ver ./run_sitl.sh --help.
set -euo pipefail

ORIG_ARGS=("$@")

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# sim/ vive em src/artex27/tools/sim -> a raiz do pacote esta dois niveis acima.
ARTEX_DIR="$(cd "${SIM_DIR}/../.." && pwd)"
WS_DIR="$(cd "${ARTEX_DIR}/../.." && pwd)"   # atlas_ws

# ---------------------------------------------------------------- defaults --
VEHICLE_FRAME="plane"        # modelo de fisica nativo do SITL (sem Gazebo)
LOCATION="ARTEX_RUNWAY"
PARAM_FILE="${ARTEX_DIR}/params/artex27_sitl.param"
MISSION=""                   # vazio -> escolhe a 1a missao de src/artex27/missions/
FENCE="${ARTEX_DIR}/missions/artex27_geofence.waypoints"
LOAD_FENCE=1
MISSION_VARIANT=""
WIND=""                      # "SPD[,DIR[,TURB]]"
SPEEDUP=1
INSTANCE=0
HEADLESS=0
NO_MAVPROXY=0
DAEMON=0
WIPE=1
LOAD_PARAMS=1
LOAD_MISSION=1
DRY_RUN=0
WORKDIR=""
EXTRA_OUTS=()
EXTRA_ARGS=()

usage() {
  cat <<'USAGE'
run_sitl.sh — ArduPlane SITL para a missao ARTEX 2000 m (SITL nativo, sem Gazebo)

USO
  ./run_sitl.sh [opcoes] [-- args extra para o sim_vehicle.py]

OPCOES
  -m, --mission FICHEIRO   missao a carregar (.waypoints | .txt | .plan).
                           Por omissao: a 1a de src/artex27/missions/.
      --variant NOME       filtra a missao por substring do nome do ficheiro
                           (ex.: --variant espiral). Ignorado se -m for dado.
      --no-mission         nao carregar missao nenhuma.
  -F, --fence FICHEIRO     geofence de inclusao no formato legado FENCE_POINT
                           (omissao: src/artex27/missions/artex27_geofence.waypoints).
                           Com FENCE_TYPE a incluir o bit2, SEM esta fence o
                           pre-arm falha com "Polygon fence(s) invalid".
      --no-fence           nao carregar geofence.
  -p, --params FICHEIRO    .param a aplicar (omissao:
                           src/artex27/params/artex27_sitl.param).
      --no-params          nao aplicar .param (so' os defaults do frame).
  -L, --location NOME      local de arranque (omissao: ARTEX_RUNWAY, que e' onde
                           a missao poe o HOME). Definidos em sim/locations.txt:
                           ARTEX_RUNWAY, ARTEX (centro do circulo), ARTEX_RUNWAY_137.
                           Tambem aceita os de Tools/autotest/locations.txt.
  -w, --wind SPD[,DIR[,TURB]]
                           vento simulado: SIM_WIND_SPD / SIM_WIND_DIR / SIM_WIND_TURB.
                           DIR = direccao DE ONDE vem o vento, graus true (omissao 180).
                           Ex.: -w 5          -> 5 m/s de 180
                                -w 5,270      -> 5 m/s de oeste
                                -w 8,270,1.5  -> idem com turbulencia
  -s, --speedup N          aceleracao da simulacao (omissao 1). NAO subir aqui:
                           os scripts de teste mexem no SIM_SPEEDUP so' nas fases
                           em que e' seguro (ver o aviso que este script imprime).
  -f, --frame FRAME        modelo do SITL (omissao: plane, o aviao generico
                           nativo; o `ap-sitl plane` usa gazebo-zephyr). Nenhum
                           dos dois e' a aeronave do ARTEX: do SITL transfere a
                           LOGICA (geometria, sequencia, failsafes, fence), nao
                           os numeros absolutos de desempenho. Ver
                           `sim_vehicle.py --list-frame ArduPlane`.
  -I, --instance N         instancia do SITL (omissao 0). Desloca todas as portas +10N.
  -d, --dir DIR            diretorio de trabalho do SITL (eeprom.bin, logs/).
                           Omissao: src/artex27/tools/sim/run/<instancia>.
      --headless           sem --map e sem --console do MAVProxy. Implica
                           --daemon (sem terminal, o MAVProxy interactivo morre).
      --daemon             MAVProxy sem shell interactiva (para correr em
                           segundo plano / CI). As saidas UDP continuam a existir.
      --no-mavproxy        sem MAVProxy; MAVLink cru em tcp:127.0.0.1:5760.
                           OBRIGATORIO se for o MAVROS a carregar a missao
                           (artex_mission): com MAVProxy os dois colidem no
                           protocolo de missao. O MAVROS liga-se assim:
                             ap-mavros tcp://127.0.0.1:5760
                           Sem MAVProxy nao ha injeccao de RC, logo os
                           failsafes de radio deixam de ser representativos.
      --no-wipe            NAO apagar a eeprom (por omissao apaga, para que o
                           .param pegue de forma deterministica).
      --out URL            saida MAVLink extra (repetivel), ex. udp:127.0.0.1:14550
      --dry-run            so' imprime o comando que iria correr.
  -h, --help               esta ajuda.

PORTAS (instancia 0; +10 por instancia)
  tcp:127.0.0.1:5762    <- SERIAL1 do SITL. E' AQUI que os scripts de teste ligam.
                           Canal proprio => o protocolo de missao nao colide com
                           o MAVProxy. Existe sempre, com ou sem MAVProxy.
  tcp:127.0.0.1:5760    <- SERIAL0: consola do SITL / MAVProxy
  udp:127.0.0.1:14550   <- MAVROS / QGroundControl (via MAVProxy)
  udp:127.0.0.1:14560   <- saida extra (MAVProxy); NAO usar para missoes

EXEMPLOS
  # tudo por omissao: missao + geofence + params do ARTEX, sem GUI
  ./run_sitl.sh --headless

  # com 5 m/s de vento de oeste
  ./run_sitl.sh --headless -w 5,270

  # variante de pouso manual
  ./run_sitl.sh --headless --variant manual_land

  # com mapa e consola do MAVProxy (precisa de X; `xhost +local:docker` no host)
  ./run_sitl.sh

  # para ser o MAVROS a carregar a missao (ver cabecalho do ficheiro)
  ./run_sitl.sh --headless --no-mavproxy --no-mission --no-fence
USAGE
}

# ------------------------------------------------------------------ parsing --
while [[ $# -gt 0 ]]; do
  case "$1" in
    -m|--mission)   MISSION="$2"; shift 2 ;;
    --variant)      MISSION_VARIANT="$2"; shift 2 ;;
    --no-mission)   LOAD_MISSION=0; shift ;;
    -F|--fence)     FENCE="$2"; shift 2 ;;
    --no-fence)     LOAD_FENCE=0; shift ;;
    -p|--params)    PARAM_FILE="$2"; shift 2 ;;
    --no-params)    LOAD_PARAMS=0; shift ;;
    -L|--location)  LOCATION="$2"; shift 2 ;;
    -w|--wind)      WIND="$2"; shift 2 ;;
    -s|--speedup)   SPEEDUP="$2"; shift 2 ;;
    -f|--frame)     VEHICLE_FRAME="$2"; shift 2 ;;
    -I|--instance)  INSTANCE="$2"; shift 2 ;;
    -d|--dir)       WORKDIR="$2"; shift 2 ;;
    --headless)     HEADLESS=1; DAEMON=1; shift ;;
    --daemon)       DAEMON=1; shift ;;
    --no-mavproxy)  NO_MAVPROXY=1; shift ;;
    --no-wipe)      WIPE=0; shift ;;
    --out)          EXTRA_OUTS+=("$2"); shift 2 ;;
    --dry-run)      DRY_RUN=1; shift ;;
    -h|--help)      usage; exit 0 ;;
    --)             shift; EXTRA_ARGS=("$@"); break ;;
    *) echo "run_sitl.sh: opcao desconhecida '$1' (ver --help)" >&2; exit 2 ;;
  esac
done

# ------------------------------------------------------------ pre-requisitos --
die() { echo "ERRO: $*" >&2; exit 1; }

# o sim_vehicle.py faz cd para o ARDUPILOT_HOME: todos os caminhos tem de ser
# absolutos antes de la' chegarem.
abspath() { python3 -c 'import os,sys; print(os.path.abspath(sys.argv[1]))' "$1"; }
[[ -n "${PARAM_FILE}" ]] && PARAM_FILE="$(abspath "${PARAM_FILE}")"
[[ -n "${MISSION}"    ]] && MISSION="$(abspath "${MISSION}")"
[[ -n "${FENCE}"      ]] && FENCE="$(abspath "${FENCE}")"
[[ -n "${WORKDIR}"    ]] && WORKDIR="$(abspath "${WORKDIR}")" 

if ! command -v sim_vehicle.py >/dev/null 2>&1; then
  cat >&2 <<EOF
ERRO: sim_vehicle.py nao esta no PATH.

Este script corre DENTRO do container atlas-sim:humble, onde o atlas-env poe
\$ARDUPILOT_HOME/Tools/autotest no PATH. A partir do host, na pasta docker/
do workspace (a do docker-compose.yml com a imagem atlas-sim:humble):

    cd <raiz-do-workspace>/docker
    docker compose run --rm dev src/artex27/tools/sim/run_sitl.sh ${ORIG_ARGS[*]:-}

(Se preferires correr no host, precisas de: pymavlink, pexpect, MAVProxy, e do
 ArduPilot compilado para SITL — ver README.md, seccao "Pre-requisitos".)
EOF
  exit 1
fi

: "${ARDUPILOT_HOME:=${WS_DIR}/src/ardupilot}"
[[ -d "${ARDUPILOT_HOME}/Tools/autotest" ]] \
  || die "ARDUPILOT_HOME invalido: ${ARDUPILOT_HOME}"

BINARY="${ARDUPILOT_HOME}/build/sitl/bin/arduplane"
REBUILD_FLAG="-N"     # -N = nao reconstruir; o binario ja existe neste workspace
if [[ ! -x "${BINARY}" ]]; then
  echo ">> binario SITL ausente (${BINARY}); o sim_vehicle.py vai compilar." >&2
  echo "   Compilacao de raiz: ~10-20 min. Equivalente manual:" >&2
  echo "     cd ${ARDUPILOT_HOME} && ./waf configure --board sitl && ./waf plane" >&2
  REBUILD_FLAG=""
fi

# locations do ARTEX sem tocar no submodulo: o sim_vehicle.py le primeiro o
# ficheiro apontado por ARDUPILOT_LOCATIONS (Tools/autotest/sim_vehicle.py,
# parse_locations()), e so' depois o Tools/autotest/locations.txt.
export ARDUPILOT_LOCATIONS="${SIM_DIR}/locations.txt"
[[ -f "${ARDUPILOT_LOCATIONS}" ]] || die "sim/locations.txt em falta"

if [[ ${LOAD_PARAMS} -eq 1 ]]; then
  [[ -f "${PARAM_FILE}" ]] || die "ficheiro de parametros nao encontrado:
    ${PARAM_FILE}
  E' produzido pelo agente de PARAMETROS do projeto ARTEX.
  Alternativas: -p <outro.param>  ou  --no-params"
fi

if [[ ${LOAD_MISSION} -eq 1 && -z "${MISSION}" ]]; then
  MDIR="${ARTEX_DIR}/missions"
  [[ -d "${MDIR}" ]] || die "diretorio de missoes nao existe: ${MDIR}
  E' produzido pelo agente de MISSAO do projeto ARTEX.
  Alternativas: -m <ficheiro>  ou  --no-mission"
  shopt -s nullglob
  CANDS=()
  for f in "${MDIR}"/*"${MISSION_VARIANT}"*.waypoints "${MDIR}"/*"${MISSION_VARIANT}"*.plan; do
    [[ "${f,,}" == *fence* ]] && continue      # a geofence nao e' uma missao
    CANDS+=("$f")
  done
  shopt -u nullglob
  [[ ${#CANDS[@]} -gt 0 ]] || die "sem ficheiros de missao em ${MDIR}${MISSION_VARIANT:+ para a variante '${MISSION_VARIANT}'}.
  Alternativas: -m <ficheiro>  ou  --no-mission"
  MISSION="${CANDS[0]}"
  [[ ${#CANDS[@]} -gt 1 ]] && echo ">> varias missoes encontradas; escolhida: ${MISSION}" >&2
fi
if [[ ${LOAD_MISSION} -eq 1 ]]; then
  [[ -f "${MISSION}" ]] || die "missao nao encontrada: ${MISSION}"
fi
if [[ ${LOAD_FENCE} -eq 1 && ! -f "${FENCE}" ]]; then
  die "geofence nao encontrada:
    ${FENCE}
  E' produzida pelo agente de MISSAO do projeto ARTEX.
  Com FENCE_TYPE a incluir o bit2 (inclusao), o pre-arm falha sem ela.
  Alternativas: -F <ficheiro>  ou  --no-fence (e por FENCE_TYPE=1)"
fi

# ----------------------------------------------------------------- comando --
TEST_PORT=$(( 14560 + INSTANCE * 10 ))
GCS_PORT=$(( 14550 + INSTANCE * 10 ))
SERIAL1_PORT=$(( 5762 + INSTANCE * 10 ))
WORKDIR="${WORKDIR:-${SIM_DIR}/run/${INSTANCE}}"
mkdir -p "${WORKDIR}"

ARGS=(-v ArduPlane -f "${VEHICLE_FRAME}" -L "${LOCATION}"
      -I "${INSTANCE}" --speedup "${SPEEDUP}" --use-dir "${WORKDIR}")
[[ -n "${REBUILD_FLAG}" ]] && ARGS+=("${REBUILD_FLAG}")
[[ ${WIPE} -eq 1 ]] && ARGS+=(-w)
[[ ${LOAD_PARAMS} -eq 1 ]] && ARGS+=(--add-param-file="${PARAM_FILE}")

# vento -> parametros SIM_* carregados ja' no arranque do binario
if [[ -n "${WIND}" ]]; then
  IFS=, read -r W_SPD W_DIR W_TURB <<< "${WIND}"
  ARGS+=(-P "SIM_WIND_SPD=${W_SPD}")
  ARGS+=(-P "SIM_WIND_DIR=${W_DIR:-180}")
  ARGS+=(-P "SIM_WIND_TURB=${W_TURB:-0}")
fi

if [[ ${NO_MAVPROXY} -eq 1 ]]; then
  ARGS+=(--no-mavproxy)
  echo ">> sem MAVProxy: SERIAL0 em tcp:127.0.0.1:$(( 5760 + INSTANCE * 10 ))"
  echo ">> ATENCAO: sem MAVProxy nao ha' injeccao de RC (o MAVProxy alimenta o"
  echo "   RC do SITL por UDP 127.0.0.1:$(( 5501 + INSTANCE * 10 )), sim_vehicle.py:1144)."
  echo "   Os cenarios de failsafe de radio deixam de ser representativos."
else
  ARGS+=(--out="udp:127.0.0.1:${TEST_PORT}" --out="udp:127.0.0.1:${GCS_PORT}")
  for o in "${EXTRA_OUTS[@]:-}"; do [[ -n "$o" ]] && ARGS+=(--out="$o"); done
  [[ ${HEADLESS} -eq 0 ]] && ARGS+=(--map --console)
  # --daemon: MAVProxy sem shell interactiva. Sem isto, lancado em segundo
  # plano (stdin fechado) o MAVProxy sai de imediato e leva o SITL com ele.
  [[ ${DAEMON} -eq 1 ]] && ARGS+=(--mavproxy-args=--daemon)
fi
[[ ${#EXTRA_ARGS[@]} -gt 0 ]] && ARGS+=("${EXTRA_ARGS[@]}")

echo "=============================================================="
echo " ARTEX 2000 m — ArduPlane SITL"
echo "   ArduPilot      : ${ARDUPILOT_HOME}"
echo "   frame          : ${VEHICLE_FRAME}   (SITL nativo, sem Gazebo)"
echo "   location       : ${LOCATION}   (${ARDUPILOT_LOCATIONS})"
echo "   parametros     : $( [[ ${LOAD_PARAMS} -eq 1 ]] && echo "${PARAM_FILE}" || echo '(nenhum)' )"
echo "   missao         : $( [[ ${LOAD_MISSION} -eq 1 ]] && echo "${MISSION}" || echo '(nenhuma)' )"
echo "   geofence       : $( [[ ${LOAD_FENCE} -eq 1 ]] && echo "${FENCE}" || echo '(nenhuma)' )"
echo "   vento          : ${WIND:-0 (sem vento)}"
echo "   speedup        : ${SPEEDUP}x"
echo "   dir de trabalho: ${WORKDIR}   (eeprom.bin, logs/)"
echo "   porta de teste : tcp:127.0.0.1:${SERIAL1_PORT}  (SERIAL1; usar nos testes)"
echo "=============================================================="
echo ">> sim_vehicle.py ${ARGS[*]}"

if [[ "${SPEEDUP%%.*}" -gt 5 ]] 2>/dev/null; then
  cat >&2 <<'WARN'
!! AVISO: speedup > 5.
   O MAVProxy alimenta o RC e o heartbeat de GCS em tempo de RELOGIO, mas o
   FCU conta RC_FS_TIMEOUT / FS_LONG_TIMEOUT em tempo de SIMULACAO. Com
   speedup alto o aviao entra em "RC Long Failsafe" / "GCS Failsafe" sozinho,
   sem haver falha nenhuma.
   Preferir: speedup 1 aqui, e deixar os scripts de teste mexerem no
   SIM_SPEEDUP so' nas fases em que isso e' seguro (eles ja' fazem isso).
WARN
fi

if [[ ${DRY_RUN} -eq 1 ]]; then
  exit 0
fi

# carregamento da missao: em segundo plano, assim que o SITL responder.
# (o sim_vehicle.py nao sabe carregar missoes; o -m/--mavproxy-args e' partido
#  por espacos em sim_vehicle.py:1164, por isso "--cmd=wp load X" nao funciona.)
if [[ ${LOAD_MISSION} -eq 1 || ${LOAD_FENCE} -eq 1 ]]; then
  (
    sleep 2
    if [[ ${LOAD_FENCE} -eq 1 ]]; then
      python3 "${SIM_DIR}/artex_sitl.py" fence \
          --url "tcp:127.0.0.1:${SERIAL1_PORT}" \
          --fence-file "${FENCE}" --timeout 120 \
        && echo ">> geofence carregada: ${FENCE}" \
        || echo "!! falhou a geofence; no MAVProxy: fence load ${FENCE}"
    fi
    if [[ ${LOAD_MISSION} -eq 1 ]]; then
      python3 "${SIM_DIR}/artex_sitl.py" upload \
          --url "tcp:127.0.0.1:${SERIAL1_PORT}" \
          --mission "${MISSION}" --timeout 120 \
        && echo ">> missao carregada: ${MISSION}" \
        || echo "!! falhou a missao; no MAVProxy: wp load ${MISSION}"
    fi
  ) &
fi

cd "${ARDUPILOT_HOME}"
exec sim_vehicle.py "${ARGS[@]}"
