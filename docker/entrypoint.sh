#!/usr/bin/env bash
# Entrypoint: carrega o ambiente (ROS + overlay do workspace + ArduPilot) e
# executa o comando pedido.

source /usr/local/bin/atlas-env

# sem DISPLAY -> modo headless por omissao
if [ -z "${DISPLAY:-}" ]; then
    export HEADLESS="${HEADLESS:-1}"
fi

exec "$@"
