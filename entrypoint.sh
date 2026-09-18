#!/bin/bash

set -e

source /opt/ros/humble/setup.bash

echo "Provided arguments: $@"

if [ -f ~/.profile ] && [ "$0" = "-bash" ]; then
    source ~/.profile
fi


exec $@
