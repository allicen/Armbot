#!/bin/bash

# Основной скрипт запуска

if [[ -z "$2" ]]; then
  docker=true
fi

[[ $docker = true ]] && dockerArg=docker || dockerArg=""

if [[ -z "$1" ]]; then
    echo "No argument supplied"
else
    "$ARMBOT_PATH/scripts/extended/actions/run.sh" "$1" "$dockerArg"
fi