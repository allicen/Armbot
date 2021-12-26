#!/bin/bash

# Основной скрипт запуска
# Запуск со своими файлами: ./scripts/armbot.sh start description=/home/e/armbot-info/launch/command_description_1019123837.txt commands=/home/e/armbot-info/launch/commands_1019123837.txt
# Запуск скрипта возможен с аргументами: ./scripts/armbot.sh start|stop|help

if [[ -z "$2" ]]; then
  docker=true
fi

[[ $docker = true ]] && dockerArg=docker || dockerArg=""

if [[ -z "$1" ]]; then
    echo "No argument supplied"
else
    "$ARMBOT_PATH/scripts/extended/actions/run.sh" "$1" "$dockerArg" "$3" "$4"
fi
