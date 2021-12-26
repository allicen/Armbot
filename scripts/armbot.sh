#!/bin/bash

# Основной скрипт запуска
# Запуск со своими файлами: ./scripts/armbot.sh start description=/home/e/armbot-info/launch/command_description_1019123837.txt commands=/home/e/armbot-info/launch/commands_1019123837.txt
# Запуск скрипта возможен с аргументами: ./scripts/armbot.sh start|stop|help

if [[ -z "$2" || $2 != false ]]; then
  docker=true
fi

[[ $docker = true ]] && dockerArg=docker || dockerArg="none"

# Пути до файлов могут передаваться во 2, 3, 4 параметре
if [[ $2 != NULL && $2 != false && $3 != NULL ]]; then
  fileArgOne="$2"
  fileArgTwo="$3"
else
  fileArgOne="$3"
  fileArgTwo="$4"
fi

if [[ -z "$1" ]]; then
    echo "No argument supplied"
else
    "$ARMBOT_PATH/scripts/extended/actions/run.sh" "$1" "$dockerArg" "$fileArgOne" "$fileArgTwo"
fi
