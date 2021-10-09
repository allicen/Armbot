#!/bin/bash

sed -i -e 's/\r$//' "$ARMBOT_PATH/scripts/functions.sh"
source "$ARMBOT_PATH/scripts/functions.sh"

pidFile="$(getPidFile)"

function killProcess {

    local procPid=$(cat "$pidFile")
    local re='^[0-9]+$'

    if ! [[ $procPid =~ $re ]] ; then
       printLog "Ошибка! PID должен быть числом"
       exit 1
    fi

    # Убить процесс и всех потомков
    kill -- -"$(ps -o pgid="$procPid" | grep -o [0-9]*)"

    # удалить файл, где записан PID
    rm "$pidFile"

    printLog "Процесс остановлен."
}

if [ -f "$pidFile" ]; then
      killProcess
else
    printLog "Файл с записью PID не существует"
fi