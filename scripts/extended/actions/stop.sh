#!/bin/bash

sed -i -e 's/\r$//' "$ARMBOT_PATH/scripts/extended/functions/functions.sh"
source "$ARMBOT_PATH/scripts/extended/functions/functions.sh"

pidFile="$(getPidFile)"

function killProcess {

    local procPid=$(cat "$pidFile")
    local re='^[0-9]+$'

    if ! [[ $procPid =~ $re ]] ; then
       printLog "Ошибка! PID должен быть числом"
       exit 1
    fi

    # Убить процесс и всех потомков
    pkill -P "$procPid"

    # удалить файл, где записан PID
    rm "$pidFile"

    printLog "Процесс остановлен."
}

if [ -f "$pidFile" ]; then
      killProcess
else
    printLog "Файл с записью PID не существует"
fi