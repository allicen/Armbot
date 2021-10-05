#!/bin/bash

logPath="$(pwd)/logs/"
logFile="log-$(date +%d-%m-%Y).log"
pidFile="$(pwd)/scripts/run.pid"

function dateTimePrint {
    date="$(date +%d.%m.%Y)"
    time="$(date +%T)"
    echo "[$date $time] — "
}

function printLog {

    if ! [[ -d "$logPath" ]]; then
        mkdir -p "$logPath"
    fi


    if ! [[ -e "$logPath$logFile" ]] ; then
        cd "$logPath"
        > "$logFile"
    fi

    printf "%s%s\n" "$(dateTimePrint)" "$1" >> "$logPath$logFile"
}

function getPidFile {
    echo "$pidFile"
}
