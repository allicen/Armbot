#!/bin/bash

logFile="$(pwd)/logs/log-$(date +%d-%m-%Y).log"
pidFile="$(pwd)/scripts/run.pid"

function dateTimePrint {
    date="$(date +%d.%m.%Y)"
    time="$(date +%T)"
    echo "[$date $time] — "
}

function printLog {
    printf "%s%s\n" "$(dateTimePrint)" "$1" >> "$logFile"
}

function getPidFile {
    echo "$pidFile"
}