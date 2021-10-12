#!/bin/bash

if [[ "$2" = 'docker' ]]; then
   ARMBOT_PATH='/workspace'
fi

logPath="$ARMBOT_PATH/logs/"
logFile="log-$(date +%d-%m-%Y).log"
pidFile="$ARMBOT_PATH/scripts/extended/run.pid"

sudo chmod ugo+rwx "$logPath$logFile"

function dateTimePrint {
   date="$(date +%d.%m.%Y)"
   time="$(date +%T)"
   echo "[$date $time] — "
}

function printLog {

   if ! [[ -d "$logPath" ]]; then
       sudo mkdir -p "$logPath"
   fi

   if ! [[ -e "$logPath$logFile" ]] ; then
       cd "$logPath"
       sudo > "$logFile"
   fi

   sudo printf "%s%s\n" "$(dateTimePrint)" "$1" >> "$logPath$logFile"
}

function getPidFile {
   echo "$pidFile"
}
