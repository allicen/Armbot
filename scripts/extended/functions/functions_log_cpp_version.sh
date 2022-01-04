#!/bin/bash

# Запись в лог версии ПО

source "$ARMBOT_PATH/scripts/extended/functions/functions.sh"

versionFile="$ARMBOT_PATH/version"

if [[ -e "$versionFile" ]]; then
    version=$(<$versionFile)
    printLog "$1Версия Armbot: $version"
else
    printLog "$1Файла с версией не существует"
fi