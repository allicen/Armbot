#!/bin/bash

# Исправление скриптов

fixFilesInDirectory() {
    local directoryActive="$1"
    if [[ -d "$directoryActive" ]]; then
        # перебрать файлы
        for entry in "$directoryActive"/*
        do
           if [[ -f "$entry" ]]; then
              sed -i -e 's/\r$//' "$entry"
              echo "Исправлен файл: $entry"
           fi
        done

        # перейти к следующей директории
        for entry in "$directoryActive"/*
        do
           if [[ -d "$entry" ]]; then
              fixFilesInDirectory "$entry"
           fi
        done
    fi
}

fixFilesInDirectory "$ARMBOT_PATH/scripts"

