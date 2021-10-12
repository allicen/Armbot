#!/bin/bash

commandsFile="$ARMBOT_PATH/data/commands.txt"
commandsDescriptionFile="$ARMBOT_PATH/data/command_description.txt"

source "$ARMBOT_PATH/scripts/extended/functions/functions.sh"

function getPath {
  firstChar=$(echo "$1" | cut -c1-1)
  if [ "$firstChar" = '/' ] && [ -f "$1" ]; then
    echo "$1"
  elif [ -f "$ARMBOT_PATH/$1" ]; then
    echo "$ARMBOT_PATH/$1"
  else
    echo NULL
  fi
}

function readArgument {
   if [[ "$(echo "$1" | cut -d "=" -f 1)" = "$2" ]]; then
     echo "$1" | cut -d "=" -f 2
   else
     echo NULL
   fi
}

printLog "Получены адреса файлов: 1) $1, 2) $2"

commandsFirstArg=$(readArgument "$1" 'commands')
commandsSecondArg=$(readArgument "$2" 'commands')
descriptionFirstArg=$(readArgument "$1" 'description')
descriptionSecondArg=$(readArgument "$2" 'description')

if [[ $commandsFirstArg != NULL && $(getPath "$commandsFirstArg") != NULL ]]; then
  commandsFile="$(getPath "$commandsFirstArg")"
elif [[ $commandsSecondArg != NULL && $(getPath "$commandsSecondArg") != NULL ]]; then
  commandsFile=$(getPath "$commandsSecondArg")
fi

if [[ $descriptionFirstArg != NULL && $(getPath "$descriptionFirstArg") != NULL ]]; then
  commandsDescriptionFile="$(getPath "$descriptionFirstArg")"
elif [[ $descriptionSecondArg != NULL && $(getPath "$descriptionSecondArg") != NULL ]]; then
  commandsDescriptionFile="$(getPath "$descriptionSecondArg")"
fi

printLog "Загружено описание команд из файла '$commandsDescriptionFile'"
printLog "Загружена последовательность выполнения команд из файла '$commandsFile'"

declare -A commandsMap

while IFS="" read -r commandDesc || [ -n "$commandDesc" ]
do
	key=$(echo "$commandDesc" | cut -d ":" -f 1)
	value=$(echo "$commandDesc" | cut -d ":" -f 2)

	commandsMap[$key]=$value

done < "$commandsDescriptionFile"

while IFS="" read -r command || [ -n "$command" ]
do
	key=$(echo "$command" | cut -d " " -f 1)
	delay=$(echo "$command" | cut -d " " -f 2)

	source devel/setup.bash
	rosrun armbot_move position _param:="$key ${commandsMap[$key]} $delay"

done < "$commandsFile"

printLog "Заканчиваю работу..."

# Удалиить файл c PID после завершения процесса
rm "$(getPidFile)"

if [ -f "$(getPidFile)" ]; then
    printLog "PID-файл не был удален."
else
    printLog "PID-файл успешно удален."
fi