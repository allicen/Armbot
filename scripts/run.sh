#!/bin/bash

commandsFile="$(pwd)/scripts/commands.txt"
commandsDescriptionFile="$(pwd)/scripts/command_description.txt"

declare -A commandsMap=()

case "$1" in
	start) 
		echo "Starting the process..."

		while read commandDesc
		do
			key=$(echo $commandDesc | cut -d ":" -f 1)
			value=$(echo $commandDesc | cut -d ":" -f 2)

			commandsMap[$1]=$key
			commandsMap[$key]=$value

		done < $commandsDescriptionFile

		while read command
		do
			key=$(echo $command | cut -d " " -f 1)
			delay=$(echo $command | cut -d " " -f 2)

			source devel/setup.bash
			rosrun armbot_move position _param:="$key ${commandsMap[$key]} $delay"

		done < $commandsFile
		;;
	*)
        echo "Command $1 not supported"
        ;;
esac

