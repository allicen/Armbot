#!/bin/bash

commandsFile="$(pwd)/data/commands.txt"
commandsDescriptionFile="$(pwd)/data/command_description.txt"

declare -A commandsMap

while IFS="" read -r commandDesc || [ -n "$commandDesc" ]
do
	key=$(echo $commandDesc | cut -d ":" -f 1)
	value=$(echo $commandDesc | cut -d ":" -f 2)

	commandsMap[$key]=$value

done < $commandsDescriptionFile

while IFS="" read -r command || [ -n "$command" ]
do
	key=$(echo $command | cut -d " " -f 1)
	delay=$(echo $command | cut -d " " -f 2)

	source devel/setup.bash
	rosrun armbot_move position _param:="$key ${commandsMap[$key]} $delay"

done < $commandsFile