#!/bin/bash

case "$1" in
	start) 
		echo "Starting the process..."

		# Заходит в docker
		sudo docker exec --tty -i armbot bash -c "cd workspace && sudo chmod +x scripts/*sh && ./scripts/get_data.sh"
		;;
	*)
        echo "Command $1 not supported"
        ;;
esac

