#!/bin/bash

sed -i -e 's/\r$//' "$ARMBOT_PATH/scripts/extended/functions/functions.sh"
source "$ARMBOT_PATH/scripts/extended/functions/functions.sh"

pidFile="$(getPidFile)"

case "$1" in
   start)
       printLog "Запускаю процесс..."

       # Заходит в docker
       if [[ "$2" = "docker" ]]; then
           printLog "Запускаю docker..."
           sudo docker exec --tty -i armbot bash -c "cd workspace && sudo chmod +x scripts/*sh &&
                                                     /workspace/scripts/docker/run_in_docker.sh $pidFile $3 $4"
       else
           echo $$ > "$pidFile"
           sudo chmod +x ./scripts/*sh
           "$ARMBOT_PATH/scripts/extended/actions/get_data.sh" "$2" "$3"
       fi
   ;;

   stop)
       printLog "Останавливаю процесс..."

       if [[ "$2" = 'docker' ]]; then
           printLog "Запускаю docker..."
           sudo docker exec --tty -i armbot bash -c "cd workspace && sudo chmod +x scripts/*sh &&
                                                   sed -i -e 's/\r$//' /workspace/scripts/extended/actions/stop.sh &&
                                                   /workspace/scripts/extended/actions/stop.sh"
       else
           "$ARMBOT_PATH/scripts/extended/actions/stop.sh"
       fi
   ;;

   help)
       echo "
 Запустите скрипт ./armbot.sh с аргументами.

 1 аргумент — название команды (обязателено). Значения:
   · help — вызов справки;
   · start — запуск скрипта;
   · stop — остановка скрипта.

 2 аргумент (для команд 'start' и 'stop') — запуск с докером (необязателено). Значения:
   · docker — запуск скрипта с докером;

 3 аргумент и 4 аргумент (для команды 'start') — путь до файла со списком команд (необязательно) и путь до файла с описанием команд. Значения:
   · commands=текстовая строка (относительный или абсолютный путь);
   · description=текстовая строка (относительный или абсолютный путь).

 commands — путь до файла со списком команд;
 description — путь до файла с описанием команд.

 Относительный без слеша (например, commands=scripts/commands.txt), абсолютный со слешем (например, commands=/home/user/scripts/commands.txt).
 "
     ;;
	   *)
       echo "Command $1 not supported
Help command: ./scripts/arnbot.sh help"
       ;;
esac

