#!/bin/bash

sed -i -e 's/\r$//' scripts/functions.sh
source ./scripts/functions.sh

pidFile="$(getPidFile)"

function killProcess {

    local procPid=$(cat "$pidFile")
    local re='^[0-9]+$'

    if ! [[ $procPid =~ $re ]] ; then
       printLog "Ошибка! PID должен быть числом"
       exit 1
    fi

    # Убить процесс и всех потомков
    kill -- -"$(ps -o pgid= "$procPid" | grep -o [0-9]*)"

    # удалить файл, где записан PID
    rm "$pidFile"

    printLog "Процесс остановлен."
}

case "$1" in
    start)
      printLog "Запускаю процесс..."
      echo $$ > "$pidFile"

      # Заходит в docker
      if [[ "$2" = "docker" ]]; then
        printLog "Запускаю docker..."
        sudo docker exec --tty -i armbot bash -c "cd workspace && sudo chmod +x scripts/*sh &&
                                                sed -i -e 's/\r$//' scripts/get_data.sh &&
                                                ./scripts/get_data.sh $3 $4"
      else
        sudo chmod +x ./scripts/*sh
        ./scripts/get_data.sh "$2" "$3"
      fi
      ;;

    stop)
      printLog "Останавливаю процесс..."
      if [ -f "$pidFile" ]; then
            killProcess
        else
            printLog "Файл с записью PID не существует"
        fi
    ;;

    help)
      echo "
  Запустите скрипт ./run.sh с аргументами.

  1 аргумент — название команды (обязателено). Значения:
    · help — вызов справки;
    · start — запуск скрипта;
    · stop — остановка скрипта;
    · restart — перезапуск скрипта.

  2 аргумент — запуск с докером (необязателено). Значения:
    · docker — запуск скрипта с докером;

  3 аргумент и 4 аргумент — путь до файла со списком команд (необязательно) и путь до файла с описанием команд. Значения:
    · commands=текстовая строка (относительный или абсолютный путь);
    · description=текстовая строка (относительный или абсолютный путь).

  commands — путь до файла со списком команд;
  description — путь до файла с описанием команд.

  Относительный без слеша (например, commands=scripts/commands.txt), абсолютный со слешем (например, commands=/home/user/scripts/commands.txt).
  "
      ;;
	  *)
      echo "Command $1 not supported"
      ;;
esac

