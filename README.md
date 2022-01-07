<h1>Armbot - инструкция</h1>
<h2>1. Настройка окружения</h2>
<p>Установить Docker с официального сайта или запустить docker-сервер командой <code>dockerd</code>.</p>
<p>Перейти в папку проекта Armbot. Armbot - название корневой папки проекта. Все пути в инструкции ниже, начинающиеся от Armbot, считаем от начала проекта.</p>
<p><strong>1.1. Документация:</strong></p>
<p>1.1.1. Перейти в папку с документацией: <code>cd docs</code></p>
<p>1.1.2. Собрать документацию: <code>./gradlew copyClientResources :back:shadowJar build</code> (нужен доступ к интернету)</p>
<p>1.1.3. Перейти в папку: <code>cd back/build/libs</code> (директория создается автоматически при сборке)</p>
<p>1.1.4. Запустить документацию из текущей папки: <code>java -jar back-0.1-all.jar</code> (можно скопировать в любую другую папку и запустить оттуда)</p>
<p><strong>Важно!</strong> По умолчанию в проекте задана Java 11. Поменять версию можно в файле Armbot/docs/back/build.gradle:</p>
<pre><code>java &#123;
  sourceCompatibility = JavaVersion.toVersion("11")
  targetCompatibility = JavaVersion.toVersion("11")
&#125;</code></pre>
<p>Для Java 8 укажите 1.8 вместо 11:</p>
<pre><code>java &#123;
  sourceCompatibility = JavaVersion.toVersion("1.8")
  targetCompatibility = JavaVersion.toVersion("1.8")
&#125;</code></pre>
<p>1.1.5. Открыть в браузере <code>localhost:9080</code></p>
<p><strong>1.2. Робот:</strong></p>
<p>1.2.1. Перейти в корневую папку проекта Armbot</p>
<p>1.2.2. Собрать окружение для робота в docker-контейнер: <code>sudo docker build -t armbot-img . --network=host --build-arg from=ubuntu:18.04</code></p>
<p>1.2.3. Сохранить docker-образ: <code>sudo docker save armbot-img > armbot-img.tar</code></p>
<p>1.2.4. Заархивировать docker-образ: <code>tar -zcvf armbot-img.tar.gz armbot-img.tar</code></p>
<p><strong>1.3. Перенести на целевую машину</strong></p>
<p>1.3.1. Скачать архив armbot-img.tar.gz на флешку. Скопировать на целевую машину</p>
<p>1.3.2. Разархивировать armbot-img.tar.gz: <code>tar -xvf armbot-img.tar.gz</code></p>
<p>1.3.3. Загрузить docker-образ робота: <code>sudo docker load < armbot-img.tar</code></p>
<p><br /></p>

<h2>2. Установить переменные среды</h2>
<p>Нужно прописать путь до проекта.</p>
<p>Добавьте переменную среды в файл <code>.bashrc</code> (каталог /home): <code>export ARMBOT_PATH='/home/e/ROS/Armbot'</code> (вместо '/home/e/ROS/Armbot' напишите свой путь к корневой папке проекта)</p>
<p>Примените изменения <code>source ~/.bashrc</code></p>
<p><br /></p>

<h2>3. Настроить скрипты</h2>
<p>3.1. Дать права на запуск скриптов: <code>sudo chmod +x scripts/*sh</code></p>
<p>3.1*. Исправить ошибку перед запуском скриптов (преобразование окончаний строк из формата DOS в формат UNIX):</p>
<p>Выполнить <code>sed -i -e 's/\r$//' "$ARMBOT_PATH/scripts/fix.sh"</code></p>
<p>Выполнить <code>./scripts/fix.sh</code></p>
<p>* - нужно делать только при первом запуске скриптов</p>
<p><br /></p>

<h2>4. Запустить робота</h2>
<p>0<sup>*</sup>. Открыть скетч <code>Armbot/src/armbot_move/control-arduino/move_arduino/move_arduino.ino</code> в Arduino IDE. Загрузить скетч.</p>
<p><i><sup>*</sup> Пропустить этот шаг, если нужный скетч уже загружен на плату Arduino.</i></p>
<p><strong>4.1. Режим разработки и отладки</strong></p>
<p>Перейти в корень проекта. Открыть 4 окна терминала.</p>
<p>1 терминал:</p>
<ul>
  <li>Запустить docker-контейнер с окружением для робота: <code>sudo ./scripts/docker/run_armbot_docker.sh</code></li>
  <li>Перейти в рабочую директорию <code>cd workspace</code></li>
  <li>Собрать проект <code>catkin_make</code> (если будут ошибки сборки, можно попробовать собрать пакеты изолированно <code>catkin_make_isolated</code>)</li>
  <li>Прописать пути <code>source devel/setup.bash</code></li>
  <li>Запустить модель робота <code>roslaunch armbot_description armbot_npoa.rviz.launch</code> (или с моделью клавиатуры
    <code>roslaunch armbot_description armbot_npoa.rviz.launch keyboard:=true</code>)</li>
</ul>
<p>2 терминал:</p>
<ul>
  <li>Зайти в docker-контейнер <code>sudo docker exec -ti armbot bash</code></li>
  <li>Перейти в рабочую директорию <code>cd workspace</code></li>
  <li>Прописать пути <code>source devel/setup.bash</code></li>
  <li>Запустить подписчик на движения <code>roslaunch armbot_move move.rviz.launch</code></li>
</ul>
<p>3 терминал:</p>
<ul>
  <li>Зайти в docker-контейнер <code>sudo docker exec -ti armbot bash</code></li>
  <li>Перейти в рабочую директорию <code>cd workspace</code></li>
  <li>Прописать пути <code>source devel/setup.bash</code></li>
  <li>Запустить подписчик движений на Arduino <code>rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200</code></li>
</ul>
<p>4 терминал:</p>
<ul>
  <li>Запустить публикацию движений с докером <code>./scripts/armbot.sh start</code> (или запустить робота из веб-интерфейса)</li>
</ul>
<p><strong>4.2. Режим запуска</strong></p>
<p>Выполнить в терминале:</p>
<ul>
  <li>Запустить docker-контейнер с окружением для робота: <code>./scripts/docker/run_armbot_docker.sh</code></li>
  <li>Перейти в рабочую директорию <code>cd workspace</code></li>
  <li>Собрать проект <code>catkin_make</code> (если будут ошибки сборки, можно попробовать собрать пакеты изолированно <code>catkin_make_isolated</code>)</li>
  <li>Прописать пути <code>source devel/setup.bash</code></li>
  <li>Запустить модель робота без визуализации <code>roslaunch armbot_move run.launch</code> (или с визуализацией
    <code>roslaunch armbot_move run.launch gui:=true</code>)</li>
</ul>
<p><br /></p>

<h2>5. Описание команд</h2>
<p><strong>Файл command_description.txt</strong>. Пример описания команды:</p>
<pre><code>direct:0.2872 4.83407e-10
right:-0.180411 -0.223463
left:-0.180411 0.223463
topLeft:0.237525 0.133799 0.0463589
</code>
</pre>
<p>На примере последней команды:</p>
<ul>
  <li>topLeft - название команды</li>
  <li>0.237525 - значение координаты X</li>
  <li>0.133799 - значение координаты Y</li>
  <li>0.0463589 - значение координаты Z (может отсутствовать, тогда берется значение из проекта)</li>
</ul>
<p>Название команды пишется слева от двоеточия. После двоеточия через пробел указываются координаты. Координата Z необязательная.</p>
<p><strong>Файл commands.txt</strong>. Пример описания перечня выполняемых команд:</p>
<pre><code>right 6
left 4
</code>
</pre>
<p>Указывается название команды и через пробел значение задержки (в секундах) после выполнения команды.</p>
<p><br /></p>

<h2>6. Как добавить модель робота</h2>
<p>6.1. Подготовить модель в Solidworks:</p>
<ol>
  <li>Подготовить модель;</li>
  <li>Создать сборку из всех деталей (можно создать несколько сборок на каждое звено и одну общую - так будет удобнее работать);</li>
  <li>Рассчитать системы координат и расставить в модели по всем звеньям;</li>
  <li>По возможности задать материалы (или можно потом задать физичесакие характеристики каждого звена) - это нужно для симулятора Gazebo;</li>
  <li>Скачать и установить плагин <a href="http://wiki.ros.org/sw_urdf_exporter">SolidWorks to URDF Exporter</a> (могут возникнуть проблемы, если Solidworks установлен не в папку по умолчанию);</li>
  <li>Задать links, joints, указать тип соединения, выбрать детали, которые будут включены в каждый link, выгрузить (ВАЖНО!!! Чтобы расчеты в модели велись по текущей программе, надо последний линк задать как "link_grip");</li>
  <li>Название папки задать как название пакета в проекте;</li>
  <li>Плагин сгенериует ROS-пакет.</li>
</ol>
<p>6.2. Перенести пакет в проект.</p>
<p>6.3. Сформировать конфигурацию робота с помощью <a href="http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html">MoveIt Setup Assistant</a>.</p>
<p>Запуск MoveIt Setup Assistant из Docker-контейнера: <code>roslaunch moveit_setup_assistant setup_assistant.launch</code></p>
<p><br /></p>

<h2>7. Обновление версии робота</h2>
<p>Версию робота необходимо поменять в 2х местах:</p>
<ul>
  <li>В проекте: <strong>Armbot/version</strong> (выводится в логи при запуске move, выводится в UI при запуске UI из проекта)</li>
  <li>В UI: <strong>Armbot/docs/back/src/main/resources</strong> (выводится в UI только если не может найти файл $ARMBOT_PATH/version)</li>
</ul>

<p><br /></p>
<h2>8. Конфиги робота</h2>
<p>В папке /home/armbot-info создайте файл Armbot.txt и укажите в нем нужные конфиги для робота. Содержимое этого файла можно поменять через пользовательский интерфейс.</p>
<p>Если не будет конфигов в файле Armbot.txt, то они будут браться по умолчанию из Armbot/src/armbot_move/src/settings.hpp</p>
<p>Пример файла с поддерживаемыми конфигами:</p>
<pre><code># Положение по оси Z
zPositionDefault = 0.06
zPositionDefaultDown = 0.012
# Ориентация по умолчанию
defaultOrientation_x = 0.999979
defaultOrientation_y = 0.00740374
defaultOrientation_z = 7.85675e-05
defaultOrientation_w = -3.01276e-06
# Позиция по умолчанию
defaultPosition_x = 0.105
defaultPosition_y = 0
defaultPosition_z = 0.05
# Сохранять координаты по вебсокету
saveWebSocket = true
# Сохранять координаты в файл
saveToFile = false
</code></pre>