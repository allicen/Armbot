<h1>Armbot</h1>
<h2>Запустить документацию</h2>
<p><code>docker run -d --name docs -p 80:80 docs</code></p>
<p><br /></p>
<hr>
<h2>Запустить робота</h2>
<p>1 терминал: <code>roslaunch armbot_move armbot.rviz.launch</code> (модель)</p>
<p>2 терминал: <code>roslaunch armbot_move move.rviz.launch</code> (подписчик на движения)</p>
<p>3 терминал: <code>./scripts/run.sh start</code> (публикатор движений)</p>
<p><br /></p>
<hr>
<p><br /></p>
<p>Собрать контейнер: <code>docker build -t armbot-img -f /home/e/ROS/Armbot/Dockerfile /home/e/ROS/Armbot --network=host --build-arg from=ubuntu:18.04</code></p>
<p>Сохранить docker-образ в tar-файл:docker <code>docker save armbot-img > /home/e/ROS/Armbot/armbot-img.tar</code></p>
<p>Запаковать в архив: <code>tar -zcvf armbot-img.tar.gz /home/e/ROS/Armbot/armbot-img.tar</code></p>
<p>Распаковать из архива: <code>tar -xvf armbot-img.tar.gz</code></p>
<p>Загрузить docker-образ с помощью команды: <code>docker load < /home/e/ROS/Armbot/armbot-img.tar</code></p>

<h3>Configure your catkin workspace (MoveIt!)</h3>
<pre>
catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
</pre>

<h3>Linux</h3>
<p>Удалить папку с содержимым: <code>rm -r devel</code></p>

<h3>Дать права на docker</h3>
<p>Дать права на выполнение скриптов: <code>sudo chmod +x docker/*sh</code></p>
<p>Создать группу docker: <code>sudo groupadd docker</code></p>
<p>Добавить пользователя в группу docker: <code>sudo usermod -aG docker $USER</code></p>
<p>Применить изменения: <code>newgrp docker</code></p>

<h3>Передача данных</h3>
<p>Запуск из корня проекта (папка Armbot).</p>
<p>Без докера: <code>./scripts/run.sh start</code></p>
<p>С докером: <code>./scripts/run.sh start docker</code></p>
<p><br /></p>
<h3>Документация</h3>
<p>Папка <code>docs</code>.</p>
<p>Собрать docker: <code>docker build -t docs .</code></p>
<p>Запустить docker: <code>docker run -d --name docs -p 80:80 docs</code></p>
<p>Загрузить образ docker: <code>docker load < docker-img/docs.tar</code></p>
<p>Перейти в браузер: <code>http://localhost:80</code></p>