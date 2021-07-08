<h1>Armbot</h1>
<p>Собрать контейнер: <code>docker build -t armbot-img -f /home/e/ROS/Armbot/Dockerfile /home/e/ROS/Armbot --network=host --build-arg from=ubuntu:18.04</code></p>
<p>Сохранить docker-образ в tar-файл:docker <code>save armbot-img > /home/e/ROS/Armbot/armbot-img.tar</code></p>
<p>Запаковать в архив: <code>tar -zcvf armbot-img.tar.gz /home/e/ROS/Armbot/armbot-img.tar</code></p>
<p>Распаковать из архива: <code>tar -xvf armbot-img.tar.gz</code></p>
<p>Загрузить docker-образ с помощью команды: <code>docker load < /home/e/ROS/Armbot/armbot-img.tar</code></p>