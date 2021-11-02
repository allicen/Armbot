<h1>Armbot</h1>
<h3>Запустить документацию</h3>
<p>1. Установить Docker с официального сайта или запустить docker-сервер командой <code>dockerd</code>.</p>
<p>2. Перейти в папку с документацией: <code>cd docs</code>.</p>
<p>3. Собрать документацию в docker-контейнер: <code>docker build -t docs-img .</code> (нужен доступ к интернету).</p>
<p>4. Сохранить docker-образ в папку docker-img: <code>docker save docs-img > docker-img/docs-img.tar</code>.</p>
<p>5. Заархивировать docker-образ: <code>tar -zcvf docker-img/docs-img.tar.gz docker-img/docs-img.tar</code>.</p>
<p>6. Скачать архивы docs-img.tar.gz и armbot-img.tar.gz на флешку. Скопировать на целевую машину.</p>
<p>7. Разархивировать docs-img.tar.gz: <code>tar -xvf docs-img.tar.gz</code>.</p>
<p>8. Загрузить docker-образ документации: <code>docker load < docs-img.tar</code></p>
<p>9. Перейти в папку проекта Armbot.</p>
<p>10. Дать права на запуск скриптов: <code>sudo chmod +x scripts/*sh</code>.</p>
<p>11. Исправить ошибку переда запуском скриптов (преобразование окончаний строк из формата DOS в формат UNIX): 
<code>sed -i -e 's/\r$//' scripts/docker/run_docs_docker.sh</code>.</p>
<p>12. Запустить docker-контейнер с документацией: <code>./scripts/docker/run_docs_docker.sh</code></p>
<p>13. Открыть документацию в браузере по адресу <code>localhost</code>.</p>
<p>14. Собрать JAR-файл из папки docs <code>./gradlew build :back:shadowJar copyClientResources</code></p>
<p>15. Запустить JAR-файл <code>java -jar back-0.1-all.jar</code></p>
<p><br /></p>
<h3>Запустить робота</h3>
<p>Как запустить робота, описано в документации во вкладке "Инструкция".</p>
<p>Отправка вебсокета <code>wscat -c ws://localhost:9080/ws/coordinate</code>, следующая строка - просто 3 числа через пробел.</p>