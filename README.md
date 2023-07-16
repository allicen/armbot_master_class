# Мастер-класс по визуализации и симуляции

Требования к оборудованию:
- ОС - Linux любой версии
- Свободное место минимум 4 Gb (docker-образ весит 3.8 Gb + собранный проект 139,6 Mb)
- Установленный docker

Установить Docker с официального сайта или запустить docker-сервер командой <code>dockerd</code>.

## 0. Склонировать проект

<code>git clone https://github.com/allicen/armbot_master_class</code>

Перейти в папку проекта armbot_master_class. Armbot_master_class - название корневой папки проекта. Все пути в инструкции ниже, начинающиеся от armbot_master_class, считаем от начала проекта.

## 1. Настройка окружения

1.1. Docker-образ

Всё необходимое окружение (Ubuntu 18.04 и ROS Melodic) собрано в docker-контейнере. 

1 вариант:

- Скачать docker-образ https://drive.google.com/file/d/1WiPhPMxtE-JzbZK_YnnyPqaBRezsOVfe/view?usp=sharing

- Разархивировать armbot-img.tar.gz: <code>tar -xvf armbot-img.tar.gz</code>

- Загрузить docker-образ робота: <code>sudo docker load < armbot-img.tar</code>

2 вариант:

- Собрать docker-образ <code>sudo docker build -t armbot-img . --network=host --build-arg from=ubuntu:18.04</code>

1.2. Установить переменные среды

Нужно прописать путь до проекта.

Добавьте переменную среды в файл <code>.bashrc</code> (каталог /home): <code>export ARMBOT_PATH='/home/e/ROS/Armbot'</code> (вместо '/home/e/ROS/Armbot' напишите свой путь к корневой папке проекта)

Примените изменения <code>source ~/.bashrc</code>

1.3. Настроить скрипты

- Дать права на запуск скриптов: <code>sudo chmod +x scripts/*sh</code>

При первом запуске скриптов нужно исправить ошибку перед запуском скриптов (преобразование окончаний строк из формата DOS в формат UNIX):

- Выполнить <code>sed -i -e 's/\r$//' "$ARMBOT_PATH/scripts/fix.sh"</code>
- Выполнить <code>./scripts/fix.sh</code>


## 2. Запустить робота

Перейти в корень проекта. Открыть 4 окна терминала.

1 терминал:
- Запустить docker-контейнер с окружением для робота: <code>sudo ./scripts/docker/run_armbot_docker.sh</code>
- Перейти в рабочую директорию <code>cd workspace</code>
- Собрать проект <code>catkin_make</code> (если будут ошибки сборки, можно попробовать собрать пакеты изолированно <code>catkin_make_isolated</code>)
- Прописать пути <code>source devel/setup.bash</code>
- Запустить модель робота <code>roslaunch armbot_description armbot_npoa.rviz.launch</code> (или с моделью клавиатуры
    <code>roslaunch armbot_description armbot_npoa.rviz.launch keyboard:=true</code>)

2 терминал:

- Зайти в docker-контейнер <code>sudo docker exec -ti armbot bash</code>
- Перейти в рабочую директорию <code>cd workspace</code>
- Прописать пути <code>source devel/setup.bash</code>
- Запустить подписчик на движения <code>roslaunch armbot_move move.rviz.launch</code>

3 терминал:

- Запустить публикацию движений <code>./scripts/armbot.sh start</code>


====================================================================

## Описание команд

#### Файл command_description.txt

Пример описания команды:
<pre><code>direct:0.2872 4.83407e-10
right:-0.180411 -0.223463
left:-0.180411 0.223463
topLeft:0.237525 0.133799 0.0463589
</code>
</pre>

На примере последней команды:

  topLeft - название команды
  0.237525 - значение координаты X
  0.133799 - значение координаты Y
  0.0463589 - значение координаты Z (может отсутствовать, тогда берется значение из проекта)

Название команды пишется слева от двоеточия. После двоеточия через пробел указываются координаты. Координата Z необязательная.

#### Файл commands.txt

Пример описания перечня выполняемых команд:
<pre><code>right 6
left 4
</code>
</pre>

Указывается название команды и через пробел значение задержки (в секундах) после выполнения команды.


## Как добавить модель робота

- Подготовить модель в Solidworks:

1. Подготовить модель;
2. Создать сборку из всех деталей (можно создать несколько сборок на каждое звено и одну общую - так будет удобнее работать);
3. Рассчитать системы координат и расставить в модели по всем звеньям;
4. По возможности задать материалы (или можно потом задать физичесакие характеристики каждого звена) - это нужно для симулятора Gazebo;
5. Скачать и установить плагин <a href="http://wiki.ros.org/sw_urdf_exporter">SolidWorks to URDF Exporter</a> (могут возникнуть проблемы, если Solidworks установлен не в папку по умолчанию);
6. Задать links, joints, указать тип соединения, выбрать детали, которые будут включены в каждый link, выгрузить (ВАЖНО!!! Чтобы расчеты в модели велись по текущей программе, надо последний линк задать как "link_grip");
7. Название папки задать как название пакета в проекте;
8. Плагин сгенериует ROS-пакет.


- Перенести пакет в проект.

- Сформировать конфигурацию робота с помощью <a href="http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html">MoveIt Setup Assistant</a>.

Запуск MoveIt Setup Assistant из Docker-контейнера: <code>roslaunch moveit_setup_assistant setup_assistant.launch</code>


## Конфиги робота

В папке /home/armbot-info создайте файл Armbot.txt и укажите в нем нужные конфиги для робота. Содержимое этого файла можно поменять через пользовательский интерфейс.
Если не будет конфигов в файле Armbot.txt, то они будут браться по умолчанию из Armbot/src/armbot_move/src/settings.hpp
Пример файла с поддерживаемыми конфигами:

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