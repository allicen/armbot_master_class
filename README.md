# Мастер-класс по визуализации робота-манипулятора для программирования в ROS

Мастер-класс разбит на несколько частей в зависимости от прогресса выполнения.

Предварительная подготовка:

1) Необходима модель робота в SolidWorks
2) В SolidWorks необходимо установить плагин для экспорта модели робота в URDF: "<a href="http://wiki.ros.org/sw_urdf_exporter">SolidWorks to URDF Exporter</a>" (чтобы плагин заработал, SolidWorks должен быть установлен в папку по умолчанию на диске C)
3) В модели робота в SolidWorks необходимо расставить системы координат
4) Выгрузить модель робота с помощью плагина "SolidWorks to URDF Exporter"

Видео:
- модель робота https://drive.google.com/file/d/1Sa0DZhuXO_XzKxlu9IbdkdT--rkhxbaI/view?usp=sharing
- расстановка системы координат https://drive.google.com/file/d/13UAgYBlZ4exlPWKb4C40eKCoWtPcW3Xc/view?usp=sharing
- пример, как установить систему координат https://drive.google.com/file/d/1t1RHBu-BO24C0cOX4EQU4zvKLbCPdeFC/view?usp=sharing
- формирование и выгрузка URDF-файла https://drive.google.com/file/d/1PEqN2CeuRYTzc6eWTYtP18fKNuXYs_MC/view?usp=sharing

Пример модели робота: 
- https://github.com/maxosprojects/dobot-arm-CAD
- https://grabcad.com/library/dobot-4/details?folder_id=2248436 

Требования к оборудованию:
- ОС - Linux любой версии
- Свободное место минимум 4 Gb (docker-образ весит 3.8 Gb + собранный проект 139,6 Mb)
- Установленный docker

Установить Docker с официального сайта или запустить docker-сервер командой <code>dockerd</code>.

<a name="nav"></a>
## Навигация по разделам

- <a href="https://github.com/allicen/armbot_master_class/tree/step1">Шаг 1 (загрузка модели робота из SolidWorks, добавление в rViz)</a>
- <a href="https://github.com/allicen/armbot_master_class/tree/step2">Шаг 2 (добавление модели робота в Gazebo из URDF, добавление mesh-модели в виде маркера)</a>
- <a href="https://github.com/allicen/armbot_master_class/tree/step3">Шаг 3 (создание мира в Gazebo - встроенные возможности, добавление объектов и текстур)</a>
- <a href="https://github.com/allicen/armbot_master_class/tree/step4">Шаг 4 (управление роботом в rViz и в Gazebo, вывод изображений с камер)</a>


=========================================

## Инструкция 

### 0. Склонировать проект

<code>git clone https://github.com/allicen/armbot_master_class</code>

Перейти в папку проекта armbot_master_class. Armbot_master_class - название корневой папки проекта. Все пути в инструкции ниже, начинающиеся от armbot_master_class, считаем от начала проекта.

### 1. Настройка окружения

1.1. Docker-образ

Всё необходимое окружение (Ubuntu 18.04 и ROS Melodic, Gazebo) собрано в docker-контейнере. 

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

### 2. Запуск робота

Переходите по <a href="#nav">разделам навигации</a>, там будут указаны шаги по запуску управления роботом для реализации того или иного шага.

#### Шаг 2 (добавление модели робота в Gazebo из URDF, добавление mesh-модели в виде маркера)

Перейти в корень проекта. Открыть терминал.

- Запустить docker-контейнер с окружением для робота: <code>sudo ./scripts/docker/run_armbot_docker.sh</code>
- Перейти в рабочую директорию <code>cd workspace</code>
- Собрать проект <code>catkin_make</code> (если будут ошибки сборки, можно попробовать собрать пакеты изолированно <code>catkin_make_isolated</code>)
- Прописать пути <code>source devel/setup.bash</code>
- Запустить модель робота:
  1) просто модель <code>roslaunch armbot_description display.launch</code>
  2) модель с клавиатурой <code>roslaunch armbot_description display.launch keyboard:=true</code>
  3) модель в Gazebo (то, что сгенерировал плагин из SolidWorks) <code>roslaunch armbot_description gazebo.launch</code>


=======================

#### Базовые команды

##### 1 терминал
- Запустить docker-контейнер с окружением для робота: <code>sudo ./scripts/docker/run_armbot_docker.sh</code>
- Перейти в рабочую директорию <code>cd workspace</code>
- Собрать проект <code>catkin_make</code> (если будут ошибки сборки, можно попробовать собрать пакеты изолированно <code>catkin_make_isolated</code>)
- Прописать пути <code>source devel/setup.bash</code>

##### 2 терминал
- Перейти в docker-контейнер <code>sudo docker exec -ti armbot bash</code>
