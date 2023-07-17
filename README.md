# Мастер-класс по визуализации робота-манипулятора для программирования в ROS

Мастер-класс разбит на несколько частей в зависимости от прогресса выполнения.

Предварительная подготовка:

1) Необходима модель робота в SolidWorks
2) В SolidWorks необходимо установить плагин для экспорта модели робота в URDF: "<a href="http://wiki.ros.org/sw_urdf_exporter">SolidWorks to URDF Exporter</a>" (чтобы плагин заработал, Solidworks должен быть установлен в папку по умолчанию на диске C)
3) В модели робота в SolidWorks необходимо расставить системы координат
4) Выгрузить модель робота с помощью плагина "SolidWorks to URDF Exporter"

Видео, как сделать предыдущие шаги: (будет позднее).

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

- <a href="https://github.com/allicen/armbot_master_class/tree/step1">Шаг 1 (загрузка модели робота из SolidWorks)</a>
- <a href="https://github.com/allicen/armbot_master_class/tree/step2">Шаг 2 (добавление модели робота в rViz)</a>
- <a href="https://github.com/allicen/armbot_master_class/tree/step3">Шаг 3 (добавление модели робота в Gazebo из URDF, добавление mesh-модели в виде маркера)</a>
- <a href="https://github.com/allicen/armbot_master_class/tree/step4">Шаг 4 (создание мира в Gazebo - встроенные возможности, добавление объектов и текстур)</a>
- <a href="https://github.com/allicen/armbot_master_class/tree/step5">Шаг 5 (пакет MoveIt!, создание конфигов робота)</a>
- <a href="https://github.com/allicen/armbot_master_class/tree/step6">Шаг 6 (управление роботом в rViz и в Gazebo)</a>
- <a href="https://github.com/allicen/armbot_master_class/tree/step7">Шаг 7 (добавление сенсоров в Gazebo, получение и обработка данных с камер на OpenCV)</a>


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