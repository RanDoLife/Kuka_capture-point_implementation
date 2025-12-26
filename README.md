# KUKA KR3

Клонируйте репозиторий

## Работа с роботом через Docker контейнер в VS Code

1. Подготовка X11 (один раз за сессию)

Для работы GUI-приложений из контейнера необходимо разрешить доступ к X-серверу.
На хосте:

    xhost +local:docker


2. Установите расширение Dev Containers в VS Code
3. Используйте Use View->Command Palette... или Ctrl+Shift+P для открытия палитры команд. Запустите
команду Dev Containers: Reopen in Container
4. Дождитесь сборки контейнера

Дальнейшую работу выполняйте с помощью терминалов в VS Code

5. Выполните следующие команды 

    sudo apt update 

    sudo apt-get install ros-noetic-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-position-controllers ros-noetic-velocity-controllers ros-noetic-ros-controllers ros-noetic-moveit 

    catkin_make

## Подключение к роботу

0. Подключите Ethernet кабель и настройте соединение
![alt text](docs/images/image.png)
ip. 192.168.1.100 255.255.255.0
1. Запустите программу

    roslaunch kuka_rsi_hw_interface test_hardware_interface.launch sim:=false

2. На роботе запустите программу ros_rsi.src. При достижения Attention!!! переведите робота в Auto режим. Нажмите OK. И продолжите запуск программы.
При успешном подключении появится сообщение в первом терминале
3. Запустите move_group для управления роботом с помощью Moveit

    roslaunch real_kuka_kr3_moveit_config move_group.launch

4. Для визуализации запустите rViz (не забудьте прописать source devel/setup.bash)

    rviz

5. В rViz добавьте добавьте панель MotionPlanning. Измените Fixed Frame на base_link

Запуск управления роботом с помощью кода

1. Запустите программу add_scene_constraints.py для построения виртуальных ограждений для робота. Дождитесь их появления в rViz.
Для лучшей видимости измените парметр прозрачности MotionPlanning->Scene Geometry->Scene Alpha.

2. Изучите и запустите демо-программу kuka_point.py

## Тестирование программ в симуляции

1. Запустите demo.launch. Эта программа не привязана к реальному роботу

    roslaunch real_kuka_kr3_moveit_config demo.launch

2. Далее можно запускать любую существующую(kuka_point.py, add_scene_constraints.py и тд) либо написанную вами программу 


# Если хотите установить ПО Basler в контейнер

1.
    sudo apt-get install usbutils
    sudo apt install udev
    sudo apt-get install ros-noetic-camera-info-manager
    sudo apt-get install ros-noetic-diagnostics
    sudo apt-get install ros-noetic-image-geometry
    sudo apt install ros-noetic-roslint
    
    usermod -aG plugdev root

2. Распакуйте архив pylon-5.2.0.13457-x86_64.tar.gz
3. Распакуйте архив и выполните установку по инструкции из файла INSTALL внутри архива.(Понадобится камера)
4. Клонирование необходимых ROS-пакетов. 
Перейдите в каталог src вашего workspace и клонируйте зависимости:

Драйвер Basler (ветка для Pylon 5)
git clone -b pylon5-legacy https://github.com/basler/pylon-ros-camera.git

Общие сообщения/утилиты (зависимости)
git clone https://github.com/dragandbot/dragandbot_common.git

5. Build
catkin_make

4. Запуск и проверка
roslaunch pylon_camera pylon_camera_node.launch
