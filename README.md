Quanser Qube ROS 2 Control Project

Dette repository-et inneholder en komplett ROS 2‑basert pipeline for å beskrive, simulere og styre en Quanser Qube – med både mock‑hardware (GenericSystem) og støtte for ekte Arduino‑tilkobling via qube_driver.

Innhold

qube_ws/
├── src/
│   ├── qube_driver/         # ROS 2 Control hardware plugin (mock + Arduino)
│   ├── qube_description/    # URDF/Xacro-modeller for Qube
│   ├── qube_bringup/        # Launch-filer og konfigurasjon
│   └── qube_controller/     # PID-kontroller for velocity-styring
└── README.md                # Denne dokumentasjonsfilen


Forutsetninger

    Ubuntu 22.04 eller lignende

    ROS 2 Jazzy (installert via sudo apt install ros-jazzy-desktop)

    ros-jazzy-ros2-control og ros-jazzy-ros2-controllers
    sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers

    colcon og standard ROS 2–verktøy tilgjengelig i PATH

Sette opp workspace

    Opprett workspace og klon pakkene
    mkdir -p ~/qube_ws/src
    cd ~/qube_ws/src

    Hent qube_driver fra GitHub
    git clone https://github.com/adamleon/qube_driver.git

    Klon eller kopier egne pakker (qube_description, qube_bringup, qube_controller) inn i src/

    Gå tilbake til workspace-roten
    cd ~/qube_ws

Bygge

colcon build --symlink-install
source install/setup.bash

Kjøre / Visualisere modellen

Kun statisk modell (ingen control) ros2 launch qube_description view_qube.launch.py

    Starter joint_state_publisher, robot_state_publisher og RViz

    Visualiserer svart kube + rød disk + hvit peker i RViz

Full Bring‑up (simulert hardware + PID‑kontroll)

ros2 launch qube_bringup bringup.launch.py

Dette gjør:

    robot_state_publisher (URDF fra controlled_qube.urdf.xacro)

    ros2_control_node med mock‑hardware (GenericSystem) konfigurasjon fra qube_driver/config/joint_controllers.yaml

    velocity_controller + joint_state_broadcaster (lastes og aktiveres)

    qube_controller (PID‑node som abonnerer på /joint_states og publiserer hastighets‑pådrag på /velocity_controller/commands)

    RViz med ferdig oppsatt visnings‑layout (view.rviz)

Endre setpoint live

ros2 param set /qube_controller setpoint 1.57

Se både /velocity_controller/command og /joint_states:

ros2 topic echo /velocity_controller/command -n 5
ros2 topic echo /joint_states -n 5

I RViz vil disken rotere mot det nye setpoint.

Kjøre mot ekte hardware (Arduino)

    Koble Arduino med Qube‑firmware på USB (f.eks. /dev/ttyUSB0).

    I bringup.launch.py, sett simulation-argument til false:
    <xacro:arg name="simulation" default="false"/>

    Launch som vanlig:
    ros2 launch qube_bringup bringup.launch.py

Samme PID‑node og controller‐setup brukes, men reell qube_driver/ArduinoHardware eksekverer mot Arduino.

Pakkeoversikt

qube_driver

    Inneholder mock_components/GenericSystem og ArduinoHardware plugins

    config/joint_controllers.yaml konfigurerer velocity_controller og joint_state_broadcaster

    launch/qube_driver.launch.py starter ros2_control_node + spawners

qube_description

    urdf/qube.macro.xacro – makrodefinisjon av svart kube + rød disk + hvit peker

    urdf/qube.urdf.xacro – enkel scene med world‑link + base_link → world joint

    launch/view_qube.launch.py for visualisering

qube_bringup

    urdf/controlled_qube.urdf.xacro – inkluderer Qube makro + ROS 2 control makro

    launch/bringup.launch.py – setter sammen robot_state_publisher, ros2_control_node, PID‑node og RViz

qube_controller

    controller.py – en ROS 2‑node med P/PI(D) regulator

    Publiserer Float64MultiArray på /velocity_controller/commands

    Abonnerer på /joint_states
