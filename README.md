Dernière mise à jour : Décembre 2025 Objectif : Rappel du fonctionnement du repo, de la stratégie Docker et de la gestion hybride des moteurs (avec/sans codeurs) pour le WAVE ROVER.

1. Architecture Infrastructure (DevOps)
Le projet est conteneurisé pour gérer la cross-compilation entre un ordinateur de dev (Mac M2 / PC) et la cible embarquée (Jetson Nano).

ROS Distro : ROS 2 Foxy Fitzroy.

Communication : CycloneDDS (configuré en multicast auto via /etc/cyclonedds.xml pour faire le pont entre le Mac et la Jetson).

Structure Docker
Le Dockerfile est multi-stage pour supporter deux environnements :

Dev (Mac M2/PC) : Cible image-generic. Basée sur ros:foxy. Utilise l'architecture ARM64 native du Mac M2 (pas d'émulation lente).

Prod (Jetson Nano) : Cible image-jetson. Basée sur dustynv/ros:foxy... (L4T r32.7.1). Nécessaire car la Jetson Nano est bloquée sous Ubuntu 18.04, mais on force l'utilisation de conteneurs Foxy.

Commandes rapides (Makefile) :

make dev-up : Lance l'environnement de dev sur Mac.

make jetson-up : Lance le conteneur sur le robot (monte /dev et le code source).

make build : Compile le workspace avec colcon.

2. Hardware : Spécificités du Rover
Ce robot est un WAVE ROVER modifié avec une configuration hybride.

Châssis : 4 roues motrices (4WD), direction par dérapage (Skid-Steering).

Contrôleur Bas Niveau : Carte "General Driver for Robots" (basée sur ESP32).

Moteurs Arrière (Modifiés) : Remplacés par des moteurs avec codeurs (Waveshare DCGM-N20-12V-EN-200RPM). Branchés sur les ports PH2.0 6-pins.

Moteurs Avant (Origine) : Moteurs sans codeurs (200 RPM). Branchés sur les ports PH2.0 2-pins.

Câblage & Synchro :

Sur la carte ESP32, les ports avant et arrière d'un même côté sont connectés en parallèle sur le même driver de puissance.

Conséquence : Impossible de piloter l'avant et l'arrière indépendamment. Ils reçoivent la même tension PWM.

Avantage : Comme tous les moteurs sont des 200 RPM, ils tournent naturellement à la même vitesse (synchronisation matérielle).

3. Stratégie de Contrôle ROS 2
Bien que le robot ait 4 roues, nous le pilotons comme un robot à 2 roues différentielles pour simplifier l'odométrie et la navigation.

A. Le Driver (waveshare_driver)
C'est une SystemInterface ros2_control (C++) à implémenter.

Responsabilité :

Communiquer en Série (JSON) avec l'ESP32.

Exposer seulement 2 joints à ROS : rear_left_wheel_joint et rear_right_wheel_joint.

Write : Envoie la commande de vitesse (ex: {"T":1, "L":0.5, "R":0.5}). L'ESP32 s'occupe du PID sur les moteurs arrière et réplique le PWM sur les moteurs avant.

Read : Lit le retour codeur (ex: {"vL":..., "vR":...}) pour calculer l'odométrie.

B. Le Contrôleur (rover_bringup)
Nous utilisons diff_drive_controller standard.

Config : Il ne connaît que les 2 roues arrière (les seules avec codeurs).

Paramètres : left_wheel_names: ["rear_left_wheel_joint"], right_wheel_names: ["rear_right_wheel_joint"].

C. URDF & Visualisation (rover_description)
Pour que la visualisation sous Rviz soit correcte (les 4 roues tournent), on utilise le système de "mimic" :

XML

<joint name="rear_left_wheel_joint" type="continuous"> ... </joint>

<joint name="front_left_wheel_joint" type="continuous">
    <mimic joint="rear_left_wheel_joint" multiplier="1.0" offset="0.0"/>
</joint>
4. Todo List (Reprise du projet)
Vérification Firmware ESP32 :

Se connecter en série direct à la carte via USB/UART.

Vérifier que l'ESP32 renvoie bien des données JSON (vitesse/position) quand on tourne les roues à la main.

Si non : Flasher un firmware compatible (ex: ugv_base_general avec support encodeur activé).

Implémenter waveshare_system.cpp :

Compléter le squelette dans src/waveshare_driver.

Intégrer une lib série (libserial) et JSON (nlohmann_json).

Faire le mapping : rad/s (ROS) <-> commande JSON (ESP32).

Vérifier le sens de rotation :

S'assurer que Avancer fait tourner les roues avant et arrière dans le même sens.

Si une roue avant tourne à l'envers : Inverser les 2 fils du connecteur moteur avan
