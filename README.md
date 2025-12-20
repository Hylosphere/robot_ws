
# 🤖 Wave Rover - ROS 2 Foxy Project

> **Projet :** Stack de navigation autonome pour robot Wave Rover (4WD).
> **Architecture :** ROS 2 Foxy Fitzroy sur Jetson Nano & Mac M2 (Docker).
> **Dernière mise à jour :** Décembre 2025

---

## 📖 Vue d'ensemble

Ce dépôt contient le code source et l'infrastructure DevOps pour piloter un robot **WaveShare Wave Rover** modifié. Le projet est conçu pour gérer la cross-compilation transparente entre un environnement de développement (Mac M2 / PC) et la cible embarquée (NVIDIA Jetson Nano).

### 🏗️ Architecture Infrastructure (DevOps)

Le projet est entièrement conteneurisé pour contourner les limitations de l'OS de la Jetson Nano (Ubuntu 18.04).

1.  **💻 Environnement Dev (Mac M2 / PC)**
    * **Cible Docker :** `image-generic`
    * **Base :** `ros:foxy` (Ubuntu 20.04)
    * **Avantage :** Utilise l'architecture **ARM64 native** des Mac Apple Silicon (compilation ultra-rapide).

2.  **🚀 Environnement Prod (Jetson Nano)**
    * **Cible Docker :** `image-jetson`
    * **Base :** `dustynv/ros:foxy-ros-base-l4t-r32.7.1`
    * **Spécificité :** Accès matériel (GPU/GPIO) via `runtime: nvidia`.

---

## 📐 Architecture du Système

Vue d'ensemble des flux de données entre le Mac (Dev), la Jetson (Prod) et le Hardware.

```mermaid
graph TD
    %% --- BLOC MAC ---
    subgraph MAC [💻 Mac M2 - Dev & Viz]
        DockerDev[Docker Dev Container]
        Rviz[Rviz2]
    end

    %% --- BLOC ROBOT ---
    subgraph JETSON [🤖 Robot - Jetson Nano]
        
        %% Interne ROS 2
        subgraph ROS_STACK [ROS 2 Control Stack]
            Teleop([Node: Teleop / Nav2]) -->|cmd_vel| CM[Controller Manager]
            
            CM -->|Charge| DDC[diff_drive_controller]
            CM -->|Charge| JSB[joint_state_broadcaster]
            
            DDC -->|Consigne rad/s| Driver[Node: waveshare_driver]
            Driver -->|Retour Vitesse| DDC
            Driver -->|Etat Joint| JSB
        end
    end

    %% --- BLOC HARDWARE ---
    subgraph HARDWARE [🔌 Hardware Bas Niveau]
        ESP32[Carte ESP32]
        Moteurs((Moteurs))
    end

    %% --- FLUX ---
    DockerDev -->|Deploy (Rsync)| JETSON
    Driver <==>|Série JSON| ESP32
    ESP32 ==>|PWM| Moteurs
    Moteurs -.->|Codeurs| ESP32
    
    DDC -.->|Odométrie| Rviz

⚙️ Hardware & Configuration Moteurs
Ce robot est un 4WD à direction par dérapage (Skid-Steering), mais piloté logiciellement comme un robot différentiel (2 roues).
Spécificités Motrices
 * Moteurs Arrière : Avec Codeurs (Utilisés pour l'odométrie).
 * Moteurs Avant : Sans Codeurs (Moteurs esclaves).
 * Synchronisation : Les moteurs avant et arrière d'un même côté sont câblés en parallèle sur l'ESP32. Ils reçoivent la même tension.
Communication Série (Driver C++)
Le driver waveshare_driver communique via /dev/ttyUSB0 avec l'ESP32 en utilisant un protocole JSON :
 * Commande (ROS -> ESP32) : {"T":1, "L":0.5, "R":0.5} (Vitesse normalisée ou PWM).
 * Feedback (ESP32 -> ROS) : {"vL":0.12, "vR":0.11} (Vitesse mesurée par les codeurs).
🚀 Quick Start (Commandes)
Le projet utilise un Makefile pour simplifier les interactions Docker.
💻 Sur le Mac (Développement)
 * Prérequis : Installer XQuartz (pour l'affichage Gazebo/Rviz) et autoriser les connexions réseau.
 * Construire l'image :
   make dev-build

 * Lancer le conteneur :
   xhost +localhost  # Autorise l'affichage graphique
make dev-up

 * Entrer dans le terminal :
   make dev-shell

🤖 Sur la Jetson (Production)
 * Prérequis : Avoir activé un SWAP file de 4Go minimum (sinon crash mémoire).
 * Entrer dans le conteneur :
   make jetson-shell

 * Lancer le robot :
   ros2 launch rover_bringup bringup.launch.py

📂 Cartographie des Fichiers Clés
Où aller pour modifier le comportement du robot ?
| Composant | Fichier Clé 📂 | Rôle |
|---|---|---|
| Géométrie | src/rover_description/urdf/rover.urdf.xacro | Définit la taille des roues, les positions et les joints mimic (avant copie arrière). |
| Paramètres | src/rover_bringup/config/control.yaml | Paramètres du diff_drive_controller (PID, Rayon roue, Covariance Odom). |
| Driver C++ | src/waveshare_driver/src/waveshare_system.cpp | Code principal. Gère la boucle de lecture/écriture sur le port Série. |
| Lancement | src/rover_bringup/launch/bringup.launch.py | Script maître. Lance le controller_manager et charge la description du robot. |
| Réseau | scripts/entrypoint.sh | Configuration automatique de l'IP et de CycloneDDS au démarrage. |
🔄 Workflow de Déploiement (Mac ➔ Jetson)
Pour éviter la surchauffe et la lenteur de la compilation sur la Jetson Nano, nous utilisons la cross-compilation native (ARM64) sur Mac suivie d'une synchronisation.
 * Configuration :
   Dans le Makefile, éditez les variables :
   JETSON_USER=votre_user
JETSON_IP=192.168.1.XX

 * Déployer :
   Depuis le Mac, lancez simplement :
   make deploy

   Cette commande compile le projet en mode Release sur le Mac, puis envoie les dossiers src, install et build sur la Jetson via rsync.
 * Appliquer :
   Sur la Jetson, sourcez le nouvel environnement :
   source install/setup.bash

🚧 Roadmap / To-Do List
 * [ ] Phase 1 : Socle de base (En cours)
   * [ ] Créer l'URDF (rover_description) avec les 4 roues et le mimic.
   * [ ] Coder le squelette du Driver C++ (waveshare_driver).
   * [ ] Configurer ros2_control pour charger le driver.
   * [ ] Valider que les moteurs tournent via ros2 topic pub /cmd_vel.
 * [ ] Phase 2 : Odométrie & Visualisation
   * [ ] Implémenter la lecture Série (JSON) dans le driver.
   * [ ] Vérifier la précision de l'odométrie dans Rviz (le robot revient-il à 0 ?).
 * [ ] Phase 3 : Navigation (Futur)
   * [ ] Ajouter le LiDAR (RPLidar ou autre).
   * [ ] Configurer SLAM Toolbox pour la cartographie.
   * [ ] Configurer Nav2 pour la navigation autonome.
<!-- end list -->

