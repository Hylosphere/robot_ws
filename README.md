# 🤖 Wave Rover - ROS 2 Foxy Project

> **Projet :** Stack de navigation autonome pour robot Wave Rover (4WD).
> **Architecture :** ROS 2 Foxy Fitzroy sur Jetson Nano & Mac M2 (Docker).
> **Dernière mise à jour :** Décembre 2025

---

## 📖 Vue d'ensemble

Ce dépôt contient le code source et l'infrastructure DevOps pour piloter un robot **WaveShare Wave Rover** modifié. Le projet est conçu pour gérer la cross-compilation transparente entre un environnement de développement (Mac M2 / PC) et la cible embarquée (NVIDIA Jetson Nano).

### 🏗️ Infrastructure Hybride

Le projet est entièrement conteneurisé pour contourner les limitations de l'OS de la Jetson Nano (Ubuntu 18.04) tout en profitant de la puissance du Mac M2.

1.  **💻 Environnement Dev (Mac M2)**
    * **Image :** `ros:foxy` (Ubuntu 20.04)
    * **Rôle :** Compilation rapide, simulation, visualisation (Rviz).
    * **Avantage :** Architecture **ARM64 native** (compatible binaire avec la Jetson).

2.  **🚀 Environnement Prod (Jetson Nano)**
    * **Image :** `dustynv/ros:foxy-ros-base-l4t-r32.7.1`
    * **Rôle :** Exécution temps réel, accès Hardware (GPU/GPIO).
    * **Spécificité :** Utilise le runtime NVIDIA pour l'accès matériel.

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

⚡ Workflow : Compilation Hybride (Mac M2 ➔ Jetson)
C'est la fonctionnalité clé. Au lieu de compiler sur la Jetson (lent, chauffe, mémoire limitée), nous utilisons le Mac comme "usine de compilation".
Pourquoi ça marche ?
 * Même Architecture CPU : Mac M2 et Jetson Nano sont tous deux ARM64. Les binaires sont compatibles.
 * Environnement Miroir : Grâce à Docker, le chemin /workspaces/robot_ws est identique sur les deux machines.
🛠️ Configuration (À faire une fois)
1. Sur la Jetson :
Créer un dossier physique et pointer le docker-compose dessus (Bind Mount) :
mkdir -p ~/robot_ws/src

Dans docker-compose.yml (service jetson) :
volumes:
  - type: bind
    source: ~/robot_ws  # Dossier physique
    target: /workspaces/robot_ws # DOIT être identique au Mac

2. Sur le Mac (Makefile) :
Configurer les variables de déploiement dans le Makefile :
JETSON_USER=jetson
JETSON_IP=192.168.1.XX

🚀 Cycle de Développement
 * Coder sur le Mac.
 * Déployer :
   make deploy

   (Compile en Release sur Mac et synchronise via rsync sur la Jetson).
 * Exécuter sur la Jetson :
   source install/setup.bash
ros2 launch rover_bringup bringup.launch.py

⚙️ Hardware & Protocole Série
Le robot est un 4WD piloté comme un différentiel (2 roues).
Moteurs
 * Arrière : Avec Codeurs (Maîtres pour l'odométrie).
 * Avant : Sans Codeurs (Suiveurs câblés en parallèle).
 * Conséquence : L'URDF utilise un tag <mimic> pour copier le mouvement arrière vers l'avant.
Protocole JSON (Driver C++)
Communication via /dev/ttyUSB0 à 115200 bauds.
 * Commande (ROS -> ESP32) :
   {"T":1, "L":0.5, "R":0.5}
   (T=Type, L/R=Vitesse normalisée ou PWM)
 * Retour (ESP32 -> ROS) :
   {"vL":0.12, "vR":0.11}
   (Vitesses mesurées en m/s ou ticks/s)
🚀 Quick Start
💻 Sur le Mac
# Construire l'image
make dev-build

# Démarrer l'environnement
make dev-up

# Entrer dans le conteneur
make dev-shell

🤖 Sur la Jetson
Prérequis : Avoir activé un SWAP file de 4Go.
# Entrer dans le conteneur
make jetson-shell

# Lancer la stack
ros2 launch rover_bringup bringup.launch.py

📂 Cartographie des Fichiers
| Composant | Fichier Clé 📂 | Rôle |
|---|---|---|
| Géométrie | src/rover_description/urdf/rover.urdf.xacro | Taille roues, positions, joints mimic. |
| Paramètres | src/rover_bringup/config/control.yaml | Config PID, Rayon roue, Odométrie. |
| Driver C++ | src/waveshare_driver/src/waveshare_system.cpp | Code principal. Interface Série/JSON. |
| Lancement | src/rover_bringup/launch/bringup.launch.py | Script maître (Control Manager + URDF). |
🚧 Roadmap / To-Do List
 * [ ] Phase 1 : Socle de base (En cours)
   * [ ] Créer l'URDF (rover_description) avec les 4 roues et le mimic.
   * [ ] Coder le squelette du Driver C++ (waveshare_driver).
   * [ ] Configurer ros2_control pour charger le driver.
   * [ ] Valider que les moteurs tournent via ros2 topic pub /cmd_vel.
 * [ ] Phase 2 : Odométrie & Visualisation
   * [ ] Implémenter la lecture Série (JSON) dans le driver.
   * [ ] Vérifier la précision de l'odométrie dans Rviz.
 * [ ] Phase 3 : Navigation (Futur)
   * [ ] Ajouter le LiDAR.
   * [ ] Configurer SLAM Toolbox & Nav2.
<!-- end list -->

