# 🤖 Wave Rover - ROS 2 Foxy Project

> **Projet :** Stack de navigation autonome pour robot Wave Rover (4WD).
> **Architecture :** ROS 2 Foxy Fitzroy sur Jetson Nano & Mac M2 (Docker).
> **Dernière mise à jour :** Décembre 2025

---

## 📖 Vue d'ensemble

Ce dépôt contient le code source et l'infrastructure DevOps pour piloter un robot **WaveShare Wave Rover** modifié. Le projet est conçu pour gérer la cross-compilation transparente entre un environnement de développement (Mac M2 / PC) et la cible embarquée (NVIDIA Jetson Nano).

### 🏗️ Architecture Infrastructure (DevOps)

Le projet est entièrement conteneurisé. Le `Dockerfile` multi-stage gère deux cibles distinctes :

1.  **💻 Environnement Dev (Mac M2 / PC)**
    * **Cible Docker :** `image-generic`
    * **Base :** `ros:foxy` (Ubuntu 20.04)
    * **Avantage :** Utilise l'architecture **ARM64 native** des Mac Apple Silicon (pas d'émulation QEMU lente).

2.  **🚀 Environnement Prod (Jetson Nano)**
    * **Cible Docker :** `image-jetson`
    * **Base :** `dustynv/ros:foxy-ros-base-l4t-r32.7.1`
    * **Spécificité :** Nécessaire car la Jetson Nano est bloquée sous Ubuntu 18.04 (JetPack 4.6), mais nous forçons l'exécution de conteneurs ROS 2 Foxy (20.04).

**Middleware :** CycloneDDS est configuré par défaut en mode multicast `auto` pour assurer le pont réseau transparent entre le Mac et la Jetson.

---

## ⚙️ Hardware : Configuration Hybride

Ce robot est un **WAVE ROVER** dont la motorisation a été modifiée pour permettre une odométrie précise.

* **Châssis :** 4 roues motrices (4WD), direction par dérapage (Skid-Steering).
* **Contrôleur Bas Niveau :** Carte "General Driver for Robots" (basée sur ESP32).

### 🔌 Détail des Moteurs

| Zone | Type de Moteur | Connecteur | Modèle / Specs |
| :--- | :--- | :--- | :--- |
| **Arrière** | **Avec Codeurs** | PH2.0 6-pins | Waveshare DCGM-N20-12V-EN-200RPM |
| **Avant** | **Sans Codeurs** | PH2.0 2-pins | Moteurs d'origine (200 RPM) |

### ⚡ Câblage & Synchronisation

Sur la carte ESP32, les ports avant et arrière d'un même côté sont **connectés électriquement en parallèle** sur le même driver de puissance.

* **Conséquence :** Impossible de piloter l'avant et l'arrière indépendamment. Ils reçoivent la même tension PWM.
* **Avantage :** Comme tous les moteurs sont des **200 RPM**, ils tournent naturellement à la même vitesse (synchronisation matérielle).

---

## 🧠 Stratégie de Contrôle ROS 2

Bien que le robot ait 4 roues physiques, nous le pilotons logiciellement comme un robot à **2 roues différentielles**.

### 1. Le Driver (`waveshare_driver`)

Il s'agit d'une `SystemInterface` **ros2_control** (C++) custom.

* **Responsabilité :** Communiquer en Série (JSON) avec l'ESP32.
* **Exposition ROS :** N'expose que 2 joints (`rear_left_wheel_joint` et `rear_right_wheel_joint`).
* **Logique Write :** Envoie la commande de vitesse globale pour un côté. L'ESP32 gère le PID sur l'arrière et réplique le PWM sur l'avant.
    * *Exemple de commande :* `{"T":1, "L":0.5, "R":0.5}`
* **Logique Read :** Lit le retour des codeurs arrière pour calculer l'odométrie.
    * *Exemple de retour :* `{"vL":0.12, "vR":0.11}`

### 2. Le Contrôleur (`rover_bringup`)

Nous utilisons le **`diff_drive_controller`** standard configuré dans `rover_bringup`.

* **Configuration :** Il ne connaît que les 2 roues arrière (les seules avec codeurs).
* **Paramètres :**
    ```yaml
    left_wheel_names: ["rear_left_wheel_joint"]
    right_wheel_names: ["rear_right_wheel_joint"]
    wheel_separation: 0.16 # À vérifier
    wheel_radius: 0.04     # À vérifier
    ```

### 3. URDF & Visualisation (`rover_description`)

Pour que la visualisation sous Rviz soit correcte (les 4 roues tournent), on utilise le système de `<mimic>`. La roue avant "copie" visuellement la position de la roue arrière.

```xml
<joint name="rear_left_wheel_joint" type="continuous"> ... </joint>

<joint name="front_left_wheel_joint" type="continuous">
    <mimic joint="rear_left_wheel_joint" multiplier="1.0" offset="0.0"/>
</joint>
