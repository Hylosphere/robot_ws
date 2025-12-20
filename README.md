Voici une version formatée, propre et structurée en Markdown, prête à être copiée-collée dans votre fichier `README.md`.

J'ai ajouté des émojis pour la lisibilité, des blocs de code pour la clarté technique et des cases à cocher pour la Todo List.

---

```markdown
# 🤖 Wave Rover - ROS 2 Foxy Project

> **Dernière mise à jour :** Décembre 2025  
> **Objectif :** Documentation de l'architecture DevOps, du hardware hybride et de la stratégie de contrôle pour le Wave Rover.

---

## 1. 🏗️ Architecture Infrastructure (DevOps)

Le projet est entièrement conteneurisé pour gérer la cross-compilation entre un ordinateur de développement (Mac M2 / PC) et la cible embarquée (Jetson Nano).

* **ROS Distro :** ROS 2 Foxy Fitzroy 🦊
* **Communication :** CycloneDDS (configuré en multicast `auto` via `/etc/cyclonedds.xml` pour le pont Mac ↔️ Jetson).

### 🐳 Structure Docker
Le `Dockerfile` est multi-stage pour supporter deux environnements distincts :

1.  **Dev (Mac M2 / PC) - Cible `image-generic`** :
    * Basée sur `ros:foxy`.
    * Utilise l'architecture **ARM64 native** du Mac M2 (pas d'émulation lente).
2.  **Prod (Jetson Nano) - Cible `image-jetson`** :
    * Basée sur `dustynv/ros:foxy` (L4T r32.7.1).
    * Nécessaire car la Jetson Nano est bloquée sous Ubuntu 18.04, mais nous forçons l'utilisation de conteneurs Foxy (20.04).

### 🛠️ Commandes rapides (Makefile)

```bash
make dev-up      # Lance l'environnement de dev sur Mac
make jetson-up   # Lance le conteneur sur le robot (monte /dev et le code source)
make build       # Compile le workspace avec colcon

```

---

## 2. ⚙️ Hardware : Spécificités du Rover

Ce robot est un **WAVE ROVER** modifié avec une configuration moteur hybride.

* **Châssis :** 4 roues motrices (4WD), direction par dérapage (Skid-Steering).
* **Contrôleur Bas Niveau :** Carte "General Driver for Robots" (basée sur ESP32).

### 🔌 Configuration Moteurs

| Zone | Type de Moteur | Connecteur | Détails |
| --- | --- | --- | --- |
| **Arrière** | **Avec Codeurs** | PH2.0 6-pins | Waveshare DCGM-N20-12V-EN-200RPM |
| **Avant** | **Sans Codeurs** | PH2.0 2-pins | Moteurs d'origine (200 RPM) |

### ⚡ Câblage & Synchronisation

Sur la carte ESP32, les ports avant et arrière d'un même côté sont **connectés en parallèle** sur le même driver de puissance.

* **Conséquence :** Impossible de piloter l'avant et l'arrière indépendamment. Ils reçoivent la même tension PWM.
* **Avantage :** Comme tous les moteurs sont des **200 RPM**, ils tournent naturellement à la même vitesse (synchronisation matérielle).

---

## 3. 🧠 Stratégie de Contrôle ROS 2

Bien que le robot ait 4 roues, nous le pilotons logiciellement comme un robot à **2 roues différentielles** pour simplifier l'odométrie et la navigation.

### A. Le Driver (`waveshare_driver`)

Il s'agit d'une `SystemInterface` **ros2_control** (C++) à implémenter.

* **Responsabilité :** Communiquer en Série (JSON) avec l'ESP32.
* **Exposition ROS :** N'expose que 2 joints (`rear_left_wheel_joint` et `rear_right_wheel_joint`).
* **Write :** Envoie la commande de vitesse. L'ESP32 gère le PID sur l'arrière et réplique le PWM sur l'avant.
* *Exemple :* `{"T":1, "L":0.5, "R":0.5}`


* **Read :** Lit le retour codeur pour calculer l'odométrie.
* *Exemple :* `{"vL":..., "vR":...}`



### B. Le Contrôleur (`rover_bringup`)

Nous utilisons le **`diff_drive_controller`** standard.

* **Config :** Il ne connaît que les 2 roues arrière (les seules avec codeurs).
* **Paramètres :**
```yaml
left_wheel_names: ["rear_left_wheel_joint"]
right_wheel_names: ["rear_right_wheel_joint"]

```



### C. URDF & Visualisation (`rover_description`)

Pour que la visualisation sous Rviz soit correcte (les 4 roues tournent), on utilise le système de `<mimic>`. La roue avant "copie" la position de la roue arrière.

```xml
<joint name="rear_left_wheel_joint" type="continuous"> ... </joint>

<joint name="front_left_wheel_joint" type="continuous">
    <mimic joint="rear_left_wheel_joint" multiplier="1.0" offset="0.0"/>
</joint>

```

---

## 4. ✅ Todo List (Reprise du projet)

### 🕵️ Vérification Firmware ESP32

* [ ] Se connecter en série direct à la carte via USB/UART.
* [ ] Vérifier que l'ESP32 renvoie bien des données JSON (vitesse/position) quand on tourne les roues à la main.
* [ ] *Si KO :* Flasher un firmware compatible (ex: `ugv_base_general` avec support encodeur activé).

### 💻 Implémenter `waveshare_system.cpp`

* [ ] Compléter le squelette dans `src/waveshare_driver`.
* [ ] Intégrer une lib série (`libserial`) et JSON (`nlohmann_json`).
* [ ] Faire le mapping : `rad/s` (ROS) ↔️ `commande JSON` (ESP32).

### 🔄 Vérification le sens de rotation

* [ ] S'assurer que la commande "Avancer" fait tourner les roues avant et arrière dans le même sens.
* [ ] *Si une roue avant tourne à l'envers :* Inverser les 2 fils du connecteur moteur avant.

```

```
