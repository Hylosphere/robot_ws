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
