cmake_minimum_required(VERSION 3.5)
project(rover_description)

# 1. Chargement des outils de construction
find_package(ament_cmake REQUIRED)

# 2. INSTALLATION (Le point critique)
# On installe le dossier "urdf" vers le dossier "share/rover_description"
# Si tu ajoutes plus tard un dossier "meshes" (pour les fichiers 3D), ajoute-le ici.
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# 3. Clôture du paquet
ament_package()