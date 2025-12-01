# Simulation de Navigation Robotique avec LiDAR

Ce projet consiste à implémenter un **robot mobile équipé d'un capteur LiDAR** pour explorer un environnement inconnu. Le système utilise **OpenCV** pour la visualisation en temps réel de la carte et de la grille d'occupation (Occupancy Grid).

## Objectifs du Projet
Le robot explore un environnement inconnu en utilisant un LiDAR pour construire une carte. Il doit éviter les obstacles, couvrir toutes les zones accessibles et naviguer efficacement. Le mode de déplacement du robot doit être determiné en scannant des tags ArUco.

## Demonstration
<p align="center">
  <img src="Images/video_demo.gif" alt="Demo"/>
</p>

Une vidéo de démonstration plus longue est disponible dans le dossier "Images".

## Documentation
- Pour le raycasting (lancer de rayons) : https://lodev.org/cgtutor/raycasting.html
- Pour les tags ArUco : https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
- Pour créer des tags ArUco : https://chev.me/arucogen/
- Pour créer la carte, nous avons utilisé un créateur de pixel art : https://www.pixilart.com/
- Nous avons utilisé deux cartes : la première pour effectuer des tests et la carte finale.
- Nous avons également utilisé l'IA pour nous aider au débogage, à la rédaction de commentaires ou en cas de blocage sur un problème.

## Fonctionnalités Clés
- **Environnements variés** avec des obstacles de tailles différentes.
- **Initialisation aléatoire du robot** sans connaissance préalable de sa position.
- **Simulation LiDAR** utilisant un algorithme de raycasting.
- **Exploration autonome** utilisant un algorithme de suivi de mur (main droite).

***Toutes les décisions du robot sont basées exclusivement sur les données du capteur LiDAR, sans accès direct ou indirect à la carte de l'environnement.***

## Dépendances
- **[OpenCV](https://opencv.org/)** : pour la visualisation graphique.
- **[CMake](https://cmake.org/)** : pour la configuration et la compilation du projet.

  ## Structure du projet
```
Robot_LIDAR/
├── CMakeLists.txt          
├── map.png                 
├── include/               
│   ├── Simulation.hpp
│   ├── Robot.hpp
│   ├── Lidar.hpp
│   ├── Map.hpp
│   ├── OccupancyGrid.hpp
│   ├── BehaviorManager.hpp
│   └── ArucoManager.hpp
└── src/                    
    ├── main.cpp
    ├── Simulation.cpp
    ├── Robot.cpp
    ├── Lidar.cpp
    ├── Map.cpp
    ├── OccupancyGrid.cpp
    ├── BehaviorManager.cpp
    └── ArucoManager.cpp
```

## Construction (Build)
**Compilation et exécution** :
```
bash
cd Robot_LIDAR
mkdir build
cd build
cmake ..
make
./main
```
## Utilisation
1. Lancer le programme pour place le robt aléatoirement sur la carte
2. Choisir le mode de déplacement :
- Utiliser la touche "1" du clavier ou scanner un tag ArUcoa avec un ID = 0 pour activer le mode de déplacment manuel
- Utiliser la touche "2" du clavier ou scanner un tag ArUco avec un ID = 1 pour activer le mode de suivi de mur
4. Une fois l'exploration terminée, appuyer sur "echap" pour fermer le programme

## Informations concernant la détection de la caméra pour les tags ArUco avec WSL
Ce projet à entièremlent été coder sur un sous-système Linux (WSL), de ce fait la caméra n'est pas directment détectée.
Pour détecter la caméra il faut suivre ces 5 étapes :
1. Ouvrir l'invité de commande Powershell en mode administrateur
2. Installer usbipd avec cette commande :
```
winget install usbipd-win
```
3. Listez les appareils USB connectés avec la commande :
 ```
usbipd list
```
4.Connecter la caméra à Linux avec ces deux commande :
```
usbipd bind --busid <VOTRE_BUSID>
usbipd attach --wsl --busid <VOTRE_BUSID>
```
Remplacer <VOTRE_BUSID> par le busid de votre caméra (exemple :  1-8 )

5. Enfin, dans votre invité de commande Linux (avant de lancer le programme), tapez cette commande :
```
sudo chmod 777 /dev/video0
```
- Normalement la caméra est detectée lors du lacemant du programme.
- Le programme n'a pas été testé sur un boot Linux normal. Il est posqible que la caméra ne soit pas détectée.
- Il est néamoins possible de tester le programme en utilisant les touches "1" et "2" du clavier

   


## Contributeurs
- Raphaël Maul
- Alexandre Raffin
- Enzo Barro
- Tanguy Villequez 
