# Simulation de Navigation Robotique avec LiDAR

Ce projet implémente une simulation robotique d'un **robot mobile équipé d'un capteur LiDAR** pour explorer un environnement inconnu. Le système utilise **OpenCV** pour la visualisation en temps réel de la carte et de la grille d'occupation (Occupancy Grid).

## Objectifs du Projet
Le robot explore un environnement inconnu en utilisant un LiDAR pour construire une carte. Il doit éviter les obstacles, couvrir toutes les zones accessibles et naviguer efficacement.

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

## Construction (Build)
- Raphaël Maul
- Alexandre Raffin
- Enzo Barro
- Tanguy Villequez 
