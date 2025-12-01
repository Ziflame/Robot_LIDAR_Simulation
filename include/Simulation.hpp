#ifndef SIMULATION_HPP
#define SIMULATION_HPP

// Inclusion des bibliothèques nécessaires
#include <opencv2/opencv.hpp>
#include "Map.hpp"
#include "Robot.hpp"
#include "Lidar.hpp"
#include "OccupancyGrid.hpp"
#include "BehaviorManager.hpp"
#include "ArucoManager.hpp"
#include <string>
#include <random>

// Classe principale gérant l'ensemble de la simulation
class Simulation {
public:
    // --- Constructeur ---
    Simulation();

    // --- Méthode Principale ---
    // Lance la boucle infinie de la simulation
    void run();

    // --- Getters (Accesseurs) ---
    // Retourne une référence constante vers la carte 
    const Map& getMap() const;
    
    // Retourne une référence constante vers le robot 
    const Robot& getRobot() const;
    
    // Retourne une référence constante vers le Lidar 
    const Lidar& getLidar() const;
    
    // Retourne une référence constante vers la grille d'occupation 
    const OccupancyGrid& getOccupancyGrid() const;
    
    // Retourne une référence modifiable vers le robot 
    Robot& getRobotMutable();

private:
    // --- Objets Composants la Simulation ---
    Map map;                        // La carte de l'environnement 
    Robot robot;                    // Le robot qui se déplace
    Lidar lidar;                    // Le capteur de distance
    OccupancyGrid occupancyGrid;    // La carte construite par le robot 
    BehaviorManager behaviorManager;// Le gestionnaire de comportements 
    ArucoManager arucoManager;      // Le gestionnaire de détection des tags

    // --- Variables d'Interface ---
    std::string windowName;         // Nom de la fenêtre d'affichage OpenCV

    
    // Vérifie si une position donnée entraîne une collision avec un mur
    // centerPos : Le point central du robot à tester
    bool checkCollision(cv::Point centerPos) const;
    
    // Positionne le robot aléatoirement sur la carte au démarrage
    // en s'assurant qu'il ne tombe pas dans un mur
    void initializeRobotPosition();
};

#endif // SIMULATION_HPP