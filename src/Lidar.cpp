#include "../include/Lidar.hpp"
#include "../include/Simulation.hpp" 
#include "../include/Robot.hpp"
#include "../include/Map.hpp"
#include <cmath>
#include <iostream>

// Définition de PI si non fournie par le compilateur
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =========================================================
// CONSTRUCTEUR
// =========================================================
Lidar::Lidar(Simulation* simulation_)
    : simulation(simulation_) // Stocke le pointeur vers l'environnement
{
}

// =========================================================
// MÉTHODE PRINCIPALE : LECTURE D'UN RAYON (ALGO DDA)
// =========================================================
double Lidar::read(int rayID) const {
    // 1. Récupération des objets nécessaires
    const Map& map = simulation->getMap();   // Accès à la carte (Murs)
    const Robot& robot = simulation->getRobot(); // Accès au robot (Position)

    // Dimensions de la carte pour éviter de sortir des limites
    int width = map.getWidth();
    int height = map.getHeight();

    // 2. Position de départ du rayon (Centre du robot)
    cv::Point robotPos = robot.getPosition();
    double startX = static_cast<double>(robotPos.x);
    double startY = static_cast<double>(robotPos.y);

    // 3. Calcul de l'angle du rayon
    double orientation = robot.getOrientation(); // Orientation du robot
    
    // L'angle du rayon = Angle du robot + Décalage du rayon (ex: -180 à +180)
    // (rayID - num_rays / 2) centre le scan devant le robot
    double rayAngle = orientation + (rayID - num_rays / 2) * (M_PI / 180.0);

    // Calcul du vecteur direction du rayon
    double rayDirX = std::cos(rayAngle);
    double rayDirY = std::sin(rayAngle);

    // --- ALGORITHME DDA (Digital Differential Analyzer) ---
    // Cet algorithme permet de parcourir une grille case par case très rapidement.
    
    // Calcul de la distance que le rayon doit parcourir pour traverser 1 unité en X ou en Y
    // (deltaDistX = distance hypoténuse pour avancer de 1 en X)
    // 1e30 est une valeur "infinie" pour éviter la division par zéro si rayDir est 0
    double deltaDistX = (rayDirX == 0) ? 1e30 : std::abs(1.0 / rayDirX);
    double deltaDistY = (rayDirY == 0) ? 1e30 : std::abs(1.0 / rayDirY);

    // Coordonnées entières de la case actuelle dans la grille de la carte
    int mapX = int(startX);
    int mapY = int(startY);

    // Variables pour l'algo de parcours
    double sideDistX; // Distance jusqu'au prochain côté vertical (X)
    double sideDistY; // Distance jusqu'au prochain côté horizontal (Y)
    int stepX;        // Direction du pas en X (+1 ou -1)
    int stepY;        // Direction du pas en Y (+1 ou -1)

    // 4. Initialisation des pas et des distances initiales (sideDist)
    if (rayDirX < 0) {
        stepX = -1; // Le rayon va vers la gauche
        sideDistX = (startX - mapX) * deltaDistX;
    } else {
        stepX = 1;  // Le rayon va vers la droite
        sideDistX = (mapX + 1.0 - startX) * deltaDistX;
    }

    if (rayDirY < 0) {
        stepY = -1; // Le rayon va vers le haut (Y diminue)
        sideDistY = (startY - mapY) * deltaDistY;
    } else {
        stepY = 1;  // Le rayon va vers le bas (Y augmente)
        sideDistY = (mapY + 1.0 - startY) * deltaDistY;
    }
    
    bool hit = false;       // A-t-on touché un mur ?
    double distance = 0.0;  // Distance parcourue

    // 5. Boucle de lancer de rayon (DDA Loop)
    while (!hit && distance < max_range) {
        // On avance dans la direction la plus courte (X ou Y) pour atteindre la prochaine intersection
        if (sideDistX < sideDistY) {
            sideDistX += deltaDistX;     // On saute à la prochaine case X
            mapX += stepX;               // On met à jour la coordonnée grille X
            distance = sideDistX - deltaDistX; // Calcul de la vraie distance parcourue
        } else {
            sideDistY += deltaDistY;     // On saute à la prochaine case Y
            mapY += stepY;               // On met à jour la coordonnée grille Y
            distance = sideDistY - deltaDistY;
        }

        // Vérification 1 : Est-ce qu'on est sorti de la carte ?
        if (mapX >= 0 && mapX < width && mapY >= 0 && mapY < height) {
            // Vérification 2 : Est-ce que cette case est un obstacle ?
            if (map.isObstacle(mapX, mapY)) {
                hit = true; // Impact confirmé
            }
        } else {
            // Sortie de carte considérée comme un impact max
            hit = true;
            distance = max_range;
        }
    }

    // Retourne la distance mesurée ou le max si rien n'a été touché
    return (hit) ? distance : max_range;
}

// =========================================================
// LECTURE COMPLÈTE
// =========================================================
std::vector<double> Lidar::readAll() const {
    std::vector<double> readings;
    readings.reserve(num_rays); // Optimisation mémoire
    
    // Boucle pour scanner les 360 degrés
    for (int i = 0; i < num_rays; i++) {
        readings.push_back(read(i));
    }
    return readings;
}

// =========================================================
// CALCUL DES POINTS D'IMPACT (Pour OccupancyGrid)
// =========================================================
std::vector<cv::Point> Lidar::getHitPoints(const Robot& robot) const {
    std::vector<cv::Point> hits;
    cv::Point pos = robot.getPosition();
    double orientation = robot.getOrientation();

    // On parcourt tous les rayons
    for (int i = 0; i < num_rays; i++) {
        double dist = read(i); // Obtient la distance du rayon i
        
        // Calcul de l'angle absolu de ce rayon
        double angle = orientation + (i - num_rays / 2) * (M_PI / 180.0);
        
        // Trigonométrie pour retrouver les coordonnées (X, Y) du point d'impact
        // x = x0 + dist * cos(theta)
        // y = y0 + dist * sin(theta)
        cv::Point p;
        p.x = pos.x + static_cast<int>(dist * std::cos(angle));
        p.y = pos.y + static_cast<int>(dist * std::sin(angle));
        
        hits.push_back(p);
    }
    
    return hits;
}

// =========================================================
// AFFICHAGE 
// =========================================================
void Lidar::draw(cv::Mat& image, const Robot& robot) const {
    cv::Point pos = robot.getPosition();
    double robotOrientation = robot.getOrientation();
    
    // Pour chaque rayon
    for (int i = 0; i < num_rays; i++) {
        double dist = read(i);
        
        // Si le rayon touche quelque chose avant sa portée maximale
        if (dist < max_range) {
            double angle = robotOrientation + (i - num_rays / 2) * (M_PI / 180.0);
            
            // Calcul du point de fin pour le dessin
            cv::Point endPoint;
            endPoint.x = pos.x + static_cast<int>(dist * std::cos(angle));
            endPoint.y = pos.y + static_cast<int>(dist * std::sin(angle));

            // Dessine une ligne rouge (BGR: 0, 0, 255) représentant le laser
            cv::line(image, pos, endPoint, cv::Scalar(0, 0, 255), 1); 
        }
    }
}

// =========================================================
// GETTERS
// =========================================================

// Retourne le nombre de rayons du capteur
int Lidar::getRayCount() const { 
    return num_rays; 
}

// Retourne la portée maximale définie
double Lidar::getMaxRange() const { 
    return max_range; 
}