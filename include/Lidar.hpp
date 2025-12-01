#ifndef LIDAR_HPP
#define LIDAR_HPP

#include <vector>
#include <opencv2/opencv.hpp>

// Déclarations anticipées pour éviter les inclusions circulaires
class Simulation;
class Robot;
class Map;

// La classe Lidar simule un capteur de distance laser à 360 degrés.
// Elle utilise un algorithme de lancer de rayons (Raycasting) pour détecter les murs.
class Lidar {
public:
    // --- 1. CONSTRUCTEUR ---
    
    // Initialise le Lidar avec un lien vers la simulation (pour lire la carte)
    Lidar(Simulation* simulation_);

    // --- 2. MÉTHODES PRINCIPALES  ---

    // Lance un seul rayon (identifié par son ID de 0 à 359) et retourne la distance
    // Utilise l'algorithme DDA (Digital Differential Analyzer) pour la rapidité
    double read(int rayID) const;

    // Lance tous les rayons (0 à 359) et retourne un vecteur contenant toutes les distances
    std::vector<double> readAll() const;

    // Convertit les distances mesurées en points (X, Y) réels dans le monde
    // C'est ce qui permet de construire la "Carte Mémoire" (OccupancyGrid)
    std::vector<cv::Point> getHitPoints(const Robot& robot) const;

    // --- 3. AFFICHAGE ---

    // Dessine les rayons laser sur l'image de simulation (lignes rouges)
    void draw(cv::Mat& image, const Robot& robot) const;

    // --- 4. GETTERS  ---

    // Retourne le nombre total de rayons (ex: 360)
    int getRayCount() const;

    // Retourne la portée maximale du capteur (ex: 250.0 pixels)
    double getMaxRange() const;

private:
    // --- CONSTANTES ---
    const int num_rays = 360;       // Résolution angulaire (1 rayon par degré)
    const double max_range = 100.0; // Portée max en pixels (Augmentée pour voir les coins)

    // --- MEMBRES ---
    Simulation* simulation; // Pointeur vers la simulation pour accéder à la Map et au Robot
};

#endif // LIDAR_HPP