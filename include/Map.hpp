#ifndef MAP_HPP
#define MAP_HPP

#include <opencv2/opencv.hpp>
#include <string>

// La classe Map gère l'environnement statique de la simulation ("Vérité Terrain").
// Elle charge une image depuis le disque où :
// - Les pixels NOIRS (0,0,0) sont considérés comme des murs.
// - Les autres pixels (blancs/gris) sont des zones libres.
class Map {
public:
    // --- 1. CONSTRUCTEUR ---
    
    // Charge l'image spécifiée par 'filename' (ex: "map.png") en mémoire.
    // Arrête le programme si l'image est introuvable.
    Map(const std::string& filename);

    // --- 2. MÉTHODES PRINCIPALES (Logique) ---

    // Vérifie si une coordonnée (x, y) donnée est un obstacle.
    // Retourne true si c'est un mur (pixel noir) ou si on est hors de la carte.
    bool isObstacle(int x, int y) const;

    // --- 3. GETTERS (Accesseurs) ---

    // Retourne l'image brute de la carte (utile pour l'affichage)
    cv::Mat getImage() const;

    // Retourne la largeur de la carte en pixels
    int getWidth() const;

    // Retourne la hauteur de la carte en pixels
    int getHeight() const;

private:
    // --- MEMBRES ---
    
    cv::Mat image; // Matrice OpenCV contenant les données de l'image (pixels BGR)
    int height;    // Hauteur de l'image (lignes / rows)
    int width;     // Largeur de l'image (colonnes / cols)
};

#endif // MAP_HPP