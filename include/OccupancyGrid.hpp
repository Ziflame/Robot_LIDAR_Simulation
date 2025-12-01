#ifndef OCCUPANCYGRID_HPP
#define OCCUPANCYGRID_HPP

#include <opencv2/opencv.hpp>
#include <vector>

// La classe OccupancyGrid gère la "mémoire" spatiale du robot.
// Elle divise le monde en une grille de cellules. Chaque cellule contient une probabilité d'occupation :
// - 127 : Zone Inconnue (Gris) - État initial
// - 255 : Zone Libre (Blanc) - Le laser a traversé cette zone
// - 0   : Obstacle (Noir) - Le laser a tapé quelque chose ici
class OccupancyGrid {
public:
    // --- 1. CONSTRUCTEUR ---
    
    // Initialise la grille en fonction de la taille de la carte réelle et de la résolution voulue.
    // width/height : Dimensions de la map réelle (ex: 500x200)
    // cellSize : Taille d'une case en pixels (ex: 10px = 1 case)
    OccupancyGrid(int width, int height, int cellSize = 1);

    // --- 2. MÉTHODES PRINCIPALES (Logique de Mapping) ---

    // Met à jour la grille en fonction des mesures du Lidar.
    // Utilise le "Raycasting" pour tracer des lignes de vide entre le robot et les obstacles.
    void update(const std::vector<cv::Point>& scanPoints, cv::Point robotPos);

    // Nettoie la carte pour boucher les petits trous et supprimer le bruit.
    // Utilise des opérations morphologiques (Dilatation/Érosion).
    void smoothGrid(int iterations = 1);

    // Vérifie le pourcentage de la carte qui a été découvert.
    // Retourne true si le ratio de zones inconnues est faible.
    bool isFullyExplored() const;

    // --- 3. AFFICHAGE ---

    // Dessine la grille sur une image affichable (conversion Grille -> Pixels).
    // Permet de visualiser ce que le robot a "compris" de son environnement.
    void draw(cv::Mat& displayImage);

    // --- 4. GETTERS (Accesseurs) ---

    // Retourne l'accès direct à la matrice brute (pour lecture ou modification avancée).
    cv::Mat& getGrid();

private:
    // --- MEMBRES ---
    
    int width;    // Largeur réelle de l'environnement (pixels)
    int height;   // Hauteur réelle de l'environnement (pixels)
    int cellSize; // Facteur d'échelle (1 case = X pixels)
    
    int gridW;    // Largeur de la grille (nombre de colonnes)
    int gridH;    // Hauteur de la grille (nombre de lignes)
    
    cv::Mat grid; // Matrice OpenCV stockant les valeurs (0, 127, 255)
};

#endif // OCCUPANCYGRID_HPP