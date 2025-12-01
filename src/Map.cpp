#include "../include/Map.hpp"
#include <iostream>
#include <cstdlib> // Pour exit()

// =========================================================
// CONSTRUCTEUR
// =========================================================
Map::Map(const std::string& filename) {
    
    // Charge l'image depuis le fichier en mode couleur (BGR)
    // IMREAD_COLOR est important car on accède aux pixels via Vec3b (3 canaux)
    image = cv::imread(filename, cv::IMREAD_COLOR);

    // Vérification de sécurité : si le chargement échoue (fichier manquant ou corrompu)
    if (image.empty()) {
        std::cerr << "ERREUR CRITIQUE : Impossible de charger la carte '" << filename << "'" << std::endl;
        std::cerr << "Verifiez que l'image est bien présente dans le dossier d'exécution (build) !" << std::endl;
        // On arrête brutalement le programme car sans carte, la simulation ne peut pas exister
        exit(1);
    }

    // Initialisation des dimensions internes pour un accès rapide
    // image.cols correspond à la largeur (axe X)
    width = image.cols;
    // image.rows correspond à la hauteur (axe Y)
    height = image.rows;
    
    std::cout << "Carte chargee avec succes: " << width << "x" << height << " pixels." << std::endl;
}

// =========================================================
// MÉTHODE PRINCIPALE : DÉTECTION D'OBSTACLE
// =========================================================
bool Map::isObstacle(int x, int y) const {
    // 1. Vérification des limites de la carte (Bounds Check)
    // Si on demande un point en dehors de l'image, on considère que c'est un mur.
    // Cela empêche le robot de sortir de l'écran.
    if (x < 0 || x >= width || y < 0 || y >= height) {
        return true; 
    }

    // 2. Accès au pixel à la coordonnée (y, x)
    // Attention : OpenCV utilise l'ordre (row, col), donc (y, x).
    // Vec3b signifie "Vector of 3 bytes" (Blue, Green, Red).
    cv::Vec3b pixel = image.at<cv::Vec3b>(y, x);
    
    // 3. Analyse de la couleur
    // On considère qu'un obstacle est un pixel parfaitement NOIR (0, 0, 0).
    // Note : OpenCV stocke les couleurs en BGR (Bleu, Vert, Rouge).
    if (pixel == cv::Vec3b(0, 0, 0)) {
        return true; // C'est un mur
    }
    
    // Si ce n'est pas noir, c'est libre (blanc, gris, etc.)
    return false;
} 

// =========================================================
// GETTERS
// =========================================================

// Retourne une copie ou une référence à la matrice image
cv::Mat Map::getImage() const {
    return image; 
}

// Retourne la largeur stockée
int Map::getWidth() const { 
    return width; 
}

// Retourne la hauteur stockée
int Map::getHeight() const { 
    return height; 
}