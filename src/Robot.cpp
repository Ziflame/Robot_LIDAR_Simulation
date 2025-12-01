#include "../include/Robot.hpp"
#include <cmath> // Pour cos, sin

// Définition de PI pour les calculs trigonométriques si non défini
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =========================================================
// CONSTRUCTEUR
// =========================================================
Robot::Robot(cv::Point startPos, int s) 
    : position(startPos),         // Initialise la position
      size(s),                    // Initialise le diamètre
      radius(s / 2),              // Calcule le rayon (utile pour les collisions)
      color(0, 255, 0),           // Couleur Vert (BGR : Blue=0, Green=255, Red=0)
      orientation_angle(0.0),     // Regarde vers la droite par défaut
      speed(1)                    // Vitesse par défaut (1px est bien pour le Grid System)
{
    // Note : J'ai mis speed à 1 pour être cohérent avec BehaviorManager
    // (car 20 pixels / 5 speed = 4 étapes, c'est un diviseur rond).
}

// =========================================================
// SETTERS
// =========================================================

void Robot::setPosition(cv::Point newPos) {
    position = newPos; // Met à jour les coordonnées internes
}

// Met à jour l'orientation en fonction de la direction du mouvement.
// C'est CRITIQUE pour que le Lidar scanne dans la bonne direction.
void Robot::updateOrientation(int dx, int dy) {
    // Cas : Mouvement vers la DROITE (X augmente)
    if (dx > 0) {
        orientation_angle = 0.0;          
    } 
    // Cas : Mouvement vers la GAUCHE (X diminue)
    else if (dx < 0) {
        orientation_angle = M_PI;        
    } 
    // Cas : Mouvement vers le BAS (Y augmente en OpenCV)
    else if (dy > 0) {
        // En maths classiques, Y vers le haut = PI/2.
        // En image (OpenCV), Y vers le bas = PI/2 aussi pour garder la rotation horaire logique.
        orientation_angle = M_PI / 2.0;   
    } 
    // Cas : Mouvement vers le HAUT (Y diminue en OpenCV)
    else if (dy < 0) {
        orientation_angle = -M_PI / 2.0;  
    }
    // Si dx=0 et dy=0, on garde l'ancienne orientation.
}

// =========================================================
// AFFICHAGE
// =========================================================

void Robot::draw(cv::Mat& displayImage) {
    // 1. Dessine le corps du robot (Cercle plein)
    cv::circle(displayImage, position, radius, color, cv::FILLED);
    
    // 2. Calcul du point bout de la ligne indiquant la direction
    cv::Point endLine;
    
    // x = cx + r * cos(theta)
    endLine.x = position.x + static_cast<int>(radius * std::cos(orientation_angle));
    
    // y = cy + r * sin(theta)
    // Note : Comme l'axe Y descend, un angle positif (Bas) donne un sin positif, 
    // donc on AJOUTE bien au Y. La logique est cohérente.
    endLine.y = position.y + static_cast<int>(radius * std::sin(orientation_angle));
    
    // 3. Dessine la ligne noire représentant la "tête" du robot
    cv::line(displayImage, position, endLine, cv::Scalar(0, 0, 0), 1);
}

// =========================================================
// GETTERS
// =========================================================

cv::Point Robot::getPosition() const { 
    return position; 
}

int Robot::getSize() const { 
    return size; 
}

double Robot::getOrientation() const { 
    return orientation_angle; 
}

int Robot::getSpeed() const {
    return speed;
}