#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <opencv2/opencv.hpp>

// La classe Robot représente l'agent physique qui se déplace dans l'environnement.
// Elle gère sa position, son orientation, sa vitesse et son affichage.
class Robot {
public:
    // --- 1. CONSTRUCTEUR ---
    
    // Initialise le robot à une position donnée avec une taille spécifique.
    // startPos : Coordonnées (x, y) de départ.
    // size : Diamètre du robot en pixels (défaut 11).
    Robot(cv::Point startPos, int size = 11);

    // --- 2. SETTERS (Modificateurs) ---
    
    // Téléporte le robot à une nouvelle position absolue.
    // Utilisé par la Simulation après vérification des collisions.
    void setPosition(cv::Point newPos);
    
    // Met à jour l'angle du robot en fonction de son vecteur de déplacement (dx, dy).
    // Assure la cohérence entre le mouvement visuel et le scan du Lidar.
    void updateOrientation(int dx, int dy);

    // --- 3. AFFICHAGE ---

    // Dessine le robot (cercle vert + trait de direction) sur l'image fournie.
    void draw(cv::Mat& displayImage);
    
    // --- 4. GETTERS (Accesseurs) ---
    
    // Retourne la position actuelle (x, y)
    cv::Point getPosition() const;

    // Retourne la taille (diamètre) du robot
    int getSize() const; 

    // Retourne l'angle d'orientation en radians
    double getOrientation() const;

    // Retourne la vitesse de déplacement (pixels par frame)
    int getSpeed() const;
    
private:
    // --- MEMBRES ---
    
    cv::Point position;       // Coordonnées actuelles du centre du robot
    cv::Scalar color;         // Couleur du robot (Vert par défaut)
    int size;                 // Diamètre du corps
    int radius;               // Rayon (taille / 2)
    int speed;                // Vitesse de déplacement (pixels par pas)
    double orientation_angle; // Angle en radians (0 = Droite, PI/2 = Bas)
};

#endif // ROBOT_HPP