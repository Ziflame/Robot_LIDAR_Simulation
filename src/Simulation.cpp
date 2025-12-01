#include "../include/Simulation.hpp"
#include <iostream>
#include <random>
#include <algorithm> // Pour std::max

// =========================================================
// CONSTRUCTEUR
// =========================================================
Simulation::Simulation() 
    // Liste d'initialisation des membres :
    : map("map.png"),                           // Charge l'image de la carte
      robot(cv::Point(0, 0), 11),               // Crée le robot à (0,0) avec une taille de 11px
      lidar(this),                              // Le Lidar a besoin d'un pointeur vers la Simu pour lire la Map
      occupancyGrid(map.getWidth(), map.getHeight()), // La grille a la même taille que la map
      behaviorManager(this),                    // Le cerveau a besoin d'accéder aux capteurs via la Simu
      arucoManager(&behaviorManager),           // Le gestionnaire ArUco pilote le BehaviorManager
      windowName("Dashboard Robot")             // Titre de la fenêtre
{
    // Crée une fenêtre OpenCV redimensionnable
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
    
    // Trouve une position aléatoire valide pour le robot (hors des murs)
    initializeRobotPosition();
    
    // Affichage des instructions dans la console au démarrage
    std::cout << "\n=== SIMULATION DEMARREE ===" << std::endl;
    std::cout << "Controles:" << std::endl;
    std::cout << "  - Touche 1: Mode MANUEL (ZQSD)" << std::endl;
    std::cout << "  - Touche 2: Mode WALL FOLLOWING" << std::endl;
    std::cout << "  - ZQSD: Deplacements en mode MANUEL" << std::endl;
    std::cout << "  - ESC: Quitter" << std::endl;
    std::cout << "================================\n" << std::endl;
}

// =========================================================
// MÉTHODE PRINCIPALE : RUN
// =========================================================
void Simulation::run() {
    bool running = true;    // Variable de contrôle de la boucle principale
    int frameCounter = 0;   // Compteur de frames pour gérer des événements périodiques

    // Boucle infinie jusqu'à demande d'arrêt
    while (running) {
        
        // 1. VISION : Capture webcam et détection des tags ArUco
        // Cela met à jour le comportement du robot si un tag est vu
        arucoManager.captureAndDetect();

        // 2. INPUTS : Gestion des entrées clavier
        // cv::waitKey(30) attend 30ms, ce qui limite la boucle à ~30 FPS
        int key = cv::waitKey(30);
        
        // Si la touche Echap (ASCII 27) est pressée, on quitte la boucle
        if (key == 27) { running = false; continue; }
        
        // Raccourcis clavier pour forcer les modes sans ArUco (Debug)
        if (key == '1') behaviorManager.setByArucoId(0);      // Force mode Manuel
        else if (key == '2') behaviorManager.setByArucoId(1); // Force mode Suivi Mur

        // Variables pour stocker le déplacement demandé par le cerveau
        int dx = 0, dy = 0;

        // 3. INTELLIGENCE : Exécution du comportement actuel
        // Le BehaviorManager décide de dx/dy en fonction du mode et des capteurs
        behaviorManager.execute(dx, dy, key);

        // 4. PHYSIQUE : Application du mouvement
        // On ne tente de bouger que si un déplacement est demandé
        if (dx != 0 || dy != 0) {
            // Calcul de la future position théorique
            cv::Point currentPos = robot.getPosition();
            cv::Point futurePos = currentPos + cv::Point(dx, dy);

            // Vérification des collisions avant d'appliquer le mouvement
            if (!checkCollision(futurePos)) {
                robot.setPosition(futurePos);      // Applique la nouvelle position
                robot.updateOrientation(dx, dy);   // Met à jour l'angle du robot (visuel + lidar)
            }
        }

        // 5. CAPTEURS : Mise à jour du Lidar et de la Carte Mémoire
        // Le Lidar lance ses rayons depuis la nouvelle position du robot
        std::vector<cv::Point> hits = lidar.getHitPoints(robot);
        
        // On met à jour la grille d'occupation avec les points d'impact
        occupancyGrid.update(hits, robot.getPosition());
        
        // 6. POST-TRAITEMENT : Nettoyage de la carte (Optionnel)
        frameCounter++;
        // Toutes les 60 frames (environ 2 sec), on lisse un peu la grille
        if (frameCounter % 60 == 0) {
            occupancyGrid.smoothGrid(1);
        }

        // 7. RENDU GRAPHIQUE (Dashboard)
        
        // A. Préparation de la vue "Simulation" (Vérité terrain)
        cv::Mat simFrame = map.getImage().clone(); // Copie de la carte originale
        lidar.draw(simFrame, robot);               // Dessin des rayons rouges
        robot.draw(simFrame);                      // Dessin du robot

        // B. Préparation de la vue "Mémoire" (Ce que le robot voit)
        cv::Mat memFrame;
        occupancyGrid.draw(memFrame);              // Conversion de la grille en image
        robot.draw(memFrame);                      // Dessin du robot pour se repérer

        // C. Récupération de la vue "Caméra" (Webcam avec réalité augmentée)
        cv::Mat camFrame = arucoManager.getFrame();

        // D. Calcul des dimensions du tableau de bord global
        // Largeur totale = max(largeur simu + largeur map, largeur caméra)
        int topRowWidth = simFrame.cols + memFrame.cols;
        int totalWidth = std::max(topRowWidth, camFrame.cols);
        // Hauteur totale = hauteur des cartes + hauteur caméra + marge
        int totalHeight = std::max(simFrame.rows, memFrame.rows) + camFrame.rows + 10;

        // Création de l'image vide du tableau de bord (fond gris foncé)
        cv::Mat dashboard = cv::Mat(totalHeight, totalWidth, CV_8UC3, cv::Scalar(40, 40, 40));

        // Copie des images dans le tableau de bord
        // 1. Simulation en haut à gauche
        simFrame.copyTo(dashboard(cv::Rect(0, 0, simFrame.cols, simFrame.rows)));
        
        // 2. Occupancy Grid en haut à droite
        memFrame.copyTo(dashboard(cv::Rect(simFrame.cols, 0, memFrame.cols, memFrame.rows)));
        
        // 3. Caméra centrée en bas
        int camX = (totalWidth - camFrame.cols) / 2; // Calcul pour centrer
        if (camX < 0) camX = 0; // Sécurité
        int camY = std::max(simFrame.rows, memFrame.rows) + 10; // Juste en dessous des cartes
        
        // Copie de l'image caméra
        camFrame.copyTo(dashboard(cv::Rect(camX, camY, camFrame.cols, camFrame.rows)));

        // Affichage final de l'image composée
        cv::imshow(windowName, dashboard);
    }
    
    // Nettoyage à la fin du programme
    cv::destroyAllWindows();
}

// =========================================================
// GETTERS 
// =========================================================

// Retourne une référence constante vers la carte
const Map& Simulation::getMap() const { 
    return map; 
}

// Retourne une référence constante vers le robot 
const Robot& Simulation::getRobot() const { 
    return robot; 
}

// Retourne une référence constante vers le lidar
const Lidar& Simulation::getLidar() const { 
    return lidar; 
}

// Retourne une référence constante vers la grille d'occupation
const OccupancyGrid& Simulation::getOccupancyGrid() const { 
    return occupancyGrid; 
}

// Retourne une référence modifiable vers le robot
// (Nécessaire pour que BehaviorManager puisse modifier l'orientation interne si besoin)
Robot& Simulation::getRobotMutable() { 
    return robot; 
}

// =========================================================
// MÉTHODES PRIVÉES 
// =========================================================

// Initialise la position du robot aléatoirement mais hors des murs
void Simulation::initializeRobotPosition() {
    std::random_device rd;  // Source d'entropie matérielle
    std::mt19937 gen(rd()); // Générateur Mersenne Twister

    int width = map.getWidth();
    int height = map.getHeight();
    int margin = robot.getSize(); // Marge pour ne pas spawner dans un mur bordure

    // Distributions uniformes pour X et Y
    std::uniform_int_distribution<> distX(margin, width - margin);
    std::uniform_int_distribution<> distY(margin, height - margin);

    bool validPositionFound = false; // Drapeau de réussite
    int attempts = 0;                // Compteur de sécurité
    
    // On essaie jusqu'à trouver une place libre ou atteindre 10000 essais
    while (!validPositionFound && attempts < 10000) {
        attempts++;
        // Génère un point candidat aléatoire
        cv::Point candidatePos(distX(gen), distY(gen));
        
        // Vérifie s'il y a collision à cet endroit
        if (!checkCollision(candidatePos)) {
            // Si c'est libre, on valide et on place le robot
            robot.setPosition(candidatePos);
            validPositionFound = true;
            std::cout << "Robot init: [" << candidatePos.x << ", " << candidatePos.y << "]" << std::endl;
        }
    }

    // Si après 10000 essais on n'a pas trouvé de place, on arrête tout
    if (!validPositionFound) {
        std::cerr << "ERREUR : Pas de place libre pour le robot." << std::endl;
        exit(-1);
    }
}

// Vérifie la collision entre le robot (cercle) et les obstacles de la carte (pixels noirs)
bool Simulation::checkCollision(cv::Point centerPos) const {
    int hit_radius = robot.getSize() / 2; // Rayon du robot
    int r2 = hit_radius * hit_radius;     // Rayon au carré (pour éviter les racines carrées)

    // On parcourt un carré autour du centre du robot (Bounding Box)
    for (int y = -hit_radius; y <= hit_radius; ++y) {
        for (int x = -hit_radius; x <= hit_radius; ++x) {
            
            // On ne vérifie que les pixels qui sont DANS le cercle du robot
            // Équation du cercle : x^2 + y^2 <= r^2
            if (x*x + y*y <= r2) {
                // Si le pixel correspondant sur la carte est un obstacle
                if (map.isObstacle(centerPos.x + x, centerPos.y + y)) {
                    return true; // Collision détectée
                }
            }
        }
    }
    return false; // Aucune collision trouvée
}