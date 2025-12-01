#include "../include/BehaviorManager.hpp"
#include "../include/Simulation.hpp"
#include "../include/Robot.hpp"
#include "../include/Lidar.hpp"
#include "../include/OccupancyGrid.hpp"
#include <iostream>
#include <cmath>

// =========================================================
// CONSTRUCTEUR
// =========================================================
BehaviorManager::BehaviorManager(Simulation* sim) 
    : simulation(sim),                // Stocke le pointeur vers la simu
      currentBehavior(Behavior::IDLE),// Démarre en mode inactif
      wallFoundForFollowing(false),   // Au début, on cherche un mur
      maneuverState(0),               // État initial de la machine à états
      stepCounter(0),                 // Compteur à 0
      FRONT_WALL_DISTANCE(7.0),       // Seuil de détection mur devant (px)
      SIDE_WALL_DISTANCE(9.0),        // Seuil de détection perte mur côté (px)
      explorationCompleted(false)     // Exploration non finie
{
    // Rien d'autre à initialiser dans le corps du constructeur
}

// =========================================================
// LOGIQUE PRINCIPALE : EXECUTE
// =========================================================

// Méthode centrale appelée à chaque frame pour déterminer le mouvement
void BehaviorManager::execute(int& dx, int& dy, int key) {
    // Par défaut, aucun mouvement
    dx = 0;
    dy = 0;
    
    // Sélectionne l'algo en fonction du mode actuel
    switch(currentBehavior) {
        case Behavior::MANUAL:
            // Mode pilotage clavier
            executeManual(dx, dy, key);
            break;
            
        case Behavior::WALL_FOLLOW:
            // Mode autonome IA
            executeWallFollow(dx, dy);
            break;
            
        default:
            // IDLE : Ne rien faire
            break;
    }
}

// Implémentation du mode MANUEL
void BehaviorManager::executeManual(int& dx, int& dy, int key) {
    // Récupère le robot pour connaître sa vitesse configurée
    const Robot& robot = simulation->getRobot();
    int speed = robot.getSpeed();
    
    // Gestion simple des touches ZQSD (ou flèches selon config)
    switch (key) {
        case 'z': case 'Z': dy = -speed; break; // Haut (Y diminue)
        case 's': case 'S': dy = speed; break;  // Bas (Y augmente)
        case 'q': case 'Q': dx = -speed; break; // Gauche (X diminue)
        case 'd': case 'D': dx = speed; break;  // Droite (X augmente)
    }
}

// Implémentation du mode WALL FOLLOW (L'IA du robot)
void BehaviorManager::executeWallFollow(int& dx, int& dy) {
    
    // 1. VÉRIFICATION DE LA FIN D'EXPLORATION
    // On accède à la grille via la simulation
    const OccupancyGrid& grid = simulation->getOccupancyGrid();
    
    // Si on n'a pas encore fini, on vérifie si la grille est complète
    // (Supposant que OccupancyGrid a une méthode isFullyExplored, sinon retirer cette partie)
     if (!explorationCompleted && grid.isFullyExplored()) {
        explorationCompleted = true;
        std::cout << "\n============================================================" << std::endl;
        std::cout << " CARTE TOTALEMENT EXPLORÉE ! LE ROBOT S'ARRÊTE. " << std::endl;
        std::cout << "============================================================\n" << std::endl;
        
        // Arrêt du robot
        dx = 0; dy = 0;
        return;
    }
    
    if (explorationCompleted) {
        dx = 0; dy = 0;
        return;
    }
    
    
    // 2. LECTURE DES CAPTEURS
    const Lidar& lidar = simulation->getLidar();
    std::vector<double> distances = lidar.readAll(); // Récupère les 360 rayons
    const Robot& robot = simulation->getRobot();
    
    double orientation = robot.getOrientation(); // Angle actuel du robot
    double speed = robot.getSpeed();             // Vitesse de déplacement
    
    // Lecture des distances clés
    double front = distances[180]; // Distance devant (index 180)
    double right = distances[270]; // Distance droite (index 270)
    
    const double WALL_DETECTION_DISTANCE = 10.0; // Seuil pour trouver le premier mur
    
    // 3. PHASE DE RECHERCHE (Tant qu'on n'a pas trouvé de mur)
    if (!wallFoundForFollowing) {
        if (front < WALL_DETECTION_DISTANCE) {
            // MUR TROUVÉ DEVANT !
            wallFoundForFollowing = true;
            
            // On tourne immédiatement à GAUCHE pour mettre le mur à notre DROITE
            // dx = speed * sin(angle), dy = -speed * cos(angle) -> Rotation -90 deg
            dx = static_cast<int>(speed * std::sin(orientation));
            dy = static_cast<int>(-speed * std::cos(orientation));
        } else {
            // PAS DE MUR -> ON AVANCE TOUT DROIT
            // dx = speed * cos(angle), dy = speed * sin(angle) -> Vecteur avant
            dx = static_cast<int>(speed * std::cos(orientation));
            dy = static_cast<int>(speed * std::sin(orientation));
        }
        return; // Fin du tour pour la phase de recherche
    }
    
    // 4. PHASE DE SUIVI (Algorithme Main Droite avec machine à états)

    // Seuils locaux
    const double WALL_DISTANCE = 9.0; // Si dist > 9.0, on a perdu le mur
    const double SAFE_DISTANCE = 7.0; // Si dist < 7.0 devant, on va taper
    
    // PRIORITÉ ABSOLUE : MUR DEVANT (Coin Intérieur)
    if (front < SAFE_DISTANCE) {
        // Virage GAUCHE immédiat pour éviter la collision
        // (On pivote sur place ou en avançant selon la géométrie, ici virage gauche pur)
        dx = static_cast<int>(speed * std::sin(orientation));
        dy = static_cast<int>(-speed * std::cos(orientation));
        
        // Reset de la manœuvre de coin extérieur car on a rencontré un obstacle
        maneuverState = 0;
        stepCounter = 0;
        return;
    }

    // MACHINE À ÉTATS POUR LES COINS EXTÉRIEURS
    
    // État 0 : Suivi normal
    if (maneuverState == 0) {
        if (right > WALL_DISTANCE) {
            // PERTE DU MUR À DROITE -> DÉTECTION DE COIN EXTÉRIEUR
            maneuverState = 1; // Passage en mode "Dégagement"
            stepCounter = 0;
            
            // Pour l'instant, on continue tout droit pour dépasser le coin
            dx = static_cast<int>(speed * std::cos(orientation));
            dy = static_cast<int>(speed * std::sin(orientation));
        } else {
            // MUR PRÉSENT -> ON LONGE TOUT DROIT
            dx = static_cast<int>(speed * std::cos(orientation));
            dy = static_cast<int>(speed * std::sin(orientation));
        }
    }
    // État 1 : Dégagement (Clearance)
    else if (maneuverState == 1) {
        // On continue d'avancer tout droit pendant quelques frames
        dx = static_cast<int>(speed * std::cos(orientation));
        dy = static_cast<int>(speed * std::sin(orientation));
        
        stepCounter++;
        // Après 5 frames (~10-20 pixels), on considère qu'on a dépassé le coin
        if (stepCounter >= 5) { 
            maneuverState = 2; // Passage en mode "Virage"
        }
    }
    // État 2 : Virage à Droite
    else if (maneuverState == 2) {
        // On tourne à DROITE pour contourner le coin
        // dx = -speed * sin, dy = speed * cos -> Rotation +90 deg
        dx = static_cast<int>(-speed * std::sin(orientation));
        dy = static_cast<int>(speed * std::cos(orientation));
        
        maneuverState = 3; // Passage en mode "Stabilisation"
        stepCounter = 0;
    }
    // État 3 : Stabilisation post-virage
    else if (maneuverState == 3) {
        // On avance tout droit dans la NOUVELLE direction
        dx = static_cast<int>(speed * std::cos(orientation));
        dy = static_cast<int>(speed * std::sin(orientation));
        
        stepCounter++;
        // Après 8 frames, on considère qu'on est stabilisé dans le nouveau couloir
        if (stepCounter >= 8) { 
            maneuverState = 0; // Retour au suivi normal
        }
    }
}

// =========================================================
// GESTION DES ÉTATS ET ARUCO
// =========================================================

// Change l'état du robot sur détection d'un Tag
void BehaviorManager::setByArucoId(int arucoId) {
    Behavior newBehavior = Behavior::IDLE;
    
    // Mapping ID -> Comportement
    switch(arucoId) {
        case 0:
            newBehavior = Behavior::MANUAL;
            break;
        case 1:
            newBehavior = Behavior::WALL_FOLLOW;
            break;
        default:
            // Si l'ID n'est pas reconnu, on log l'info mais on ne change rien
            // std::cout << "ArUco ID inconnu: " << arucoId << std::endl;
            return;
    }
    
    // On ne reset que si le comportement change vraiment
    if (newBehavior != currentBehavior) {
        currentBehavior = newBehavior;
        std::cout << ">>> CHANGEMENT COMPORTEMENT: " << getBehaviorName() << std::endl;
        
        // Réinitialise les compteurs de navigation pour partir proprement
        reset();
    }
}

// Réinitialise la mémoire interne (appelé au changement de mode)
void BehaviorManager::reset() {
    wallFoundForFollowing = false; // On devra rechercher un mur
    maneuverState = 0;             // Reset machine à états
    stepCounter = 0;               // Reset compteur
    // Note : On ne reset pas explorationCompleted pour garder la progression
}

// =========================================================
// GETTERS
// =========================================================

Behavior BehaviorManager::getCurrentBehavior() const {
    return currentBehavior;
}

std::string BehaviorManager::getBehaviorName() const {
    switch(currentBehavior) {
        case Behavior::MANUAL:      return "MANUEL";
        case Behavior::WALL_FOLLOW: return "WALL FOLLOWING";
        default:                    return "IDLE";
    }
}