#ifndef BEHAVIORMANAGER_HPP
#define BEHAVIORMANAGER_HPP

#include <opencv2/opencv.hpp>
#include <string>

// Déclaration anticipée pour éviter les inclusions circulaires
class Simulation; 
class Robot;

// Énumération fortement typée pour définir les états possibles du robot
enum class Behavior {
    MANUAL = 0,      // Le robot est piloté au clavier (ZQSD)
    WALL_FOLLOW = 1, // Le robot suit les murs de manière autonome
    IDLE = -1        // État par défaut (ne fait rien)
};

// La classe BehaviorManager est le "Cerveau" du robot.
// Elle décide du prochain mouvement (dx, dy) en fonction de l'état actuel
// et des données des capteurs (Lidar, Grille).
class BehaviorManager {
public:
    // --- 1. CONSTRUCTEUR ---
    
    // Initialise le manager avec un lien vers la simulation (pour accéder aux capteurs/robot)
    BehaviorManager(Simulation* sim);
    
    // --- 2. MÉTHODES PRINCIPALES (Logique de contrôle) ---
    
    // Change le comportement du robot en fonction de l'ID d'un tag ArUco détecté
    // ID 0 -> Manuel, ID 1 -> Suivi de mur
    void setByArucoId(int arucoId);
    
    // Calcule le déplacement (dx, dy) pour la frame actuelle.
    // Cette méthode est appelée à chaque tour de boucle par Simulation::run()
    void execute(int& dx, int& dy, int key);
    
    // Réinitialise la mémoire interne de l'algorithme de navigation
    // (Utile quand on change de mode ou qu'on redémarre)
    void reset();

    // --- 3. GETTERS (Accesseurs) ---
    
    // Retourne l'état actuel (enum)
    Behavior getCurrentBehavior() const;

    // Retourne le nom de l'état actuel en toutes lettres (pour l'affichage HUD)
    std::string getBehaviorName() const;
    
private:
    // --- MEMBRES (Données) ---
    
    Simulation* simulation;    // Pointeur vers la simulation principale
    Behavior currentBehavior;  // L'état actuel du robot
    
    // Variables pour l'algorithme de suivi de mur
    bool wallFoundForFollowing; // Est-ce qu'on a trouvé le premier mur ?
    int maneuverState;          // État de la manœuvre (0=Suivi, 1=Dégagement, 2=Virage, 3=Stabilisation)
    int stepCounter;            // Compteur pour temporiser les actions (avancer X frames)
    bool explorationCompleted;  // Est-ce que la carte est finie ?

    // Constantes de distances (en pixels)
    const double SIDE_WALL_DISTANCE;  // Distance idéale au mur latéral
    const double FRONT_WALL_DISTANCE; // Distance d'arrêt face à un mur

    // --- MÉTHODES PRIVÉES (Implémentation des algos) ---
    
    // Gère le déplacement manuel via les touches ZQSD
    void executeManual(int& dx, int& dy, int key);

    // Gère l'algorithme autonome de suivi de mur (Main Droite / Main Gauche selon config)
    void executeWallFollow(int& dx, int& dy);
};

#endif // BEHAVIORMANAGER_HPP