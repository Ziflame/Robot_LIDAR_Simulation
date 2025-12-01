#include "../include/Simulation.hpp"
#include <iostream>

// =========================================================
// POINT D'ENTRÉE DU PROGRAMME
// =========================================================
int main() {
    
    // 1. Création de l'instance principale de la simulation.
     // Cela va charger la carte, créer le robot, initialiser la fenêtre, etc.
    Simulation sim;
        
    // 2. Lancement de la boucle principale.
    // Cette méthode contient la boucle while(true) et ne rendra la main
    // que lorsque l'utilisateur appuiera sur ESC ou fermera la fenêtre.
    sim.run();

  
    return 0;
}