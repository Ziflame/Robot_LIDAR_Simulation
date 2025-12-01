#include "../include/OccupancyGrid.hpp"

// =========================================================
// CONSTRUCTEUR
// =========================================================
OccupancyGrid::OccupancyGrid(int w, int h, int cellS) 
    // Initialisation des membres via la liste d'initialisation
    : width(w), height(h), cellSize(cellS) 
{
    // Calcul de la largeur de la grille (ex: 500px / 10 = 50 cases)
    gridW = width / cellSize;
    
    // Calcul de la hauteur de la grille (ex: 200px / 10 = 20 cases)
    gridH = height / cellSize;
    
    // Création de la matrice image (niveau de gris 8 bits - 1 canal)
    // On remplit tout avec 127 (Gris) pour dire "Zone Inconnue" au départ
    grid = cv::Mat(gridH, gridW, CV_8UC1, cv::Scalar(127));
}

// =========================================================
// MISE À JOUR DE LA CARTE (Update)
// =========================================================
void OccupancyGrid::update(const std::vector<cv::Point>& scanPoints, cv::Point robotPos) {
    // Conversion de la position réelle du robot en coordonnées "Grille"
    // (ex: Robot à 105,105 avec cellSize=10 devient Case 10,10)
    cv::Point gridRobot = robotPos / cellSize;

    // On parcourt chaque point d'impact détecté par le Lidar
    for (const auto& point : scanPoints) {
        
        // Conversion du point d'impact réel en coordonnées "Grille"
        cv::Point gridHit = point / cellSize;
        
        // Vérification de sécurité : si le point est hors de la grille, on l'ignore
        if (gridHit.x < 0 || gridHit.x >= gridW || gridHit.y < 0 || gridHit.y >= gridH) {
            continue;
        }

        // Création d'un itérateur de ligne (Algorithme de Bresenham optimisé par OpenCV)
        // Cet outil va nous donner la liste de toutes les cases entre le robot et l'impact
        cv::LineIterator it(grid, gridRobot, gridHit);

        // On parcourt chaque case le long du rayon laser
        for (int i = 0; i < it.count; i++, ++it) {
            // Récupère les coordonnées de la case courante
            cv::Point cell = it.pos();
            
            // Seconde vérification de sécurité (pour ne pas écrire hors mémoire)
            if (cell.x < 0 || cell.x >= gridW || cell.y < 0 || cell.y >= gridH) {
                continue;
            }

            // Vérifie si on est arrivé à la dernière case du rayon (là où ça a tapé)
            bool isEnd = (i == it.count - 1);

            if (isEnd) {
                // --- GESTION DE L'OBSTACLE (Fin du rayon) ---
                
                // Calcul de la distance réelle pour valider l'obstacle
                double dist = cv::norm(point - robotPos);
                
                // Si l'impact est à moins de 98 pixels (portée max théorique ~100),
                // on considère que c'est un vrai mur et pas juste la limite du capteur.
                if (dist < 98.0) {
                    // On marque la case comme OBSTACLE (0 = Noir)
                    grid.at<uchar>(cell) = 0; 
                }
            } else {
                // --- GESTION DE L'ESPACE LIBRE (Le long du rayon) ---
                
                // Si le rayon passe par ici, c'est que c'est vide.
                // PROTECTION CRITIQUE : On ne remplace JAMAIS un obstacle (0) par du vide.
                // Si la case est déjà noire (0), on ne fait rien.
                // Sinon (Gris ou Blanc), on la marque comme LIBRE (255 = Blanc).
                if (grid.at<uchar>(cell) != 0) {
                    grid.at<uchar>(cell) = 255;
                }
            }
        }
    }
}

// =========================================================
// NETTOYAGE DE LA CARTE (Post-Processing)
// =========================================================
void OccupancyGrid::smoothGrid(int iterations) {
    // 1. Création d'un masque binaire où les obstacles sont blancs (255) et le reste noir (0)
    // L'opérateur (grid == 0) crée cette image binaire automatiquement
    cv::Mat obstacleMask = (grid == 0);
    
    // 2. Définition de l'élément structurant (un carré de 3x3 pixels)
    // C'est la forme qu'on va utiliser pour "boucher les trous"
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    
    // 3. Application de la "Fermeture" (Closing) morphologique
    // Fermeture = Dilatation suivie d'une Érosion.
    // Cela permet de relier les points noirs proches et de combler les petits interstices.
    cv::morphologyEx(obstacleMask, obstacleMask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), iterations);
    
    // 4. Réapplication du masque nettoyé sur la grille principale
    // Partout où le masque dit "Obstacle", on force la grille à 0 (Noir).
    grid.setTo(0, obstacleMask);
}

// =========================================================
// ANALYSE D'EXPLORATION
// =========================================================
bool OccupancyGrid::isFullyExplored() const {
    
    int unexploredCount = 0; // Compteur de cases grises
    
    // Parcours de toute la grille
    for (int y = 0; y < gridH; y++) {
        for (int x = 0; x < gridW; x++) {
            // Lecture de la valeur du pixel
            uchar value = grid.at<uchar>(y, x);
            
            // Si c'est gris (127), c'est inexploré
            if (value == 127) { 
                unexploredCount++;
            }
        }
    }
    
    // Calcul du ratio : (Nombre de cases grises) / (Nombre total de cases)
    int totalCells = gridW * gridH;
    double unexploredRatio = (double)unexploredCount / totalCells;
    
    // Si moins de 31.1% de la carte est grise, on considère que c'est fini.
    // (Valeur empirique pour tolérer les zones inaccessibles derrière les murs)
    return unexploredRatio < 0.311; 
}

// =========================================================
// AFFICHAGE (Rendu Graphique)
// =========================================================
void OccupancyGrid::draw(cv::Mat& displayImage) {
    // Initialise l'image de sortie avec un fond gris
    // CV_8UC3 = Image couleur 3 canaux (pour pouvoir afficher en BGR)
    displayImage = cv::Mat(height, width, CV_8UC3, cv::Scalar(127, 127, 127));

    // Parcours de chaque case de la grille logique
    for (int y = 0; y < gridH; y++) {
        for (int x = 0; x < gridW; x++) {
            
            // Récupère la valeur logique (0, 127, 255)
            uchar value = grid.at<uchar>(y, x);
            cv::Scalar color;
            
            // Choix de la couleur d'affichage
            if (value == 255) {
                color = cv::Scalar(255, 255, 255); // Blanc pour Libre
            } else if (value == 0) {
                color = cv::Scalar(0, 0, 0);       // Noir pour Obstacle
            } else {
                color = cv::Scalar(127, 127, 127); // Gris pour Inconnu
            }

            // Calcul du rectangle correspondant sur l'image finale
            // On multiplie par cellSize pour "upscaler" la grille vers la résolution d'écran
            cv::Rect rect(x * cellSize, y * cellSize, cellSize, cellSize);
            
            // Dessin du rectangle plein
            cv::rectangle(displayImage, rect, color, cv::FILLED);
        }
    }
}

// =========================================================
// GETTERS
// =========================================================

// Retourne une référence vers la matrice brute
cv::Mat& OccupancyGrid::getGrid() {
    return grid;
}