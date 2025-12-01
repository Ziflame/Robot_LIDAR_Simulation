#include "../include/ArucoManager.hpp"
#include "../include/BehaviorManager.hpp" 
#include <iostream>


// =========================================================
// CONSTRUCTEUR
// =========================================================
ArucoManager::ArucoManager(BehaviorManager* behaviorMgr) 
    : behaviorManager(behaviorMgr) // Initialise le pointeur vers le gestionnaire de comportement
{
    // Tentative d'ouverture de la caméra (Index 0 = Webcam par défaut)
    // CAP_V4L2 est l'API "Video for Linux 2", souvent plus stable sous Linux
    cap.open(0, cv::CAP_V4L2);

    // Configuration de la caméra pour optimiser la fluidité
    // MJPG permet de compresser les images au niveau matériel pour avoir plus de FPS via USB
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // Définition de la résolution standard 640x480 (suffisant et rapide)
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // Vérification si la caméra est bien accessible
    if (!cap.isOpened()) {
        std::cerr << "ERREUR CRITIQUE : Impossible d'ouvrir la caméra !" <<  std::endl;
        // Si erreur, on crée une image noire vide pour éviter le crash du programme
        currentFrame = cv::Mat::zeros(480, 640, CV_8UC3);
    } else {
         std::cout << "Camera initialisee (V4L2 + MJPG)." << std::endl;
    }

    // Initialisation du dictionnaire ArUco
    // DICT_4X4_50 signifie : Tags de 4x4 bits (simple à détecter), avec 50 IDs possibles (0 à 49)
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    
    // Création des paramètres par défaut pour le détecteur
    parameters = cv::aruco::DetectorParameters::create();
}

// =========================================================
// DESTRUCTEUR
// =========================================================
ArucoManager::~ArucoManager() {
    // Si la caméra est encore ouverte, on la libère pour que d'autres applis puissent l'utiliser
    if (cap.isOpened()) {
        cap.release();
    }
}

// =========================================================
// MÉTHODE PRINCIPALE : CAPTURE ET DETECTION
// =========================================================
void ArucoManager::captureAndDetect() {
    // Si pas de caméra, on ne fait rien
    if (!cap.isOpened()) return;

    cv::Mat rawFrame;
    // Capture d'une nouvelle frame depuis le flux vidéo
    cap >> rawFrame;

    // Vérification si la frame est valide (parfois les premières frames sont vides)
    if (rawFrame.empty()) {
        std::cerr << "Attention: Frame vide." << std::endl;
        return;
    }

    // On clone l'image brute pour travailler dessus sans modifier le buffer interne de la caméra
    currentFrame = rawFrame.clone();

    // Préparation des conteneurs pour les résultats de la détection
    std::vector<int> ids; // Liste des IDs trouvés (ex: [0, 5])
    std::vector<std::vector<cv::Point2f>> corners; // Coordonnées des 4 coins pour chaque tag trouvé
    std::vector<std::vector<cv::Point2f>> rejected; // Candidats rejetés (bruit, formes carrées non valides)

    // Lancement de l'algorithme de détection ArUco
    cv::aruco::detectMarkers(rawFrame, dictionary, corners, ids, parameters, rejected);

    // Si au moins un marqueur a été détecté
    if (ids.size() > 0) {
        // Dessine les contours verts et l'ID sur l'image pour le feedback visuel
        cv::aruco::drawDetectedMarkers(currentFrame, corners, ids);

        // On prend le premier tag détecté pour piloter le robot
        int id = ids[0]; 

        // Si le pointeur vers BehaviorManager est valide, on lui envoie l'ordre
        if (behaviorManager) {
            // Cette méthode va changer l'état du robot (Manuel vs Auto) selon l'ID
            behaviorManager->setByArucoId(id);
        }

        // Dessine l'interface utilisateur (HUD) avec l'ID détecté
        drawOverlay(currentFrame, id);
    } else {
        // Aucun tag vu : on dessine quand même l'interface (avec ID = -1)
        drawOverlay(currentFrame, -1); 
    }
}

// =========================================================
// MÉTHODE PRIVÉE : DESSIN DE L'INTERFACE (HUD)
// =========================================================
void ArucoManager::drawOverlay(cv::Mat& img, int detectedId) {
    // 1. Création d'un bandeau semi-transparent en haut de l'image
    cv::Mat overlay = img.clone();
    
    // Dessine un rectangle noir plein sur le haut de l'image (hauteur 100px)
    cv::rectangle(overlay, cv::Point(0, 0), cv::Point(img.cols, 100), cv::Scalar(0, 0, 0), cv::FILLED);
    
    // Fusionne le rectangle noir avec l'image originale
    // alpha=0.4 (transparence), beta=0.6 (image originale)
    cv::addWeighted(overlay, 0.4, img, 0.6, 0, img);
    
    // 2. Préparation du texte selon l'ID détecté
    std::string tagText;
    cv::Scalar tagColor;

    if (detectedId >= 0) {
        tagText = "Tag detecte: ID " + std::to_string(detectedId);
        tagColor = cv::Scalar(0, 255, 0); // Vert si tag trouvé
    } else {
        tagText = "Aucun tag detecte";
        tagColor = cv::Scalar(0, 0, 255); // Rouge si rien
    }

    // Écriture du statut du Tag
    cv::putText(img, tagText, cv::Point(15, 30), 
            cv::FONT_HERSHEY_SIMPLEX, 0.7, tagColor, 2);
    
    // 3. Affichage du Mode actuel du robot (via BehaviorManager)
    if(behaviorManager) {
        // Récupère le nom du mode (MANUEL, WALL FOLLOWING...)
        std::string modeName = behaviorManager->getBehaviorName();
        std::string modeText = "MODE: " + modeName;
        
        // Choix de la couleur selon le mode pour bien visualiser
        cv::Scalar modeColor;
        if (modeName == "MANUEL") {
            modeColor = cv::Scalar(255, 200, 0); // Cyan/Jaune pour Manuel
        } else if (modeName == "WALL FOLLOWING") {
            modeColor = cv::Scalar(0, 165, 255); // Orange pour Auto
        } else {
            modeColor = cv::Scalar(200, 200, 200); // Gris par défaut
        }
        
        // Écriture du mode en plus gros
        cv::putText(img, modeText, cv::Point(15, 65), 
                cv::FONT_HERSHEY_DUPLEX, 0.8, modeColor, 2);
    }
    
    // 4. Affichage des instructions statiques (Aide utilisateur)
    std::string instructions = "Montrez Tag 0 (Manuel) ou Tag 1 (Wall-Follow)";
    cv::putText(img, instructions, cv::Point(15, 90), 
            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(180, 180, 180), 1);
}

// =========================================================
// GETTERS (Implémentation)
// =========================================================

// Retourne l'image courante (traitée avec l'overlay)
cv::Mat ArucoManager::getFrame() const {
    // Si l'image est vide (caméra non initialisée ou erreur), on retourne du noir
    if (currentFrame.empty()) {
        return cv::Mat::zeros(480, 640, CV_8UC3);
    }
    return currentFrame;
}