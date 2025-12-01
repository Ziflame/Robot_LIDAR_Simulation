#ifndef ARUCOMANAGER_HPP
#define ARUCOMANAGER_HPP

// Inclusion des bibliothèques OpenCV pour la capture vidéo et le traitement d'image
#include <opencv2/opencv.hpp>
// Inclusion spécifique pour le module ArUco (Réalité Augmentée / Fiducial Markers)
#include <opencv2/aruco.hpp>
#include <vector>

// Déclaration anticipée de la classe BehaviorManager.
// Cela permet de dire au compilateur que cette classe existe sans inclure tout son fichier .hpp ici,
// ce qui évite les inclusions circulaires (Aruco a besoin de Behavior, Behavior a besoin de Aruco).
class BehaviorManager; 

// La classe ArucoManager gère la caméra et la détection des codes visuels (Tags).
// Elle agit comme l'oeil du robot et envoie des ordres au cerveau (BehaviorManager).
class ArucoManager {
public:
    // --- 1. CONSTRUCTEUR & DESTRUCTEUR ---
    
    // Constructeur : Initialise la caméra et les paramètres de détection ArUco
    // Prend en paramètre un pointeur vers le BehaviorManager pour pouvoir lui envoyer des commandes.
    ArucoManager(BehaviorManager* behaviorMgr);
    
    // Destructeur : Libère proprement les ressources (ferme la caméra)
    ~ArucoManager();

    // --- 2. MÉTHODES PRINCIPALES  ---
    
    // Capture une image, détecte les tags, met à jour le comportement et dessine l'interface
    void captureAndDetect();

    // --- 3. GETTERS  ---
    
    // Retourne l'image actuelle traitée (avec les dessins par-dessus)
    // Utile pour l'affichage dans la fenêtre principale
    cv::Mat getFrame() const;

private:
    // --- MEMBRES ---
    
    BehaviorManager* behaviorManager; // Lien vers le cerveau du robot
    cv::VideoCapture cap;             // Objet OpenCV gérant le flux vidéo physique
    cv::Mat currentFrame;             // La dernière image capturée et traitée
    
    // Paramètres spécifiques à la librairie ArUco
    cv::Ptr<cv::aruco::Dictionary> dictionary;      // Le dictionnaire de tags autorisé 
    cv::Ptr<cv::aruco::DetectorParameters> parameters; // Paramètres de l'algorithme de détection

    // --- MÉTHODES PRIVÉES  ---
    
    // Dessine l'interface utilisateur (HUD) sur l'image : texte, bandeau noir, ID détecté
    void drawOverlay(cv::Mat& img, int detectedId);
};

#endif // ARUCOMANAGER_HPP