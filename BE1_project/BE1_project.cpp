#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_white.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <algorithm>
#include <cmath>

#define M_PI 3.14159265358          // Définition de Pi

#include <random>
static std::default_random_engine engine(10); // random seed = 10
static std::uniform_real_distribution<double> uniform(0, 1);

class Vector {
public:
    explicit Vector(const double &X = 0, const double &Y = 0, const double &Z = 0) {
        x = X;
        y = Y;
        z = Z;
    };
    double x;
    double y;
    double z;
    double normeCarre() {
        return x * x + y * y + z * z;
    }
};

Vector operator+(const Vector &a, const Vector &b) {
    Vector c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    c.z = a.z + b.z;
    return c;
}

Vector operator-(const Vector &a, const Vector &b) {
    Vector c(a.x - b.x, a.y - b.y, a.z - b.z);
    return c;
}

Vector operator*(double a, const Vector &b) {
    Vector c;
    c.x = a * b.x;
    c.y = a * b.y;
    c.z = a * b.z;
    return c;
}

Vector operator*(const Vector &b, double a) {
    Vector c;
    c.x = a * b.x;
    c.y = a * b.y;
    c.z = a * b.z;
    return c;
}

Vector operator*(const Vector &a, const Vector &b) {
    Vector c;
    c.x = a.x * b.x;
    c.y = a.y * b.y;
    c.z = a.z * b.z;
    return c;
}

Vector operator/(const Vector &b, double a) {
    Vector c(b.x/a, b.y/a, b.z/a);
    return c;
}

double dot(const Vector &a, const Vector &b) {
    return a.x * b.x + a.y*b.y + a.z*b.z;
}

Vector Normalisation(const Vector &a) {
    Vector c;
    double norme;
    norme = sqrt((a.x * a.x) + (a.y * a.y) + (a.z * a.z));
    c.x = a.x / norme ;
    c.y = a.y / norme;
    c.z = a.z / norme;
    return c;
}

Vector prodVect(const Vector& a, const Vector& b) {      // produit vectoriel de deux vecteurs
    Vector c;
    c.x = a.y * b.z - a.z * b.y;
    c.y = a.z * b.x - a.x * b.z;
    c.z = a.x * b.y - a.y * b.x;
    return c;
}

double variableGaussX(double u1, double u2) {
    return sqrt(-2 * log(u1)) * cos(2 * M_PI * u2);
}

double variableGaussY(double u1, double u2) {
    return sqrt(-2 * log(u1)) * sin(2 * M_PI * u2);
}


class Ray {
public:
    Ray(const Vector& C, const Vector& D) : C(C), D(D) {}
    Vector C;   // Centre
    Vector D;   // Direcion
};

class Sphere {
public:
    explicit Sphere(const Vector& Ce, const double& R, const Vector& Albedo, const bool& isMirrorSphere = false, const bool& isTransparentSphere = false) : Ce(Ce), R(R), Albedo(Albedo), isMirrorSphere(isMirrorSphere), isTransparentSphere(isTransparentSphere){
    }
    bool intersection(const Ray& rayon, Vector& P, Vector& N, double &x, bool &isMirror, bool &isTransparent) {
        isMirror = isMirrorSphere;
        isTransparent = isTransparentSphere;
        double a = 1;
        double b = 2 * dot(rayon.D, (rayon.C - Ce));
        double c = ((rayon.C - Ce).normeCarre() - R * R);
        double delta = b * b - 4 * a * c;
        if (delta < 0) {                            // Pas d'intersection de la lumière
            return false;
        }
        else {         // Intersection : devant ou derrière la caméra ?
            double x1 = (-b - sqrt(delta))/(2*a);
            double x2 = (-b + sqrt(delta))/(2*a);   // x2 > x1 donc plus loin
            if (x2 < 0) {                   // Les deux intersections sont derrières la caméra
                return false;
            }
            else {                          // Au moins une intersection devant la caméra
                if (x1 > 0) {               // x1 (le plus proche) est devant ?
                    x = x1;
                }
                else {                      // sinon on prend x2 (le plus éloigné)
                    x = x2;
                }
                P = rayon.C + x * rayon.D;  // Point d'intersection rayon-sphère
                N = Normalisation(P - Ce);  // Vecteur normal à la sphère, normalisé
                return true;
            }            
        }
    }
    Vector Ce;  // Centre
    double R;   // Rayon
    Vector Albedo;  // Propriété de couleurs
    bool isMirrorSphere;
    bool isTransparentSphere;
};

class Scene {
public:
    Scene() {}
    std::vector<Sphere> Spheres;                    // Liste des sphères de la scène
    bool intersection(const Ray& rayon, Vector& P, Vector& N, Vector& Albedo, double &X, bool &isMirroir, bool &isTransparent) {
        isMirroir = false;
        isTransparent = false;
        bool isGlobalIntersected = false;
        double GlobalX = 1E99;                      // Le point d'intersection x qui nous interesse (le minimum)
        for (int i = 0; i < Spheres.size(); i++) {
            Vector Pi, Ni;
            double LocalX;
            bool miroir, transparent;
            if (Spheres[i].intersection(rayon, Pi, Ni, LocalX, miroir, transparent)) {
                isGlobalIntersected = true;
                if (LocalX < GlobalX) {         // Si on trouve un point plus proche, on prend les valeurs locales
                    isMirroir = miroir;
                    isTransparent = transparent;
                    GlobalX = std::min(LocalX, GlobalX);      
                    P = Pi;
                    N = Ni;
                    Albedo = Spheres[i].Albedo;

                }
            }
        }
        X = GlobalX;
        return isGlobalIntersected;
    }
    void addSphere(const Sphere& s) {
        Spheres.push_back(s);
    }

    Vector getColor(const Ray& rayon, int rebond) {
        if (rebond > nbRebondsMax) {
            return Vector(0, 0, 0);
        }
        else {
            Vector P, N, AlbedoSphere;
            double Xsphere;
            // P : point d'intersection rayon-sphère le plus proche, N vecteur normal, Albedo de la sphère affichée
            bool isMirror, isTransparent;
            bool isIntersected = intersection(rayon, P, N, AlbedoSphere, Xsphere, isMirror, isTransparent);                
            if (isIntersected) {
                if (isMirror) {                 // Si miroir
                    Ray rayonReflechi(P+0.0001*N, rayon.D - 2 * dot(rayon.D, N) * N);
                    return getColor(rayonReflechi, rebond + 1);
                }
                else if (isTransparent) {           // Si transparence
                    Ray rayonRefracte(Vector(0, 0, 0), Vector(0, 0, 0));
                    if (dot(rayon.D, N) < 0) {      // Rayon entrant dans la sphère
                        if ((1 - ((milieuAir / milieuSphere) * (milieuAir / milieuSphere) * (1 - dot(rayon.D, N) * dot(rayon.D, N)))) >= 0) {
                            rayonRefracte.D = (milieuAir / milieuSphere) * (rayon.D - dot(rayon.D, N) * N) - N * sqrt(1 - ((milieuAir / milieuSphere) * (milieuAir / milieuSphere) * (1 - dot(rayon.D, N) * dot(rayon.D, N))));
                            rayonRefracte.C = P - 0.0001 * N;
                        }
                        else {
                            return Vector(0, 0, 0);
                        }
                    }
                    else {                           // Rayon sortant -> N devient -N
                        if ((1 - ((milieuSphere / milieuAir) * (milieuSphere / milieuAir) * (1 - dot(rayon.D, N) * dot(rayon.D, N)))) >= 0) {
                            rayonRefracte.D = (milieuSphere / milieuAir) * (rayon.D - dot(rayon.D, N) * N) + N * sqrt(1 - ((milieuSphere / milieuAir) * (milieuSphere / milieuAir) * (1 - dot(rayon.D, N) * dot(rayon.D, N))));
                            rayonRefracte.C = P + 0.0001 * N;
                        }
                        else {
                            return Vector(0, 0, 0);
                        }
                    }
                    return getColor(rayonRefracte, rebond + 1);
                }
                else {
                    // Eclairage direct
                    Vector PL = L - P;
                    double d = sqrt(PL.normeCarre());           // distance de P à la lumière
                    Ray ShadowRay(P + 0.0001 * N, (L - P) / d);          // 0.001N pour soigner le bruit
                    Vector ShadowP, ShadowN, ShadowAlbedo;
                    double ShadowX;
                    bool ShadowMirror, ShadowTransparent;
                    bool ombre = intersection(ShadowRay, ShadowP, ShadowN, ShadowAlbedo, ShadowX, ShadowMirror, ShadowTransparent);
                    
                    // Eclairage indirect
                    Vector couleurEclairageIndirect = getEclairageIndirect(N, P, AlbedoSphere, rebond);
                    if (ombre && ShadowX < d) {             // S'il y a qqc entre P et la lumière, alors ombre
                        return Vector(0, 0, 0) + couleurEclairageIndirect;
                    }
                    else {
                        // pixels lus de haut en bas : i => (H-i-1)
                        // max(0, dot(...)) pour éviter une intensité négative si la lumière est derrière la sphère
                        Vector couleurSortie;
                        couleurSortie.x = std::min(255., std::pow(((I / (4 * M_PI * d * d)) * (AlbedoSphere.x / M_PI) * std::max(0., dot(N, PL / d))), 1 / gamma));
                        couleurSortie.y = std::min(255., std::pow(((I / (4 * M_PI * d * d)) * (AlbedoSphere.y / M_PI) * std::max(0., dot(N, PL / d))), 1 / gamma));
                        couleurSortie.z = std::min(255., std::pow(((I / (4 * M_PI * d * d)) * (AlbedoSphere.z / M_PI) * std::max(0., dot(N, PL / d))), 1 / gamma));
                        return couleurSortie + couleurEclairageIndirect;
                    }
                }
            }
            else {          // Plan noir car pas d'intersection
                return Vector(0, 0, 0);
            }
        }
    }

    Vector getEclairageIndirect(const Vector& N, const Vector& P, const Vector& AlbedoSphere, int rebond) {
        double x1 = uniform(engine);
        double x2 = uniform(engine);
        Vector direction_aleatoire_repere_local(cos(2 * M_PI * x1) * sqrt(1 - x2), sin(2 * M_PI * x1) * sqrt(1 - x2), sqrt(x2));
        Vector aleatoire(uniform(engine)-0.5, uniform(engine)-0.5, uniform(engine)-0.5);
        Vector tangent1 = Normalisation(prodVect(N, aleatoire));
        Vector tangent2 = prodVect(tangent1, N);
        Vector direction_aleatoire = direction_aleatoire_repere_local.z * N + direction_aleatoire_repere_local.x * tangent1 + direction_aleatoire_repere_local.y * tangent2;
        Ray rayon_aleatoire(P + 0.001 * N, direction_aleatoire);
        return getColor(rayon_aleatoire, rebond + 1) * AlbedoSphere;
    }


    Vector L;
    double I;
    double gamma;
    double milieuAir;
    double milieuSphere;
    int nbRebondsMax;
};

int main() {
    int W = 512;
    int H = 512;

    Sphere S1(Vector(-10, 0, 0), 10, Vector(1., 1., 1.), false, false); // (Centre, Rayon, Albedo, Miroir ?, Transparente ?)
    Sphere S2(Vector(10, 0, -20), 10, Vector(0, 1, 0));
    Sphere Ssol(Vector(0, -2000, 0), 1990, Vector(0.6, 0.6, 0.6));      // sol
    Sphere Splafond(Vector(0, 2100, 0), 1960, Vector(0, 0, 1));          // plafond
    Sphere Sderrier(Vector(0, 0, 1000), 940, Vector(1, 0, 0));          // Ajout d'un mur derrière la caméra
    Sphere Sdevant(Vector(0, 0, -1000), 940, Vector(1, 0, 1));          // Ajout d'un mur de fond
    Sphere Scote1(Vector(-2000, 0, 0), 1940, Vector(1, 1, 0));
    Sphere Scote2(Vector(2000, 0, 0), 1940, Vector(0, 1, 1));

    Scene scene;
    scene.addSphere(S1);
    scene.addSphere(S2);
    scene.addSphere(Ssol);
    scene.addSphere(Splafond);
    scene.addSphere(Sderrier);
    scene.addSphere(Sdevant);
    scene.addSphere(Scote1);
    scene.addSphere(Scote2);

    scene.gamma = 2.2;                         // Correction de couleur

    Vector C(0, 0, 55);                         // Caméra
    double champVisuel = 60 * M_PI / 180;       // Champ visuel 60° converti en radian
    double profondeurChamp = 55.;               // Distance de la profondeur de champ (pour voir l'objet net)

    scene.L = Vector(-10, 20, 40);              // Source de la lumière
    scene.I = 1E9;                              // Intensité de la lumière
    scene.milieuAir = 1.0;                      // Indices des milieux pour la transparence
    scene.milieuSphere = 1.4;
    scene.nbRebondsMax = 2;                     // Nombre de rebonds maximum des rayons
    int nbRayonsParPixel = 80;

    std::vector<unsigned char> image(W * H * 3, 0);
#pragma omp parallel for
    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {

            Vector color;
            for (int k = 0; k < nbRayonsParPixel; k++) {

                // anti aliasing
                double u1, u2, x, y;
                u1 = uniform(engine);
                u2 = uniform(engine);
                x = variableGaussX(u1, u2) + 0.5;
                y = variableGaussY(u2, u2) + 0.5;

                // Création de la direction u de la caméra vers le pixel
                Vector u(j - W / 2. + x, i - H / 2. + y, -H / (2 * tan(champVisuel / 2)));
                u = Normalisation(u);               // u normalisé
                // Ray rayonScene(C, u);               // Rayon qui part de la caméra vers le pixel

                // profondeur de champ
                double v1, v2;
                v1 = (uniform(engine) - 0.5) * 5.0;
                v2 = (uniform(engine) - 0.5) * 5.0;
                Vector planMiseAuPoint(C.x + (u * profondeurChamp).x, C.y + (u * profondeurChamp).y, C.z + (u * profondeurChamp).z); // plan de mise au point
                Vector OrigineRayon(C.x+v1,C.y+v2, C.z);                                        // Origine du rayon décalé

                Vector v = Normalisation(planMiseAuPoint - OrigineRayon);// direction du pixel de la camera (C) vers le plan de mise au point
                Ray rayonScene(OrigineRayon, v);

                color = color + (scene.getColor(rayonScene, 0) / nbRayonsParPixel);              // 0 rebond 
            }

            image[((H - i - 1) * W + j) * 3 + 0] = color.x;
            image[((H - i - 1) * W + j) * 3 + 1] = color.y;
            image[((H - i - 1) * W + j) * 3 + 2] = color.z;
        }
    }
    stbi_write_png("imageProfondeurChamp.png", W, H, 3, &image[0], 0);

    return 0;
}