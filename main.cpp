#include "./include/raylib.h"
#include <math.h>
#include <vector>

// --- CONFIGURACIÓN BÁSICA ---
const int SCREEN_WIDTH = 1024;
const int SCREEN_HEIGHT = 768;

// --- ESTRUCTURA DEL COCHE ---
struct Car {
    Vector2 position;
    float rotation; // En grados
    float speed;
    float sensorDistances[5]; // Lo que leerá tu Red Neuronal
    bool isCrashed;

    Car(float startX, float startY) {
        position = {startX, startY};
        rotation = 0.0f;
        speed = 0.0f;
        isCrashed = false;
        for(int i=0; i<5; i++) sensorDistances[i] = 100.0f; // Distancia máxima por defecto
    }
};

// Función auxiliar: Intersección de dos líneas (para los láseres contra los muros)
// Devuelve true si chocan y guarda la distancia exacta.
bool GetLineIntersectionDist(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, float &outDist) {
    float den = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
    if (den == 0) return false;

    float t = ((p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)) / den;
    float u = -((p1.x - p2.x) * (p1.y - p3.y) - (p1.y - p2.y) * (p1.x - p3.x)) / den;

    if (t > 0 && t < 1 && u > 0 && u < 1) {
        // Hay colisión, calculamos la distancia desde p1
        Vector2 pt = { p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y) };
        outDist = sqrt(pow(pt.x - p1.x, 2) + pow(pt.y - p1.y, 2));
        return true;
    }
    return false;
}

int main(void) {
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Simulador Genético - IA Autónoma");
    SetTargetFPS(60);

    // Creamos un coche en la línea de salida
    Car myCar(100.0f, SCREEN_HEIGHT / 2.0f);

    // Definimos un circuito simple (una caja con un muro en medio de ejemplo)
    // En tu proyecto final, puedes añadir más puntos para hacer curvas.
    std::vector<std::pair<Vector2, Vector2>> trackWalls = {
        {{50, 50}, {950, 50}},     // Muro Superior
        {{950, 50}, {950, 700}},   // Muro Derecho
        {{950, 700}, {50, 700}},   // Muro Inferior
        {{50, 700}, {50, 50}},     // Muro Izquierdo
        {{400, 50}, {400, 400}}    // Obstáculo interno
    };

    // Ángulos de los 5 láseres relativos al morro del coche
    float sensorAngles[5] = {-90.0f, -45.0f, 0.0f, 45.0f, 90.0f};

    while (!WindowShouldClose()) {
        
        // ====================================================================
        // 1. ZONA DE INTEGRACIÓN DE TU IA (Aquí entra tu Algoritmo Genético)
        // ====================================================================
        
        if (!myCar.isCrashed) {
            // AHORA MISMO: Control manual para que lo pruebes.
            // TU MISIÓN: Borrar este control manual y hacer que estas variables 
            // las decida la salida (output) de tu Red Neuronal.
            
            float inputAcelerar = 0.0f;
            float inputGiro = 0.0f;

            if (IsKeyDown(KEY_UP)) inputAcelerar = 1.0f;
            if (IsKeyDown(KEY_DOWN)) inputAcelerar = -1.0f;
            if (IsKeyDown(KEY_LEFT)) inputGiro = -1.0f;
            if (IsKeyDown(KEY_RIGHT)) inputGiro = 1.0f;

            // Físicas básicas del coche basadas en los inputs
            myCar.speed += inputAcelerar * 0.1f;
            myCar.speed *= 0.95f; // Fricción
            myCar.rotation += inputGiro * (myCar.speed * 0.5f); 

            // Actualizar posición con trigonometría básica
            myCar.position.x += cos(myCar.rotation * DEG2RAD) * myCar.speed;
            myCar.position.y += sin(myCar.rotation * DEG2RAD) * myCar.speed;
        }

        // ====================================================================
        // 2. ACTUALIZACIÓN DE SENSORES Y COLISIONES (El Mundo)
        // ====================================================================
        
        float maxSensorDist = 150.0f; // Longitud máxima del láser
        for (int i = 0; i < 5; i++) {
            myCar.sensorDistances[i] = maxSensorDist; // Reset a distancia máxima
            
            // Calculamos hacia dónde apunta este láser
            float rayAngle = (myCar.rotation + sensorAngles[i]) * DEG2RAD;
            Vector2 rayEnd = {
                myCar.position.x + cos(rayAngle) * maxSensorDist,
                myCar.position.y + sin(rayAngle) * maxSensorDist
            };

            // Comprobamos si el láser choca contra algún muro
            for (auto wall : trackWalls) {
                float dist;
                if (GetLineIntersectionDist(myCar.position, rayEnd, wall.first, wall.second, dist)) {
                    if (dist < myCar.sensorDistances[i]) {
                        myCar.sensorDistances[i] = dist; // Guardamos la distancia más corta
                    }
                }
            }

            // Si cualquier sensor lee una distancia muy pequeña (ej. < 5 px), el coche ha chocado
            if (myCar.sensorDistances[i] < 5.0f) {
                myCar.isCrashed = true; 
                // AQUÍ LE DARÍAS EL FITNESS FINAL A ESTE COCHE EN TU ALGORITMO GENÉTICO
            }
        }

        // ====================================================================
        // 3. RENDERIZADO GRÁFICO (Lo que ves por pantalla)
        // ====================================================================
        BeginDrawing();
        ClearBackground(DARKGRAY);

        // Dibujar Muros
        for (auto wall : trackWalls) {
            DrawLineEx(wall.first, wall.second, 4.0f, WHITE);
        }

        // Dibujar el coche (un rectángulo rojo) si no ha chocado
        if (!myCar.isCrashed) {
            Rectangle carRect = { myCar.position.x, myCar.position.y, 20.0f, 10.0f };
            Vector2 carOrigin = { 10.0f, 5.0f }; // Centro geométrico
            DrawRectanglePro(carRect, carOrigin, myCar.rotation, RED);

            // Dibujar los sensores láser (en verde o amarillo si detectan algo cerca)
            for (int i = 0; i < 5; i++) {
                float rayAngle = (myCar.rotation + sensorAngles[i]) * DEG2RAD;
                Vector2 actualRayEnd = {
                    myCar.position.x + cos(rayAngle) * myCar.sensorDistances[i],
                    myCar.position.y + sin(rayAngle) * myCar.sensorDistances[i]
                };
                Color rayColor = (myCar.sensorDistances[i] < maxSensorDist) ? ORANGE : GREEN;
                DrawLineV(myCar.position, actualRayEnd, rayColor);
                DrawCircleV(actualRayEnd, 3.0f, rayColor); // Puntito donde choca el láser
            }
        } else {
            DrawText("¡CRASH!", (int)myCar.position.x - 20, (int)myCar.position.y - 20, 20, RED);
        }

        // HUD de ayuda
        DrawText("Usa las flechas para conducir y probar los sensores.", 10, 10, 20, LIGHTGRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}