#include "Car.h"
#include "Config.h"
#include "raymath.h"
#include <cmath>

#include "TrackManager.h"

Car::Car(float startX, float startY, float startRot) {
    Reset(startX, startY, startRot, true);
}

void Car::Reset(float startX, float startY, float startRot, bool fullReset) {
    position = {startX, startY};
    rotation = startRot;
    speed = 0.0f;
    isCrashed = false;
    if (fullReset) {
        accumulatedFitness = 0.0f;
        fitness = 0.0f;
    }
    timeAlive = 0;
    distanceTraveled = 0.0f;
    for(int i=0; i<5; i++) sensorDistances[i] = 100.0f;
}

void Car::UpdatePhysics(float inputAcelerar, float inputGiro, const std::unordered_map<uint64_t, std::vector<std::pair<Vector2, Vector2>>> &spatialGrid, int timer) {
    if (isCrashed) return;
    timeAlive++;
    
    // Lógica básica de aceleración y frenado
    if (inputAcelerar > 0) speed += Config::CAR_ACCEL_RATE;
    else if (inputAcelerar < 0) speed -= Config::CAR_BRAKE_RATE;
    else {
        // Si no hay input, aplicamos fricción para detener gradualmente el coche
        if (speed > 0) { speed -= Config::CAR_FRICTION; if (speed < 0) speed = 0; }
        else if (speed < 0) { speed += Config::CAR_FRICTION; if (speed > 0) speed = 0; }
    }

    // Limitamos la velocidad a los máximos permitidos (hacia adelante y hacia atrás)
    if (speed > Config::CAR_MAX_SPEED_FORWARD) speed = Config::CAR_MAX_SPEED_FORWARD;
    if (speed < Config::CAR_MAX_SPEED_BACKWARD) speed = Config::CAR_MAX_SPEED_BACKWARD;

    // Solo permitimos girar si el coche está en movimiento.
    // El radio de giro se invierte si vamos marcha atrás (direction = -1).
    if (speed != 0) {
        float direction = (speed > 0) ? 1.0f : -1.0f;
        // Escalar el giro relativo a la velocidad para un control más realista
        rotation += inputGiro * Config::CAR_TURN_SPEED * direction * (std::abs(speed) / Config::CAR_MAX_SPEED_FORWARD); 
    } 

    // Actualizamos posición usando trigonometría simple basándonos en la velocidad y el ángulo actual
    position.x += cos(rotation * DEG2RAD) * speed;
    position.y += sin(rotation * DEG2RAD) * speed;

    // Incrementamos fitness: premia la velocidad pero penaliza el girar en exceso
    if (speed > 0) {
        distanceTraveled += speed - (std::abs(inputGiro) * Config::CAR_TURN_PENALTY); 
    }

    // Cálculo de Raycasting para los 5 sensores (ojos del coche)
    for (int i = 0; i < 5; i++) {
        sensorDistances[i] = Config::CAR_MAX_SENSOR_DIST;
        
        // Calculamos el final teórico del rayo según su ángulo fijo
        float rayAngle = (rotation + Config::SENSOR_ANGLES[i]) * DEG2RAD;
        Vector2 rayEnd = { position.x + cos(rayAngle) * Config::CAR_MAX_SENSOR_DIST, position.y + sin(rayAngle) * Config::CAR_MAX_SENSOR_DIST };

        // Comprobamos intersección del rayo con TODAS las paredes de la pista
        int miCeldaX = position.x / TrackManager::GRID_CELL_SIZE;
        int miCeldaY = position.y / TrackManager::GRID_CELL_SIZE;

        for(int gridX = miCeldaX - 2; gridX <= miCeldaX + 2; gridX++){
            for(int gridY = miCeldaY - 2; gridY<= miCeldaY + 2; gridY++){
                uint64_t key = TrackManager::GetGridKey(gridX, gridY);
                if(spatialGrid.find(key) != spatialGrid.end()){
                    for (auto line : spatialGrid.at(key)){
                        float dist;
                        if (TrackManager::GetLineIntersectionDist(position, rayEnd, line.first, line.second, dist)) {
                            if (dist < sensorDistances[i]) sensorDistances[i] = dist;
                        }
                    }
                }
            }
        }
        
        // El fitness general es la distancia viajada castigada por el tiempo (promueve coches rápidos)
        fitness = accumulatedFitness + distanceTraveled - timeAlive;
        
        // Condiciones de "Muerte" (Crash): 
        // 1. Chocar de frente con pared.
        // 2. Quedarse atascado por mucho tiempo y tener fitness negativo.
        // 3. Estar yendo demasiado rápido marcha atrás (trampas de IA).
        if (sensorDistances[i] < Config::CAR_CRASH_DIST_THRESHOLD || (timer > Config::CAR_STALL_TIME_THRESHOLD && fitness < 0) || speed < Config::CAR_STALL_SPEED_THRESHOLD) {
            isCrashed = true; 
        }
    }
}
