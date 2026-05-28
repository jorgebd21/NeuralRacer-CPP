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
    velocity = {0.0f, 0.0f};
    acceleration = {0.0f, 0.0f};
    width = Config::CAR_HALF_WIDTH*2.0f;
    height = Config::CAR_HALF_LENGTH*2.0f;
    isCrashed = false;
    nextCheckPointIndex = -1;
    totalCheckPointsCrossed = 0;
    if (fullReset) {
        accumulatedFitness = 0.0f;
        fitness = 0.0f;
    }
    timeAlive = 0;
    timeSinceLastCheckpoint = 0;
    distanceTraveled = 0.0f;
    for(int i=0; i<5; i++) sensorDistances[i] = 100.0f;
}

void Car::UpdatePhysics(float inputAcelerar, float inputGiro, const std::unordered_map<uint64_t, std::vector<std::pair<Vector2, Vector2>>> &spatialGrid, int timer, const std::vector<std::pair<Vector2, Vector2>>& trackCheckpoints) {
    if (isCrashed) return;
    timeAlive++;
    timeSinceLastCheckpoint++;

    float velocidadLongitudinal = (velocity.x * cos(rotation * DEG2RAD)) + (velocity.y * sin(rotation * DEG2RAD));
    float velocidadLateral = (-velocity.x * sin(rotation * DEG2RAD)) + (velocity.y * cos(rotation * DEG2RAD));

    float fuerzaMotor = 0.0;
    if(inputAcelerar > 0.0f) {
        fuerzaMotor = inputAcelerar * Config::ENGINE_POWER;
    }else{
        fuerzaMotor = -inputAcelerar * Config::BRAKING_POWER;
    }
    float fuerzaDrag = -velocidadLongitudinal * std::abs(velocidadLongitudinal) * Config::DRAG_MULTIPLIER;
    float aceleracionLong = (fuerzaMotor + fuerzaDrag) / Config::CAR_MASS;
    velocidadLongitudinal += aceleracionLong;
    
    float fuerzaLateral = -velocidadLateral * Config::CORNERING_STIFFNESS;
    float aceleracionLateral = fuerzaLateral / Config::CAR_MASS;
    if(std::abs(aceleracionLateral) > std::abs(velocidadLateral)) {
        aceleracionLateral = 0.0f;
    }
    velocidadLateral += aceleracionLateral;

    rotation += inputGiro * Config::CAR_TURN_SPEED * (velocidadLongitudinal > 0 ? 1.0f : -1.0f);

    velocity.x = velocidadLongitudinal * cos(rotation * DEG2RAD) + velocidadLateral * sin(rotation * DEG2RAD);
    velocity.y = velocidadLongitudinal * sin(rotation * DEG2RAD) - velocidadLateral * cos(rotation * DEG2RAD);

    // Guardamos la posicion antigua para comprobar si ha pasado por un checkpoint
    Vector2 oldPosition;
    oldPosition = position;

    position.x = position.x + velocity.x;
    position.y = position.y + velocity.y;

    // Comprobamos si ha pasado por un checkpoint
    if (nextCheckPointIndex == -1) {
        for (size_t i = 0; i < trackCheckpoints.size(); i++) {
            auto cp = trackCheckpoints[i];
            Vector2 interseccion_basura;
            if(TrackManager::GetSegmentIntersection(oldPosition, position, cp.first, cp.second, interseccion_basura)){
                nextCheckPointIndex = (i + 1) % trackCheckpoints.size();
                totalCheckPointsCrossed++;
                timeSinceLastCheckpoint = 0;
                break;
            }
        }
    } else {
        auto cp = trackCheckpoints[nextCheckPointIndex];
        Vector2 interseccion_basura;
        if(TrackManager::GetSegmentIntersection(oldPosition, position, cp.first, cp.second, interseccion_basura)){
            nextCheckPointIndex = (nextCheckPointIndex + 1) % trackCheckpoints.size();
            totalCheckPointsCrossed++;
            timeSinceLastCheckpoint = 0;
        }
    }

    // Incrementamos fitness: premia la velocidad pero penaliza el girar en exceso
    if (GetSpeed() > 0) {
        distanceTraveled += GetSpeed() - (std::abs(inputGiro) * Config::CAR_TURN_PENALTY); 
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
        fitness = accumulatedFitness + distanceTraveled + (totalCheckPointsCrossed * 100) - timeAlive;
        
        // Condiciones de "Muerte" (Crash): 
        // 1. Chocar de frente con pared.
        // 2. Quedarse atascado por mucho tiempo.
        // 3. Estar yendo demasiado rápido marcha atrás (trampas de IA).
        if (sensorDistances[i] < Config::CAR_HALF_WIDTH || timeSinceLastCheckpoint > Config::CAR_MAX_TIME_WITHOUT_CHECKPOINT || GetSpeed() < Config::CAR_STALL_SPEED_THRESHOLD) {
            isCrashed = true; 
        }
    }
}
