#include "Car.h"
#include "Config.h"
#include "raymath.h"
#include <cmath>
#include <algorithm>

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

void Car::UpdatePhysics(float inputAcelerar, float inputGiro, const std::unordered_map<uint64_t, std::vector<std::pair<Vector2, Vector2>>> &spatialGrid, int timer, const std::vector<Vector2>& trackCheckpoints) {
    if (isCrashed) return;
    timeAlive++;
    timeSinceLastCheckpoint++;

    float velocidadLongitudinal = (velocity.x * cos(rotation * DEG2RAD)) + (velocity.y * sin(rotation * DEG2RAD));
    float velocidadLateral = (-velocity.x * sin(rotation * DEG2RAD)) + (velocity.y * cos(rotation * DEG2RAD));

    float fuerzaMotor = 0.0;
    if(inputAcelerar > 0.0f) {
        fuerzaMotor = inputAcelerar * Config::ENGINE_POWER;
    }else{
        fuerzaMotor = inputAcelerar * Config::BRAKING_POWER;
    }
    float fuerzaDrag = -velocidadLongitudinal * std::abs(velocidadLongitudinal) * Config::DRAG_MULTIPLIER;
    float fuerzaTotal = fuerzaMotor + fuerzaDrag;
    float aceleracionLong = (fuerzaTotal / Config::CAR_MASS);
    float multiplicadorGiro = 1.0f - (aceleracionLong * Config::WEIGHT_TRANSFER_FACTOR);
    multiplicadorGiro = std::clamp(multiplicadorGiro, 0.5f, 1.5f);
    velocidadLongitudinal += aceleracionLong;

    float multiplicadorTraccion = 1.0f + (aceleracionLong * Config::WEIGHT_TRANSFER_FACTOR);
    multiplicadorTraccion = std::clamp(multiplicadorTraccion, 0.5f, 1.5f);

    float fuerzaLateral = -velocidadLateral * (Config::CORNERING_STIFFNESS * multiplicadorTraccion);
    float aceleracionLateral = fuerzaLateral / Config::CAR_MASS;
    if(std::abs(aceleracionLateral) > std::abs(velocidadLateral)) {
        aceleracionLateral = 0.0f;
    }
    velocidadLateral += aceleracionLateral;

    rotation += inputGiro * multiplicadorGiro * Config::CAR_TURN_SPEED * (velocidadLongitudinal > 0 ? 1.0f : -1.0f);

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
            Vector2 cp = trackCheckpoints[i];
            float dist = sqrt(pow(position.x - cp.x, 2) + pow(position.y - cp.y, 2));
            if(dist <= 65.0f){
                nextCheckPointIndex = (i + 1) % trackCheckpoints.size();
                totalCheckPointsCrossed++;
                timeSinceLastCheckpoint = 0;
                break;
            }
        }
    } else {
        Vector2 cp = trackCheckpoints[nextCheckPointIndex];
        float dist = sqrt(pow(position.x - cp.x, 2) + pow(position.y - cp.y, 2));
        if(dist <= 65.0f){
            nextCheckPointIndex = (nextCheckPointIndex + 1) % trackCheckpoints.size();
            totalCheckPointsCrossed++;
            timeSinceLastCheckpoint = 0;
        }
    }

    // 1. Pre-calculamos las flechas direccionales base
    float morroX = cos(rotation * DEG2RAD) * Config::CAR_HALF_LENGTH;
    float morroY = sin(rotation * DEG2RAD) * Config::CAR_HALF_LENGTH;
    float derechaX = -sin(rotation * DEG2RAD) * Config::CAR_HALF_WIDTH;
    float derechaY = cos(rotation * DEG2RAD) * Config::CAR_HALF_WIDTH;
    // 2. Calculamos las 4 esquinas combinando esas flechas
    Vector2 esq_DD = { position.x + morroX + derechaX, position.y + morroY + derechaY }; // Delantera Derecha
    Vector2 esq_DI = { position.x + morroX - derechaX, position.y + morroY - derechaY }; // Delantera Izquierda
    Vector2 esq_TD = { position.x - morroX + derechaX, position.y - morroY + derechaY }; // Trasera Derecha
    Vector2 esq_TI = { position.x - morroX - derechaX, position.y - morroY - derechaY }; // Trasera Izquierda

    float rayAngle = (rotation + Config::SENSOR_ANGLES[0]) * DEG2RAD;
    Vector2 rayEnd[5];
    for (int i = 0; i < 5; i++) {
        sensorDistances[i] = Config::CAR_MAX_SENSOR_DIST; // Reseteamos la memoria de impactos aquí arriba
        float anguloRayo = (rotation + Config::SENSOR_ANGLES[i]) * DEG2RAD;
        rayEnd[i] = { position.x + cos(anguloRayo) * Config::CAR_MAX_SENSOR_DIST, position.y + sin(anguloRayo) * Config::CAR_MAX_SENSOR_DIST };
    }

    int miCeldaX = position.x / TrackManager::GRID_CELL_SIZE;
    int miCeldaY = position.y / TrackManager::GRID_CELL_SIZE;
    for(int gridX = miCeldaX - 2; gridX <= miCeldaX + 2; gridX++){
        for(int gridY = miCeldaY - 2; gridY <= miCeldaY + 2; gridY++){
            uint64_t key = TrackManager::GetGridKey(gridX, gridY);
            if(spatialGrid.find(key) != spatialGrid.end()){
                for (auto lineaPared : spatialGrid.at(key)){
                    if (!isCrashed) {
                        Vector2 basura;
                        bool morro = TrackManager::GetSegmentIntersection(esq_DI, esq_DD, lineaPared.first, lineaPared.second, basura);
                        bool culo  = TrackManager::GetSegmentIntersection(esq_TI, esq_TD, lineaPared.first, lineaPared.second, basura);
                        bool der   = TrackManager::GetSegmentIntersection(esq_TD, esq_DD, lineaPared.first, lineaPared.second, basura);
                        bool izq   = TrackManager::GetSegmentIntersection(esq_TI, esq_DI, lineaPared.first, lineaPared.second, basura);
                        if (morro || culo || der || izq) isCrashed = true;
                    }
              
                    for (int i = 0; i < 5; i++) {
                        float dist;
                        // Fíjate cómo pasamos el rayEnd[i] del array que calculamos arriba
                        if (TrackManager::GetLineIntersectionDist(position, rayEnd[i], lineaPared.first, lineaPared.second, dist)) {
                            if (dist < sensorDistances[i]) sensorDistances[i] = dist; // Nos quedamos con el choque más cercano
                        }
                    }
                }
            }
        }
    }

    // Incrementamos fitness: premia la velocidad pero penaliza el girar en exceso
    if (GetSpeed() > 0){
        distanceTraveled += GetSpeed() - (std::abs(inputGiro) * Config::CAR_TURN_PENALTY); 
    }
    fitness = accumulatedFitness + distanceTraveled + (totalCheckPointsCrossed * 1000);

    if (timeSinceLastCheckpoint > Config::CAR_MAX_TIME_WITHOUT_CHECKPOINT || velocity.x < Config::CAR_STALL_SPEED_THRESHOLD) {
        isCrashed = true; 
    }
}
