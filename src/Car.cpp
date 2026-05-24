#include "Car.h"
#include "Config.h"
#include "raymath.h"
#include <cmath>

// Helper global forward declaration para GetLineIntersectionDist
// Esta función ahora estará en TrackManager, pero podemos acceder a ella
// O mejor, la movemos a TrackManager y aquí incluimos TrackManager.h
// Para evitar dependencia circular con TrackManager en este punto, definimos una auxiliar o esperamos a implementar TrackManager
#include "TrackManager.h" 

Car::Car(float startX, float startY, float startRot) {
    Reset(startX, startY, startRot);
}

void Car::Reset(float startX, float startY, float startRot) {
    position = {startX, startY};
    rotation = startRot;
    speed = 0.0f;
    isCrashed = false;
    fitness = 0.0f;
    timeAlive = 0;
    distanceTraveled = 0.0f;
    for(int i=0; i<5; i++) sensorDistances[i] = 100.0f;
}

void Car::UpdatePhysics(float inputAcelerar, float inputGiro, const std::vector<std::pair<Vector2, Vector2>>& trackWalls, int timer) {
    if (isCrashed) return;
    timeAlive++;
    
    if (inputAcelerar > 0) speed += Config::CAR_ACCEL_RATE;
    else if (inputAcelerar < 0) speed -= Config::CAR_BRAKE_RATE;
    else {
        if (speed > 0) { speed -= Config::CAR_FRICTION; if (speed < 0) speed = 0; }
        else if (speed < 0) { speed += Config::CAR_FRICTION; if (speed > 0) speed = 0; }
    }

    if (speed > Config::CAR_MAX_SPEED_FORWARD) speed = Config::CAR_MAX_SPEED_FORWARD;
    if (speed < Config::CAR_MAX_SPEED_BACKWARD) speed = Config::CAR_MAX_SPEED_BACKWARD;

    if (speed != 0) {
        float direction = (speed > 0) ? 1.0f : -1.0f;
        rotation += inputGiro * Config::CAR_TURN_SPEED * direction * (std::abs(speed) / Config::CAR_MAX_SPEED_FORWARD); 
    } 

    position.x += cos(rotation * DEG2RAD) * speed;
    position.y += sin(rotation * DEG2RAD) * speed;

    if (speed > 0) {
        distanceTraveled += speed - (std::abs(inputGiro) * Config::CAR_TURN_PENALTY); 
    }

    for (int i = 0; i < 5; i++) {
        sensorDistances[i] = Config::CAR_MAX_SENSOR_DIST;
        float rayAngle = (rotation + Config::SENSOR_ANGLES[i]) * DEG2RAD;
        Vector2 rayEnd = { position.x + cos(rayAngle) * Config::CAR_MAX_SENSOR_DIST, position.y + sin(rayAngle) * Config::CAR_MAX_SENSOR_DIST };

        for (auto wall : trackWalls) {
            float dist;
            if (TrackManager::GetLineIntersectionDist(position, rayEnd, wall.first, wall.second, dist)) {
                if (dist < sensorDistances[i]) sensorDistances[i] = dist;
            }
        }
        
        fitness = distanceTraveled - timeAlive;
        if (sensorDistances[i] < Config::CAR_CRASH_DIST_THRESHOLD || (timer > Config::CAR_STALL_TIME_THRESHOLD && fitness < 0) || speed < Config::CAR_STALL_SPEED_THRESHOLD) {
            isCrashed = true; 
        }
    }
}
