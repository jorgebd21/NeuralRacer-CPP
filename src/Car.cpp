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
    
    float maxSpeedForward = 4.0f;
    float maxSpeedBackward = -1.5f;
    float accelRate = 0.04f;
    float brakeRate = 0.1f;
    float friction = 0.015f;

    if (inputAcelerar > 0) speed += accelRate;
    else if (inputAcelerar < 0) speed -= brakeRate;
    else {
        if (speed > 0) { speed -= friction; if (speed < 0) speed = 0; }
        else if (speed < 0) { speed += friction; if (speed > 0) speed = 0; }
    }

    if (speed > maxSpeedForward) speed = maxSpeedForward;
    if (speed < maxSpeedBackward) speed = maxSpeedBackward;

    float turnSpeed = 3.5f;
    if (speed != 0) {
        float direction = (speed > 0) ? 1.0f : -1.0f;
        rotation += inputGiro * turnSpeed * direction * (std::abs(speed) / maxSpeedForward); 
    } 

    position.x += cos(rotation * DEG2RAD) * speed;
    position.y += sin(rotation * DEG2RAD) * speed;

    if (speed > 0) {
        distanceTraveled += speed - (std::abs(inputGiro) * 0.5f); 
    }

    float maxSensorDist = 150.0f;
    for (int i = 0; i < 5; i++) {
        sensorDistances[i] = maxSensorDist;
        float rayAngle = (rotation + Config::SENSOR_ANGLES[i]) * DEG2RAD;
        Vector2 rayEnd = { position.x + cos(rayAngle) * maxSensorDist, position.y + sin(rayAngle) * maxSensorDist };

        for (auto wall : trackWalls) {
            float dist;
            if (TrackManager::GetLineIntersectionDist(position, rayEnd, wall.first, wall.second, dist)) {
                if (dist < sensorDistances[i]) sensorDistances[i] = dist;
            }
        }
        
        fitness = distanceTraveled - timeAlive;
        if (sensorDistances[i] < 5.0f || (timer > 100 && fitness < 0) || speed < -0.2f) {
            isCrashed = true; 
        }
    }
}
