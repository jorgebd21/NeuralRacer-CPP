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
    width = Config::CAR_HALF_WIDTH * 2.0f;
    height = Config::CAR_HALF_LENGTH * 2.0f;
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
    for(int i = 0; i < 5; i++) sensorDistances[i] = 100.0f;
}

void Car::UpdatePhysics(float inputAccelerate, float inputTurn, const std::unordered_map<uint64_t, std::vector<std::pair<Vector2, Vector2>>> &spatialGrid, int timer, const std::vector<Vector2>& trackCheckpoints) {
    if (isCrashed) return;
    timeAlive++;
    timeSinceLastCheckpoint++;

    float longitudinalVelocity = (velocity.x * cos(rotation * DEG2RAD)) + (velocity.y * sin(rotation * DEG2RAD));
    float lateralVelocity = (-velocity.x * sin(rotation * DEG2RAD)) + (velocity.y * cos(rotation * DEG2RAD));

    float engineForce = 0.0;
    if(inputAccelerate > 0.0f) {
        engineForce = inputAccelerate * Config::ENGINE_POWER;
    } else {
        engineForce = inputAccelerate * Config::BRAKING_POWER;
    }
    float dragForce = -longitudinalVelocity * std::abs(longitudinalVelocity) * Config::DRAG_MULTIPLIER;
    float totalForce = engineForce + dragForce;
    float longitudinalAcceleration = (totalForce / Config::CAR_MASS);
    float turnMultiplier = 1.0f - (longitudinalAcceleration * Config::WEIGHT_TRANSFER_FACTOR);
    turnMultiplier = std::clamp(turnMultiplier, 0.5f, 1.5f);
    longitudinalVelocity += longitudinalAcceleration;

    float tractionMultiplier = 1.0f + (longitudinalAcceleration * Config::WEIGHT_TRANSFER_FACTOR);
    tractionMultiplier = std::clamp(tractionMultiplier, 0.5f, 1.5f);

    float lateralForce = -lateralVelocity * (Config::CORNERING_STIFFNESS * tractionMultiplier);
    float lateralAcceleration = lateralForce / Config::CAR_MASS;
    if(std::abs(lateralAcceleration) > std::abs(lateralVelocity)) {
        lateralAcceleration = 0.0f;
    }
    lateralVelocity += lateralAcceleration;

    rotation += inputTurn * turnMultiplier * Config::CAR_TURN_SPEED * (longitudinalVelocity > 0 ? 1.0f : -1.0f);

    velocity.x = longitudinalVelocity * cos(rotation * DEG2RAD) + lateralVelocity * sin(rotation * DEG2RAD);
    velocity.y = longitudinalVelocity * sin(rotation * DEG2RAD) - lateralVelocity * cos(rotation * DEG2RAD);

    // Store old position to check if a checkpoint has been crossed
    Vector2 oldPosition;
    oldPosition = position;

    position.x = position.x + velocity.x;
    position.y = position.y + velocity.y;

    // Check if the car has passed through a checkpoint
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

    // 1. Pre-calculate base directional vectors
    float noseX = cos(rotation * DEG2RAD) * Config::CAR_HALF_LENGTH;
    float noseY = sin(rotation * DEG2RAD) * Config::CAR_HALF_LENGTH;
    float rightX = -sin(rotation * DEG2RAD) * Config::CAR_HALF_WIDTH;
    float rightY = cos(rotation * DEG2RAD) * Config::CAR_HALF_WIDTH;
    
    // 2. Calculate the 4 corners combining those vectors
    Vector2 cornerFR = { position.x + noseX + rightX, position.y + noseY + rightY }; // Front Right
    Vector2 cornerFL = { position.x + noseX - rightX, position.y + noseY - rightY }; // Front Left
    Vector2 cornerRR = { position.x - noseX + rightX, position.y - noseY + rightY }; // Rear Right
    Vector2 cornerRL = { position.x - noseX - rightX, position.y - noseY - rightY }; // Rear Left

    float baseRayAngle = (rotation + Config::SENSOR_ANGLES[0]) * DEG2RAD;
    Vector2 rayEnd[5];
    for (int i = 0; i < 5; i++) {
        sensorDistances[i] = Config::CAR_MAX_SENSOR_DIST; // Reset impact memory here
        float rayAngle = (rotation + Config::SENSOR_ANGLES[i]) * DEG2RAD;
        rayEnd[i] = { position.x + cos(rayAngle) * Config::CAR_MAX_SENSOR_DIST, position.y + sin(rayAngle) * Config::CAR_MAX_SENSOR_DIST };
    }

    int myCellX = position.x / TrackManager::GRID_CELL_SIZE;
    int myCellY = position.y / TrackManager::GRID_CELL_SIZE;
    for(int gridX = myCellX - 2; gridX <= myCellX + 2; gridX++){
        for(int gridY = myCellY - 2; gridY <= myCellY + 2; gridY++){
            uint64_t key = TrackManager::GetGridKey(gridX, gridY);
            if(spatialGrid.find(key) != spatialGrid.end()){
                for (auto wallLine : spatialGrid.at(key)){
                    if (!isCrashed) {
                        Vector2 unused;
                        bool noseHit = TrackManager::GetSegmentIntersection(cornerFL, cornerFR, wallLine.first, wallLine.second, unused);
                        bool tailHit = TrackManager::GetSegmentIntersection(cornerRL, cornerRR, wallLine.first, wallLine.second, unused);
                        bool rightHit = TrackManager::GetSegmentIntersection(cornerRR, cornerFR, wallLine.first, wallLine.second, unused);
                        bool leftHit = TrackManager::GetSegmentIntersection(cornerRL, cornerFL, wallLine.first, wallLine.second, unused);
                        if (noseHit || tailHit || rightHit || leftHit) isCrashed = true;
                    }
              
                    for (int i = 0; i < 5; i++) {
                        float dist;
                        // Passing rayEnd[i] from the array calculated above
                        if (TrackManager::GetLineIntersectionDist(position, rayEnd[i], wallLine.first, wallLine.second, dist)) {
                            if (dist < sensorDistances[i]) sensorDistances[i] = dist; // Keep the closest collision
                        }
                    }
                }
            }
        }
    }

    // Increment fitness: rewards speed but penalizes excessive turning
    if (GetSpeed() > 0){
        distanceTraveled += GetSpeed() - (std::abs(inputTurn) * Config::CAR_TURN_PENALTY); 
    }
    fitness = accumulatedFitness + distanceTraveled + (totalCheckPointsCrossed * 100.0f);

    float currentLongitudinalVelocity = (velocity.x * cos(rotation * DEG2RAD)) + (velocity.y * sin(rotation * DEG2RAD));
    if (timeSinceLastCheckpoint > Config::CAR_MAX_TIME_WITHOUT_CHECKPOINT || currentLongitudinalVelocity < Config::CAR_STALL_SPEED_THRESHOLD) {
        isCrashed = true; 
    }
}
