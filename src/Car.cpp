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
    prevRotation = startRot;
    velocity = {0.0f, 0.0f};
    acceleration = {0.0f, 0.0f};
    width = Config::CAR_HALF_WIDTH * 2.0f;
    height = Config::CAR_HALF_LENGTH * 2.0f;
    isCrashed = false;
    justCrashed = false;
    nextCheckPointIndex = -1;
    totalCheckPointsCrossed = 0;
    if (fullReset) {
        accumulatedFitness = 0.0f;
        fitness = 0.0f;
    }
    timeAlive = 0;
    timeSinceLastCheckpoint = 0;
    distanceTraveled = 0.0f;
    angularVelocity = 0.0f;
    longitudinalVelocity = 0.0f;
    lateralVelocity = 0.0f;
    frontSensorDelta = 0.0f;
    frontSensorHistoryIdx = 0;
    stallTicks = 0;

    for (int i = 0; i < FRONT_HISTORY_SIZE; i++) {
        frontSensorHistory[i] = Config::CAR_FRONT_SENSOR_DIST;
    }
    for (int i = 0; i < 5; i++) {
        sensorDistances[i] = (i == 2) ? Config::CAR_FRONT_SENSOR_DIST : Config::CAR_MAX_SENSOR_DIST;
    }
}

void Car::UpdatePhysics(float inputAccelerate, float inputTurn, const std::unordered_map<uint64_t, std::vector<std::pair<Vector2, Vector2>>> &spatialGrid, int timer, const std::vector<Vector2>& trackCheckpoints) {
    if (isCrashed) return;
    timeAlive++;
    timeSinceLastCheckpoint++;

    float rad = rotation * DEG2RAD;
    longitudinalVelocity = (velocity.x * cos(rad)) + (velocity.y * sin(rad));
    lateralVelocity = (-velocity.x * sin(rad)) + (velocity.y * cos(rad));

    float engineForce = 0.0f;
    if (inputAccelerate > 0.0f) {
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
    if (std::abs(lateralAcceleration) > std::abs(lateralVelocity)) {
        lateralVelocity = 0.0f;
        lateralAcceleration = 0.0f;
    } else {
        lateralVelocity += lateralAcceleration;
    }

    // Steering without sudden reverse inversion when coasting/braking around zero
    float rotationDelta = inputTurn * turnMultiplier * Config::CAR_TURN_SPEED;
    rotation += rotationDelta;
    angularVelocity = rotationDelta;
    prevRotation = rotation;

    // Correct local-to-world rotation projection
    rad = rotation * DEG2RAD;
    velocity.x = longitudinalVelocity * cos(rad) - lateralVelocity * sin(rad);
    velocity.y = longitudinalVelocity * sin(rad) + lateralVelocity * cos(rad);

    // Store old position to check if a checkpoint has been crossed
    Vector2 oldPosition = position;

    position.x = position.x + velocity.x;
    position.y = position.y + velocity.y;

    // Check if the car has passed through a checkpoint
    if (nextCheckPointIndex == -1) {
        for (size_t i = 0; i < trackCheckpoints.size(); i++) {
            Vector2 cp = trackCheckpoints[i];
            float dist = sqrt(pow(position.x - cp.x, 2) + pow(position.y - cp.y, 2));
            if(dist <= Config::CHECKPOINT_RADIUS){
                nextCheckPointIndex = (int)(i + 1) % (int)trackCheckpoints.size();
                totalCheckPointsCrossed++;
                timeSinceLastCheckpoint = 0;
                break;
            }
        }
    } else {
        Vector2 cp = trackCheckpoints[nextCheckPointIndex];
        float dist = sqrt(pow(position.x - cp.x, 2) + pow(position.y - cp.y, 2));
        if(dist <= Config::CHECKPOINT_RADIUS){
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

    Vector2 rayEnd[5];
    for (int i = 0; i < 5; i++) {
        float maxDist = (i == 2) ? Config::CAR_FRONT_SENSOR_DIST : Config::CAR_MAX_SENSOR_DIST;
        sensorDistances[i] = maxDist; // Reset impact memory here
        float rayAngle = (rotation + Config::SENSOR_ANGLES[i]) * DEG2RAD;
        rayEnd[i] = { position.x + cos(rayAngle) * maxDist, position.y + sin(rayAngle) * maxDist };
    }

    int myCellX = (int)std::floor(position.x / TrackManager::GRID_CELL_SIZE);
    int myCellY = (int)std::floor(position.y / TrackManager::GRID_CELL_SIZE);

    // 1. OBB Collision against immediate 3x3 neighboring cells around the vehicle
    for (int gx = myCellX - 1; gx <= myCellX + 1 && !isCrashed; gx++) {
        for (int gy = myCellY - 1; gy <= myCellY + 1 && !isCrashed; gy++) {
            uint64_t key = TrackManager::GetGridKey(gx, gy);
            auto it = spatialGrid.find(key);
            if (it != spatialGrid.end()) {
                for (const auto& wallLine : it->second) {
                    Vector2 unused;
                    if (TrackManager::GetSegmentIntersection(cornerFL, cornerFR, wallLine.first, wallLine.second, unused) ||
                        TrackManager::GetSegmentIntersection(cornerRL, cornerRR, wallLine.first, wallLine.second, unused) ||
                        TrackManager::GetSegmentIntersection(cornerRR, cornerFR, wallLine.first, wallLine.second, unused) ||
                        TrackManager::GetSegmentIntersection(cornerRL, cornerFL, wallLine.first, wallLine.second, unused)) {
                        isCrashed = true;
                        justCrashed = true;
                        break;
                    }
                }
            }
        }
    }

    // 2. Localized Raycasting: test only grid cells covered by each ray's bounding box
    for (int i = 0; i < 5; i++) {
        int rMinX = (int)std::floor(std::min(position.x, rayEnd[i].x) / TrackManager::GRID_CELL_SIZE);
        int rMaxX = (int)std::floor(std::max(position.x, rayEnd[i].x) / TrackManager::GRID_CELL_SIZE);
        int rMinY = (int)std::floor(std::min(position.y, rayEnd[i].y) / TrackManager::GRID_CELL_SIZE);
        int rMaxY = (int)std::floor(std::max(position.y, rayEnd[i].y) / TrackManager::GRID_CELL_SIZE);

        for (int gx = rMinX; gx <= rMaxX; gx++) {
            for (int gy = rMinY; gy <= rMaxY; gy++) {
                uint64_t key = TrackManager::GetGridKey(gx, gy);
                auto it = spatialGrid.find(key);
                if (it != spatialGrid.end()) {
                    for (const auto& wallLine : it->second) {
                        float dist;
                        if (TrackManager::GetLineIntersectionDist(position, rayEnd[i], wallLine.first, wallLine.second, dist)) {
                            if (dist < sensorDistances[i]) sensorDistances[i] = dist;
                        }
                    }
                }
            }
        }
    }

    // Update temporal delta for front sensor (index 2) via circular history buffer
    frontSensorDelta = sensorDistances[2] - frontSensorHistory[frontSensorHistoryIdx];
    frontSensorHistory[frontSensorHistoryIdx] = sensorDistances[2];
    frontSensorHistoryIdx = (frontSensorHistoryIdx + 1) % FRONT_HISTORY_SIZE;

    // Fitness breakdown:
    //  - Checkpoints (250 each): balanced milestone reward
    //  - Distance * 1.0:         dominant continuous progress signal
    //  - Speed * 0.5:            cruising speed bonus
    if (GetSpeed() > 0.0f) {
        distanceTraveled += GetSpeed();
    }
    fitness = accumulatedFitness
            + (totalCheckPointsCrossed * 250.0f)
            + (distanceTraveled * 1.0f)
            + (GetSpeed() * 0.5f);

    // Controlled braking / stall filter
    if (GetSpeed() < 0.1f) {
        stallTicks++;
    } else {
        stallTicks = 0;
    }

    if (!isCrashed && (timeSinceLastCheckpoint > Config::CAR_MAX_TIME_WITHOUT_CHECKPOINT || 
        longitudinalVelocity < Config::CAR_STALL_SPEED_THRESHOLD ||
        stallTicks > Config::CAR_STALL_TIME_THRESHOLD)) {
        isCrashed = true; 
        justCrashed = true;
    }
}
