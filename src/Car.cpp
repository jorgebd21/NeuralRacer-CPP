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
    nextCheckPointIndex = 0;
    totalCheckPointsCrossed = 0;
    currentLapTicks = 0;
    if (fullReset) {
        accumulatedFitness = 0.0f;
        fitness = 0.0f;
        totalLapBonus = 0.0f;
        bestLapTicks = 0;
        lapsCompleted = 0;
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
    currentLapTicks++;

    // 1. Steering dynamics: rotate vehicle heading first
    float engineForceEst = (inputAccelerate > 0.0f) ? (inputAccelerate * Config::ENGINE_POWER) : (inputAccelerate * Config::BRAKING_POWER);
    float dragForceEst = -longitudinalVelocity * std::abs(longitudinalVelocity) * Config::DRAG_MULTIPLIER;
    float longAccelEst = (engineForceEst + dragForceEst) / Config::CAR_MASS;
    float turnMultiplier = std::clamp(1.0f - (longAccelEst * Config::WEIGHT_TRANSFER_FACTOR), 0.5f, 1.5f);

    float rotationDelta = inputTurn * turnMultiplier * Config::CAR_TURN_SPEED;
    rotation += rotationDelta;
    angularVelocity = rotationDelta;
    prevRotation = rotation;

    // 2. Inertial decomposition: project existing world velocity onto the NEW heading frame
    // Conserving linear momentum naturally produces the slip angle and centrifugal lateral velocity (v_lat)
    float rad = rotation * DEG2RAD;
    float localForward = (velocity.x * cos(rad)) + (velocity.y * sin(rad));
    float localLateral = (-velocity.x * sin(rad)) + (velocity.y * cos(rad));

    // 3. Longitudinal forces: Engine/Brakes, Aero Drag, and Tire Cornering Scrub Resistance
    float engineForce = (inputAccelerate > 0.0f) ? (inputAccelerate * Config::ENGINE_POWER) : (inputAccelerate * Config::BRAKING_POWER);
    float dragForce = -localForward * std::abs(localForward) * Config::DRAG_MULTIPLIER;
    float scrubForce = -localForward * std::abs(localLateral) * Config::TIRE_SCRUB_FACTOR;
    float totalLongForce = engineForce + dragForce + scrubForce;
    float longitudinalAcceleration = totalLongForce / Config::CAR_MASS;
    localForward += longitudinalAcceleration;

    // 4. Lateral tire forces with dynamic friction limit (tire saturation for realistic drift)
    float tractionMultiplier = std::clamp(1.0f + (longitudinalAcceleration * Config::WEIGHT_TRANSFER_FACTOR), 0.5f, 1.5f);
    float lateralForce = -localLateral * (Config::CORNERING_STIFFNESS * tractionMultiplier);
    float maxLateralGrip = Config::TIRE_GRIP_LIMIT * Config::CAR_MASS * tractionMultiplier;

    if (std::abs(lateralForce) > maxLateralGrip) {
        // Exceeded friction limit: tires saturate into dynamic drift
        lateralForce = std::copysign(maxLateralGrip, lateralForce);
        localLateral += (lateralForce / Config::CAR_MASS);
    } else {
        // Within friction limit: critical damping prevents endless numeric oscillation
        float lateralAcceleration = lateralForce / Config::CAR_MASS;
        if (std::abs(lateralAcceleration) > std::abs(localLateral)) {
            localLateral = 0.0f;
        } else {
            localLateral += lateralAcceleration;
        }
    }

    // 5. Reconstruct world velocity from updated chassis local velocities
    velocity.x = localForward * cos(rad) - localLateral * sin(rad);
    velocity.y = localForward * sin(rad) + localLateral * cos(rad);
    longitudinalVelocity = localForward;
    lateralVelocity = localLateral;

    // Store old position to check if a checkpoint has been crossed
    Vector2 oldPosition = position;

    position.x = position.x + velocity.x;
    position.y = position.y + velocity.y;

    // Check if the car has passed through the target checkpoint
    if (!trackCheckpoints.empty() && nextCheckPointIndex >= 0 && nextCheckPointIndex < (int)trackCheckpoints.size()) {
        Vector2 cpCurr = trackCheckpoints[nextCheckPointIndex];
        float distSq = (position.x - cpCurr.x) * (position.x - cpCurr.x) + (position.y - cpCurr.y) * (position.y - cpCurr.y);
        if (distSq <= Config::CHECKPOINT_RADIUS * Config::CHECKPOINT_RADIUS) {
            int numCp = (int)trackCheckpoints.size();
            Vector2 cpPrev = trackCheckpoints[(nextCheckPointIndex - 1 + numCp) % numCp];
            Vector2 trackDir = { cpCurr.x - cpPrev.x, cpCurr.y - cpPrev.y };
            float trackLen = std::sqrt(trackDir.x * trackDir.x + trackDir.y * trackDir.y);
            if (trackLen > 0.001f) {
                trackDir.x /= trackLen;
                trackDir.y /= trackLen;
            } else {
                trackDir = { 1.0f, 0.0f };
            }

            float carHeadingX = cos(rotation * DEG2RAD);
            float carHeadingY = sin(rotation * DEG2RAD);
            float headingAlignment = carHeadingX * trackDir.x + carHeadingY * trackDir.y;
            float velocityAlignment = (velocity.x * trackDir.x + velocity.y * trackDir.y);

            // Directional validation: vehicle must be oriented and moving in the track progression direction
            if (headingAlignment > 0.2f && velocityAlignment > 0.0f) {
                totalCheckPointsCrossed++;
                timeSinceLastCheckpoint = 0;
                nextCheckPointIndex = (nextCheckPointIndex + 1) % numCp;

                // Normalized Lap Time Bonus with Fastest Lap (Purple Sector) Milestone
                if (totalCheckPointsCrossed % numCp == 0) {
                    lapsCompleted++;
                    int lapTime = currentLapTicks;
                    int targetBudget = numCp * 32; // ~2.2 px/tick nominal pacing budget
                    int ticksSaved = std::max(0, targetBudget - lapTime);
                    float lapBonus = 1500.0f + (8.0f * (float)ticksSaved);

                    // Purple sector: bonus for setting or beating personal best lap time
                    if (bestLapTicks == 0 || lapTime < bestLapTicks) {
                        lapBonus += 500.0f;
                        bestLapTicks = lapTime;
                    }

                    totalLapBonus += lapBonus;
                    currentLapTicks = 0;
                }
            }
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
    //  - Total Lap Bonus: normalized time-to-complete + fastest lap milestones
    //  - Distance * 0.5: smooth exploration gradient (calibrated to prevent lateral weaving)
    //  - Speed * 0.5: cruising speed bonus
    if (GetSpeed() > 0.0f) {
        distanceTraveled += GetSpeed();
    }
    fitness = accumulatedFitness
            + (totalCheckPointsCrossed * 250.0f)
            + totalLapBonus
            + (distanceTraveled * 0.5f)
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
