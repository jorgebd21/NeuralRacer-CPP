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
        adjustedFitness = 0.0f;
        speciesId = -1;
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

    prevLongitudinalVelocity = 0.0f;
    prevLateralVelocity = 0.0f;
    longAccel = 0.0f;
    latAccel = 0.0f;
    prevLongAccel = 0.0f;
    prevLatAccel = 0.0f;
    jerkLong = 0.0f;
    jerkLat = 0.0f;
    slipAngle = 0.0f;
    kammUtilization = 0.0f;
    tireThermals.Reset(75.0f);
    currentSurface = SurfaceType::Asphalt;
    currentSurfaceFriction = 1.0f;

    prevSteerSign = 0;
    steeringZeroCrossings = 0;
    prevAccelInput = 0.0f;
    throttleChatter = 0.0f;

    currentTrackOffset = 0.0f;
    sumTrackOffset = 0.0f;
    trackOffsetSamples = 0;
    lastApexDistance = 0.0f;
    lastApexSpeed = 0.0f;

    currentLapSamples.clear();
    if (fullReset) {
        bestLapSamples.clear();
    }

    for (int i = 0; i < FRONT_HISTORY_SIZE; i++) {
        frontSensorHistory[i] = Config::CAR_FRONT_SENSOR_DIST;
    }
    closestCompetitorId = -1;
    draftingFactor = 0.0f;
    brain.ResetState();
    for (int i = 0; i < 5; i++) {
        float maxDist = (i == 2) ? Config::CAR_FRONT_SENSOR_DIST : Config::CAR_MAX_SENSOR_DIST;
        sensorDistances[i] = maxDist;
        competitorDistances[i] = maxDist;
        competitorClosingVel[i] = 0.0f;
    }
}

void Car::UpdatePhysics(float inputAccelerate, float inputTurn, const FlatSpatialGrid &spatialGrid, int timer, const std::vector<Vector2>& trackCheckpoints, const VehicleSpatialGrid* vehicleGrid, uint32_t myCarIndex) {
    if (isCrashed) return;
    timeAlive++;
    timeSinceLastCheckpoint++;
    currentLapTicks++;

    // 1. Steering dynamics: speed-dependent Ackermann kinematic scaling with angular yaw inertia
    float engineForceEst = (inputAccelerate > 0.0f) ? (inputAccelerate * Config::ENGINE_POWER) : (inputAccelerate * Config::BRAKING_POWER);
    float dragForceEst = -longitudinalVelocity * std::abs(longitudinalVelocity) * Config::DRAG_MULTIPLIER;
    float longAccelEst = (engineForceEst + dragForceEst) / Config::CAR_MASS;
    float turnWeightMultiplier = std::clamp(1.0f - (longAccelEst * Config::WEIGHT_TRANSFER_FACTOR), 0.5f, 1.5f);

    // Realistic steering authority: Full turning authority at cornering speeds (1.0 - 3.5 px/tick)
    // with aerodynamic stability damping above 3.5 px/tick and rolling engagement from rest.
    float speedDamping = 1.0f / (1.0f + 0.15f * std::max(0.0f, std::abs(longitudinalVelocity) - 3.5f));
    float rollingEngagement = std::clamp(std::abs(longitudinalVelocity) / 0.3f, 0.0f, 1.0f);
    float targetYawRate = inputTurn * turnWeightMultiplier * Config::CAR_TURN_SPEED * speedDamping * rollingEngagement;

    // Angular inertia / low-pass filter (eliminates unphysical 1-tick bang-bang steering oscillation exploit)
    angularVelocity = (angularVelocity * 0.65f) + (targetYawRate * 0.35f);
    rotation += angularVelocity;
    prevRotation = rotation;

    // 2. Inertial decomposition: project existing world velocity onto the NEW heading frame
    float rad = rotation * DEG2RAD;
    float localForward = (velocity.x * cos(rad)) + (velocity.y * sin(rad));
    float localLateral = (-velocity.x * sin(rad)) + (velocity.y * cos(rad));

    // 2.5 Multi-Agent Aerodynamic Slipstream Wake & Drafting
    draftingFactor = 0.0f;
    if (vehicleGrid != nullptr && std::abs(localForward) > 0.5f) {
        int vCellSize = vehicleGrid->GetCellSize();
        int searchRadiusCells = static_cast<int>(std::ceil(Config::DRAFTING_MAX_DISTANCE / static_cast<float>(vCellSize)));
        int myCellX = static_cast<int>(std::floor(position.x / static_cast<float>(vCellSize)));
        int myCellY = static_cast<int>(std::floor(position.y / static_cast<float>(vCellSize)));

        Vector2 egoHeading = { cosf(rad), sinf(rad) };

        for (int gx = myCellX - searchRadiusCells; gx <= myCellX + searchRadiusCells; ++gx) {
            for (int gy = myCellY - searchRadiusCells; gy <= myCellY + searchRadiusCells; ++gy) {
                const auto* cell = vehicleGrid->GetCell(gx, gy);
                if (!cell) continue;

                for (uint8_t k = 0; k < cell->count; ++k) {
                    const VehicleOBB& lead = vehicleGrid->GetVehicle(cell->vehicleIndices[k]);
                    if (lead.carIndex == myCarIndex || !lead.active) continue;

                    // Relative vector from ego to lead vehicle
                    Vector2 d = { lead.position.x - position.x, lead.position.y - position.y };
                    float leadRad = lead.rotation * (3.14159265f / 180.0f);
                    Vector2 leadHeading = { cosf(leadRad), sinf(leadRad) };
                    Vector2 leadNormal = { -sinf(leadRad), cosf(leadRad) };

                    // Directional alignment: both cars heading in roughly the same direction
                    float alignment = egoHeading.x * leadHeading.x + egoHeading.y * leadHeading.y;
                    if (alignment < 0.70f) continue;

                    // Lead vehicle must be ahead along heading
                    float dLong = d.x * leadHeading.x + d.y * leadHeading.y;
                    if (dLong > 0.0f && dLong <= Config::DRAFTING_MAX_DISTANCE) {
                        float yLat = std::abs(d.x * leadNormal.x + d.y * leadNormal.y);
                        if (yLat <= Config::DRAFTING_MAX_WIDTH) {
                            float factorLong = (dLong * dLong) / (2.0f * Config::DRAFTING_SIGMA_LONG * Config::DRAFTING_SIGMA_LONG);
                            float factorLat = (yLat * yLat) / (2.0f * Config::DRAFTING_SIGMA_LAT * Config::DRAFTING_SIGMA_LAT);
                            float factor = std::exp(-factorLong - factorLat);
                            if (factor > draftingFactor) {
                                draftingFactor = factor;
                            }
                        }
                    }
                }
            }
        }
    }

    // 3. Friction Circle / Kamm Ellipse Coupling:
    // When cornering or sliding, lateral tire force demand restricts available forward engine traction
    float tractionMultiplier = std::clamp(1.0f + (longAccelEst * Config::WEIGHT_TRANSFER_FACTOR), 0.5f, 1.5f);
    
    currentSurface = spatialGrid.SampleSurface(position);
    currentSurfaceFriction = SurfaceDynamics::GetFrictionCoefficient(currentSurface);
    float rollingResistanceFactor = SurfaceDynamics::GetRollingResistanceFactor(currentSurface);
    float thermalGrip = tireThermals.thermalGripFactor;
    float dirtyAirPenalty = 1.0f - draftingFactor * Config::DRAFTING_DIRTY_AIR_PENALTY;
    float effectiveGripLimit = Config::TIRE_GRIP_LIMIT * currentSurfaceFriction * thermalGrip * dirtyAirPenalty;

    float maxLateralGrip = effectiveGripLimit * Config::CAR_MASS * tractionMultiplier;
    float lateralDemand = std::abs(localLateral) * Config::CORNERING_STIFFNESS;
    float lateralDemandRatio = std::clamp(lateralDemand / std::max(1.0f, maxLateralGrip), 0.0f, 1.0f);
    float tractionBudget = std::sqrt(std::max(0.0f, 1.0f - (lateralDemandRatio * lateralDemandRatio)));

    // Longitudinal forces: Engine/Brakes (governed by traction budget and surface adhesion), Aero Drag (with drafting slipstream reduction), Rolling Resistance, and Induced Turning Drag
    // Longitudinal forces: Engine/Brakes (governed by traction budget and surface adhesion), Aero Drag (with drafting slipstream reduction), Rolling Resistance, and Induced Turning Drag
    float engineForce = 0.0f;
    if (inputAccelerate > 0.0f) {
        // Forward engine propulsion
        engineForce = inputAccelerate * Config::ENGINE_POWER * tractionBudget * currentSurfaceFriction;
    } else if (inputAccelerate < 0.0f) {
        // Pure resistive braking: decelerates the car towards 0, strictly cannot propel in reverse
        float brakeForce = std::abs(inputAccelerate) * Config::BRAKING_POWER * currentSurfaceFriction;
        if (localForward > 0.01f) {
            float maxBrakeForce = localForward * Config::CAR_MASS;
            engineForce = -std::min(brakeForce, maxBrakeForce);
        } else if (localForward < -0.01f) {
            float maxBrakeForce = -localForward * Config::CAR_MASS;
            engineForce = std::min(brakeForce, maxBrakeForce);
        } else {
            engineForce = 0.0f;
            localForward = 0.0f;
        }
    }
    float effectiveDragMultiplier = Config::DRAG_MULTIPLIER * (1.0f - draftingFactor * Config::DRAFTING_MAX_DRAG_REDUCTION);
    float dragForce = -localForward * std::abs(localForward) * effectiveDragMultiplier;
    float rollForce = (std::abs(localForward) > 0.01f) 
        ? -std::copysign(Config::ROLLING_RESISTANCE * rollingResistanceFactor, localForward) 
        : 0.0f;
    float steeringInducedDrag = -localForward * (inputTurn * inputTurn) * (Config::CAR_TURN_PENALTY * Config::CAR_MASS * 0.05f);
    float tireScrubCoeff = Config::TIRE_SCRUB_FACTOR * (1.0f / std::max(0.2f, currentSurfaceFriction));
    float tireScrubForce = (std::abs(localForward) > 0.01f) ? -std::copysign(std::abs(localLateral) * tireScrubCoeff, localForward) : 0.0f;

    float totalLongForce = engineForce + dragForce + rollForce + steeringInducedDrag + tireScrubForce;
    float longitudinalAcceleration = totalLongForce / Config::CAR_MASS;
    localForward += longitudinalAcceleration;

    // Enforce no reverse propulsion: braking and rolling friction cannot invert velocity
    if (inputAccelerate <= 0.0f && localForward < 0.0f) {
        localForward = 0.0f;
    }

    // 4. Lateral tire forces with dynamic friction limit (tire saturation for realistic drift)
    float lateralForce = -localLateral * (Config::CORNERING_STIFFNESS * tractionMultiplier * currentSurfaceFriction * thermalGrip);
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

    // Apex curb oscillation impulse (rumble strip vibration perturbation)
    float curbImpulse = SurfaceDynamics::GetCurbOscillationImpulse(currentSurface);
    if (curbImpulse > 0.0f && std::abs(localForward) > 0.5f) {
        float rumblePhase = std::sin(static_cast<float>(timer) * 1.5f + position.x * 0.2f + position.y * 0.2f);
        localLateral += rumblePhase * curbImpulse * 0.15f;
    }

    // 5. Reconstruct world velocity from updated chassis local velocities
    velocity.x = localForward * cos(rad) - localLateral * sin(rad);
    velocity.y = localForward * sin(rad) + localLateral * cos(rad);
    longitudinalVelocity = localForward;
    lateralVelocity = localLateral;

    // Advanced dynamic acceleration and jerk telemetry
    longAccel = longitudinalVelocity - prevLongitudinalVelocity;
    latAccel = lateralVelocity - prevLateralVelocity;
    jerkLong = longAccel - prevLongAccel;
    jerkLat = latAccel - prevLatAccel;
    prevLongAccel = longAccel;
    prevLatAccel = latAccel;
    prevLongitudinalVelocity = longitudinalVelocity;
    prevLateralVelocity = lateralVelocity;

    // Kamm friction utilization & Slip Angle
    float totalAccelMag = std::sqrt(longAccel * longAccel + latAccel * latAccel);
    float gripBudget = std::max(0.01f, effectiveGripLimit * tractionMultiplier);
    kammUtilization = totalAccelMag / gripBudget;
    slipAngle = std::atan2(lateralVelocity, std::max(0.1f, std::abs(longitudinalVelocity))) * (180.0f / 3.14159265f);

    // Dynamic lumped-parameter tire thermal state update
    tireThermals.Update(longitudinalVelocity, lateralVelocity, slipAngle, tractionMultiplier);

    // Control Ergonomics & Chatter
    int steerSign = (inputTurn > 0.05f) ? 1 : (inputTurn < -0.05f ? -1 : 0);
    if (steerSign != 0 && prevSteerSign != 0 && steerSign != prevSteerSign) {
        steeringZeroCrossings++;
    }
    prevSteerSign = steerSign;
    throttleChatter += std::abs(inputAccelerate - prevAccelInput);
    prevAccelInput = inputAccelerate;

    position.x = position.x + velocity.x;
    position.y = position.y + velocity.y;

    // Check if the car has passed through the target checkpoint
    if (!trackCheckpoints.empty() && nextCheckPointIndex >= 0 && nextCheckPointIndex < (int)trackCheckpoints.size()) {
        int numCp = (int)trackCheckpoints.size();
        Vector2 cpCurr = trackCheckpoints[nextCheckPointIndex];
        Vector2 cpPrev = trackCheckpoints[(nextCheckPointIndex - 1 + numCp) % numCp];

        // Track centerline offset
        float segX = cpCurr.x - cpPrev.x;
        float segY = cpCurr.y - cpPrev.y;
        float segLenSq = segX * segX + segY * segY;
        if (segLenSq > 1.0f) {
            float u = std::clamp(((position.x - cpPrev.x) * segX + (position.y - cpPrev.y) * segY) / segLenSq, 0.0f, 1.0f);
            Vector2 proj = { cpPrev.x + u * segX, cpPrev.y + u * segY };
            float distToCenter = std::sqrt((position.x - proj.x) * (position.x - proj.x) + (position.y - proj.y) * (position.y - proj.y));
            currentTrackOffset = std::clamp(distToCenter / 35.0f, 0.0f, 1.0f);
            sumTrackOffset += currentTrackOffset;
            trackOffsetSamples++;
        }

        float distSq = (position.x - cpCurr.x) * (position.x - cpCurr.x) + (position.y - cpCurr.y) * (position.y - cpCurr.y);
        if (distSq <= Config::CHECKPOINT_RADIUS * Config::CHECKPOINT_RADIUS) {
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
                lastApexDistance = std::sqrt(distSq);
                lastApexSpeed = GetSpeed();
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
                        bestLapSamples = currentLapSamples;
                    }

                    totalLapBonus += lapBonus;
                    currentLapTicks = 0;
                    currentLapSamples.clear();
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

    int cellSize = spatialGrid.GetCellSize();
    int myCellX = (int)std::floor(position.x / static_cast<float>(cellSize));
    int myCellY = (int)std::floor(position.y / static_cast<float>(cellSize));

    // 1. OBB Collision against immediate 3x3 neighboring cells around the vehicle
    for (int gx = myCellX - 1; gx <= myCellX + 1 && !isCrashed; gx++) {
        for (int gy = myCellY - 1; gy <= myCellY + 1 && !isCrashed; gy++) {
            const GridCell* cell = spatialGrid.GetCell(gx, gy);
            if (cell) {
                for (uint32_t k = 0; k < cell->count; ++k) {
                    const auto& wallLine = spatialGrid.GetCellSegment(cell, k);
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

    // 1.5 Inter-Agent Collision Resolution against neighboring vehicles
    if (!isCrashed && vehicleGrid != nullptr) {
        int vCellSize = vehicleGrid->GetCellSize();
        int vCellX = static_cast<int>(std::floor(position.x / static_cast<float>(vCellSize)));
        int vCellY = static_cast<int>(std::floor(position.y / static_cast<float>(vCellSize)));

        Vector2 myCorners[4] = { cornerFL, cornerFR, cornerRR, cornerRL };
        Vector2 myHeading = { cosf(rotation * DEG2RAD), sinf(rotation * DEG2RAD) };

        for (int gx = vCellX - 1; gx <= vCellX + 1 && !isCrashed; ++gx) {
            for (int gy = vCellY - 1; gy <= vCellY + 1 && !isCrashed; ++gy) {
                const auto* cell = vehicleGrid->GetCell(gx, gy);
                if (!cell) continue;

                for (uint8_t k = 0; k < cell->count && !isCrashed; ++k) {
                    const VehicleOBB& comp = vehicleGrid->GetVehicle(cell->vehicleIndices[k]);
                    if (comp.carIndex == myCarIndex || !comp.active) continue;

                    // Broadphase distance check between centers
                    float dx = position.x - comp.position.x;
                    float dy = position.y - comp.position.y;
                    float distSq = dx * dx + dy * dy;
                    if (distSq < 4.0f) continue; // Ignore vehicles overlapping at identical spawn positions
                    constexpr float MAX_CONTACT_DIST = (Config::CAR_HALF_LENGTH * 2.0f);
                    if (distSq > MAX_CONTACT_DIST * MAX_CONTACT_DIST) continue;

                    // Narrowphase: check edge-edge intersections between ego and competitor OBB
                    bool contact = false;
                    for (int me = 0; me < 4 && !contact; ++me) {
                        Vector2 m1 = myCorners[me];
                        Vector2 m2 = myCorners[(me + 1) % 4];
                        for (int ce = 0; ce < 4 && !contact; ++ce) {
                            Vector2 c1 = comp.corners[ce];
                            Vector2 c2 = comp.corners[(ce + 1) % 4];
                            Vector2 hit;
                            if (TrackManager::GetSegmentIntersection(m1, m2, c1, c2, hit)) {
                                contact = true;
                            }
                        }
                    }

                    if (contact) {
                        float dist = std::sqrt(distSq);
                        Vector2 n = (dist > 0.001f) ? Vector2{ dx / dist, dy / dist } : Vector2{ 1.0f, 0.0f };

                        // Relative velocity vector (ego relative to competitor)
                        Vector2 vRel = { velocity.x - comp.velocity.x, velocity.y - comp.velocity.y };
                        float relSpeed = std::sqrt(vRel.x * vRel.x + vRel.y * vRel.y);

                        float compRad = comp.rotation * DEG2RAD;
                        Vector2 compHeading = { cosf(compRad), sinf(compRad) };
                        float headingAlignment = std::abs(myHeading.x * compHeading.x + myHeading.y * compHeading.y);

                        // Terminal condition: violent perpendicular T-bone impact (high speed + low heading alignment)
                        if (relSpeed > 4.5f && headingAlignment < 0.35f) {
                            isCrashed = true;
                            justCrashed = true;
                        } else {
                            // Non-terminal elastic restitution
                            constexpr float RESTITUTION = 0.40f;
                            float vn = vRel.x * n.x + vRel.y * n.y;
                            if (vn < 0.0f) {
                                float impulse = -((1.0f + RESTITUTION) * 0.5f) * vn;
                                velocity.x += impulse * n.x;
                                velocity.y += impulse * n.y;

                                // Position push-back to resolve interpenetration
                                position.x += n.x * 0.75f;
                                position.y += n.y * 0.75f;

                                // Deflection angular yaw perturbation
                                angularVelocity += std::copysign(0.35f, -localLateral);
                            }
                        }
                    }
                }
            }
        }
    }

    // 2. Localized Raycasting: test only grid cells covered by each ray's bounding box
    closestCompetitorId = -1;
    float closestCompDist = 999999.0f;
    for (int i = 0; i < 5; i++) {
        float maxDist = (i == 2) ? Config::CAR_FRONT_SENSOR_DIST : Config::CAR_MAX_SENSOR_DIST;
        competitorDistances[i] = maxDist;
        competitorClosingVel[i] = 0.0f;

        int rMinX = (int)std::floor(std::min(position.x, rayEnd[i].x) / static_cast<float>(cellSize));
        int rMaxX = (int)std::floor(std::max(position.x, rayEnd[i].x) / static_cast<float>(cellSize));
        int rMinY = (int)std::floor(std::min(position.y, rayEnd[i].y) / static_cast<float>(cellSize));
        int rMaxY = (int)std::floor(std::max(position.y, rayEnd[i].y) / static_cast<float>(cellSize));

        for (int gx = rMinX; gx <= rMaxX; gx++) {
            for (int gy = rMinY; gy <= rMaxY; gy++) {
                const GridCell* cell = spatialGrid.GetCell(gx, gy);
                if (cell) {
                    for (uint32_t k = 0; k < cell->count; ++k) {
                        const auto& wallLine = spatialGrid.GetCellSegment(cell, k);
                        float dist;
                        if (TrackManager::GetLineIntersectionDist(position, rayEnd[i], wallLine.first, wallLine.second, dist)) {
                            if (dist < sensorDistances[i]) sensorDistances[i] = dist;
                        }
                    }
                }
            }
        }

        // Multi-Agent Raycasting against competitor vehicles
        if (vehicleGrid != nullptr) {
            int vCellSize = vehicleGrid->GetCellSize();
            int vrMinX = (int)std::floor(std::min(position.x, rayEnd[i].x) / static_cast<float>(vCellSize));
            int vrMaxX = (int)std::floor(std::max(position.x, rayEnd[i].x) / static_cast<float>(vCellSize));
            int vrMinY = (int)std::floor(std::min(position.y, rayEnd[i].y) / static_cast<float>(vCellSize));
            int vrMaxY = (int)std::floor(std::max(position.y, rayEnd[i].y) / static_cast<float>(vCellSize));

            Vector2 rayDir = { rayEnd[i].x - position.x, rayEnd[i].y - position.y };
            float rayLen = std::sqrt(rayDir.x * rayDir.x + rayDir.y * rayDir.y);
            Vector2 rayDirNorm = (rayLen > 0.001f) ? Vector2{ rayDir.x / rayLen, rayDir.y / rayLen } : Vector2{ 0.0f, 0.0f };

            for (int gx = vrMinX; gx <= vrMaxX; gx++) {
                for (int gy = vrMinY; gy <= vrMaxY; gy++) {
                    const auto* vCell = vehicleGrid->GetCell(gx, gy);
                    if (!vCell) continue;
                    for (uint8_t k = 0; k < vCell->count; ++k) {
                        const VehicleOBB& comp = vehicleGrid->GetVehicle(vCell->vehicleIndices[k]);
                        if (comp.carIndex == myCarIndex || !comp.active) continue;

                        for (int e = 0; e < 4; ++e) {
                            Vector2 p1 = comp.GetSegmentStart(e);
                            Vector2 p2 = comp.GetSegmentEnd(e);
                            float dist;
                            if (TrackManager::GetLineIntersectionDist(position, rayEnd[i], p1, p2, dist)) {
                                if (dist > 5.0f) {
                                    if (dist < sensorDistances[i]) {
                                        sensorDistances[i] = dist;
                                    }
                                    if (dist < competitorDistances[i]) {
                                        competitorDistances[i] = dist;
                                        float relVx = comp.velocity.x - velocity.x;
                                        float relVy = comp.velocity.y - velocity.y;
                                        float closingSpeed = -(relVx * rayDirNorm.x + relVy * rayDirNorm.y);
                                        competitorClosingVel[i] = closingSpeed;

                                        if (dist < closestCompDist) {
                                            closestCompDist = dist;
                                            closestCompetitorId = static_cast<int>(comp.carIndex);
                                        }
                                    }
                                }
                            }
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
    //  - Distance * 0.5: forward exploration gradient (strictly forward momentum along track)
    //  - Forward Speed * 1.5: cruising speed incentive to prevent ultra-slow crawling
    if (longitudinalVelocity > 0.15f) {
        distanceTraveled += longitudinalVelocity;
    }
    // Prevent degenerate idling: distance reward before crossing checkpoint 0 is capped
    float effectiveDistance = (totalCheckPointsCrossed > 0) ? distanceTraveled : std::min(distanceTraveled, 80.0f);
    fitness = accumulatedFitness
            + (totalCheckPointsCrossed * 250.0f)
            + totalLapBonus
            + (effectiveDistance * 0.5f)
            + (std::max(0.0f, longitudinalVelocity) * 1.5f);

    // Forward progression filter: must maintain forward velocity (> 0.15 px/tick)
    // Standing still, reversing or oscillating in place accumulates stallTicks
    if (longitudinalVelocity < 0.15f) {
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

    // 6. Record high-frequency telemetry sample if car is actively racing
    if (!isCrashed) {
        float jacobian[OUTPUT_NODES][INPUT_NODES];
        brain.ComputeJacobian(jacobian);
        float sensSteerFront = jacobian[1][2];
        float sensSteerLat = jacobian[1][1] - jacobian[1][3];

        TelemetrySample sample{};
        sample.tick = timer;
        sample.lapTick = currentLapTicks;
        sample.distance = distanceTraveled;
        sample.position = position;
        sample.speed = GetSpeed();
        sample.vLong = longitudinalVelocity;
        sample.vLat = lateralVelocity;
        sample.angularVel = angularVelocity;
        sample.throttleInput = inputAccelerate;
        sample.steerInput = inputTurn;
        sample.longAccel = longAccel;
        sample.latAccel = latAccel;
        sample.kammUtil = kammUtilization;
        sample.slipAngle = slipAngle;
        sample.jerkLong = jerkLong;
        sample.jerkLat = jerkLat;
        sample.trackOffset = currentTrackOffset;
        sample.tireSurfaceTemp = tireThermals.surfaceTemp;
        sample.tireBulkTemp = tireThermals.bulkTemp;
        sample.tireThermalGrip = tireThermals.thermalGripFactor;
        sample.surfaceFriction = currentSurfaceFriction;
        for (int s = 0; s < 5; s++) sample.sensorDistances[s] = sensorDistances[s];
        sample.sensSteerFront = sensSteerFront;
        sample.sensSteerLat = sensSteerLat;
        currentLapSamples.push_back(sample);
    }
}
