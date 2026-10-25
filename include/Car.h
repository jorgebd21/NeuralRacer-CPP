#pragma once

#include "Brain.h"
#include "raylib.h"
#include <vector>
#include <utility>
#include <unordered_map>
#include <cmath>

/**
 * @brief Structure representing a car entity in the simulation.
 * 
 * Contains both physical components (position, velocity) and 
 * elements necessary for the genetic algorithm (brain, fitness).
 */
struct Car {
    Vector2 position;
    float rotation;
    Vector2 velocity;
    Vector2 acceleration;

    float width;
    float height;

    float sensorDistances[5];
    bool isCrashed;
    bool justCrashed;
    
    float fitness;
    float accumulatedFitness;
    int timeAlive;
    int timeSinceLastCheckpoint;
    float distanceTraveled;

    int nextCheckPointIndex;
    int totalCheckPointsCrossed;
    
    // Lap time performance & racing line optimization metrics
    float totalLapBonus;
    int currentLapTicks;
    int bestLapTicks;
    int lapsCompleted;
    
    // Physical state tracking for POMDP / Delta state observability
    float prevRotation;
    float angularVelocity;
    float longitudinalVelocity;
    float lateralVelocity;
    float frontSensorDelta;

    // Track lookahead preview for predictive cornering and braking horizon
    float lookaheadAngle;
    float lookaheadDistance;

    static constexpr int FRONT_HISTORY_SIZE = 8;
    float frontSensorHistory[FRONT_HISTORY_SIZE];
    int frontSensorHistoryIdx;
    int stallTicks;

    Brain brain;

    /**
     * @brief Constructor that initializes the car at an initial position and rotation.
     * 
     * @param startX Initial position on the X axis.
     * @param startY Initial position on the Y axis.
     * @param startRot Initial rotation in degrees (default 0.0f).
     */
    Car(float startX, float startY, float startRot = 0.0f);
    
    /**
     * @brief Resets the physical state of the car and resets genetic metrics.
     * 
     * @param startX New initial position on the X axis.
     * @param startY New initial position on the Y axis.
     * @param startRot New initial rotation in degrees (default 0.0f).
     */
    void Reset(float startX, float startY, float startRot = 0.0f, bool fullReset = true);
    
    /**
     * @brief Calculates the scalar magnitude of the current velocity (Pythagorean Theorem).
     */
    float GetSpeed() const { return std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y); }

    /**
     * @brief Returns the average speed across the agent's active lifetime.
     */
    float GetAverageSpeed() const { return timeAlive > 0 ? (distanceTraveled / timeAlive) : 0.0f; }

    /**
     * @brief Computes predictive lookahead curvature angle and distance to upcoming apex.
     * 
     * @param trackCheckpoints Ordered list of circuit checkpoints.
     * @param outAngle Output relative angle in range [-1.0, 1.0] (-180 to +180 deg).
     * @param outDist Output distance to apex normalized [0.0, 1.0] across 350px horizon.
     */
    void CalculateLookahead(const std::vector<Vector2>& trackCheckpoints, float &outAngle, float &outDist) const;

    /**
     * @brief Evaluates the brain using the 11-dimensional state space with predictive lookahead
     * and maps target speed policy to physical pedal pressure (accelerator or brake).
     * 
     * @param outAccelerate Calculated pedal pressure: >0 throttle [0.15, 1.0], <0 brake [-1.0, -0.1].
     * @param outTurn Calculated steering output [-1.0, 1.0].
     * @param trackCheckpoints Optional track checkpoint array for lookahead calculation.
     */
    void Think(float &outAccelerate, float &outTurn, const std::vector<Vector2>& trackCheckpoints = {}) {
        if (!trackCheckpoints.empty()) {
            const_cast<Car*>(this)->CalculateLookahead(trackCheckpoints, 
                const_cast<Car*>(this)->lookaheadAngle, 
                const_cast<Car*>(this)->lookaheadDistance);
        }

        float rawSpeedPolicy = 0.0f;
        brain.Evaluate(sensorDistances, frontSensorDelta, longitudinalVelocity, lateralVelocity, angularVelocity, lookaheadAngle, lookaheadDistance, rawSpeedPolicy, outTurn);

        // Target Speed Mapping:
        // rawSpeedPolicy in [-1.0, 1.0] maps smoothly to [1.6, 5.6] px/tick
        constexpr float MIN_TARGET_SPEED = 1.6f;
        constexpr float MAX_TARGET_SPEED = 5.6f;
        float normTarget = (rawSpeedPolicy + 1.0f) * 0.5f; // [0.0, 1.0]
        float targetSpeed = MIN_TARGET_SPEED + normTarget * (MAX_TARGET_SPEED - MIN_TARGET_SPEED);

        // Longitudinal Driver Controller:
        // Converts target speed error into physical pedal pressure (accelerator or brake)
        float currentSpeed = std::max(0.0f, longitudinalVelocity);
        float speedError = targetSpeed - currentSpeed;

        if (speedError >= 0.0f) {
            // Accelerating to reach target speed
            outAccelerate = std::clamp(speedError / 1.5f, 0.15f, 1.0f);
        } else {
            // Exceeding target speed: apply brake pressure
            outAccelerate = std::clamp(speedError / 1.5f, -1.0f, -0.1f);
        }
    }

    /**
     * @brief Updates the physics of the car according to its inputs and calculates collisions.
     * 
     * @param inputAccelerate Acceleration value (-1.0 to 1.0).
     * @param inputTurn Turn value (-1.0 to 1.0).
     * @param spatialGrid Spatial grid containing the track walls for collision detection and sensors.
     * @param timer The current lifetime of the car in the simulation.
     */
    void UpdatePhysics(float inputAccelerate, float inputTurn, const std::unordered_map<uint64_t, std::vector<std::pair<Vector2, Vector2>>> &spatialGrid, int timer, const std::vector<Vector2>& trackCheckpoints);
};