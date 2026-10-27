#pragma once

#include "Brain.h"
#include "Config.h"
#include "FixedRingBuffer.h"
#include "FlatSpatialGrid.h"
#include "TelemetrySample.h"
#include "TireThermals.h"
#include "SurfaceDynamics.h"
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

    // Advanced Vehicle Dynamics & Kamm Friction Circle
    float prevLongitudinalVelocity;
    float prevLateralVelocity;
    float longAccel;
    float latAccel;
    float prevLongAccel;
    float prevLatAccel;
    float jerkLong;
    float jerkLat;
    float slipAngle;
    float kammUtilization;

    // Thermodynamic tire degradation model
    TireThermalState tireThermals;

    // Track surface dynamics & adhesion
    SurfaceType currentSurface;
    float currentSurfaceFriction;

    // Control Ergonomics & Chatter
    int prevSteerSign;
    int steeringZeroCrossings;
    float prevAccelInput;
    float throttleChatter;

    // Racing line & track utilization
    float currentTrackOffset;
    float sumTrackOffset;
    int trackOffsetSamples;
    float lastApexDistance;
    float lastApexSpeed;

    // Preallocated fixed-capacity ring buffers for zero-allocation telemetry logging
    FixedRingBuffer<TelemetrySample, Config::MAX_GENERATION_TIME> currentLapSamples;
    FixedRingBuffer<TelemetrySample, Config::MAX_GENERATION_TIME> bestLapSamples;

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
     * @brief Evaluates the brain using the 9-dimensional state space.
     * 
     * @param outAccelerate Calculated acceleration output.
     * @param outTurn Calculated steering output.
     */
    void Think(float &outAccelerate, float &outTurn) {
        brain.Evaluate(sensorDistances, frontSensorDelta, longitudinalVelocity, lateralVelocity, angularVelocity, outAccelerate, outTurn);
    }

    /**
     * @brief Updates the physics of the car according to its inputs and calculates collisions.
     * 
     * @param inputAccelerate Acceleration value (-1.0 to 1.0).
     * @param inputTurn Turn value (-1.0 to 1.0).
     * @param spatialGrid Spatial grid containing the track walls for collision detection and sensors.
     * @param timer The current lifetime of the car in the simulation.
     */
    void UpdatePhysics(float inputAccelerate, float inputTurn, const FlatSpatialGrid &spatialGrid, int timer, const std::vector<Vector2>& trackCheckpoints);
};