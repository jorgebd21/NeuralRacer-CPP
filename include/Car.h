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
    
    float fitness;
    float accumulatedFitness;
    int timeAlive;
    int timeSinceLastCheckpoint;
    float distanceTraveled;

    int nextCheckPointIndex;
    int totalCheckPointsCrossed;
    
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
     * @brief Updates the physics of the car according to its inputs and calculates collisions.
     * 
     * @param inputAccelerate Acceleration value (-1.0 to 1.0).
     * @param inputTurn Turn value (-1.0 to 1.0).
     * @param spatialGrid Spatial grid containing the track walls for collision detection and sensors.
     * @param timer The current lifetime of the car in the simulation.
     */
    void UpdatePhysics(float inputAccelerate, float inputTurn, const std::unordered_map<uint64_t, std::vector<std::pair<Vector2, Vector2>>> &spatialGrid, int timer, const std::vector<Vector2>& trackCheckpoints);
};