#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <cstdint>
#include "raylib.h"
#include "Car.h"
#include "FlatSpatialGrid.h"
#include "SurfaceDynamics.h"

/**
 * @brief Behavioral incoherence diagnostic bitmask flags.
 * Identifies unphysical, contradictory, or suicidal cognitive & physical states.
 */
enum IncoherenceFlags : uint32_t {
    INCOH_NONE                  = 0,
    INCOH_STEER_INTO_LEFT_WALL  = 1 << 0, // Left wall imminent (< 25px) yet agent commands steering left (< -0.2)
    INCOH_STEER_INTO_RIGHT_WALL = 1 << 1, // Right wall imminent (< 25px) yet agent commands steering right (> 0.2)
    INCOH_FULL_THROTTLE_WALL    = 1 << 2, // Front obstacle imminent (< 80px) at high speed (> 2.0px/t) yet full throttle (> 0.5)
    INCOH_BRAKE_ON_STRAIGHT     = 1 << 3, // Open straight (S2 > 250px, sides clear) yet harsh braking (< -0.3)
    INCOH_BANGBANG_STEERING     = 1 << 4, // Violent inter-tick steering reversal (|Δsteer| > 1.2)
    INCOH_GRIP_COLLAPSE         = 1 << 5, // Tire saturation (Kamm > 1.2) coupled with extreme chassis yaw drift (|slip| > 25 deg)
    INCOH_REVERSE_THRUST        = 1 << 6, // Standing still / negligible forward momentum (vLong < 0.05) yet applying reverse braking (< 0)
    INCOH_JACOBIAN_INVERTED     = 1 << 7, // Steering sensitivity inverted: obstacle on left increases steering to the left
    INCOH_STALLING              = 1 << 8  // Stagnant without forward progression (accumulating stallTicks > 30)
};

/**
 * @brief Single tick comprehensive sample containing complete physics, cognition and environmental data.
 */
struct ExhaustiveSample {
    int tick{0};
    Vector2 position{0.0f, 0.0f};
    float rotation{0.0f};
    float speed{0.0f};
    float vLong{0.0f};
    float vLat{0.0f};
    float angularVelocity{0.0f};
    float longAccel{0.0f};
    float latAccel{0.0f};
    float jerkLong{0.0f};
    float jerkLat{0.0f};
    float slipAngle{0.0f};
    float kammUtil{0.0f};
    
    // Surface & Tire Thermals
    SurfaceType surface{SurfaceType::Asphalt};
    float surfaceFriction{1.0f};
    float tireSurfaceTemp{80.0f};
    float tireBulkTemp{80.0f};
    float tireThermalGrip{1.0f};
    
    // Cognition / Neural inputs (12)
    float inputs[INPUT_NODES]{0.0f};
    // Hidden layer activations (10)
    float hiddenActivations[HIDDEN_NODES]{0.0f};
    float neuralSaturation{0.0f};
    
    // Control outputs
    float outThrottle{0.0f};
    float outSteer{0.0f};
    
    // Analytical Jacobians (XAI Sensitivity Gradients)
    float jacobianSteerFront{0.0f};    // d(Steer) / d(Lidar_Front)
    float jacobianSteerLat{0.0f};      // d(Steer) / d(Lidar_Left) - d(Steer) / d(Lidar_Right)
    float jacobianThrottleFront{0.0f}; // d(Throttle) / d(Lidar_Front)
    
    // Track & Navigation
    float trackOffset{0.0f};
    float trackWidthRatio{0.0f};
    float targetCPAngle{0.0f};
    int currentCP{0};
    int checkpointsCrossed{0};
    int timeSinceLastCP{0};
    int stallTicks{0};
    float fitness{0.0f};
    
    // Incoherence detection
    uint32_t incoherenceFlags{INCOH_NONE};
};

/**
 * @brief Summary statistics of an exhaustive telemetry session.
 */
struct ExhaustiveSummary {
    int totalTicks{0};
    int checkpointsCrossed{0};
    float finalFitness{0.0f};
    float avgSpeed{0.0f};
    float maxSpeed{0.0f};
    float avgKammUtil{0.0f};
    float maxKammUtil{0.0f};
    float avgSlipAngle{0.0f};
    float maxSlipAngle{0.0f};
    float avgJerk{0.0f};
    
    // Coherence scores (0.0 to 100.0%)
    float steeringCoherencePct{100.0f};  // Percentage of ticks free from steering into walls
    float throttleCoherencePct{100.0f};  // Percentage of ticks free from blind acceleration or straight braking
    float gripStabilityPct{100.0f};      // Percentage of ticks within linear tire grip
    float globalCoherenceScore{100.0f};  // Overall weighted behavioral coherence index
    
    // Incoherence occurrence counts
    int countSteerIntoWall{0};
    int countFullThrottleWall{0};
    int countBrakeOnStraight{0};
    int countBangBangSteering{0};
    int countGripCollapse{0};
    int countReverseThrust{0};
    int countJacobianInverted{0};
    int countStalling{0};
    
    bool crashed{false};
    int crashTick{0};
    std::string crashReason;
};

/**
 * @brief High-performance exhaustive recorder & behavioral auditor.
 */
class ExhaustiveTelemetryRecorder {
public:
    ExhaustiveTelemetryRecorder(size_t reserveTicks = 2500);

    void Reset();
    void RecordTick(const Car& car, 
                    const FlatSpatialGrid& grid, 
                    int tick, 
                    const std::vector<Vector2>& checkpoints, 
                    const std::vector<Vector2>& denseLine,
                    float trackWidth = 35.0f);

    void Finalize(bool crashed, const std::string& crashReason = "");

    const std::vector<ExhaustiveSample>& GetSamples() const { return samples; }
    const ExhaustiveSummary& GetSummary() const { return summary; }

    bool ExportCSV(const std::string& filepath) const;
    bool ExportJSON(const std::string& filepath) const;
    void PrintSummaryTable(std::ostream& os = std::cout) const;
    void PrintAnomalyTimeline(std::ostream& os = std::cout, int maxEntries = 50) const;

private:
    std::vector<ExhaustiveSample> samples;
    ExhaustiveSummary summary;
    float prevSteerInput{0.0f};
};
