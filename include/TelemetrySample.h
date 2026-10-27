#pragma once
#include "raylib.h"

/**
 * @brief High-frequency telemetry time-series record captured tick-by-tick (60 Hz).
 * 
 * Modeled after professional telemetry systems (MoTeC i2 / McLaren ATLAS)
 * for vehicle dynamics, tire adhesion, control ergonomics, and neural sensitivity.
 */
struct TelemetrySample {
    int tick;
    int lapTick;
    float distance;
    Vector2 position;
    float speed;
    float vLong;
    float vLat;
    float angularVel;
    float throttleInput;
    float steerInput;
    float longAccel;
    float latAccel;
    float kammUtil;
    float slipAngle;
    float jerkLong;
    float jerkLat;
    float trackOffset;
    float tireSurfaceTemp;
    float tireBulkTemp;
    float tireThermalGrip;
    float surfaceFriction;
    float sensorDistances[5];
    float sensSteerFront; // Analytical Jacobian: d(Steer) / d(Lidar_Front)
    float sensSteerLat;   // Analytical Jacobian: d(Steer) / d(Lidar_Left) - d(Steer) / d(Lidar_Right)
};
