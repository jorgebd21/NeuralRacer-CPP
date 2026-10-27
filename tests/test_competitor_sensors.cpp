#include "Car.h"
#include "FlatSpatialGrid.h"
#include "VehicleSpatialGrid.h"
#include <cassert>
#include <iostream>
#include <cmath>

void TestCompetitorRaycastDetection() {
    FlatSpatialGrid trackGrid(2000.0f, 2000.0f, 50.0f);
    VehicleSpatialGrid vehicleGrid(2000.0f, 2000.0f, 50.0f);
    std::vector<Vector2> checkpoints;

    // Ego car at (200, 300), heading 0 degrees (pointing +X)
    // Front ray reaches up to 200 + 450 = 650.0 px
    Car ego(200.0f, 300.0f, 0.0f);
    ego.velocity = { 4.0f, 0.0f };
    ego.longitudinalVelocity = 4.0f;

    // Competitor car at (300, 300), heading 0 degrees
    // Competitor rear edge is at 300 - 8.5 = 291.5 px
    // Expected intersection distance from ego: 291.5 - 200.0 = 91.5 px
    Vector2 compPos = { 300.0f, 300.0f };
    Vector2 compVel = { 1.5f, 0.0f };
    float compRot = 0.0f;

    vehicleGrid.Clear();
    vehicleGrid.RegisterVehicle(42, compPos, compVel, compRot);

    ego.UpdatePhysics(0.5f, 0.0f, trackGrid, 0, checkpoints, &vehicleGrid, 0);

    // Sensor 2 is front sensor (0 deg offset)
    assert(ego.sensorDistances[2] < Config::CAR_FRONT_SENSOR_DIST);
    assert(std::abs(ego.sensorDistances[2] - 91.5f) < 0.5f);
    assert(ego.competitorDistances[2] == ego.sensorDistances[2]);
    assert(ego.closestCompetitorId == 42);

    // Closing velocity along +X: ego (4.0) vs comp (1.5) -> closing speed = +2.5 px/tick
    assert(std::abs(ego.competitorClosingVel[2] - 2.5f) < 0.1f);

    std::cout << "[PASS] TestCompetitorRaycastDetection passed (Distance: " 
              << ego.competitorDistances[2] << " px, Closing Vel: " 
              << ego.competitorClosingVel[2] << " px/tick)." << std::endl;
}

void TestCompetitorOutsideConeIgnored() {
    FlatSpatialGrid trackGrid(2000.0f, 2000.0f, 50.0f);
    VehicleSpatialGrid vehicleGrid(2000.0f, 2000.0f, 50.0f);
    std::vector<Vector2> checkpoints;

    Car ego(200.0f, 300.0f, 0.0f);

    // Place competitor behind ego car at (100, 300) where forward rays won't hit it
    Vector2 compPos = { 100.0f, 300.0f };
    Vector2 compVel = { 0.0f, 0.0f };

    vehicleGrid.Clear();
    vehicleGrid.RegisterVehicle(7, compPos, compVel, 0.0f);

    ego.UpdatePhysics(0.0f, 0.0f, trackGrid, 0, checkpoints, &vehicleGrid, 0);

    // Front sensor should be unaffected
    assert(ego.sensorDistances[2] == Config::CAR_FRONT_SENSOR_DIST);
    assert(ego.competitorDistances[2] == Config::CAR_FRONT_SENSOR_DIST);
    assert(ego.closestCompetitorId == -1);

    std::cout << "[PASS] TestCompetitorOutsideConeIgnored passed." << std::endl;
}

void TestNeuralInputClosingVelocity() {
    FlatSpatialGrid trackGrid(2000.0f, 2000.0f, 50.0f);
    VehicleSpatialGrid vehicleGrid(2000.0f, 2000.0f, 50.0f);
    std::vector<Vector2> checkpoints;

    Car ego(200.0f, 300.0f, 0.0f);
    ego.velocity = { 5.0f, 0.0f };
    ego.longitudinalVelocity = 5.0f;

    vehicleGrid.Clear();
    vehicleGrid.RegisterVehicle(9, { 300.0f, 300.0f }, { 2.0f, 0.0f }, 0.0f);

    ego.UpdatePhysics(0.5f, 0.0f, trackGrid, 0, checkpoints, &vehicleGrid, 0);

    float accel = 0.0f, turn = 0.0f;
    ego.Think(accel, turn);

    assert(ego.brain.last_input[9] > 0.20f && ego.brain.last_input[9] < 0.35f);
    assert(!std::isnan(accel) && !std::isnan(turn));

    std::cout << "[PASS] TestNeuralInputClosingVelocity passed (Front closing input: " 
              << ego.brain.last_input[9] << ")." << std::endl;
}

int main() {
    std::cout << "Running Competitor Sensor Tests..." << std::endl;
    TestCompetitorRaycastDetection();
    TestCompetitorOutsideConeIgnored();
    TestNeuralInputClosingVelocity();
    std::cout << "All Competitor Sensor tests completed successfully!" << std::endl;
    return 0;
}
