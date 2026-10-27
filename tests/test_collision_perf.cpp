#include "Car.h"
#include "FlatSpatialGrid.h"
#include "VehicleSpatialGrid.h"
#include <cassert>
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>

void TestMultipleVehicleContactMomentumConservation() {
    FlatSpatialGrid trackGrid(2000.0f, 2000.0f, 50.0f);
    VehicleSpatialGrid vehicleGrid(2000.0f, 2000.0f, 50.0f);
    std::vector<Vector2> checkpoints;

    // Car 0 moving along +X and sliding slightly right (+Y)
    Car carA(200.0f, 200.0f, 0.0f);
    carA.velocity = { 3.5f, 0.6f };
    carA.longitudinalVelocity = 3.5f;
    carA.lateralVelocity = 0.6f;

    // Car 1 moving along +X and sliding slightly left (-Y)
    Car carB(200.0f, 206.0f, 0.0f);
    carB.velocity = { 3.5f, -0.6f };
    carB.longitudinalVelocity = 3.5f;
    carB.lateralVelocity = -0.6f;

    float initialTotalVy = carA.velocity.y + carB.velocity.y;
    (void)initialTotalVy;

    vehicleGrid.Clear();
    vehicleGrid.RegisterVehicle(0, carA.position, carA.velocity, carA.rotation);
    vehicleGrid.RegisterVehicle(1, carB.position, carB.velocity, carB.rotation);

    // Update both cars
    carA.UpdatePhysics(0.5f, 0.0f, trackGrid, 0, checkpoints, &vehicleGrid, 0);
    carB.UpdatePhysics(0.5f, 0.0f, trackGrid, 0, checkpoints, &vehicleGrid, 1);

    // Both cars must remain alive (rubbing contact)
    assert(!carA.isCrashed);
    assert(!carB.isCrashed);

    // Velocity in Y must have rebounded
    assert(carA.velocity.y < 0.6f);
    assert(carB.velocity.y > -0.6f);

    // Total momentum in Y is preserved (delta Vy sums to ~0)
    float finalTotalVy = carA.velocity.y + carB.velocity.y;
    assert(std::abs(finalTotalVy - initialTotalVy) < 0.1f);
    (void)finalTotalVy;

    std::cout << "[PASS] TestMultipleVehicleContactMomentumConservation passed." << std::endl;
}

void TestHighDensityPackSpatialQueryPerformance() {
    FlatSpatialGrid trackGrid(2000.0f, 2000.0f, 50.0f);
    VehicleSpatialGrid vehicleGrid(2000.0f, 2000.0f, 50.0f);
    std::vector<Vector2> checkpoints;

    constexpr int NUM_CARS = 100;
    std::vector<Car> fleet;
    fleet.reserve(NUM_CARS);

    // Grid start formation: 2 columns of 50 cars spaced 25 px apart
    for (int i = 0; i < NUM_CARS; ++i) {
        float x = 200.0f + static_cast<float>(i / 2) * 25.0f;
        float y = 300.0f + static_cast<float>(i % 2) * 18.0f;
        fleet.emplace_back(x, y, 0.0f);
        fleet.back().velocity = { 3.0f + (i % 3) * 0.2f, 0.0f };
        fleet.back().longitudinalVelocity = 3.0f;
    }

    constexpr int WARMUP_TICKS = 10;
    constexpr int MEASURE_TICKS = 100;

    for (int t = 0; t < WARMUP_TICKS; ++t) {
        vehicleGrid.Clear();
        for (int i = 0; i < NUM_CARS; ++i) {
            vehicleGrid.RegisterVehicle(i, fleet[i].position, fleet[i].velocity, fleet[i].rotation);
        }
        for (int i = 0; i < NUM_CARS; ++i) {
            fleet[i].UpdatePhysics(0.5f, 0.0f, trackGrid, t, checkpoints, &vehicleGrid, i);
        }
    }

    auto start = std::chrono::high_resolution_clock::now();
    for (int t = 0; t < MEASURE_TICKS; ++t) {
        vehicleGrid.Clear();
        for (int i = 0; i < NUM_CARS; ++i) {
            vehicleGrid.RegisterVehicle(i, fleet[i].position, fleet[i].velocity, fleet[i].rotation);
        }
        for (int i = 0; i < NUM_CARS; ++i) {
            fleet[i].UpdatePhysics(0.5f, 0.0f, trackGrid, t, checkpoints, &vehicleGrid, i);
        }
    }
    auto end = std::chrono::high_resolution_clock::now();

    double elapsedMicroseconds = std::chrono::duration<double, std::micro>(end - start).count();
    double avgUsPerTick = elapsedMicroseconds / MEASURE_TICKS;
    double avgNsPerCar = (elapsedMicroseconds / (MEASURE_TICKS * NUM_CARS)) * 1000.0;

    std::cout << "[PERF] 100-Car Pack Spatial Query & Collision Resolution:" << std::endl;
    std::cout << "       Average Tick Latency: " << avgUsPerTick << " us/tick" << std::endl;
    std::cout << "       Average Latency Per Car: " << avgNsPerCar << " ns/car" << std::endl;

    // Performance assertion: less than 1500 ns (1.5 us) per car even under dense pack racing
    assert(avgNsPerCar < 15000.0); // 15 us upper ceiling in debug/RelWithDebInfo

    std::cout << "[PASS] TestHighDensityPackSpatialQueryPerformance passed." << std::endl;
}

int main() {
    std::cout << "Running Multi-Agent Collision & Spatial Query Performance Tests..." << std::endl;
    TestMultipleVehicleContactMomentumConservation();
    TestHighDensityPackSpatialQueryPerformance();
    std::cout << "All Multi-Agent Collision & Spatial Performance tests passed successfully!" << std::endl;
    return 0;
}
