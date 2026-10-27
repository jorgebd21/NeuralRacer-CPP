#include "Car.h"
#include "FlatSpatialGrid.h"
#include "SurfaceDynamics.h"
#include "TireThermals.h"
#include <cassert>
#include <iostream>
#include <cmath>

void TestSurfaceGripModulation() {
    FlatSpatialGrid grid(1000.0f, 1000.0f, 50.0f);
    // Setup cell (2, 2) as Asphalt, cell (4, 4) as GravelTrap
    grid.SetCellSurface(2, 2, SurfaceType::Asphalt);
    grid.SetCellSurface(4, 4, SurfaceType::GravelTrap);

    std::vector<Vector2> checkpoints;

    // Car on asphalt
    Car carAsphalt(125.0f, 125.0f, 0.0f);
    carAsphalt.tireThermals.Reset(90.0f); // optimal temperature

    // Car on gravel
    Car carGravel(225.0f, 225.0f, 0.0f);
    carGravel.tireThermals.Reset(90.0f);

    // Apply 1 tick full throttle
    carAsphalt.UpdatePhysics(1.0f, 0.0f, grid, 0, checkpoints);
    carGravel.UpdatePhysics(1.0f, 0.0f, grid, 0, checkpoints);

    assert(carAsphalt.currentSurface == SurfaceType::Asphalt);
    assert(carGravel.currentSurface == SurfaceType::GravelTrap);
    assert(carAsphalt.currentSurfaceFriction == 1.00f);
    assert(carGravel.currentSurfaceFriction == 0.25f);

    // Forward acceleration on asphalt must be strictly greater than on gravel
    assert(carAsphalt.longitudinalVelocity > carGravel.longitudinalVelocity);

    std::cout << "[PASS] TestSurfaceGripModulation passed (Asphalt: " 
              << carAsphalt.longitudinalVelocity << " px/tick vs Gravel: " 
              << carGravel.longitudinalVelocity << " px/tick)." << std::endl;
}

void TestThermalGripCoupling() {
    FlatSpatialGrid grid(1000.0f, 1000.0f, 50.0f);
    // Uniform asphalt
    for (int y = 0; y < grid.GetRows(); ++y) {
        for (int x = 0; x < grid.GetCols(); ++x) {
            grid.SetCellSurface(x, y, SurfaceType::Asphalt);
        }
    }

    std::vector<Vector2> checkpoints;

    // Car with optimal tires (90 C)
    Car carOptimal(200.0f, 200.0f, 0.0f);
    carOptimal.tireThermals.Reset(90.0f);
    carOptimal.velocity = { 5.0f, 0.0f };
    carOptimal.longitudinalVelocity = 5.0f;

    // Car with blistered tires (135 C)
    Car carBlistered(200.0f, 200.0f, 0.0f);
    carBlistered.tireThermals.Reset(135.0f);
    carBlistered.velocity = { 5.0f, 0.0f };
    carBlistered.longitudinalVelocity = 5.0f;

    // Apply identical hard steer cornering input
    carOptimal.UpdatePhysics(0.5f, 1.0f, grid, 0, checkpoints);
    carBlistered.UpdatePhysics(0.5f, 1.0f, grid, 0, checkpoints);

    // Blistered tires should have lower thermal grip factor
    assert(carOptimal.tireThermals.thermalGripFactor > carBlistered.tireThermals.thermalGripFactor);

    std::cout << "[PASS] TestThermalGripCoupling passed (Optimal Grip Factor: " 
              << carOptimal.tireThermals.thermalGripFactor << " vs Blistered: " 
              << carBlistered.tireThermals.thermalGripFactor << ")." << std::endl;
}

void TestCurbOscillationImpulse() {
    FlatSpatialGrid grid(1000.0f, 1000.0f, 50.0f);
    grid.SetCellSurface(3, 3, SurfaceType::ApexCurbs);

    std::vector<Vector2> checkpoints;
    Car car(175.0f, 175.0f, 0.0f);
    car.velocity = { 4.0f, 0.0f };
    car.longitudinalVelocity = 4.0f;

    // Driving straight on apex curb with zero steer
    car.UpdatePhysics(0.5f, 0.0f, grid, 10, checkpoints);

    assert(car.currentSurface == SurfaceType::ApexCurbs);
    // Curb rumble introduces micro-oscillation in lateral velocity
    assert(std::abs(car.lateralVelocity) > 0.0f);

    std::cout << "[PASS] TestCurbOscillationImpulse passed (Lateral chatter: " 
              << car.lateralVelocity << " px/tick)." << std::endl;
}

int main() {
    std::cout << "Running Kamm-Thermal Coupling Tests..." << std::endl;
    TestSurfaceGripModulation();
    TestThermalGripCoupling();
    TestCurbOscillationImpulse();
    std::cout << "All Kamm-Thermal Coupling tests completed successfully!" << std::endl;
    return 0;
}
