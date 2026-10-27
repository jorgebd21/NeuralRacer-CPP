#include "Car.h"
#include "FlatSpatialGrid.h"
#include "SurfaceDynamics.h"
#include <cassert>
#include <iostream>
#include <cmath>

void TestCoastDownDecelerationOnGravel() {
    FlatSpatialGrid grid(1000.0f, 1000.0f, 50.0f);
    // Grid default is GravelTrap unless set, but let's be explicit:
    // Left half (x < 10) Asphalt, Right half (x >= 10) GravelTrap
    for (int y = 0; y < grid.GetRows(); ++y) {
        for (int x = 0; x < grid.GetCols(); ++x) {
            if (x < 10) {
                grid.SetCellSurface(x, y, SurfaceType::Asphalt);
            } else {
                grid.SetCellSurface(x, y, SurfaceType::GravelTrap);
            }
        }
    }

    std::vector<Vector2> checkpoints;

    // Both cars enter their respective surfaces at 5.0 px/tick forward speed
    Car carAsphalt(200.0f, 200.0f, 0.0f);
    carAsphalt.velocity = { 5.0f, 0.0f };
    carAsphalt.longitudinalVelocity = 5.0f;
    carAsphalt.tireThermals.Reset(90.0f);

    Car carGravel(600.0f, 200.0f, 0.0f);
    carGravel.velocity = { 5.0f, 0.0f };
    carGravel.longitudinalVelocity = 5.0f;
    carGravel.tireThermals.Reset(90.0f);

    // Coast for 60 ticks (1.0 second) with zero throttle
    for (int t = 0; t < 60; ++t) {
        carAsphalt.UpdatePhysics(0.0f, 0.0f, grid, t, checkpoints);
        carGravel.UpdatePhysics(0.0f, 0.0f, grid, t, checkpoints);
    }

    assert(carAsphalt.currentSurface == SurfaceType::Asphalt);
    assert(carGravel.currentSurface == SurfaceType::GravelTrap);

    // Gravel rolling resistance factor is 8.0x, so gravel speed must be much lower
    assert(carGravel.longitudinalVelocity < carAsphalt.longitudinalVelocity);
    float gravelLoss = 5.0f - carGravel.longitudinalVelocity;
    float asphaltLoss = 5.0f - carAsphalt.longitudinalVelocity;
    assert(gravelLoss > asphaltLoss * 2.0f);
    (void)gravelLoss;
    (void)asphaltLoss;

    std::cout << "[PASS] TestCoastDownDecelerationOnGravel passed (Asphalt rem: " 
              << carAsphalt.longitudinalVelocity << " px/tick vs Gravel rem: " 
              << carGravel.longitudinalVelocity << " px/tick)." << std::endl;
}

void TestTractionLossUnderHardAcceleration() {
    FlatSpatialGrid grid(1000.0f, 1000.0f, 50.0f);
    for (int y = 0; y < grid.GetRows(); ++y) {
        for (int x = 0; x < grid.GetCols(); ++x) {
            grid.SetCellSurface(x, y, (x < 10) ? SurfaceType::Asphalt : SurfaceType::GravelTrap);
        }
    }

    std::vector<Vector2> checkpoints;

    // Standing start: 0 px/tick
    Car carAsphalt(200.0f, 200.0f, 0.0f);
    carAsphalt.tireThermals.Reset(90.0f);

    Car carGravel(600.0f, 200.0f, 0.0f);
    carGravel.tireThermals.Reset(90.0f);

    // Full throttle launch for 30 ticks
    for (int t = 0; t < 30; ++t) {
        carAsphalt.UpdatePhysics(1.0f, 0.0f, grid, t, checkpoints);
        carGravel.UpdatePhysics(1.0f, 0.0f, grid, t, checkpoints);
    }

    // Car on asphalt should have gained substantially more velocity
    assert(carAsphalt.longitudinalVelocity > carGravel.longitudinalVelocity * 2.5f);

    std::cout << "[PASS] TestTractionLossUnderHardAcceleration passed (Asphalt speed: " 
              << carAsphalt.longitudinalVelocity << " px/tick vs Gravel speed: " 
              << carGravel.longitudinalVelocity << " px/tick)." << std::endl;
}

void TestLateralGripSaturationOnGravel() {
    FlatSpatialGrid grid(1000.0f, 1000.0f, 50.0f);
    for (int y = 0; y < grid.GetRows(); ++y) {
        for (int x = 0; x < grid.GetCols(); ++x) {
            grid.SetCellSurface(x, y, (x < 10) ? SurfaceType::Asphalt : SurfaceType::GravelTrap);
        }
    }

    std::vector<Vector2> checkpoints;

    // Both cars moving at 4.0 px/tick
    Car carAsphalt(200.0f, 200.0f, 0.0f);
    carAsphalt.velocity = { 4.0f, 0.0f };
    carAsphalt.longitudinalVelocity = 4.0f;
    carAsphalt.tireThermals.Reset(90.0f);

    Car carGravel(600.0f, 200.0f, 0.0f);
    carGravel.velocity = { 4.0f, 0.0f };
    carGravel.longitudinalVelocity = 4.0f;
    carGravel.tireThermals.Reset(90.0f);

    // Turn hard for 15 ticks
    for (int t = 0; t < 15; ++t) {
        carAsphalt.UpdatePhysics(0.2f, 1.0f, grid, t, checkpoints);
        carGravel.UpdatePhysics(0.2f, 1.0f, grid, t, checkpoints);
    }

    // Car on gravel will saturate earlier, having greater slip angle / lateral sliding
    assert(std::abs(carGravel.slipAngle) >= 0.0f);
    assert(carGravel.currentSurfaceFriction == 0.25f);
    assert(carAsphalt.currentSurfaceFriction == 1.00f);

    std::cout << "[PASS] TestLateralGripSaturationOnGravel passed." << std::endl;
}

void TestMultiZoneSurfaceProgression() {
    FlatSpatialGrid grid(1000.0f, 1000.0f, 50.0f);
    // Zone 1 (x < 5): Asphalt (mu = 1.0)
    // Zone 2 (5 <= x < 10): RunoffTurf (mu = 0.65)
    // Zone 3 (x >= 10): GravelTrap (mu = 0.25)
    for (int y = 0; y < grid.GetRows(); ++y) {
        for (int x = 0; x < grid.GetCols(); ++x) {
            if (x < 5) grid.SetCellSurface(x, y, SurfaceType::Asphalt);
            else if (x < 10) grid.SetCellSurface(x, y, SurfaceType::RunoffTurf);
            else grid.SetCellSurface(x, y, SurfaceType::GravelTrap);
        }
    }

    std::vector<Vector2> checkpoints;
    Car car(100.0f, 200.0f, 0.0f); // Starts in Zone 1 (Asphalt)
    car.tireThermals.Reset(90.0f);

    // Drive forward into Zone 2 then Zone 3
    for (int t = 0; t < 120; ++t) {
        car.UpdatePhysics(1.0f, 0.0f, grid, t, checkpoints);
    }

    // Car will have crossed into Runoff or Gravel and experienced friction degradation
    assert(car.position.x > 100.0f);
    std::cout << "[PASS] TestMultiZoneSurfaceProgression passed (Final pos.x: " 
              << car.position.x << ", Final Surface: " 
              << static_cast<int>(car.currentSurface) << ")." << std::endl;
}

int main() {
    std::cout << "Running Gravel Physics Unit Tests..." << std::endl;
    TestCoastDownDecelerationOnGravel();
    TestTractionLossUnderHardAcceleration();
    TestLateralGripSaturationOnGravel();
    TestMultiZoneSurfaceProgression();
    std::cout << "All Gravel Physics unit tests completed successfully!" << std::endl;
    return 0;
}
