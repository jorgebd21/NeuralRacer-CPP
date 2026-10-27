#include "Car.h"
#include "FlatSpatialGrid.h"
#include "VehicleSpatialGrid.h"
#include <cassert>
#include <iostream>
#include <cmath>

void TestSideBySideRubbingRestitution() {
    FlatSpatialGrid trackGrid(2000.0f, 2000.0f, 50.0f);
    VehicleSpatialGrid vehicleGrid(2000.0f, 2000.0f, 50.0f);
    std::vector<Vector2> checkpoints;

    // Car A at (200, 200), moving forward along +X with a slight lateral drift towards Car B (+Y)
    Car carA(200.0f, 200.0f, 0.0f);
    carA.velocity = { 4.0f, 0.8f };
    carA.longitudinalVelocity = 4.0f;
    carA.lateralVelocity = 0.8f;

    // Car B at (200, 206.5) - overlapping slightly (width is 8.0, center distance 6.5 < 8.0)
    // Moving parallel along +X
    Vector2 compPos = { 200.0f, 206.5f };
    Vector2 compVel = { 4.0f, -0.8f };
    float compRot = 0.0f;

    vehicleGrid.Clear();
    vehicleGrid.RegisterVehicle(1, compPos, compVel, compRot);

    carA.UpdatePhysics(0.5f, 0.0f, trackGrid, 0, checkpoints, &vehicleGrid, 0);

    // Side-by-side contact must NOT trigger terminal crash
    assert(!carA.isCrashed);

    // Normal restitution impulse should push Car A away in Y (velocity.y should have reversed or decreased)
    assert(carA.velocity.y < 0.8f);

    std::cout << "[PASS] TestSideBySideRubbingRestitution passed (Car A alive, post-contact vy: " 
              << carA.velocity.y << " px/tick)." << std::endl;
}

void TestTerminalTBoneCrash() {
    FlatSpatialGrid trackGrid(2000.0f, 2000.0f, 50.0f);
    VehicleSpatialGrid vehicleGrid(2000.0f, 2000.0f, 50.0f);
    std::vector<Vector2> checkpoints;

    // Car A heading along +X (0 deg) at high speed (5.0 px/tick)
    Car carA(200.0f, 200.0f, 0.0f);
    carA.velocity = { 5.0f, 0.0f };
    carA.longitudinalVelocity = 5.0f;

    // Car B intersecting at 90 degrees (heading +Y) at 5.0 px/tick
    // Centered at (200, 200) for a violent T-bone collision
    Vector2 compPos = { 200.0f, 200.0f };
    Vector2 compVel = { 0.0f, 5.0f };
    float compRot = 90.0f;

    vehicleGrid.Clear();
    vehicleGrid.RegisterVehicle(2, compPos, compVel, compRot);

    carA.UpdatePhysics(0.5f, 0.0f, trackGrid, 0, checkpoints, &vehicleGrid, 0);

    // Violent perpendicular high-speed T-bone MUST result in terminal crash
    assert(carA.isCrashed);
    assert(carA.justCrashed);

    std::cout << "[PASS] TestTerminalTBoneCrash passed (Terminal crash triggered as expected)." << std::endl;
}

int main() {
    std::cout << "Running Inter-Agent Collision Tests..." << std::endl;
    TestSideBySideRubbingRestitution();
    TestTerminalTBoneCrash();
    std::cout << "All Inter-Agent Collision tests completed successfully!" << std::endl;
    return 0;
}
