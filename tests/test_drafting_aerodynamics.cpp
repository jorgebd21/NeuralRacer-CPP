#include "Car.h"
#include "FlatSpatialGrid.h"
#include "VehicleSpatialGrid.h"
#include <cassert>
#include <iostream>
#include <cmath>

void TestSlipstreamWakeDetection() {
    FlatSpatialGrid trackGrid(2000.0f, 2000.0f, 50.0f);
    VehicleSpatialGrid vehicleGrid(2000.0f, 2000.0f, 50.0f);
    std::vector<Vector2> checkpoints;

    // Ego car at (200, 300), heading 0 degrees
    Car ego(200.0f, 300.0f, 0.0f);
    ego.velocity = { 5.0f, 0.0f };
    ego.longitudinalVelocity = 5.0f;

    // Lead car at (260, 300), heading 0 degrees, moving at 5.0 px/tick
    vehicleGrid.Clear();
    vehicleGrid.RegisterVehicle(1, { 260.0f, 300.0f }, { 5.0f, 0.0f }, 0.0f);

    ego.UpdatePhysics(0.5f, 0.0f, trackGrid, 0, checkpoints, &vehicleGrid, 0);

    // Should detect slipstream
    assert(ego.draftingFactor > 0.5f);
    // Calculated: exp(-60^2 / (2 * 60^2)) = exp(-0.5) ~= 0.6065
    assert(std::abs(ego.draftingFactor - 0.6065f) < 0.05f);

    std::cout << "[PASS] TestSlipstreamWakeDetection passed (draftingFactor: " 
              << ego.draftingFactor << ")." << std::endl;
}

void TestStraightawayDraftingOvertakeAdvantage() {
    FlatSpatialGrid trackGrid(2000.0f, 2000.0f, 50.0f);
    VehicleSpatialGrid vehicleGrid(2000.0f, 2000.0f, 50.0f);
    std::vector<Vector2> checkpoints;

    // Car 1: in clean air (no vehicles ahead)
    Car carClean(200.0f, 200.0f, 0.0f);
    carClean.velocity = { 4.0f, 0.0f };
    carClean.longitudinalVelocity = 4.0f;

    // Car 2: in slipstream behind a lead car
    Car carDrafting(200.0f, 400.0f, 0.0f);
    carDrafting.velocity = { 4.0f, 0.0f };
    carDrafting.longitudinalVelocity = 4.0f;

    vehicleGrid.Clear();
    vehicleGrid.RegisterVehicle(99, { 250.0f, 400.0f }, { 6.0f, 0.0f }, 0.0f);

    // Run 1 tick full throttle
    carClean.UpdatePhysics(1.0f, 0.0f, trackGrid, 0, checkpoints, nullptr, 0);
    carDrafting.UpdatePhysics(1.0f, 0.0f, trackGrid, 0, checkpoints, &vehicleGrid, 1);

    // Drafting car suffers less drag, hence reaches higher forward velocity
    assert(carDrafting.draftingFactor > 0.0f);
    assert(carDrafting.longitudinalVelocity > carClean.longitudinalVelocity);

    std::cout << "[PASS] TestStraightawayDraftingOvertakeAdvantage passed (Drafting speed: " 
              << carDrafting.longitudinalVelocity << " px/tick vs Clean speed: " 
              << carClean.longitudinalVelocity << " px/tick)." << std::endl;
}

void TestPerpendicularOrTrailingCarNoDraft() {
    FlatSpatialGrid trackGrid(2000.0f, 2000.0f, 50.0f);
    VehicleSpatialGrid vehicleGrid(2000.0f, 2000.0f, 50.0f);
    std::vector<Vector2> checkpoints;

    Car ego(200.0f, 300.0f, 0.0f);
    ego.velocity = { 4.0f, 0.0f };
    ego.longitudinalVelocity = 4.0f;

    // Competitor is BEHIND ego (at x = 150)
    vehicleGrid.Clear();
    vehicleGrid.RegisterVehicle(5, { 150.0f, 300.0f }, { 4.0f, 0.0f }, 0.0f);

    ego.UpdatePhysics(0.5f, 0.0f, trackGrid, 0, checkpoints, &vehicleGrid, 0);

    assert(ego.draftingFactor == 0.0f);

    std::cout << "[PASS] TestPerpendicularOrTrailingCarNoDraft passed." << std::endl;
}

int main() {
    std::cout << "Running Drafting Aerodynamics Tests..." << std::endl;
    TestSlipstreamWakeDetection();
    TestStraightawayDraftingOvertakeAdvantage();
    TestPerpendicularOrTrailingCarNoDraft();
    std::cout << "All Drafting Aerodynamics tests completed successfully!" << std::endl;
    return 0;
}
