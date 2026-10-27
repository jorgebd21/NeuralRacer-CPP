#include "TireThermals.h"
#include "Car.h"
#include "FlatSpatialGrid.h"
#include <cassert>
#include <iostream>
#include <cmath>

void TestGripFactorCurve() {
    // 1. Cold range (< 85 C)
    float gripCold = TireThermalState::ComputeThermalGripFactor(40.0f);
    assert(gripCold < 1.0f);
    assert(gripCold >= 0.75f);
    (void)gripCold;

    // 2. Optimal window [85 C, 100 C]
    assert(TireThermalState::ComputeThermalGripFactor(85.0f) == 1.00f);
    assert(TireThermalState::ComputeThermalGripFactor(92.5f) == 1.00f);
    assert(TireThermalState::ComputeThermalGripFactor(100.0f) == 1.00f);

    // 3. Overheating window (100 C, 115 C]
    float gripWarm = TireThermalState::ComputeThermalGripFactor(110.0f);
    assert(gripWarm < 1.0f && gripWarm >= 0.88f);
    (void)gripWarm;

    // 4. Critical blistering window (> 115 C)
    float gripBlister = TireThermalState::ComputeThermalGripFactor(130.0f);
    assert(gripBlister < 0.88f && gripBlister >= 0.55f);
    (void)gripBlister;

    std::cout << "[PASS] TestGripFactorCurve passed." << std::endl;
}

void TestThermodynamicHeatingAndCooling() {
    TireThermalState state;
    state.Reset(75.0f);
    assert(state.surfaceTemp == 75.0f);
    assert(state.bulkTemp == 75.0f);

    // High lateral slip scenario: drifting at 4.0 px/tick, slip angle 25 degrees
    float initialSurf = state.surfaceTemp;
    for (int i = 0; i < 180; ++i) { // 3 seconds at 60 Hz
        state.Update(3.0f, 2.5f, 25.0f, 1.2f, 1.0f / 60.0f);
    }
    assert(state.surfaceTemp > initialSurf);
    (void)initialSurf;
    // Surface heats faster than carcass bulk
    assert(state.surfaceTemp > state.bulkTemp);

    // Car stops, ambient cooling kicks in
    float hotSurf = state.surfaceTemp;
    for (int i = 0; i < 600; ++i) { // 10 seconds of rest
        state.Update(0.0f, 0.0f, 0.0f, 1.0f, 1.0f / 60.0f);
    }
    assert(state.surfaceTemp < hotSurf);
    (void)hotSurf;
    assert(state.surfaceTemp >= TireThermalState::AMBIENT_TEMP);

    std::cout << "[PASS] TestThermodynamicHeatingAndCooling passed." << std::endl;
}

void TestCarThermalIntegration() {
    Car car(100.0f, 100.0f, 0.0f);
    FlatSpatialGrid grid(1000.0f, 1000.0f, 50.0f);
    std::vector<Vector2> checkpoints;

    assert(car.tireThermals.surfaceTemp == 75.0f);

    // Accelerate and turn hard to generate tire slip
    for (int i = 0; i < 120; ++i) {
        car.UpdatePhysics(1.0f, 1.0f, grid, i, checkpoints);
    }

    assert(car.tireThermals.surfaceTemp != 75.0f);
    assert(car.tireThermals.thermalGripFactor > 0.0f && car.tireThermals.thermalGripFactor <= 1.0f);

    std::cout << "[PASS] TestCarThermalIntegration passed." << std::endl;
}

int main() {
    std::cout << "Running Tire Thermodynamics Tests..." << std::endl;
    TestGripFactorCurve();
    TestThermodynamicHeatingAndCooling();
    TestCarThermalIntegration();
    std::cout << "All Tire Thermodynamics tests completed successfully!" << std::endl;
    return 0;
}
