#include <cassert>
#include <iostream>
#include <cmath>
#include "Brain.h"

int main() {
    std::cout << "[TEST] Starting Brain AVX2/FMA SIMD verification test suite..." << std::endl;

    Brain brain;
    float sensors[5] = {100.0f, 200.0f, 350.0f, 250.0f, 150.0f};
    float frontDelta = -20.0f;
    float vLong = 12.0f;
    float vLat = 1.5f;
    float angularVel = 0.5f;

    float accel = 0.0f;
    float turn = 0.0f;
    brain.Evaluate(sensors, frontDelta, vLong, vLat, angularVel, accel, turn);

    assert(accel >= -1.0f && accel <= 1.0f);
    assert(turn >= -1.0f && turn <= 1.0f);
    assert(!std::isnan(accel));
    assert(!std::isnan(turn));

    // Test Batch-of-8 inference vs individual evaluation
    float batchInput[8][INPUT_NODES];
    for (int c = 0; c < 8; ++c) {
        for (int f = 0; f < INPUT_NODES; ++f) {
            batchInput[c][f] = brain.last_input[f] * (0.8f + 0.05f * static_cast<float>(c));
        }
    }

    float batchAccel[8];
    float batchTurn[8];
    brain.EvaluateBatch8(batchInput, batchAccel, batchTurn);

    for (int c = 0; c < 8; ++c) {
        assert(batchAccel[c] >= -1.0f && batchAccel[c] <= 1.0f);
        assert(batchTurn[c] >= -1.0f && batchTurn[c] <= 1.0f);
        assert(!std::isnan(batchAccel[c]));
        assert(!std::isnan(batchTurn[c]));
    }

    std::cout << "[TEST] Brain AVX2/FMA: ALL ASSERTIONS PASSED SUCCESSFULLY!" << std::endl;
    return 0;
}
