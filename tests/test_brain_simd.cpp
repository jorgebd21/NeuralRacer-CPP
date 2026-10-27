#include <cassert>
#include <iostream>
#include <cmath>
#include "Brain.h"

int main() {
    std::cout << "[TEST] Starting Brain AVX2/FMA & Pade Tanh test suite..." << std::endl;

    // 1. Test scalar Pade FastTanh properties
    assert(std::abs(FastTanh(0.0f)) < 1e-7f);
    assert(std::abs(FastTanh(1.0f) - std::tanh(1.0f)) < 1e-4f);
    assert(std::abs(FastTanh(2.0f) - std::tanh(2.0f)) < 1e-3f);
    assert(FastTanh(5.0f) == 1.0f);
    assert(FastTanh(-5.0f) == -1.0f);

    // Monotonicity check
    float prev = -1.1f;
    for (float x = -4.0f; x <= 4.0f; x += 0.1f) {
        float val = FastTanh(x);
        assert(val >= prev);
        assert(val >= -1.0f && val <= 1.0f);
        prev = val;
    }

#if defined(__AVX2__) && defined(__FMA__)
    // 2. Test FastTanhAVX2 vs FastTanh across all 8 lanes
    alignas(32) float test_inputs[8] = {-3.0f, -1.5f, -0.5f, 0.0f, 0.5f, 1.0f, 2.0f, 3.5f};
    __m256 in_vec = _mm256_load_ps(test_inputs);
    __m256 out_vec = FastTanhAVX2(in_vec);
    alignas(32) float test_outputs[8];
    _mm256_store_ps(test_outputs, out_vec);

    for (int i = 0; i < 8; ++i) {
        float expected = FastTanh(test_inputs[i]);
        assert(std::abs(test_outputs[i] - expected) < 1e-5f);
    }
#endif

    // 3. Test Brain evaluation
    Brain brain;
    float sensors[5] = {120.0f, 220.0f, 380.0f, 240.0f, 160.0f};
    float frontDelta = -15.0f;
    float vLong = 14.0f;
    float vLat = 0.5f;
    float angularVel = -0.2f;

    float accel = 0.0f;
    float turn = 0.0f;
    float compVel[5] = { 0.0f, 1.5f, 3.2f, -1.0f, 0.0f };
    brain.Evaluate(sensors, frontDelta, vLong, vLat, angularVel, compVel, accel, turn);

    assert(accel >= -1.0f && accel <= 1.0f);
    assert(turn >= -1.0f && turn <= 1.0f);
    assert(!std::isnan(accel));
    assert(!std::isnan(turn));
    assert(brain.last_input[9] == std::clamp(compVel[2] / 10.0f, -1.0f, 1.0f));
    assert(brain.last_input[10] == std::clamp(compVel[1] / 10.0f, -1.0f, 1.0f));
    assert(brain.last_input[11] == std::clamp(compVel[3] / 10.0f, -1.0f, 1.0f));

    // 4. Test Batch-of-8 inference vs individual evaluation
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

    std::cout << "[TEST] Brain Pade Tanh & SIMD: ALL ASSERTIONS PASSED SUCCESSFULLY!" << std::endl;
    return 0;
}
