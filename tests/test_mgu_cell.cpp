#include "Brain.h"
#include "Car.h"
#include <cassert>
#include <iostream>
#include <cmath>

void TestMGUResetState() {
    Brain brain;
    brain.ResetState();
    for (int i = 0; i < RECURRENT_NODES; ++i) {
        assert(brain.h_state[i] == 0.0f);
        assert(brain.last_f_gate[i] == 0.0f);
    }
    std::cout << "[PASS] TestMGUResetState passed." << std::endl;
}

void TestMGUForwardEquations() {
    Brain brain;
    brain.ResetState();

    float input[INPUT_NODES];
    for (int j = 0; j < INPUT_NODES; ++j) {
        input[j] = 0.5f;
    }

    float accel = 0.0f;
    float turn = 0.0f;
    brain.StepMGU(input, accel, turn);

    assert(accel >= -1.0f && accel <= 1.0f);
    assert(turn >= -1.0f && turn <= 1.0f);
    assert(!std::isnan(accel));
    assert(!std::isnan(turn));

    // Verify gates are in (0, 1) range
    for (int i = 0; i < RECURRENT_NODES; ++i) {
        assert(brain.last_f_gate[i] > 0.0f && brain.last_f_gate[i] < 1.0f);
        assert(brain.h_state[i] >= -1.0f && brain.h_state[i] <= 1.0f);
    }

    std::cout << "[PASS] TestMGUForwardEquations passed." << std::endl;
}

void TestMGUTemporalWorkingMemory() {
    Brain brain;
    brain.ResetState();

    // Pulse event at tick 0
    float pulseInput[INPUT_NODES];
    for (int j = 0; j < INPUT_NODES; ++j) pulseInput[j] = 1.0f;

    float a0 = 0.0f, t0 = 0.0f;
    brain.StepMGU(pulseInput, a0, t0);
    (void)a0; (void)t0;

    // Check magnitude of recurrent state after pulse
    float energy0 = 0.0f;
    for (int i = 0; i < RECURRENT_NODES; ++i) {
        energy0 += brain.h_state[i] * brain.h_state[i];
    }
    assert(energy0 > 0.01f);

    // Feed zero input for 5 consecutive ticks
    float zeroInput[INPUT_NODES] = { 0.0f };
    for (int step = 0; step < 5; ++step) {
        float a = 0.0f, t = 0.0f;
        brain.StepMGU(zeroInput, a, t);
        (void)a; (void)t;
    }

    // Working memory verification: recurrent hidden state retains memory of pulse
    float energy5 = 0.0f;
    for (int i = 0; i < RECURRENT_NODES; ++i) {
        energy5 += brain.h_state[i] * brain.h_state[i];
    }
    assert(energy5 > 0.0001f);

    std::cout << "[PASS] TestMGUTemporalWorkingMemory passed (State Energy: t0=" 
              << energy0 << ", t5=" << energy5 << ")." << std::endl;
}

void TestCarMGUIntegration() {
    Car car(100.0f, 100.0f, 0.0f);
    car.brain.useRecurrent = true;
    car.Reset(100.0f, 100.0f, 0.0f);

    for (int i = 0; i < RECURRENT_NODES; ++i) {
        assert(car.brain.h_state[i] == 0.0f);
    }

    float accel = 0.0f, turn = 0.0f;
    car.Think(accel, turn);

    assert(accel >= -1.0f && accel <= 1.0f);
    assert(turn >= -1.0f && turn <= 1.0f);

    std::cout << "[PASS] TestCarMGUIntegration passed." << std::endl;
}

int main() {
    std::cout << "Running Minimal Gated Unit (MGU) Recurrent Cell Tests..." << std::endl;
    TestMGUResetState();
    TestMGUForwardEquations();
    TestMGUTemporalWorkingMemory();
    TestCarMGUIntegration();
    std::cout << "All MGU Recurrent Cell tests completed successfully!" << std::endl;
    return 0;
}
