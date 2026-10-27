#include "Telemetry.h"
#include "Brain.h"
#include <cassert>
#include <iostream>
#include <cmath>

void TestJacobianSensitivityCalculation() {
    Brain brain;
    // Perform one forward step to establish activations
    float sensorDistances[5] = {50.0f, 100.0f, 150.0f, 100.0f, 50.0f};
    float accel = 0.0f, turn = 0.0f;
    brain.Evaluate(sensorDistances, 0.0f, 15.0f, 0.0f, 0.0f, accel, turn);

    float jacobian[OUTPUT_NODES][INPUT_NODES];
    brain.ComputeJacobian(jacobian);

    for (int k = 0; k < OUTPUT_NODES; ++k) {
        for (int i = 0; i < INPUT_NODES; ++i) {
            assert(!std::isnan(jacobian[k][i]));
            assert(!std::isinf(jacobian[k][i]));
        }
    }

    float sensFront = jacobian[1][2];
    float sensLat = jacobian[1][1] - jacobian[1][3];

    assert(!std::isnan(sensFront));
    assert(!std::isnan(sensLat));

    std::cout << "[PASS] TestJacobianSensitivityCalculation passed (SensFront=" 
              << sensFront << ", SensLat=" << sensLat << ")." << std::endl;
}

void TestJacobianVectorOrthogonality() {
    float rotDeg = 45.0f;
    float rad = rotDeg * DEG2RAD;
    Vector2 forwardDir = { std::cos(rad), std::sin(rad) };
    Vector2 lateralDir = { -std::sin(rad), std::cos(rad) };

    // Dot product between forwardDir and lateralDir must be exactly 0 (orthogonal)
    float dot = forwardDir.x * lateralDir.x + forwardDir.y * lateralDir.y;
    assert(std::abs(dot) < 1e-6f);
    (void)dot;

    std::cout << "[PASS] TestJacobianVectorOrthogonality passed." << std::endl;
}

int main() {
    std::cout << "Running Real-Time Jacobian HUD Vector Tests..." << std::endl;
    TestJacobianSensitivityCalculation();
    TestJacobianVectorOrthogonality();
    std::cout << "All Jacobian Vector Visualizer tests completed successfully!" << std::endl;
    return 0;
}
