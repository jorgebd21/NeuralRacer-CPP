#include "Telemetry.h"
#include "TelemetrySample.h"
#include <cassert>
#include <iostream>
#include <vector>

void TestHeatmapColorThresholds() {
    // 1. High Kamm utilization (> 0.85) -> RED
    Color cRedKamm = Telemetry::ComputeHeatmapColor(15.0f, 0.95f, 1.0f);
    assert(cRedKamm.r > 200 && cRedKamm.g == 0);
    (void)cRedKamm;

    // 2. Heavy braking (throttle < -0.3) -> RED
    Color cRedBraking = Telemetry::ComputeHeatmapColor(15.0f, 0.20f, -0.8f);
    assert(cRedBraking.r > 200 && cRedBraking.g == 0);
    (void)cRedBraking;

    // 3. Trail braking / Moderate slip (kamm in 0.60..0.85) -> ORANGE
    Color cOrange = Telemetry::ComputeHeatmapColor(10.0f, 0.70f, 0.2f);
    assert(cOrange.r > 200 && cOrange.g > 100);
    (void)cOrange;

    // 4. Low speed / threshold (kamm in 0.40..0.60) -> YELLOW
    Color cYellow = Telemetry::ComputeHeatmapColor(11.0f, 0.45f, 0.5f);
    assert(cYellow.r > 200 && cYellow.g > 200);
    (void)cYellow;

    // 5. Full throttle / high speed / high traction -> GREEN
    Color cGreen = Telemetry::ComputeHeatmapColor(18.0f, 0.15f, 1.0f);
    assert(cGreen.g > 200 && cGreen.r == 0);
    (void)cGreen;

    std::cout << "[PASS] TestHeatmapColorThresholds passed." << std::endl;
}

void TestRacingLineDrawingIntegrity() {
    std::vector<TelemetrySample> lap;
    for (int i = 0; i < 50; ++i) {
        TelemetrySample s{};
        s.position = {static_cast<float>(i * 10), static_cast<float>(i * 5)};
        s.speed = 5.0f + static_cast<float>(i) * 0.3f;
        s.kammUtil = 0.2f + 0.01f * static_cast<float>(i);
        s.throttleInput = 1.0f;
        lap.push_back(s);
    }
    Telemetry::RecordFastestLap(lap);

    // Verify buffer exists and has >= 2 samples
    assert(Telemetry::GetFastestLapBuffer().size() == 50);

    std::cout << "[PASS] TestRacingLineDrawingIntegrity passed." << std::endl;
}

int main() {
    std::cout << "Running Racing Line Heatmap & Velocity Gradient Tests..." << std::endl;
    TestHeatmapColorThresholds();
    TestRacingLineDrawingIntegrity();
    std::cout << "All Racing Line Heatmap tests completed successfully!" << std::endl;
    return 0;
}
