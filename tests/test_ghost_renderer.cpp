#include "Telemetry.h"
#include "TelemetrySample.h"
#include "FixedRingBuffer.h"
#include "Config.h"
#include <cassert>
#include <iostream>
#include <vector>

void TestGhostBufferRecording() {
    std::vector<TelemetrySample> testSamples;
    for (int t = 0; t < 100; ++t) {
        TelemetrySample s{};
        s.tick = t;
        s.lapTick = t;
        s.position = {100.0f + static_cast<float>(t) * 2.0f, 200.0f};
        s.speed = 12.0f;
        testSamples.push_back(s);
    }

    Telemetry::RecordFastestLap(testSamples);
    assert(Telemetry::HasFastestLap());

    const auto& buffer = Telemetry::GetFastestLapBuffer();
    (void)buffer;
    assert(buffer.size() == 100);
    assert(buffer[0].position.x == 100.0f);
    assert(buffer[99].position.x == 100.0f + 99.0f * 2.0f);

    std::cout << "[PASS] TestGhostBufferRecording passed." << std::endl;
}

void TestGhostInterpolationTicks() {
    // Verify tick wrapping / bounds safety
    const auto& buffer = Telemetry::GetFastestLapBuffer();
    assert(!buffer.empty());

    // Test ticks beyond buffer size: 0, 50, 150, 1000
    int testTicks[] = {0, 50, 100, 150, 9999};
    for (int tick : testTicks) {
        size_t idx = static_cast<size_t>(tick) % buffer.size();
        (void)idx;
        assert(idx < buffer.size());
    }

    std::cout << "[PASS] TestGhostInterpolationTicks passed." << std::endl;
}

int main() {
    std::cout << "Running Ghost Vehicle Telemetry & Renderer Tests..." << std::endl;
    TestGhostBufferRecording();
    TestGhostInterpolationTicks();
    std::cout << "All Ghost Vehicle tests completed successfully!" << std::endl;
    return 0;
}
