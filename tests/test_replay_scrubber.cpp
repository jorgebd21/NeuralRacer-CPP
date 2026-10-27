#include "TelemetrySample.h"
#include <cassert>
#include <iostream>
#include <vector>
#include <algorithm>

void TestTimelineScrubbingMath() {
    int totalSamples = 3600; // 60 seconds at 60Hz
    int barX = 100;
    int barW = 1000;

    // Test mouse clicks along the bar
    float mousePositions[] = {100.0f, 600.0f, 1100.0f, 50.0f, 1200.0f};
    float expectedRatios[] = {0.0f, 0.5f, 1.0f, 0.0f, 1.0f}; // Clamped
    (void)expectedRatios;

    for (int i = 0; i < 5; ++i) {
        float ratio = std::clamp((mousePositions[i] - static_cast<float>(barX)) / static_cast<float>(barW), 0.0f, 1.0f);
        assert(std::abs(ratio - expectedRatios[i]) < 1e-5f);
        float frame = ratio * static_cast<float>(totalSamples - 1);
        (void)ratio;
        (void)frame;
        assert(frame >= 0.0f && frame <= static_cast<float>(totalSamples - 1));
    }

    std::cout << "[PASS] TestTimelineScrubbingMath passed." << std::endl;
}

void TestPlaybackRateStepping() {
    float frame = 0.0f;
    float rate1 = 0.25f;
    float rate2 = 1.0f;
    float rate3 = 4.0f;

    // Advance 4 steps at 0.25x
    for (int i = 0; i < 4; ++i) frame += rate1;
    assert(std::abs(frame - 1.0f) < 1e-5f);

    // Advance 1 step at 1.0x
    frame += rate2;
    assert(std::abs(frame - 2.0f) < 1e-5f);

    // Advance 2 steps at 4.0x
    frame += rate3 * 2.0f;
    assert(std::abs(frame - 10.0f) < 1e-5f);

    std::cout << "[PASS] TestPlaybackRateStepping passed." << std::endl;
}

void TestTimelineLooping() {
    float frame = 99.0f;
    float totalFrames = 100.0f;
    float rate = 2.0f;

    frame += rate;
    if (frame >= totalFrames) {
        frame = 0.0f;
    }
    assert(frame == 0.0f);

    std::cout << "[PASS] TestTimelineLooping passed." << std::endl;
}

int main() {
    std::cout << "Running 60Hz Replay Timeline Scrubber Tests..." << std::endl;
    TestTimelineScrubbingMath();
    TestPlaybackRateStepping();
    TestTimelineLooping();
    std::cout << "All Replay Scrubber tests completed successfully!" << std::endl;
    return 0;
}
