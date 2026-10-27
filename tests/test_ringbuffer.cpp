#include <cassert>
#include <iostream>
#include "FixedRingBuffer.h"

struct TestSample {
    int id;
    float value;
};

int main() {
    std::cout << "[TEST] Starting FixedRingBuffer test suite..." << std::endl;

    FixedRingBuffer<TestSample, 4> buffer;
    assert(buffer.empty());
    assert(buffer.size() == 0);
    assert(buffer.capacity() == 4);

    // Test sequential push
    buffer.push_back({1, 10.0f});
    buffer.push_back({2, 20.0f});
    buffer.push_back({3, 30.0f});

    assert(!buffer.empty());
    assert(buffer.size() == 3);
    assert(buffer[0].id == 1);
    assert(buffer[1].id == 2);
    assert(buffer[2].id == 3);
    assert(buffer.front().id == 1);
    assert(buffer.back().id == 3);

    // Test filling to capacity
    buffer.push_back({4, 40.0f});
    assert(buffer.size() == 4);
    assert(buffer[3].id == 4);

    // Test circular overwrite (ring buffer semantics)
    buffer.push_back({5, 50.0f}); // Should overwrite id 1
    assert(buffer.size() == 4);
    assert(buffer[0].id == 2);
    assert(buffer[1].id == 3);
    assert(buffer[2].id == 4);
    assert(buffer[3].id == 5);
    assert(buffer.front().id == 2);
    assert(buffer.back().id == 5);

    // Test range-based for loop with iterator
    int expectedId = 2;
    for (const auto& item : buffer) {
        assert(item.id == expectedId);
        expectedId++;
    }

    // Test copy constructor (should flatten layout)
    FixedRingBuffer<TestSample, 4> copyBuf = buffer;
    assert(copyBuf.size() == 4);
    assert(copyBuf[0].id == 2);
    assert(copyBuf[3].id == 5);

    // Test clear
    buffer.clear();
    assert(buffer.empty());
    assert(buffer.size() == 0);

    // Push into cleared buffer
    buffer.push_back({10, 100.0f});
    assert(buffer.size() == 1);
    assert(buffer[0].id == 10);

    std::cout << "[TEST] FixedRingBuffer: ALL ASSERTIONS PASSED SUCCESSFULLY!" << std::endl;
    return 0;
}
