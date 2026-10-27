#include <cassert>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <new>
#include <vector>

#include "Car.h"
#include "Config.h"
#include "FlatSpatialGrid.h"
#include "TrackManager.h"

// Global allocation interception tracker
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"

static thread_local bool g_trackAllocations = false;
static thread_local std::size_t g_allocationCount = 0;

void* operator new(std::size_t size) {
    if (g_trackAllocations) {
        g_allocationCount++;
    }
    void* p = std::malloc(size);
    if (!p) throw std::bad_alloc();
    return p;
}

void* operator new[](std::size_t size) {
    if (g_trackAllocations) {
        g_allocationCount++;
    }
    void* p = std::malloc(size);
    if (!p) throw std::bad_alloc();
    return p;
}

void operator delete(void* p) noexcept {
    std::free(p);
}

void operator delete[](void* p) noexcept {
    std::free(p);
}

void operator delete(void* p, std::size_t) noexcept {
    std::free(p);
}

void operator delete[](void* p, std::size_t) noexcept {
    std::free(p);
}

#pragma GCC diagnostic pop

int main() {
    std::cout << "================================================================" << std::endl;
    std::cout << "[BENCHMARK] High-Frequency 0-Allocation & HPC Throughput Suite" << std::endl;
    std::cout << "================================================================" << std::endl;

    // 1. Setup synthetic F1 track environment
    std::vector<Segment> walls = {
        {{100.0f, 100.0f}, {800.0f, 100.0f}},
        {{800.0f, 100.0f}, {800.0f, 600.0f}},
        {{800.0f, 600.0f}, {100.0f, 600.0f}},
        {{100.0f, 600.0f}, {100.0f, 100.0f}},
        {{200.0f, 200.0f}, {700.0f, 200.0f}},
        {{700.0f, 200.0f}, {700.0f, 500.0f}},
        {{700.0f, 500.0f}, {200.0f, 500.0f}},
        {{200.0f, 500.0f}, {200.0f, 200.0f}}
    };

    std::vector<Vector2> checkpoints = {
        {150.0f, 150.0f}, {450.0f, 150.0f}, {750.0f, 150.0f},
        {750.0f, 350.0f}, {750.0f, 550.0f}, {450.0f, 550.0f},
        {150.0f, 550.0f}, {150.0f, 350.0f}
    };

    FlatSpatialGrid grid(1200.0f, 800.0f, 100);
    grid.Build(walls);

    // 2. Preallocate 100 agent population
    std::vector<Car> population;
    population.reserve(Config::POPULATION_SIZE);
    for (int i = 0; i < Config::POPULATION_SIZE; ++i) {
        population.emplace_back(150.0f, 150.0f, 0.0f);
    }

    // Warm-up tick
    for (auto& car : population) {
        float accel = 0.0f, turn = 0.0f;
        car.Think(accel, turn);
        car.UpdatePhysics(accel, turn, grid, 0, checkpoints);
    }

    std::cout << "[INFO] Initialized population: " << population.size() << " agents." << std::endl;
    std::cout << "[INFO] Commencing 1,000 tick hot-path execution..." << std::endl;

    // 3. Enable strict allocation monitoring on hot path
    g_allocationCount = 0;
    g_trackAllocations = true;

    const int TOTAL_TICKS = 1000;
    auto start = std::chrono::high_resolution_clock::now();

    for (int tick = 1; tick <= TOTAL_TICKS; ++tick) {
        for (auto& car : population) {
            float accel = 0.0f, turn = 0.0f;
            car.Think(accel, turn);
            car.UpdatePhysics(accel, turn, grid, tick, checkpoints);
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    g_trackAllocations = false;

    auto durationUs = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    double totalCarTicks = static_cast<double>(TOTAL_TICKS * Config::POPULATION_SIZE);
    double carTicksPerSec = (totalCarTicks / static_cast<double>(durationUs)) * 1e6;
    double nsPerCarTick = (static_cast<double>(durationUs) * 1000.0) / totalCarTicks;

    std::cout << "----------------------------------------------------------------" << std::endl;
    std::cout << "Dynamic Allocations Detected in Hot Path: " << g_allocationCount << std::endl;
    std::cout << "Total Sim Time (100k Car-Ticks):           " << durationUs / 1000.0 << " ms" << std::endl;
    std::cout << "Average Latency Per Car-Tick:              " << nsPerCarTick << " ns" << std::endl;
    std::cout << "Simulation Throughput:                     " << static_cast<uint64_t>(carTicksPerSec) << " car-ticks/sec" << std::endl;
    std::cout << "----------------------------------------------------------------" << std::endl;

    assert(g_allocationCount == 0 && "CRITICAL: Detected dynamic heap allocation in hot path!");

    std::cout << "[PASS] ZERO DYNAMIC ALLOCATIONS VERIFIED ACROSS 100,000 TICKS!" << std::endl;
    std::cout << "================================================================" << std::endl;

    return 0;
}
