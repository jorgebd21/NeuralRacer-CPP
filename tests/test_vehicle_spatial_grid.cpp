#include "VehicleSpatialGrid.h"
#include <cassert>
#include <iostream>
#include <cmath>

void TestVehicleOBBCorners() {
    VehicleSpatialGrid grid(1000.0f, 1000.0f, 50.0f);
    Vector2 pos = { 200.0f, 200.0f };
    Vector2 vel = { 3.0f, 0.0f };
    float rot = 0.0f; // Pointing along +X

    grid.RegisterVehicle(0, pos, vel, rot);
    assert(grid.GetVehicleCount() == 1);

    const VehicleOBB& obb = grid.GetVehicle(0);
    assert(obb.carIndex == 0);
    assert(obb.position.x == 200.0f && obb.position.y == 200.0f);

    // With rot = 0, nose is +X (length 8.5), right is +Y (width 4.0)
    // FL: nose - right = (200 + 8.5, 200 - 4.0) = (208.5, 196.0)
    // FR: nose + right = (200 + 8.5, 200 + 4.0) = (208.5, 204.0)
    // RR: -nose + right = (200 - 8.5, 200 + 4.0) = (191.5, 204.0)
    // RL: -nose - right = (200 - 8.5, 200 - 4.0) = (191.5, 196.0)
    assert(std::abs(obb.corners[0].x - 208.5f) < 0.01f);
    assert(std::abs(obb.corners[0].y - 196.0f) < 0.01f);
    assert(std::abs(obb.corners[1].x - 208.5f) < 0.01f);
    assert(std::abs(obb.corners[1].y - 204.0f) < 0.01f);
    assert(std::abs(obb.corners[2].x - 191.5f) < 0.01f);
    assert(std::abs(obb.corners[2].y - 204.0f) < 0.01f);
    assert(std::abs(obb.corners[3].x - 191.5f) < 0.01f);
    assert(std::abs(obb.corners[3].y - 196.0f) < 0.01f);

    // Verify 4 edges are connected
    for (int e = 0; e < 4; ++e) {
        Vector2 start = obb.GetSegmentStart(e);
        Vector2 end = obb.GetSegmentEnd(e);
        assert(start.x == obb.corners[e].x && start.y == obb.corners[e].y);
        assert(end.x == obb.corners[(e + 1) % 4].x && end.y == obb.corners[(e + 1) % 4].y);
        (void)start;
        (void)end;
    }

    std::cout << "[PASS] TestVehicleOBBCorners passed." << std::endl;
}

void TestVehicleSpatialBinningAndSparseClear() {
    VehicleSpatialGrid grid(1000.0f, 1000.0f, 50.0f);

    // Register 10 vehicles
    for (uint32_t i = 0; i < 10; ++i) {
        grid.RegisterVehicle(i, { 100.0f + i * 50.0f, 150.0f }, { 1.0f, 0.0f }, 0.0f);
    }
    assert(grid.GetVehicleCount() == 10);

    // Check cell (2, 3) which corresponds to (100, 150)
    int gx = static_cast<int>(100.0f / 50.0f);
    int gy = static_cast<int>(150.0f / 50.0f);
    const auto* cell = grid.GetCell(gx, gy);
    assert(cell != nullptr);
    assert(cell->count > 0);
    assert(cell->vehicleIndices[0] == 0);
    (void)cell;

    // Fast sparse clear
    grid.Clear();
    assert(grid.GetVehicleCount() == 0);
    const auto* cellAfter = grid.GetCell(gx, gy);
    assert(cellAfter != nullptr);
    assert(cellAfter->count == 0);
    (void)cellAfter;

    // Register again to ensure zero memory reallocation
    grid.RegisterVehicle(42, { 500.0f, 500.0f }, { 0.0f, 0.0f }, 45.0f);
    assert(grid.GetVehicleCount() == 1);
    const auto* cellCenter = grid.GetCell(10, 10);
    assert(cellCenter != nullptr);
    assert(cellCenter->count == 1);
    assert(cellCenter->vehicleIndices[0] == 0); // 0-th registered vehicle this frame
    assert(grid.GetVehicle(cellCenter->vehicleIndices[0]).carIndex == 42);
    (void)cellCenter;

    std::cout << "[PASS] TestVehicleSpatialBinningAndSparseClear passed." << std::endl;
}

int main() {
    std::cout << "Running Vehicle Spatial Grid Tests..." << std::endl;
    TestVehicleOBBCorners();
    TestVehicleSpatialBinningAndSparseClear();
    std::cout << "All Vehicle Spatial Grid tests completed successfully!" << std::endl;
    return 0;
}
