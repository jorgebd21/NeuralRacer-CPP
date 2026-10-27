#include <cassert>
#include <iostream>
#include <vector>
#include "FlatSpatialGrid.h"
#include "TrackManager.h"
#include "SurfaceDynamics.h"

int main() {
    std::cout << "[TEST] Starting TrackManager surface zones verification suite..." << std::endl;

    // Create a simple track with a straightaway and a 90-degree corner
    std::vector<Vector2> centerPoints;
    // Straight 1: (200, 200) to (600, 200)
    for (float x = 200.0f; x <= 600.0f; x += 10.0f) {
        centerPoints.push_back({x, 200.0f});
    }
    // Turn 1: arc from (600, 200) to (700, 300)
    for (float a = -PI/2.0f; a <= 0.0f; a += 0.1f) {
        centerPoints.push_back({600.0f + 100.0f * cosf(a + PI/2.0f), 300.0f + 100.0f * sinf(a)});
    }
    // Straight 2: (700, 300) to (700, 600)
    for (float y = 300.0f; y <= 600.0f; y += 10.0f) {
        centerPoints.push_back({700.0f, y});
    }
    // Loop back: (700, 600) to (200, 600) to (200, 200)
    for (float x = 700.0f; x >= 200.0f; x -= 20.0f) {
        centerPoints.push_back({x, 600.0f});
    }
    for (float y = 600.0f; y >= 200.0f; y -= 20.0f) {
        centerPoints.push_back({200.0f, y});
    }

    FlatSpatialGrid grid(1000.0f, 800.0f, 25); // 25px fine cell resolution
    float trackWidth = 35.0f;

    TrackManager::GenerateSurfaceZones(centerPoints, trackWidth, grid);

    // 1. Verify asphalt on centerline of straightaway (e.g. at 400, 200)
    SurfaceType surfCenter = grid.SampleSurface({400.0f, 200.0f});
    assert(surfCenter == SurfaceType::Asphalt);
    assert(grid.SampleFriction({400.0f, 200.0f}) == 1.00f);

    // 2. Verify runoff turf along straightaway margin (halfWidth = 17.5, test at 200 - 40 = 160)
    SurfaceType surfTurf = grid.SampleSurface({400.0f, 160.0f});
    assert(surfTurf == SurfaceType::RunoffTurf || surfTurf == SurfaceType::ApexCurbs);
    assert(grid.SampleFriction({400.0f, 160.0f}) < 1.00f);

    // 3. Verify gravel trap far off-track (e.g. at 400, 50)
    SurfaceType surfGravel = grid.SampleSurface({400.0f, 50.0f});
    assert(surfGravel == SurfaceType::GravelTrap);
    assert(grid.SampleFriction({400.0f, 50.0f}) == 0.25f);

    std::cout << "[TEST] TrackManager Surface Zones: ALL ASSERTIONS PASSED SUCCESSFULLY!" << std::endl;
    return 0;
}
