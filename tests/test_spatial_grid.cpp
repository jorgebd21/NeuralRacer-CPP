#include <cassert>
#include <iostream>
#include "FlatSpatialGrid.h"

int main() {
    std::cout << "[TEST] Starting FlatSpatialGrid test suite..." << std::endl;

    FlatSpatialGrid grid(1000.0f, 800.0f, 100);
    assert(grid.GetCellSize() == 100);
    assert(grid.GetCols() == 12); // ceil(1000/100) + 2 = 12
    assert(grid.GetRows() == 10); // ceil(800/100) + 2 = 10
    assert(grid.TotalCells() == 120);

    // Out of bounds queries
    assert(grid.GetCell(-1, 0) == nullptr);
    assert(grid.GetCell(0, -1) == nullptr);
    assert(grid.GetCell(12, 5) == nullptr);
    assert(grid.GetCell(5, 10) == nullptr);

    // In bounds query before build
    const GridCell* cell = grid.GetCell(0, 0);
    assert(cell != nullptr);
    assert(cell->segments.empty());

    // Add wall segment spanning from (150, 150) to (250, 150)
    // Cells: gridX in [1, 2], gridY = 1
    std::vector<Segment> walls = {
        {{150.0f, 150.0f}, {250.0f, 150.0f}}
    };
    grid.Build(walls);

    // Cell (1, 1) and (2, 1) should have the segment
    const GridCell* c11 = grid.GetCell(1, 1);
    assert(c11 != nullptr);
    assert(c11->segments.size() == 1);
    assert(c11->segments[0].first.x == 150.0f);

    const GridCell* c21 = grid.GetCell(2, 1);
    assert(c21 != nullptr);
    assert(c21->segments.size() == 1);

    // Cell (0, 0) should have 0 segments
    const GridCell* c00 = grid.GetCell(0, 0);
    assert(c00 != nullptr);
    assert(c00->segments.empty());

    // Rebuild with empty walls
    grid.Build({});
    assert(grid.GetCell(1, 1)->segments.empty());

    std::cout << "[TEST] FlatSpatialGrid: ALL ASSERTIONS PASSED SUCCESSFULLY!" << std::endl;
    return 0;
}
