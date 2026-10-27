#include <cassert>
#include <iostream>
#include "FlatSpatialGrid.h"

int main() {
    std::cout << "[TEST] Starting FlatSpatialGrid pooled memory arena test suite..." << std::endl;

    FlatSpatialGrid grid(1000.0f, 800.0f, 100);
    assert(grid.GetCellSize() == 100);
    assert(grid.GetCols() == 12);
    assert(grid.GetRows() == 10);
    assert(grid.TotalCells() == 120);

    // Out of bounds queries
    assert(grid.GetCell(-1, 0) == nullptr);
    assert(grid.GetCell(0, -1) == nullptr);
    assert(grid.GetCell(12, 5) == nullptr);
    assert(grid.GetCell(5, 10) == nullptr);

    // Add wall segments
    std::vector<Segment> walls = {
        {{150.0f, 150.0f}, {250.0f, 150.0f}}, // Seg 0: cells (1,1) and (2,1)
        {{150.0f, 150.0f}, {150.0f, 250.0f}}  // Seg 1: cells (1,1) and (1,2)
    };
    grid.Build(walls);

    // Verify segment arena
    assert(grid.GetSegmentArena().size() == 2);
    assert(grid.GetSegment(0).first.x == 150.0f);
    assert(grid.GetSegment(1).second.y == 250.0f);

    // Cell (1, 1) should contain both segments 0 and 1
    const GridCell* c11 = grid.GetCell(1, 1);
    assert(c11 != nullptr);
    assert(c11->count == 2);
    assert(grid.GetCellSegment(c11, 0).first.x == 150.0f);
    assert(grid.GetCellSegment(c11, 1).second.y == 250.0f);

    // Cell (2, 1) should contain only segment 0
    const GridCell* c21 = grid.GetCell(2, 1);
    assert(c21 != nullptr);
    assert(c21->count == 1);
    assert(grid.GetCellSegment(c21, 0).second.x == 250.0f);

    // Cell (0, 0) should be empty
    const GridCell* c00 = grid.GetCell(0, 0);
    assert(c00 != nullptr);
    assert(c00->count == 0);

    // Clear grid
    grid.Clear();
    assert(grid.GetSegmentArena().empty());
    assert(grid.GetIndexArena().empty());
    assert(grid.GetCell(1, 1)->count == 0);

    std::cout << "[TEST] FlatSpatialGrid Arena: ALL ASSERTIONS PASSED SUCCESSFULLY!" << std::endl;
    return 0;
}
