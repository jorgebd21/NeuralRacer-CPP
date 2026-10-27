#pragma once

#include "raylib.h"
#include <vector>
#include <utility>
#include <algorithm>
#include <cmath>
#include <cstdint>

/**
 * @brief 2D line segment representing a track boundary barrier.
 */
using Segment = std::pair<Vector2, Vector2>;

/**
 * @brief Flat spatial cell containing segments overlapping its spatial bounds.
 */
struct GridCell {
    std::vector<Segment> segments;
};

/**
 * @brief High-performance 1D contiguous row-major spatial grid (FlatSpatialGrid).
 * 
 * Deprecates pointer-chasing and dynamic bucket collisions from std::unordered_map.
 * Uses direct linear indexing: idx = y * COLS + x, providing O(1) deterministic
 * lookups with contiguous CPU cache lines.
 */
class FlatSpatialGrid {
public:
    static constexpr int DEFAULT_CELL_SIZE = 100;

private:
    int m_cellSize = DEFAULT_CELL_SIZE;
    int m_cols = 0;
    int m_rows = 0;
    float m_width = 0.0f;
    float m_height = 0.0f;
    std::vector<GridCell> m_grid;

public:
    FlatSpatialGrid() = default;

    FlatSpatialGrid(float worldWidth, float worldHeight, int cellSize = DEFAULT_CELL_SIZE) {
        Init(worldWidth, worldHeight, cellSize);
    }

    /**
     * @brief Preallocates grid memory for given world dimensions.
     */
    void Init(float worldWidth, float worldHeight, int cellSize = DEFAULT_CELL_SIZE) {
        m_width = worldWidth;
        m_height = worldHeight;
        m_cellSize = cellSize > 0 ? cellSize : DEFAULT_CELL_SIZE;
        m_cols = static_cast<int>(std::ceil(worldWidth / static_cast<float>(m_cellSize))) + 2;
        m_rows = static_cast<int>(std::ceil(worldHeight / static_cast<float>(m_cellSize))) + 2;
        m_grid.assign(static_cast<size_t>(m_cols * m_rows), GridCell{});
    }

    /**
     * @brief Clears segments from all cells while retaining preallocated container capacity.
     */
    void Clear() {
        for (auto& cell : m_grid) {
            cell.segments.clear();
        }
    }

    /**
     * @brief Rebuilds the spatial partition from track boundary wall segments.
     */
    void Build(const std::vector<Segment>& walls) {
        Clear();
        for (const auto& wall : walls) {
            int grid1X = static_cast<int>(std::floor(wall.first.x / static_cast<float>(m_cellSize)));
            int grid1Y = static_cast<int>(std::floor(wall.first.y / static_cast<float>(m_cellSize)));
            int grid2X = static_cast<int>(std::floor(wall.second.x / static_cast<float>(m_cellSize)));
            int grid2Y = static_cast<int>(std::floor(wall.second.y / static_cast<float>(m_cellSize)));

            int startX = std::max(0, std::min(grid1X, grid2X));
            int startY = std::max(0, std::min(grid1Y, grid2Y));
            int endX = std::min(m_cols - 1, std::max(grid1X, grid2X));
            int endY = std::min(m_rows - 1, std::max(grid1Y, grid2Y));

            for (int x = startX; x <= endX; ++x) {
                for (int y = startY; y <= endY; ++y) {
                    m_grid[static_cast<size_t>(y * m_cols + x)].segments.push_back(wall);
                }
            }
        }
    }

    /**
     * @brief Retrieves cell pointer by 2D discrete grid coordinates with bounds safety.
     * @return const GridCell* Pointer to cell, or nullptr if out of bounds.
     */
    [[nodiscard]] inline const GridCell* GetCell(int x, int y) const noexcept {
        if (x >= 0 && x < m_cols && y >= 0 && y < m_rows) {
            return &m_grid[static_cast<size_t>(y * m_cols + x)];
        }
        return nullptr;
    }

    [[nodiscard]] inline int GetCellSize() const noexcept { return m_cellSize; }
    [[nodiscard]] inline int GetCols() const noexcept { return m_cols; }
    [[nodiscard]] inline int GetRows() const noexcept { return m_rows; }
    [[nodiscard]] inline size_t TotalCells() const noexcept { return m_grid.size(); }
};
