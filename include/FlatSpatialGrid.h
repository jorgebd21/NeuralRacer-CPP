#pragma once

#include "raylib.h"
#include "SurfaceDynamics.h"
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
 * @brief Lightweight index-based spatial grid cell slice.
 * Compact 8-byte structure maximizing L1 CPU cache residency.
 */
struct GridCell {
    uint32_t startIndex = 0;
    uint32_t count = 0;
};

/**
 * @brief High-performance 1D contiguous row-major spatial grid with pooled memory arena.
 * 
 * Segments and cell references are packed into contiguous linear memory arenas (std::vector<Segment>
 * and std::vector<uint32_t> indices), eliminating per-cell dynamic vector allocations and
 * maximizing cache locality during 60 Hz neighborhood sweeps.
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
    std::vector<Segment> m_segmentArena;
    std::vector<uint32_t> m_indexArena;
    std::vector<SurfaceType> m_surfaceGrid;

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
        m_segmentArena.clear();
        m_indexArena.clear();
        m_surfaceGrid.assign(static_cast<size_t>(m_cols * m_rows), SurfaceType::RunoffTurf);
    }

    /**
     * @brief Clears segments and index arena while retaining preallocated container capacity.
     */
    void Clear() {
        for (auto& cell : m_grid) {
            cell.startIndex = 0;
            cell.count = 0;
        }
        m_segmentArena.clear();
        m_indexArena.clear();
        std::fill(m_surfaceGrid.begin(), m_surfaceGrid.end(), SurfaceType::RunoffTurf);
    }

    /**
     * @brief Packs segments into contiguous memory arenas and indexes them into grid cells.
     */
    void Build(const std::vector<Segment>& walls) {
        m_segmentArena = walls;
        m_indexArena.clear();

        // Temporary builder bins for each cell
        std::vector<std::vector<uint32_t>> tempCellIndices(m_grid.size());

        for (uint32_t segIdx = 0; segIdx < static_cast<uint32_t>(walls.size()); ++segIdx) {
            const auto& wall = walls[segIdx];
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
                    tempCellIndices[static_cast<size_t>(y * m_cols + x)].push_back(segIdx);
                }
            }
        }

        // Pack bins into contiguous index arena
        for (size_t i = 0; i < m_grid.size(); ++i) {
            m_grid[i].startIndex = static_cast<uint32_t>(m_indexArena.size());
            m_grid[i].count = static_cast<uint32_t>(tempCellIndices[i].size());
            m_indexArena.insert(m_indexArena.end(), tempCellIndices[i].begin(), tempCellIndices[i].end());
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

    /**
     * @brief Retrieves segment reference from the segment memory arena by index.
     */
    [[nodiscard]] inline const Segment& GetSegment(uint32_t index) const noexcept {
        return m_segmentArena[index];
    }

    /**
     * @brief Retrieves segment index in the segment arena for a cell's local element.
     */
    [[nodiscard]] inline uint32_t GetCellSegmentIndex(const GridCell* cell, uint32_t localIndex) const noexcept {
        return m_indexArena[cell->startIndex + localIndex];
    }

    /**
     * @brief Retrieves segment reference directly from the arena using cell local index.
     */
    [[nodiscard]] inline const Segment& GetCellSegment(const GridCell* cell, uint32_t localIndex) const noexcept {
        return m_segmentArena[m_indexArena[cell->startIndex + localIndex]];
    }

    [[nodiscard]] inline const std::vector<Segment>& GetSegmentArena() const noexcept { return m_segmentArena; }
    [[nodiscard]] inline const std::vector<uint32_t>& GetIndexArena() const noexcept { return m_indexArena; }
    [[nodiscard]] inline const std::vector<SurfaceType>& GetSurfaceGrid() const noexcept { return m_surfaceGrid; }
    [[nodiscard]] inline int GetCellSize() const noexcept { return m_cellSize; }
    [[nodiscard]] inline int GetCols() const noexcept { return m_cols; }
    [[nodiscard]] inline int GetRows() const noexcept { return m_rows; }
    [[nodiscard]] inline size_t TotalCells() const noexcept { return m_grid.size(); }

    /**
     * @brief Sets the surface classification for a specific grid cell.
     */
    inline void SetCellSurface(int x, int y, SurfaceType type) noexcept {
        if (x >= 0 && x < m_cols && y >= 0 && y < m_rows) {
            m_surfaceGrid[static_cast<size_t>(y * m_cols + x)] = type;
        }
    }

    /**
     * @brief Gets the surface type for discrete grid coordinates.
     */
    [[nodiscard]] inline SurfaceType GetCellSurface(int x, int y) const noexcept {
        if (x >= 0 && x < m_cols && y >= 0 && y < m_rows) {
            return m_surfaceGrid[static_cast<size_t>(y * m_cols + x)];
        }
        return SurfaceType::GravelTrap;
    }

    /**
     * @brief Samples the track surface type at continuous world coordinates in O(1).
     */
    [[nodiscard]] inline SurfaceType SampleSurface(Vector2 pos) const noexcept {
        int gx = static_cast<int>(std::floor(pos.x / static_cast<float>(m_cellSize)));
        int gy = static_cast<int>(std::floor(pos.y / static_cast<float>(m_cellSize)));
        return GetCellSurface(gx, gy);
    }

    /**
     * @brief Samples instantaneous Coulomb friction coefficient (\mu) at world coordinates in O(1).
     */
    [[nodiscard]] inline float SampleFriction(Vector2 pos) const noexcept {
        return SurfaceDynamics::GetFrictionCoefficient(SampleSurface(pos));
    }
};
