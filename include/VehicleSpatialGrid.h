#pragma once

#include "raylib.h"
#include "Config.h"
#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>

/**
 * @brief Oriented Bounding Box (OBB) representation for dynamic vehicles in 2D space.
 */
struct VehicleOBB {
    uint32_t carIndex = 0;              ///< Original index of vehicle in the population
    Vector2 position = { 0.0f, 0.0f };  ///< World position center
    Vector2 velocity = { 0.0f, 0.0f };  ///< Instantaneous chassis velocity vector
    float rotation = 0.0f;              ///< Heading orientation in degrees
    Vector2 corners[4];                 ///< 0: Front-Left, 1: Front-Right, 2: Rear-Right, 3: Rear-Left
    bool active = false;

    /**
     * @brief Retrieves starting point of the specified OBB edge [0..3].
     */
    [[nodiscard]] inline Vector2 GetSegmentStart(int edge) const noexcept {
        return corners[edge % 4];
    }

    /**
     * @brief Retrieves end point of the specified OBB edge [0..3].
     */
    [[nodiscard]] inline Vector2 GetSegmentEnd(int edge) const noexcept {
        return corners[(edge + 1) % 4];
    }
};

/**
 * @brief Dynamic secondary spatial hash grid for inter-vehicle OBB registration and queries.
 * 
 * Optimized for zero-allocation per-frame clearance and queries via sparse active cell tracking.
 */
class VehicleSpatialGrid {
public:
    static constexpr int MAX_CARS_PER_CELL = 16;

    struct Cell {
        uint8_t count = 0;
        uint16_t vehicleIndices[MAX_CARS_PER_CELL];
    };

    /**
     * @brief Constructs preallocated vehicle spatial acceleration grid.
     */
    VehicleSpatialGrid(float worldWidth = Config::WORLD_WIDTH, 
                       float worldHeight = Config::WORLD_HEIGHT, 
                       float cellSize = 60.0f)
        : m_worldWidth(worldWidth),
          m_worldHeight(worldHeight),
          m_cellSize(static_cast<int>(cellSize)),
          m_cols(static_cast<int>(std::ceil(worldWidth / cellSize))),
          m_rows(static_cast<int>(std::ceil(worldHeight / cellSize))) {
        m_cells.resize(static_cast<size_t>(m_cols * m_rows));
        m_registeredVehicles.reserve(Config::POPULATION_SIZE);
        m_activeCellIndices.reserve(Config::POPULATION_SIZE * 4);
    }

    /**
     * @brief Sparse O(K) frame clearance resetting only cells populated in the previous tick.
     */
    void Clear() noexcept {
        for (uint32_t idx : m_activeCellIndices) {
            m_cells[idx].count = 0;
        }
        m_activeCellIndices.clear();
        m_registeredVehicles.clear();
    }

    /**
     * @brief Registers an active vehicle into the spatial grid computing its 4-vertex OBB.
     */
    void RegisterVehicle(uint32_t carIndex, Vector2 position, Vector2 velocity, float rotation) noexcept {
        float rad = rotation * (3.14159265f / 180.0f);
        float cosA = cosf(rad);
        float sinA = sinf(rad);
        float noseX = cosA * Config::CAR_HALF_LENGTH;
        float noseY = sinA * Config::CAR_HALF_LENGTH;
        float rightX = -sinA * Config::CAR_HALF_WIDTH;
        float rightY = cosA * Config::CAR_HALF_WIDTH;

        VehicleOBB obb;
        obb.carIndex = carIndex;
        obb.position = position;
        obb.velocity = velocity;
        obb.rotation = rotation;
        obb.corners[0] = { position.x + noseX - rightX, position.y + noseY - rightY }; // Front-Left
        obb.corners[1] = { position.x + noseX + rightX, position.y + noseY + rightY }; // Front-Right
        obb.corners[2] = { position.x - noseX + rightX, position.y - noseY + rightY }; // Rear-Right
        obb.corners[3] = { position.x - noseX - rightX, position.y - noseY - rightY }; // Rear-Left
        obb.active = true;

        uint16_t regIdx = static_cast<uint16_t>(m_registeredVehicles.size());
        m_registeredVehicles.push_back(obb);

        // Calculate axis-aligned bounding box enclosing the OBB in grid space
        float minX = std::min({ obb.corners[0].x, obb.corners[1].x, obb.corners[2].x, obb.corners[3].x });
        float maxX = std::max({ obb.corners[0].x, obb.corners[1].x, obb.corners[2].x, obb.corners[3].x });
        float minY = std::min({ obb.corners[0].y, obb.corners[1].y, obb.corners[2].y, obb.corners[3].y });
        float maxY = std::max({ obb.corners[0].y, obb.corners[1].y, obb.corners[2].y, obb.corners[3].y });

        int gx0 = std::clamp(static_cast<int>(std::floor(minX / static_cast<float>(m_cellSize))), 0, m_cols - 1);
        int gx1 = std::clamp(static_cast<int>(std::floor(maxX / static_cast<float>(m_cellSize))), 0, m_cols - 1);
        int gy0 = std::clamp(static_cast<int>(std::floor(minY / static_cast<float>(m_cellSize))), 0, m_rows - 1);
        int gy1 = std::clamp(static_cast<int>(std::floor(maxY / static_cast<float>(m_cellSize))), 0, m_rows - 1);

        for (int y = gy0; y <= gy1; ++y) {
            for (int x = gx0; x <= gx1; ++x) {
                uint32_t cellIdx = static_cast<uint32_t>(y * m_cols + x);
                Cell& cell = m_cells[cellIdx];
                if (cell.count == 0) {
                    m_activeCellIndices.push_back(cellIdx);
                }
                if (cell.count < MAX_CARS_PER_CELL) {
                    cell.vehicleIndices[cell.count++] = regIdx;
                }
            }
        }
    }

    /**
     * @brief Retrieves cell pointer by discrete grid coordinates with bounds safety.
     */
    [[nodiscard]] inline const Cell* GetCell(int gx, int gy) const noexcept {
        if (gx >= 0 && gx < m_cols && gy >= 0 && gy < m_rows) {
            return &m_cells[static_cast<size_t>(gy * m_cols + gx)];
        }
        return nullptr;
    }

    /**
     * @brief Retrieves registered vehicle OBB by its registered index.
     */
    [[nodiscard]] inline const VehicleOBB& GetVehicle(uint16_t regIdx) const noexcept {
        return m_registeredVehicles[regIdx];
    }

    [[nodiscard]] inline size_t GetVehicleCount() const noexcept {
        return m_registeredVehicles.size();
    }

    [[nodiscard]] inline int GetCols() const noexcept { return m_cols; }
    [[nodiscard]] inline int GetRows() const noexcept { return m_rows; }
    [[nodiscard]] inline int GetCellSize() const noexcept { return m_cellSize; }
    [[nodiscard]] inline const std::vector<VehicleOBB>& GetAllVehicles() const noexcept { return m_registeredVehicles; }

private:
    float m_worldWidth;
    float m_worldHeight;
    int m_cellSize;
    int m_cols;
    int m_rows;
    std::vector<Cell> m_cells;
    std::vector<VehicleOBB> m_registeredVehicles;
    std::vector<uint32_t> m_activeCellIndices;
};
