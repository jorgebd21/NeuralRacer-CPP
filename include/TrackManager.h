#pragma once

#include "raylib.h"
#include <vector>
#include <string>
#include <utility>
#include <cstdint>
#include <unordered_map>

/**
 * @brief Namespace for utilities relating to track generation, processing, and loading.
 * 
 * Contains mathematical functions for interpolation (Catmull-Rom), collision detection, 
 * and reading/writing map files.
 */
namespace TrackManager {
    const int GRID_CELL_SIZE = 100;

    inline uint64_t GetGridKey(int x, int y) {
        return ((uint64_t)(uint32_t)x << 32) | (uint32_t)y;
    }

    /**
     * @brief Calculates the distance to the intersection of two line segments.
     * 
     * @param p1 Start of the first segment.
     * @param p2 End of the first segment.
     * @param p3 Start of the second segment.
     * @param p4 End of the second segment.
     * @param outDist Resulting distance if an intersection exists.
     * @return true If an intersection exists.
     * @return false If there is no intersection.
     */
    bool GetLineIntersectionDist(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, float &outDist);

    /**
     * @brief Interpolates a point using a Catmull-Rom spline curve.
     * 
     * @param p0 Previous control point.
     * @param p1 Start point of the segment.
     * @param p2 End point of the segment.
     * @param p3 Next control point.
     * @param t Interpolation value [0.0, 1.0].
     * @return Vector2 The interpolated point on the curve.
     */
    Vector2 GetCatmullRomPoint(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3, float t);

    /**
     * @brief Generates intermediate points along a list of points using splines.
     * 
     * @param points Base nodes of the circuit.
     * @param segmentsPerCurve Resolution of the curve (how many points to generate per base segment).
     * @return std::vector<Vector2> The smoothed path.
     */
    std::vector<Vector2> GenerateSplinePoints(const std::vector<Vector2>& points, int segmentsPerCurve = 15);

    /**
     * @brief Applies a smoothing filter to a series of points to round the curves.
     * 
     * @param pts Vector of points to smooth (modified in-place).
     * @param radius Reach of smoothing forwards and backwards.
     * @param passes Number of iterations for the smoothing algorithm.
     */
    void SmoothPoints(std::vector<Vector2>& pts, int radius, int passes);

    /**
     * @brief Checks if two line segments intersect and gets the exact point.
     * 
     * @param p1 Start of the first segment.
     * @param p2 End of the first segment.
     * @param p3 Start of the second segment.
     * @param p4 End of the second segment.
     * @param outIntersection Exact coordinate of the intersection.
     * @return true If the segments intersect.
     * @return false If they do not intersect.
     */
    bool GetSegmentIntersection(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, Vector2& outIntersection);

    /**
     * @brief Detects and removes loops where the circuit crosses itself.
     * 
     * @param pts The list of points that defines the circuit.
     */
    void RemoveSelfIntersections(std::vector<Vector2>& pts);

    /**
     * @brief Generates the outer and inner walls of the circuit from its center line.
     * 
     * @param centerPoints Points that define the optimal route of the circuit.
     * @param trackWidth Total width of the track.
     * @param outWalls Vector where the track borders will be added.
     * @param outCheckpoints Vector where the checkpoints will be stored.
     * @param startPosition Initial position to generate checkpoints appropriately.
     */
    void GenerateBordersFromCenterLine(const std::vector<Vector2>& centerPoints, float trackWidth, std::vector<std::pair<Vector2, Vector2>>& outWalls, std::vector<Vector2>& outCheckpoints, Vector2 startPosition);

    /**
     * @brief Determines the optimal initial position and rotation for the cars.
     * 
     * @param proceduralPoints The center nodes of the circuit.
     * @param startPosition Output parameter for the initial position.
     * @param startRotation Output parameter for the initial rotation in degrees.
     */
    void CalculateStartGrid(const std::vector<Vector2>& proceduralPoints, Vector2& startPosition, float& startRotation);

    /**
     * @brief Loads a complete circuit (walls and starting point) from a text file.
     * 
     * @param filename Path to the circuit file.
     * @param outWalls Vector where the map borders will be added.
     * @param outStartPos Output parameter for the starting position read from the map.
     * @param outStartRot Output parameter for the calculated or obtained starting rotation.
     * @param outCheckpoints Vector where the checkpoints will be stored.
     */
    void LoadTrackFromFile(const std::string& filename, std::vector<std::pair<Vector2, Vector2>>& outWalls, Vector2& outStartPos, float& outStartRot, std::vector<Vector2>& outCheckpoints);

    /**
     * @brief Explores the maps directory and returns a list of available names.
     * 
     * @return std::vector<std::string> Names of the files found in the maps folder.
     */
    std::vector<std::string> ScanMapFiles();
}
