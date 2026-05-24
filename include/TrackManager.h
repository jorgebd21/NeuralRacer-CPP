#pragma once

#include "raylib.h"
#include <vector>
#include <string>
#include <utility>

namespace TrackManager {
    bool GetLineIntersectionDist(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, float &outDist);
    Vector2 GetCatmullRomPoint(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3, float t);
    std::vector<Vector2> GenerateSplinePoints(const std::vector<Vector2>& points, int segmentsPerCurve = 15);
    void SmoothPoints(std::vector<Vector2>& pts, int radius, int passes);
    bool GetSegmentIntersection(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, Vector2& outIntersection);
    void RemoveSelfIntersections(std::vector<Vector2>& pts);
    void GenerateBordersFromCenterLine(const std::vector<Vector2>& centerPoints, float trackWidth, std::vector<std::pair<Vector2, Vector2>>& outWalls);
    void CalculateStartGrid(const std::vector<Vector2>& puntosProcedurales, Vector2& startPosition, float& startRotation);
    void LoadTrackFromFile(const std::string& filename, std::vector<std::pair<Vector2, Vector2>>& outWalls, Vector2& outStartPos, float& outStartRot);
    std::vector<std::string> ScanMapFiles();
}
