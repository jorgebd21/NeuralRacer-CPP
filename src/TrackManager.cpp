#include "TrackManager.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <filesystem>
#include "json.hpp"


namespace TrackManager {

constexpr int MAX_INTERSECTION_ITER = 5000;
constexpr int MAX_INTERSECTION_SEARCH = 150;
constexpr int BORDER_STEP = 8;
constexpr int SMOOTH_RADIUS = 4;
constexpr int SMOOTH_PASSES = 5;
constexpr int TRACK_SPLINE_SEGMENTS = 50;
constexpr float TRACK_WIDTH = 65.0f;

bool GetLineIntersectionDist(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, float &outDist) {
    // Calculate the denominator with the determinant of the lines
    float den = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
    if (std::abs(den) < 1e-6f) return false; // Parallel or coincident lines
    
    // t and u represent the relative intersection point on both segments
    float t = ((p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)) / den;
    float u = -((p1.x - p2.x) * (p1.y - p3.y) - (p1.y - p2.y) * (p1.x - p3.x)) / den;
    
    // If t and u are between 0 and 1, the intersection occurs within the segments themselves
    if (t > 0 && t < 1 && u > 0 && u < 1) {
        Vector2 pt = { p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y) };
        outDist = sqrt(pow(pt.x - p1.x, 2) + pow(pt.y - p1.y, 2)); // Save the distance from p1 to the intersection
        return true;
    }
    return false;
}

Vector2 GetCatmullRomPoint(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3, float t) {
    float t2 = t * t;
    float t3 = t2 * t;
    Vector2 result;
    result.x = 0.5f * ((2.0f * p1.x) + (-p0.x + p2.x) * t + (2.0f * p0.x - 5.0f * p1.x + 4.0f * p2.x - p3.x) * t2 + (-p0.x + 3.0f * p1.x - 3.0f * p2.x + p3.x) * t3);
    result.y = 0.5f * ((2.0f * p1.y) + (-p0.y + p2.y) * t + (2.0f * p0.y - 5.0f * p1.y + 4.0f * p2.y - p3.y) * t2 + (-p0.y + 3.0f * p1.y - 3.0f * p2.y + p3.y) * t3);
    return result;
}

std::vector<Vector2> GenerateSplinePoints(const std::vector<Vector2>& points, int segmentsPerCurve) {
    std::vector<Vector2> densePoints;
    if (points.size() < 3) return densePoints;
    
    int n = points.size();
    for (int i = 0; i < n; i++) {
        // Take 4 consecutive points for the Catmull-Rom Spline. 
        // Use modulo to close the track in a continuous loop.
        Vector2 p0 = points[(i - 1 + n) % n];
        Vector2 p1 = points[i];
        Vector2 p2 = points[(i + 1) % n];
        Vector2 p3 = points[(i + 2) % n];
        
        // Interpolate 'segmentsPerCurve' points along this curve segment
        for (int j = 0; j < segmentsPerCurve; j++) {
            float t = (float)j / segmentsPerCurve;
            Vector2 pt = GetCatmullRomPoint(p0, p1, p2, p3, t);
            densePoints.push_back(pt);
        }
    }
    return densePoints;
}

void SmoothPoints(std::vector<Vector2>& pts, int radius, int passes) {
    int n = pts.size();
    for (int pass = 0; pass < passes; pass++) {
        std::vector<Vector2> tmp(n);
        for (int i = 0; i < n; i++) {
            Vector2 acc = {0, 0};
            int count = 0;
            for (int j = -radius; j <= radius; j++) {
                int idx = (i + j + n) % n;
                acc.x += pts[idx].x;
                acc.y += pts[idx].y;
                count++;
            }
            tmp[i] = {acc.x / count, acc.y / count};
        }
        pts = tmp;
    }
}

bool GetSegmentIntersection(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, Vector2& outIntersection) {
    float den = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
    if (std::abs(den) < 1e-6f) return false;
    float t = ((p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)) / den;
    float u = -((p1.x - p2.x) * (p1.y - p3.y) - (p1.y - p2.y) * (p1.x - p3.x)) / den;
    if (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f) {
        outIntersection = { p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y) };
        return true;
    }
    return false;
}

void RemoveSelfIntersections(std::vector<Vector2>& pts) {
    int n = pts.size();
    if (n < 4) return;
    bool foundIntersection = true;
    int maxIterations = MAX_INTERSECTION_ITER;
    int iterations = 0;
    while (foundIntersection && iterations++ < maxIterations) {
        foundIntersection = false;
        n = pts.size();
        for (int i = 0; i < n; i++) {
            Vector2 p1 = pts[i];
            Vector2 p2 = pts[(i + 1) % n];
            int maxSearch = std::min(n - 1, MAX_INTERSECTION_SEARCH); 
            for (int k = 2; k < maxSearch; k++) {
                int j = (i + k) % n;
                Vector2 p3 = pts[j];
                Vector2 p4 = pts[(j + 1) % n];
                Vector2 intersectPt;
                if (GetSegmentIntersection(p1, p2, p3, p4, intersectPt)) {
                    std::vector<Vector2> newPts;
                    newPts.reserve(n);
                    if (i < j) {
                        for (int idx = 0; idx <= i; idx++) newPts.push_back(pts[idx]);
                        newPts.push_back(intersectPt);
                        for (int idx = j + 1; idx < n; idx++) newPts.push_back(pts[idx]);
                    } else {
                        for (int idx = j + 1; idx <= i; idx++) newPts.push_back(pts[idx]);
                        newPts.push_back(intersectPt);
                    }
                    pts = newPts;
                    foundIntersection = true;
                    break;
                }
            }
            if (foundIntersection) break;
        }
    }
}

void GenerateBordersFromCenterLine(const std::vector<Vector2>& centerPoints, float trackWidth, std::vector<std::pair<Vector2, Vector2>>& outWalls, std::vector<Vector2>& outCheckpoints, Vector2 startPosition) {
    int n = centerPoints.size();
    if (n < 2) return;
    float halfWidth = trackWidth / 2.0f;
    std::vector<Vector2> outerPoints(n);
    std::vector<Vector2> innerPoints(n);
    int step = BORDER_STEP;
    for (int i = 0; i < n; i++) {
        // We get an "earlier" and "later" point further away (given by 'step')
        // to calculate a smoothed tangent, instead of using immediately adjacent points.
        Vector2 prev = centerPoints[(i - step + n) % n];
        Vector2 next = centerPoints[(i + step) % n];
        
        // Calculate the vector direction
        Vector2 dir = {next.x - prev.x, next.y - prev.y};
        float length = sqrt(dir.x * dir.x + dir.y * dir.y);
        if (length < 0.0001f) dir = {1.0f, 0.0f}; // Fallback if points accidentally coincide
        else { dir.x /= length; dir.y /= length; }
        
        // Rotate the tangent 90 degrees to get the Normal (perpendicular)
        Vector2 normal = {-dir.y, dir.x};
        
        // Extrude outwards and inwards applying the defined width
        outerPoints[i] = {centerPoints[i].x + normal.x * halfWidth, centerPoints[i].y + normal.y * halfWidth};
        innerPoints[i] = {centerPoints[i].x - normal.x * halfWidth, centerPoints[i].y - normal.y * halfWidth};
    }

    int startIndex = 0;
    float minDist = 1e9f;
    for (int i = 0; i < n; i++) {
        float d = (centerPoints[i].x - startPosition.x) * (centerPoints[i].x - startPosition.x) + 
                  (centerPoints[i].y - startPosition.y) * (centerPoints[i].y - startPosition.y);
        if (d < minDist) {
            minDist = d;
            startIndex = i;
        }
    }

    int numCheckpoints = std::min(5, n);
    if (numCheckpoints > 0) {
        std::vector<int> midpoints;
        for (int i = TRACK_SPLINE_SEGMENTS / 2; i < n; i += TRACK_SPLINE_SEGMENTS) {
            midpoints.push_back(i);
        }

        if (midpoints.size() > 0) {
            int startMidpointIdx = 0;
            int minMidpointDist = 1e9;
            for (size_t i = 0; i < midpoints.size(); i++) {
                int dist = std::min(std::abs(midpoints[i] - startIndex), n - std::abs(midpoints[i] - startIndex));
                if (dist < minMidpointDist) {
                    minMidpointDist = dist;
                    startMidpointIdx = i;
                }
            }

            int midpointsCount = midpoints.size();
            for (int c = 1; c < numCheckpoints; c++) {
                int mIdx = (startMidpointIdx + (c * midpointsCount) / numCheckpoints) % midpointsCount;
                int centerIdx = midpoints[mIdx];
                
                Vector2 prev = centerPoints[(centerIdx - 1 + n) % n];
                Vector2 next = centerPoints[(centerIdx + 1) % n];
                Vector2 dir = {next.x - prev.x, next.y - prev.y};
                float length = sqrt(dir.x * dir.x + dir.y * dir.y);
                if (length > 0.0001f) { dir.x /= length; dir.y /= length; }
                else { dir = {1.0f, 0.0f}; }
                
                outCheckpoints.push_back(centerPoints[centerIdx]);
            }
        }
        
        outCheckpoints.push_back(centerPoints[startIndex]);
    }

    RemoveSelfIntersections(outerPoints);
    RemoveSelfIntersections(innerPoints);
    SmoothPoints(outerPoints, SMOOTH_RADIUS, SMOOTH_PASSES);
    SmoothPoints(innerPoints, SMOOTH_RADIUS, SMOOTH_PASSES);
    RemoveSelfIntersections(outerPoints);
    RemoveSelfIntersections(innerPoints);
    for (size_t i = 0; i < outerPoints.size(); i++) {
        size_t nextIdx = (i + 1) % outerPoints.size();
        outWalls.push_back({outerPoints[i], outerPoints[nextIdx]});
    }
    for (size_t i = 0; i < innerPoints.size(); i++) {
        size_t nextIdx = (i + 1) % innerPoints.size();
        outWalls.push_back({innerPoints[i], innerPoints[nextIdx]});
    }
}

void CalculateStartGrid(const std::vector<Vector2>& proceduralPoints, Vector2& startPosition, float& startRotation) {
    if (proceduralPoints.empty()) return;
    int n = proceduralPoints.size();
    float maxDist = 0;
    int bestIdx = 0;
    for (int i = 0; i < n; i++) {
        Vector2 p1 = proceduralPoints[i];
        Vector2 p2 = proceduralPoints[(i+1)%n];
        float d = (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y);
        
        // Save the longest segment found
        if (d > maxDist) {
            maxDist = d;
            bestIdx = i;
        }
    }
    
    // Position the cars right in the middle of the straightest segment to give them room to accelerate
    Vector2 p1 = proceduralPoints[bestIdx];
    Vector2 p2 = proceduralPoints[(bestIdx+1)%n];
    startPosition.x = (p1.x + p2.x) / 2.0f;
    startPosition.y = (p1.y + p2.y) / 2.0f;
    
    // Calculate the angle so the car faces the direction of the segment
    startRotation = atan2(p2.y - p1.y, p2.x - p1.x) * (180.0f / PI);
}

void LoadTrackFromFile(const std::string& filename, std::vector<std::pair<Vector2, Vector2>>& outWalls, Vector2& outStartPos, float& outStartRot, std::vector<Vector2>& outCheckpoints) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open " << filename << std::endl;
        return;
    }
    std::vector<Vector2> centerPoints;
    nlohmann::json j;
    try {
        j = nlohmann::json::parse(file);
    } catch (const nlohmann::json::parse_error& e) {
        std::cerr << "Error: Failed to parse track file '" << filename << "': " << e.what() << std::endl;
        return;
    }
    
    // Support both old and new keys for loading track files
    if (j.contains("puntos_centrales")) {
        for (const auto& point : j["puntos_centrales"]) {
            centerPoints.push_back({point["x"], point["y"]});
        }
    } else if (j.contains("center_points")) {
        for (const auto& point : j["center_points"]) {
            centerPoints.push_back({point["x"], point["y"]});
        }
    }
    
    if (centerPoints.size() >= 3) {
        CalculateStartGrid(centerPoints, outStartPos, outStartRot);
        std::vector<Vector2> denseCenterLine = GenerateSplinePoints(centerPoints, TRACK_SPLINE_SEGMENTS);
        GenerateBordersFromCenterLine(denseCenterLine, TRACK_WIDTH, outWalls, outCheckpoints, outStartPos);
    }
}

std::vector<std::string> ScanMapFiles() {
    std::vector<std::string> mapFiles;
    for (const auto& entry : std::filesystem::directory_iterator("data/tracks")) {
        if (entry.path().extension() == ".json") {
            std::string name = entry.path().filename().string();
            if (name.rfind("pista_", 0) == 0 || name.rfind("procedural_track_", 0) == 0) {
                mapFiles.push_back("data/tracks/" + name);
            }
        }
    }
    if (mapFiles.empty()) {
        // Fallback or default
        std::ifstream defaultCheck("data/tracks/pista_facil.json");
        if (defaultCheck.is_open()) {
            mapFiles.push_back("data/tracks/pista_facil.json");
        }
    }
    std::sort(mapFiles.begin(), mapFiles.end());
    return mapFiles;
}

} // namespace TrackManager
