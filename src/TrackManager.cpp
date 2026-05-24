#include "TrackManager.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <filesystem>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

namespace TrackManager {

constexpr int MAX_INTERSECTION_ITER = 5000;
constexpr int MAX_INTERSECTION_SEARCH = 150;
constexpr int BORDER_STEP = 8;
constexpr int SMOOTH_RADIUS = 4;
constexpr int SMOOTH_PASSES = 5;
constexpr int TRACK_SPLINE_SEGMENTS = 50;
constexpr float TRACK_WIDTH = 65.0f;

bool GetLineIntersectionDist(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, float &outDist) {
    float den = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
    if (den == 0) return false;
    float t = ((p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)) / den;
    float u = -((p1.x - p2.x) * (p1.y - p3.y) - (p1.y - p2.y) * (p1.x - p3.x)) / den;
    if (t > 0 && t < 1 && u > 0 && u < 1) {
        Vector2 pt = { p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y) };
        outDist = sqrt(pow(pt.x - p1.x, 2) + pow(pt.y - p1.y, 2));
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
        Vector2 p0 = points[(i - 1 + n) % n];
        Vector2 p1 = points[i];
        Vector2 p2 = points[(i + 1) % n];
        Vector2 p3 = points[(i + 2) % n];
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

void GenerateBordersFromCenterLine(const std::vector<Vector2>& centerPoints, float trackWidth, std::vector<std::pair<Vector2, Vector2>>& outWalls) {
    int n = centerPoints.size();
    if (n < 2) return;
    float halfWidth = trackWidth / 2.0f;
    std::vector<Vector2> outerPoints(n);
    std::vector<Vector2> innerPoints(n);
    int step = BORDER_STEP;
    for (int i = 0; i < n; i++) {
        Vector2 prev = centerPoints[(i - step + n) % n];
        Vector2 next = centerPoints[(i + step) % n];
        Vector2 dir = {next.x - prev.x, next.y - prev.y};
        float length = sqrt(dir.x * dir.x + dir.y * dir.y);
        if (length < 0.0001f) dir = {1.0f, 0.0f};
        else { dir.x /= length; dir.y /= length; }
        Vector2 normal = {-dir.y, dir.x};
        outerPoints[i] = {centerPoints[i].x + normal.x * halfWidth, centerPoints[i].y + normal.y * halfWidth};
        innerPoints[i] = {centerPoints[i].x - normal.x * halfWidth, centerPoints[i].y - normal.y * halfWidth};
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

void CalculateStartGrid(const std::vector<Vector2>& puntosProcedurales, Vector2& startPosition, float& startRotation) {
    if (puntosProcedurales.empty()) return;
    int n = puntosProcedurales.size();
    float maxDist = 0;
    int bestIdx = 0;
    for (int i = 0; i < n; i++) {
        Vector2 p1 = puntosProcedurales[i];
        Vector2 p2 = puntosProcedurales[(i+1)%n];
        float d = (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y);
        if (d > maxDist) {
            maxDist = d;
            bestIdx = i;
        }
    }
    Vector2 p1 = puntosProcedurales[bestIdx];
    Vector2 p2 = puntosProcedurales[(bestIdx+1)%n];
    startPosition.x = (p1.x + p2.x) / 2.0f;
    startPosition.y = (p1.y + p2.y) / 2.0f;
    startRotation = atan2(p2.y - p1.y, p2.x - p1.x) * (180.0f / PI);
}

void LoadTrackFromFile(const std::string& filename, std::vector<std::pair<Vector2, Vector2>>& outWalls, Vector2& outStartPos, float& outStartRot) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: No se pudo abrir " << filename << std::endl;
        return;
    }
    std::vector<Vector2> centerPoints;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        float x, y;
        if (ss >> x >> y) {
            centerPoints.push_back({x, y});
        }
    }
    if (centerPoints.size() >= 3) {
        CalculateStartGrid(centerPoints, outStartPos, outStartRot);
        std::vector<Vector2> denseCenterLine = GenerateSplinePoints(centerPoints, TRACK_SPLINE_SEGMENTS);
        GenerateBordersFromCenterLine(denseCenterLine, TRACK_WIDTH, outWalls);
    }
}

std::vector<std::string> ScanMapFiles() {
    std::vector<std::string> mapFiles;
    for (const auto& entry : std::filesystem::directory_iterator("data/tracks")) {
        if (entry.path().extension() == ".txt") {
            std::string name = entry.path().filename().string();
            if (name.rfind("pista_", 0) == 0) {
                mapFiles.push_back("data/tracks/" + name);
            }
        }
    }
    if (mapFiles.empty()) mapFiles.push_back("data/tracks/pista_facil.txt");
    std::sort(mapFiles.begin(), mapFiles.end());
    return mapFiles;
}

} // namespace TrackManager
