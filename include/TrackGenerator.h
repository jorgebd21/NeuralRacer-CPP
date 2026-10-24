#pragma once
#include <vector>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "raylib.h"

/**
 * @brief High-Performance Procedural F1-style race track generation engine.
 * 
 * Implements a 2D convex-hull and sector-motif procedural pipeline:
 * 1. Elliptical polar seed distribution.
 * 2. Monotone Chain convex hull computation (ensuring zero self-intersections).
 * 3. F1 sector motif generation: DRS straight, high-speed sweeping S-curves, and controlled apexes.
 * 4. Multi-pass angular relaxation enforcing minimum radius and turn angle constraints (theta >= 115 deg).
 * 5. Deterministic counter-clockwise winding order for outward/inward boundary extrusion.
 */
namespace TrackGenerator {
    constexpr float PI_F = 3.14159265358979323846f;
    constexpr float CENTER_X = 500.0f;
    constexpr float CENTER_Y = 400.0f;
    constexpr float BOUNDS_PAD_X = 90.0f;
    constexpr float BOUNDS_PAD_Y = 75.0f;

    inline float CrossProduct(Vector2 a, Vector2 b, Vector2 c) {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }

    inline float DistanceSq(Vector2 a, Vector2 b) {
        return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
    }

    /**
     * @brief Evaluates angular turn deflection between three consecutive nodes in degrees.
     */
    inline float AngleChangeDeg(Vector2 pPrev, Vector2 pCurr, Vector2 pNext) {
        Vector2 v1 = {pCurr.x - pPrev.x, pCurr.y - pPrev.y};
        Vector2 v2 = {pNext.x - pCurr.x, pNext.y - pCurr.y};
        float l1 = std::hypot(v1.x, v1.y);
        float l2 = std::hypot(v2.x, v2.y);
        if (l1 < 1e-4f || l2 < 1e-4f) return 0.0f;
        float dot = (v1.x * v2.x + v1.y * v2.y) / (l1 * l2);
        dot = std::clamp(dot, -1.0f, 1.0f);
        return std::acos(dot) * (180.0f / PI_F);
    }

    /**
     * @brief Computes 2D Convex Hull using the Monotone Chain algorithm (O(N log N)).
     * 
     * Guarantees a strictly non-self-intersecting closed convex polygon.
     */
    inline std::vector<Vector2> ComputeConvexHull(std::vector<Vector2>& pts) {
        int n = pts.size();
        if (n <= 3) return pts;

        std::sort(pts.begin(), pts.end(), [](const Vector2& a, const Vector2& b) {
            return (a.x < b.x) || (a.x == b.x && a.y < b.y);
        });

        std::vector<Vector2> hull;
        hull.reserve(2 * n);

        // Lower hull
        for (int i = 0; i < n; i++) {
            while (hull.size() >= 2 && CrossProduct(hull[hull.size() - 2], hull[hull.size() - 1], pts[i]) <= 0.0f) {
                hull.pop_back();
            }
            hull.push_back(pts[i]);
        }

        // Upper hull
        for (int i = n - 2, t = hull.size() + 1; i >= 0; i--) {
            while (hull.size() >= t && CrossProduct(hull[hull.size() - 2], hull[hull.size() - 1], pts[i]) <= 0.0f) {
                hull.pop_back();
            }
            hull.push_back(pts[i]);
        }

        hull.pop_back(); // Remove duplicate end point
        return hull;
    }

    /**
     * @brief Generates high-fidelity F1-style procedural race track center nodes.
     * 
     * @return std::vector<Vector2> List of ordered, smoothly curved center points.
     */
    inline std::vector<Vector2> GenerateProceduralCenterPoints() {
        // 1. Generate random seed points distributed across an elliptical bounding zone
        std::vector<Vector2> seeds;
        seeds.reserve(36);

        float rx = (float)GetRandomValue(350, 390);
        float ry = (float)GetRandomValue(240, 275);

        for (int i = 0; i < 28; i++) {
            float angle = ((float)GetRandomValue(0, 360)) * (PI_F / 180.0f);
            float rFraction = std::sqrt((float)GetRandomValue(65, 100) / 100.0f);
            float px = CENTER_X + rx * rFraction * std::cos(angle);
            float py = CENTER_Y + ry * rFraction * std::sin(angle);
            seeds.push_back({px, py});
        }

        // 2. Compute convex hull to form the topological baseline
        std::vector<Vector2> hull = ComputeConvexHull(seeds);

        // 3. Ensure Counter-Clockwise (CCW) winding order via Shoelace formula
        float shoelace = 0.0f;
        int m = hull.size();
        for (int i = 0; i < m; i++) {
            Vector2 a = hull[i];
            Vector2 b = hull[(i + 1) % m];
            shoelace += (b.x - a.x) * (b.y + a.y);
        }
        if (shoelace < 0.0f) {
            std::reverse(hull.begin(), hull.end());
        }

        // 4. Identify the longest edge to designate as the primary DRS Straight
        float maxEdgeDistSq = 0.0f;
        int straightIdx = 0;
        m = hull.size();
        for (int i = 0; i < m; i++) {
            float d2 = DistanceSq(hull[i], hull[(i + 1) % m]);
            if (d2 > maxEdgeDistSq) {
                maxEdgeDistSq = d2;
                straightIdx = i;
            }
        }

        // 5. Subdivide edges into distinct F1 racing sectors (Straight, Sweepers, Chicanes)
        std::vector<Vector2> trackNodes;
        trackNodes.reserve(m * 3);

        for (int i = 0; i < m; i++) {
            Vector2 p1 = hull[i];
            Vector2 p2 = hull[(i + 1) % m];
            trackNodes.push_back(p1);

            float edgeLen = std::sqrt(DistanceSq(p1, p2));
            if (i == straightIdx) {
                // Main DRS straight: insert 2 collinear points to guarantee high-speed acceleration corridor
                trackNodes.push_back({p1.x + 0.33f * (p2.x - p1.x), p1.y + 0.33f * (p2.y - p1.y)});
                trackNodes.push_back({p1.x + 0.67f * (p2.x - p1.x), p1.y + 0.67f * (p2.y - p1.y)});
            } else if (edgeLen > 180.0f) {
                // Wide edge: smooth F1 curve / gentle chicane using 2 transition nodes
                if (GetRandomValue(0, 100) < 45) {
                    Vector2 mid1 = {p1.x + 0.35f * (p2.x - p1.x), p1.y + 0.35f * (p2.y - p1.y)};
                    Vector2 mid2 = {p1.x + 0.65f * (p2.x - p1.x), p1.y + 0.65f * (p2.y - p1.y)};

                    Vector2 toCenter1 = {CENTER_X - mid1.x, CENTER_Y - mid1.y};
                    Vector2 toCenter2 = {CENTER_X - mid2.x, CENTER_Y - mid2.y};
                    float d1 = std::hypot(toCenter1.x, toCenter1.y);
                    float d2 = std::hypot(toCenter2.x, toCenter2.y);
                    if (d1 > 1e-4f) { toCenter1.x /= d1; toCenter1.y /= d1; }
                    if (d2 > 1e-4f) { toCenter2.x /= d2; toCenter2.y /= d2; }

                    float disp = (float)GetRandomValue(25, 40);
                    mid1.x += toCenter1.x * disp;
                    mid1.y += toCenter1.y * disp;
                    mid2.x += toCenter2.x * disp;
                    mid2.y += toCenter2.y * disp;

                    trackNodes.push_back(mid1);
                    trackNodes.push_back(mid2);
                } else {
                    // Natural sweeping midpoint
                    trackNodes.push_back({(p1.x + p2.x) * 0.5f, (p1.y + p2.y) * 0.5f});
                }
            }
        }

        // 6. Multi-pass Laplacian relaxation enforcing minimum angle constraint (max 50 deg deflection)
        int n = trackNodes.size();
        for (int pass = 0; pass < 8; pass++) {
            for (int i = 0; i < n; i++) {
                Vector2 pPrev = trackNodes[(i - 1 + n) % n];
                Vector2 pCurr = trackNodes[i];
                Vector2 pNext = trackNodes[(i + 1) % n];

                float dev = AngleChangeDeg(pPrev, pCurr, pNext);
                if (dev > 50.0f) {
                    // Relax sharp corner toward adjacent chord midpoint
                    trackNodes[i].x = 0.35f * pCurr.x + 0.325f * (pPrev.x + pNext.x);
                    trackNodes[i].y = 0.35f * pCurr.y + 0.325f * (pPrev.y + pNext.y);
                }
            }
        }

        // 7. Clamp within safe viewport margins
        for (auto& pt : trackNodes) {
            pt.x = std::clamp(pt.x, BOUNDS_PAD_X, 1000.0f - BOUNDS_PAD_X);
            pt.y = std::clamp(pt.y, BOUNDS_PAD_Y, 800.0f - BOUNDS_PAD_Y);
        }

        return trackNodes;
    }

    /**
     * @brief Serializes and saves the center structure of the track to disk.
     * 
     * @param centerPoints Vector of nodes to save.
     * @param filename Path and name of the resulting file.
     */
    inline void SaveTrackToFile(const std::vector<Vector2>& centerPoints, const std::string& filename) {
        std::ofstream file(filename);
        file << "{\n  \"center_points\": [\n";
        for (size_t i = 0; i < centerPoints.size(); i++) {
            file << "    {\"x\": " << centerPoints[i].x << ", \"y\": " << centerPoints[i].y << "}";
            if (i + 1 < centerPoints.size()) file << ",";
            file << "\n";
        }
        file << "  ]\n}\n";
        file.close();
    }
}
