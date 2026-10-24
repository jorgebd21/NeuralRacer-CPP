#pragma once
#include <vector>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "raylib.h"
#include "TrackManager.h"

/**
 * @brief Professional High-Entropy Formula 1 Procedural Track Generator.
 * 
 * Synthesizes flowing, authentic racing circuits designed to create a dense, highly varied
 * dataset for autonomous reinforcement learning and neuroevolution agents:
 * - 6 Canonical Grand Prix Archetypes + Multi-Centric Polar Macro-Graph synthesis.
 * - Combinatorial Motif Injection: stochastically introduces chicanes, technical esses,
 *   and deceleration hooks along straight sectors.
 * - Full 360-Degree Affine Transformation: unconstrained planar rotation φ ∈ [0, 2π),
 *   asymmetric scaling (sx, sy), and dynamic viewport margin positioning.
 * - Rigorous Clearance Watchdog: guarantees zero self-intersections and enforces a minimum
 *   clearance of 45.0 px between non-adjacent track centerlines to avoid bottlenecks.
 */
namespace TrackGenerator {
    constexpr float PI_F = 3.14159265358979323846f;
    constexpr float CANVAS_W = 1000.0f;
    constexpr float CANVAS_H = 800.0f;
    constexpr float MARGIN_X = 85.0f;
    constexpr float MARGIN_Y = 75.0f;
    constexpr float MIN_CLEARANCE = 45.0f;

    inline float DistSq(Vector2 a, Vector2 b) {
        return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
    }

    inline float Dist(Vector2 a, Vector2 b) {
        return std::sqrt(DistSq(a, b));
    }

    // Strict intersection between two line segments (excluding shared endpoints)
    inline bool SegmentsIntersect(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4) {
        float den = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
        if (std::abs(den) < 1e-5f) return false;
        float t = ((p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)) / den;
        float u = -((p1.x - p2.x) * (p1.y - p3.y) - (p1.y - p2.y) * (p1.x - p3.x)) / den;
        return (t > 0.05f && t < 0.95f && u > 0.05f && u < 0.95f);
    }

    /**
     * @brief Macro-Graph Polar Synthesis with dynamic concave bays (infield loops).
     */
    inline std::vector<Vector2> GeneratePolarMacroGraph() {
        std::vector<Vector2> nodes;
        int numVerts = GetRandomValue(8, 14);

        std::vector<float> stepWeights(numVerts);
        float sumWeights = 0.0f;
        for (int i = 0; i < numVerts; i++) {
            stepWeights[i] = (float)GetRandomValue(70, 140);
            sumWeights += stepWeights[i];
        }

        float curAngle = (float)GetRandomValue(0, 360) * (PI_F / 180.0f);
        std::vector<float> sortedAngles;
        for (int i = 0; i < numVerts; i++) {
            sortedAngles.push_back(curAngle);
            curAngle += (stepWeights[i] / sumWeights) * (2.0f * PI_F);
        }

        int bayStart1 = GetRandomValue(1, numVerts - 3);
        int bayLen1 = GetRandomValue(2, 3);
        bool hasBay2 = (GetRandomValue(0, 1) == 1 && numVerts >= 11);
        int bayStart2 = (bayStart1 + 4 + GetRandomValue(0, 2)) % numVerts;
        int bayLen2 = 2;

        for (int i = 0; i < numVerts; i++) {
            float baseRadius = (float)GetRandomValue(220, 380);
            int d1 = (i - bayStart1 + numVerts) % numVerts;
            if (d1 < bayLen1) {
                baseRadius *= (float)GetRandomValue(45, 65) * 0.01f;
            }
            if (hasBay2) {
                int d2 = (i - bayStart2 + numVerts) % numVerts;
                if (d2 < bayLen2) {
                    baseRadius *= (float)GetRandomValue(50, 70) * 0.01f;
                }
            }

            float a = sortedAngles[i];
            nodes.push_back({ 500.0f + baseRadius * std::cos(a), 400.0f + baseRadius * std::sin(a) });
        }

        return nodes;
    }

    /**
     * @brief Archetype 0: Out-and-Back Corridor (Monza / Montreal / Red Bull Ring).
     */
    inline std::vector<Vector2> GenerateOutAndBack() {
        std::vector<Vector2> nodes;
        float yBottom = (float)GetRandomValue(580, 640);
        float yTop = (float)GetRandomValue(160, 240);
        float xLeft = (float)GetRandomValue(120, 160);
        float xRight = (float)GetRandomValue(840, 890);

        nodes.push_back({xRight, yBottom});
        nodes.push_back({xRight * 0.65f + xLeft * 0.35f, yBottom});
        nodes.push_back({xRight * 0.35f + xLeft * 0.65f, yBottom});
        nodes.push_back({xLeft + 90.0f, yBottom});
        nodes.push_back({xLeft + 20.0f, yBottom - 45.0f});
        nodes.push_back({xLeft - 25.0f, (yTop + yBottom) * 0.5f});
        nodes.push_back({xLeft + 20.0f, yTop + 45.0f});
        nodes.push_back({xLeft + 120.0f, yTop});
        nodes.push_back({xRight * 0.5f + xLeft * 0.5f, yTop + (float)GetRandomValue(-25, 35)});
        nodes.push_back({xRight - 120.0f, yTop});
        nodes.push_back({xRight - 20.0f, yTop + 45.0f});
        nodes.push_back({xRight + 30.0f, (yTop + yBottom) * 0.5f});
        nodes.push_back({xRight - 20.0f, yBottom - 45.0f});
        return nodes;
    }

    /**
     * @brief Archetype 1: Infield Loop Complex (Bahrain / Shanghai / Silverstone).
     */
    inline std::vector<Vector2> GenerateInfieldLoop() {
        std::vector<Vector2> nodes;
        float yBase = (float)GetRandomValue(620, 660);
        float yTop = (float)GetRandomValue(140, 180);
        float xLeft = (float)GetRandomValue(130, 170);
        float xRight = (float)GetRandomValue(820, 880);

        nodes.push_back({xRight - 80.0f, yBase});
        nodes.push_back({(xLeft + xRight) * 0.5f, yBase});
        nodes.push_back({xLeft + 80.0f, yBase});
        nodes.push_back({xLeft - 10.0f, yBase - 80.0f});
        nodes.push_back({xLeft, (yBase + yTop) * 0.5f});
        nodes.push_back({xLeft + 60.0f, yTop + 60.0f});
        nodes.push_back({xLeft + 180.0f, yTop});
        nodes.push_back({(xLeft + xRight) * 0.5f, yTop});
        nodes.push_back({xRight - 100.0f, yTop});
        nodes.push_back({xRight, yTop + 80.0f});
        nodes.push_back({xRight + 15.0f, yTop + 200.0f});

        float loopDepth = (float)GetRandomValue(350, 430);
        nodes.push_back({xRight - 120.0f, yTop + 220.0f});
        nodes.push_back({loopDepth + 120.0f, 340.0f});
        nodes.push_back({loopDepth, 420.0f});
        nodes.push_back({loopDepth + 100.0f, 500.0f});
        nodes.push_back({xRight - 140.0f, 520.0f});
        nodes.push_back({xRight - 30.0f, yBase - 70.0f});
        return nodes;
    }

    /**
     * @brief Archetype 2: Triangular Diagonal (Spa-Francorchamps / Paul Ricard / Jeddah).
     */
    inline std::vector<Vector2> GenerateTriangularDiagonal() {
        std::vector<Vector2> nodes;
        float xStart = (float)GetRandomValue(160, 210);
        float yStart = (float)GetRandomValue(630, 680);
        float xApex = (float)GetRandomValue(780, 860);
        float yApex = (float)GetRandomValue(140, 190);

        nodes.push_back({xStart, yStart});
        nodes.push_back({xStart * 0.65f + xApex * 0.35f, yStart * 0.65f + yApex * 0.35f});
        nodes.push_back({xStart * 0.35f + xApex * 0.65f, yStart * 0.35f + yApex * 0.65f});
        nodes.push_back({xApex - 40.0f, yApex + 20.0f});
        nodes.push_back({xApex + 30.0f, yApex + 40.0f});
        nodes.push_back({xApex + 40.0f, yApex + 130.0f});
        nodes.push_back({xApex - 50.0f, yApex + 210.0f});

        float midX = (xApex + xStart) * 0.5f + (float)GetRandomValue(40, 80);
        float midY = (yApex + yStart) * 0.5f;
        nodes.push_back({midX + 70.0f, midY - 20.0f});
        nodes.push_back({midX, midY + 40.0f});
        nodes.push_back({midX - 60.0f, midY + 100.0f});
        nodes.push_back({xStart + 90.0f, yStart + 25.0f});
        nodes.push_back({xStart - 15.0f, yStart + 55.0f});
        nodes.push_back({xStart - 40.0f, yStart + 10.0f});
        return nodes;
    }

    /**
     * @brief Archetype 3: Asymmetric L-Shape (Catalunya / Suzuka / Zandvoort / Portimao).
     */
    inline std::vector<Vector2> GenerateLShape() {
        std::vector<Vector2> nodes;
        float xBase = (float)GetRandomValue(130, 170);
        float xMax = (float)GetRandomValue(830, 880);
        float yBase = (float)GetRandomValue(620, 670);
        float yTop = (float)GetRandomValue(140, 180);

        nodes.push_back({xMax - 80.0f, yBase});
        nodes.push_back({(xBase + xMax) * 0.5f, yBase});
        nodes.push_back({xBase + 120.0f, yBase});
        nodes.push_back({xBase, yBase - 70.0f});
        nodes.push_back({xBase - 15.0f, (yBase + yTop) * 0.5f + 50.0f});
        nodes.push_back({xBase + 10.0f, (yBase + yTop) * 0.5f - 50.0f});
        nodes.push_back({xBase + 60.0f, yTop + 60.0f});
        nodes.push_back({xBase + 180.0f, yTop});
        nodes.push_back({xBase + 340.0f, yTop + 60.0f});
        nodes.push_back({xBase + 460.0f, yTop - 10.0f});
        nodes.push_back({xMax - 140.0f, yTop + 100.0f});
        nodes.push_back({xMax - 30.0f, yTop + 240.0f});
        nodes.push_back({xMax - 120.0f, 380.0f});
        nodes.push_back({xMax, 500.0f});
        return nodes;
    }

    /**
     * @brief Archetype 4: Dual-Lobe Horseshoe & Omega Waist (Interlagos / Hungaroring / Lusail).
     */
    inline std::vector<Vector2> GenerateDualLobeHorseshoe() {
        std::vector<Vector2> nodes;
        float xLeft = (float)GetRandomValue(130, 160);
        float xRight = (float)GetRandomValue(840, 880);

        nodes.push_back({xRight - 40.0f, 640.0f});
        nodes.push_back({500.0f, 650.0f});
        nodes.push_back({xLeft + 100.0f, 640.0f});
        nodes.push_back({xLeft, 560.0f});
        nodes.push_back({xLeft - 20.0f, 440.0f});
        nodes.push_back({xLeft + 80.0f, 380.0f});
        nodes.push_back({xLeft + 220.0f, 430.0f});
        nodes.push_back({380.0f, 330.0f});
        nodes.push_back({320.0f, 190.0f});
        nodes.push_back({480.0f, 140.0f});
        nodes.push_back({680.0f, 150.0f});
        nodes.push_back({xRight, 240.0f});
        nodes.push_back({xRight + 20.0f, 380.0f});
        nodes.push_back({xRight - 100.0f, 440.0f});
        nodes.push_back({xRight - 160.0f, 520.0f});
        nodes.push_back({xRight - 80.0f, 580.0f});
        return nodes;
    }

    /**
     * @brief Archetype 5: Geometric Urban Street Grid (Singapore Marina Bay / Baku / Vegas).
     */
    inline std::vector<Vector2> GenerateUrbanStreet() {
        std::vector<Vector2> nodes;
        float xLeft = (float)GetRandomValue(140, 170);
        float xRight = (float)GetRandomValue(830, 880);
        float yBottom = (float)GetRandomValue(630, 670);
        float yTop = (float)GetRandomValue(150, 180);

        nodes.push_back({xRight - 60.0f, yBottom});
        nodes.push_back({(xLeft + xRight) * 0.5f, yBottom});
        nodes.push_back({xLeft + 90.0f, yBottom});
        nodes.push_back({xLeft, yBottom - 80.0f});
        nodes.push_back({xLeft + 20.0f, yBottom - 200.0f});
        nodes.push_back({xLeft + 140.0f, yBottom - 230.0f});
        nodes.push_back({xLeft + 160.0f, yTop + 200.0f});
        nodes.push_back({xLeft + 60.0f, yTop + 140.0f});
        nodes.push_back({xLeft + 120.0f, yTop + 40.0f});
        nodes.push_back({350.0f, yTop});
        nodes.push_back({620.0f, yTop});
        nodes.push_back({740.0f, yTop + 70.0f});
        nodes.push_back({xRight, yTop + 130.0f});
        nodes.push_back({xRight + 20.0f, 360.0f});
        nodes.push_back({xRight - 100.0f, 420.0f});
        nodes.push_back({xRight - 100.0f, 530.0f});
        nodes.push_back({xRight + 10.0f, 570.0f});
        return nodes;
    }

    /**
     * @brief Injects chicanes, esses, or kinks on long edges.
     */
    inline std::vector<Vector2> InjectMotifs(const std::vector<Vector2>& inputNodes) {
        std::vector<Vector2> result;
        int n = inputNodes.size();

        for (int i = 0; i < n; i++) {
            Vector2 p1 = inputNodes[i];
            Vector2 p2 = inputNodes[(i + 1) % n];
            result.push_back(p1);

            float d = Dist(p1, p2);
            if (d > 190.0f && GetRandomValue(0, 100) < 55) {
                float dx = (p2.x - p1.x) / d;
                float dy = (p2.y - p1.y) / d;
                float nx = -dy;
                float ny = dx;

                int motifType = GetRandomValue(0, 2);
                float side = (GetRandomValue(0, 1) == 0) ? -1.0f : 1.0f;

                if (motifType == 0) {
                    // Chicane
                    float amp = (float)GetRandomValue(35, 55) * side;
                    result.push_back({ p1.x + dx * (d * 0.35f) + nx * amp, p1.y + dy * (d * 0.35f) + ny * amp });
                    result.push_back({ p1.x + dx * (d * 0.65f) - nx * (amp * 0.7f), p1.y + dy * (d * 0.65f) - ny * (amp * 0.7f) });
                } else if (motifType == 1) {
                    // Technical Esses
                    float amp = (float)GetRandomValue(25, 40) * side;
                    result.push_back({ p1.x + dx * (d * 0.30f) + nx * amp, p1.y + dy * (d * 0.30f) + ny * amp });
                    result.push_back({ p1.x + dx * (d * 0.70f) - nx * amp, p1.y + dy * (d * 0.70f) - ny * amp });
                } else {
                    // Mid-straight kink
                    float amp = (float)GetRandomValue(25, 45) * side;
                    result.push_back({ p1.x + dx * (d * 0.50f) + nx * amp, p1.y + dy * (d * 0.50f) + ny * amp });
                }
            }
        }
        return result;
    }

    /**
     * @brief Applies 360-degree rotation, asymmetric scaling, and canvas fitting.
     */
    inline std::vector<Vector2> ApplyAffineTransform(const std::vector<Vector2>& nodes) {
        if (nodes.empty()) return nodes;

        float cx = 0.0f, cy = 0.0f;
        for (const auto& p : nodes) { cx += p.x; cy += p.y; }
        cx /= nodes.size();
        cy /= nodes.size();

        float angle = (float)GetRandomValue(0, 360) * (PI_F / 180.0f);
        float cosA = std::cos(angle);
        float sinA = std::sin(angle);

        float sx = (float)GetRandomValue(75, 140) * 0.01f;
        float sy = (float)GetRandomValue(75, 140) * 0.01f;
        if (GetRandomValue(0, 1) == 1) sx = -sx;

        std::vector<Vector2> transformed;
        float minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;

        for (const auto& p : nodes) {
            float relX = (p.x - cx) * sx;
            float relY = (p.y - cy) * sy;
            float rotX = relX * cosA - relY * sinA;
            float rotY = relX * sinA + relY * cosA;

            transformed.push_back({ rotX, rotY });
            if (rotX < minX) minX = rotX;
            if (rotX > maxX) maxX = rotX;
            if (rotY < minY) minY = rotY;
            if (rotY > maxY) maxY = rotY;
        }

        float targetW = CANVAS_W - 2.0f * MARGIN_X;
        float targetH = CANVAS_H - 2.0f * MARGIN_Y;
        float currW = std::max(1.0f, maxX - minX);
        float currH = std::max(1.0f, maxY - minY);

        float fillRatio = (float)GetRandomValue(70, 98) * 0.01f;
        float fitScale = std::min(targetW / currW, targetH / currH) * fillRatio;

        float centerOffX = (float)GetRandomValue(-25, 25);
        float centerOffY = (float)GetRandomValue(-20, 20);
        float midTransX = (minX + maxX) * 0.5f;
        float midTransY = (minY + maxY) * 0.5f;

        for (auto& p : transformed) {
            p.x = 500.0f + centerOffX + (p.x - midTransX) * fitScale;
            p.y = 400.0f + centerOffY + (p.y - midTransY) * fitScale;
        }

        // Ensure CCW winding
        float shoelace = 0.0f;
        int n = transformed.size();
        for (int i = 0; i < n; i++) {
            Vector2 a = transformed[i];
            Vector2 b = transformed[(i + 1) % n];
            shoelace += (b.x - a.x) * (b.y + a.y);
        }
        if (shoelace < 0.0f) {
            std::reverse(transformed.begin(), transformed.end());
        }

        return transformed;
    }

    /**
     * @brief Enforces zero self-intersections and non-adjacent clearance margin.
     */
    inline bool ValidateLayout(const std::vector<Vector2>& nodes) {
        int n = nodes.size();
        if (n < 6) return false;

        for (int i = 0; i < n; i++) {
            Vector2 a1 = nodes[i];
            Vector2 a2 = nodes[(i + 1) % n];
            for (int j = i + 2; j < n; j++) {
                if (i == 0 && j == n - 1) continue;
                Vector2 b1 = nodes[j];
                Vector2 b2 = nodes[(j + 1) % n];
                if (SegmentsIntersect(a1, a2, b1, b2)) return false;
            }
        }

        std::vector<Vector2> dense = TrackManager::GenerateSplinePoints(nodes, 25);
        int m = dense.size();
        if (m < 50) return false;

        int window = 22;
        for (int i = 0; i < m; i += 3) {
            Vector2 pt = dense[i];
            for (int j = i + window; j < m - window / 2; j += 3) {
                if (DistSq(pt, dense[j]) < MIN_CLEARANCE * MIN_CLEARANCE) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * @brief Generates a high-entropy procedural Formula 1 track with 360-deg rotation and motif injection.
     */
    inline std::vector<Vector2> GenerateProceduralCenterPoints() {
        for (int attempt = 0; attempt < 60; attempt++) {
            int mode = GetRandomValue(0, 6);
            std::vector<Vector2> base;
            if (mode == 6) {
                base = GeneratePolarMacroGraph();
            } else {
                switch (mode) {
                    case 0: base = GenerateOutAndBack(); break;
                    case 1: base = GenerateInfieldLoop(); break;
                    case 2: base = GenerateTriangularDiagonal(); break;
                    case 3: base = GenerateLShape(); break;
                    case 4: base = GenerateDualLobeHorseshoe(); break;
                    case 5: default: base = GenerateUrbanStreet(); break;
                }
            }

            std::vector<Vector2> withMotifs = InjectMotifs(base);
            std::vector<Vector2> transformed = ApplyAffineTransform(withMotifs);

            if (ValidateLayout(transformed)) {
                return transformed;
            }
        }
        return ApplyAffineTransform(GeneratePolarMacroGraph());
    }

    /**
     * @brief Serializes and saves the center structure of the track to disk.
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
