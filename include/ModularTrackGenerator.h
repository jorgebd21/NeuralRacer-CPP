#pragma once
#include <vector>
#include <cmath>
#include <string>
#include <cstdlib>
#include <algorithm>
#include <fstream>
#include <iostream>
#include "raylib.h"
#include "TrackManager.h"

/**
 * @brief High-Performance Modular Track Synthesis Engine (SE(2) Tile Coupling).
 * 
 * Synthesizes Grand Prix racing circuits from an expanded discrete catalog of 12 module families
 * using lean curvature control vertices to optimize memory cache occupancy, spatial hashing,
 * and 60 FPS real-time rendering.
 */
namespace ModularTrackGenerator {
    constexpr float PI_F = 3.14159265358979323846f;
    constexpr float MIN_CLEARANCE = 38.0f;
    constexpr float MIN_RADIUS = 68.0f; // Enforces minimum post-scale radius compatible with physical adhesion limits

    struct Port {
        Vector2 pos;
        float heading;
    };

    struct ModuleDef {
        std::string id;
        float deltaHeading;
        Vector2 localExitPos;
        std::vector<Vector2> localPoints;
    };

    inline float DistSq(Vector2 a, Vector2 b) {
        return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
    }
    inline float Dist(Vector2 a, Vector2 b) {
        return std::sqrt(DistSq(a, b));
    }

    inline float NormalizeAngle(float a) {
        while (a > PI_F) a -= 2.0f * PI_F;
        while (a < -PI_F) a += 2.0f * PI_F;
        return a;
    }

    inline ModuleDef MakeStraight(const std::string& id, float length) {
        ModuleDef m;
        m.id = id;
        m.deltaHeading = 0.0f;
        m.localExitPos = { length, 0.0f };
        if (length > 160.0f) {
            m.localPoints.push_back({ length * 0.333f, 0.0f });
            m.localPoints.push_back({ length * 0.666f, 0.0f });
        } else if (length > 80.0f) {
            m.localPoints.push_back({ length * 0.5f, 0.0f });
        }
        m.localPoints.push_back({ length, 0.0f });
        return m;
    }

    inline ModuleDef MakeArc(const std::string& id, float radius, float angleRad, bool turnLeft) {
        ModuleDef m;
        m.id = id;
        m.deltaHeading = turnLeft ? angleRad : -angleRad;

        int numSteps = std::max(1, (int)std::round(radius * angleRad / 55.0f));
        float sign = turnLeft ? 1.0f : -1.0f;

        for (int i = 1; i <= numSteps; i++) {
            float t = (float)i / numSteps;
            float a = t * angleRad;
            float px = radius * std::sin(a);
            float py = sign * radius * (1.0f - std::cos(a));
            m.localPoints.push_back({ px, py });
        }
        m.localExitPos = m.localPoints.back();
        return m;
    }

    inline ModuleDef MakeDoubleApex(const std::string& id, float r1, float r2, bool turnLeft) {
        ModuleDef m;
        m.id = id;
        float a1 = PI_F * 0.25f;
        float a2 = PI_F * 0.50f;
        m.deltaHeading = turnLeft ? (a1 + a2) : -(a1 + a2);
        float sign = turnLeft ? 1.0f : -1.0f;

        m.localPoints.push_back({ r1 * std::sin(a1 * 0.5f), sign * r1 * (1.0f - std::cos(a1 * 0.5f)) });
        Vector2 p1 = { r1 * std::sin(a1), sign * r1 * (1.0f - std::cos(a1)) };
        m.localPoints.push_back(p1);

        Vector2 c2 = { p1.x - r2 * std::sin(a1), p1.y + sign * r2 * std::cos(a1) };
        float midA = a1 + a2 * 0.5f;
        m.localPoints.push_back({ c2.x + r2 * std::sin(midA), c2.y - sign * r2 * std::cos(midA) });
        float endA = a1 + a2;
        m.localPoints.push_back({ c2.x + r2 * std::sin(endA), c2.y - sign * r2 * std::cos(endA) });
        m.localExitPos = m.localPoints.back();
        return m;
    }

    inline ModuleDef MakeChicane(const std::string& id, float length, float lateralOffset) {
        ModuleDef m;
        m.id = id;
        m.deltaHeading = 0.0f;
        m.localExitPos = { length, lateralOffset };
        m.localPoints.push_back({ length * 0.333f, lateralOffset * 0.40f });
        m.localPoints.push_back({ length * 0.666f, lateralOffset * 0.90f });
        m.localPoints.push_back({ length, lateralOffset });
        return m;
    }

    inline ModuleDef MakeSnakeEsses(const std::string& id, float length, float amplitude) {
        ModuleDef m;
        m.id = id;
        m.deltaHeading = 0.0f;
        m.localExitPos = { length, 0.0f };
        m.localPoints.push_back({ length * 0.25f, amplitude });
        m.localPoints.push_back({ length * 0.50f, 0.0f });
        m.localPoints.push_back({ length * 0.75f, -amplitude });
        m.localPoints.push_back({ length, 0.0f });
        return m;
    }

    inline std::vector<ModuleDef> BuildCatalog() {
        std::vector<ModuleDef> cat;
        cat.push_back(MakeStraight("STR_50", 50.0f));
        cat.push_back(MakeStraight("STR_90", 90.0f));
        cat.push_back(MakeStraight("STR_150", 150.0f));
        cat.push_back(MakeStraight("STR_240", 240.0f));

        cat.push_back(MakeArc("CRV_30_R260_L", 260.0f, PI_F / 6.0f, true));
        cat.push_back(MakeArc("CRV_30_R260_R", 260.0f, PI_F / 6.0f, false));

        cat.push_back(MakeArc("CRV_45_R160_L", 160.0f, PI_F / 4.0f, true));
        cat.push_back(MakeArc("CRV_45_R160_R", 160.0f, PI_F / 4.0f, false));

        cat.push_back(MakeArc("CRV_60_R150_L", 150.0f, PI_F / 3.0f, true));
        cat.push_back(MakeArc("CRV_60_R150_R", 150.0f, PI_F / 3.0f, false));

        cat.push_back(MakeArc("CRV_75_R140_L", 140.0f, PI_F * 0.4167f, true));
        cat.push_back(MakeArc("CRV_75_R140_R", 140.0f, PI_F * 0.4167f, false));

        cat.push_back(MakeArc("CRV_90_R130_L", 130.0f, PI_F / 2.0f, true));
        cat.push_back(MakeArc("CRV_90_R130_R", 130.0f, PI_F / 2.0f, false));

        cat.push_back(MakeArc("CRV_120_R120_L", 120.0f, PI_F * 0.6667f, true));
        cat.push_back(MakeArc("CRV_120_R120_R", 120.0f, PI_F * 0.6667f, false));

        cat.push_back(MakeDoubleApex("CRV_DBL_APEX_L", 160.0f, 110.0f, true));
        cat.push_back(MakeDoubleApex("CRV_DBL_APEX_R", 160.0f, 110.0f, false));

        cat.push_back(MakeArc("HRP_180_R120_L", 120.0f, PI_F, true));
        cat.push_back(MakeArc("HRP_180_R120_R", 120.0f, PI_F, false));

        cat.push_back(MakeChicane("CHK_S_L40", 130.0f, 40.0f));
        cat.push_back(MakeChicane("CHK_S_R40", 130.0f, -40.0f));
        cat.push_back(MakeChicane("CHK_BUS_STOP_L", 170.0f, 60.0f));
        cat.push_back(MakeChicane("CHK_BUS_STOP_R", 170.0f, -60.0f));

        cat.push_back(MakeSnakeEsses("ESS_SNAKE_L", 210.0f, 35.0f));
        cat.push_back(MakeSnakeEsses("ESS_SNAKE_R", 210.0f, -35.0f));

        return cat;
    }

    inline void AttachModule(const ModuleDef& def, const Port& inPort, Port& outPort, std::vector<Vector2>& outWorldPoints) {
        float cosH = std::cos(inPort.heading);
        float sinH = std::sin(inPort.heading);
        outWorldPoints.clear();
        for (const auto& lp : def.localPoints) {
            float wx = inPort.pos.x + lp.x * cosH - lp.y * sinH;
            float wy = inPort.pos.y + lp.x * sinH + lp.y * cosH;
            outWorldPoints.push_back({ wx, wy });
        }
        float ex = inPort.pos.x + def.localExitPos.x * cosH - def.localExitPos.y * sinH;
        float ey = inPort.pos.y + def.localExitPos.x * sinH + def.localExitPos.y * cosH;
        outPort.pos = { ex, ey };
        outPort.heading = NormalizeAngle(inPort.heading + def.deltaHeading);
    }

    inline bool TryHermiteBridge(const Port& curPort, const Port& startPort, std::vector<Vector2>& outBridgePoints) {
        float d = Dist(curPort.pos, startPort.pos);
        if (d < 120.0f || d > 450.0f) return false;

        float toStartX = startPort.pos.x - curPort.pos.x;
        float toStartY = startPort.pos.y - curPort.pos.y;
        float dirAngle = std::atan2(toStartY, toStartX);
        
        // Strict angular threshold: max 28.8 degrees deviation
        float angleDiffCur = std::abs(NormalizeAngle(dirAngle - curPort.heading));
        if (angleDiffCur > PI_F * 0.16f) return false;

        float angleDiffStart = std::abs(NormalizeAngle(startPort.heading - dirAngle));
        if (angleDiffStart > PI_F * 0.16f) return false;

        float tangentLen = d * 0.45f;
        Vector2 m0 = { std::cos(curPort.heading) * tangentLen, std::sin(curPort.heading) * tangentLen };
        Vector2 m1 = { std::cos(startPort.heading) * tangentLen, std::sin(startPort.heading) * tangentLen };

        outBridgePoints.clear();
        int steps = std::clamp((int)std::round(d / 55.0f), 2, 6);
        for (int i = 1; i <= steps; i++) {
            float t = (float)i / (steps + 1);
            float t2 = t * t;
            float t3 = t2 * t;
            float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
            float h10 = t3 - 2.0f * t2 + t;
            float h01 = -2.0f * t3 + 3.0f * t2;
            float h11 = t3 - t2;
            outBridgePoints.push_back({
                h00 * curPort.pos.x + h10 * m0.x + h01 * startPort.pos.x + h11 * m1.x,
                h00 * curPort.pos.y + h10 * m0.y + h01 * startPort.pos.y + h11 * m1.y
            });
        }
        return true;
    }

    inline bool CheckClearance(const std::vector<Vector2>& newPts, const std::vector<Vector2>& allPts) {
        if (allPts.size() < 6) return true;
        int safeLimit = std::max(0, (int)allPts.size() - 4);
        for (const auto& np : newPts) {
            for (int i = 0; i < safeLimit; i++) {
                if (DistSq(np, allPts[i]) < MIN_CLEARANCE * MIN_CLEARANCE) {
                    return false;
                }
            }
        }
        return true;
    }

    inline bool SegmentsStrictlyIntersect(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4) {
        float den = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
        if (std::abs(den) < 1e-5f) return false;
        float t = ((p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)) / den;
        float u = -((p1.x - p2.x) * (p1.y - p3.y) - (p1.y - p2.y) * (p1.x - p3.x)) / den;
        return (t > 0.05f && t < 0.95f && u > 0.05f && u < 0.95f);
    }

    /**
     * @brief Comprehensive pre-flight geometric quality watchdog for candidate tracks.
     */
    inline bool ValidateTrackCandidate(const std::vector<Vector2>& pts) {
        if (pts.size() < 12) return false;
        std::vector<Vector2> dense = TrackManager::GenerateSplinePoints(pts, 16);
        int n = dense.size();
        if (n < 60) return false;

        // 1. Curvature radius check (3-point circumradius with numerical step = 3)
        int step = 3;
        for (int i = 0; i < n; i += step) {
            Vector2 p0 = dense[(i - step + n) % n];
            Vector2 p1 = dense[i];
            Vector2 p2 = dense[(i + step) % n];
            float a = std::hypot(p1.x - p0.x, p1.y - p0.y);
            float b = std::hypot(p2.x - p1.x, p2.y - p1.y);
            float c = std::hypot(p2.x - p0.x, p2.y - p0.y);
            float area = 0.5f * std::abs((p1.x - p0.x)*(p2.y - p0.y) - (p1.y - p0.y)*(p2.x - p0.x));
            if (area > 1e-4f && a > 1e-4f && b > 1e-4f && c > 1e-4f) {
                float radius = (a * b * c) / (4.0f * area);
                if (radius < MIN_RADIUS) return false;
            }
        }

        // 2. Minimum non-adjacent clearance along periodic loop
        for (int i = 0; i < n; i += 3) {
            for (int j = i + 1; j < n; j += 3) {
                int loopDist = std::min(j - i, n - (j - i));
                if (loopDist < 26) continue;
                if (DistSq(dense[i], dense[j]) < 40.0f * 40.0f) return false;
            }
        }

        // 3. Wall self-intersection check
        std::vector<std::pair<Vector2, Vector2>> walls;
        std::vector<Vector2> checkpoints;
        Vector2 startPos;
        float startRot;
        TrackManager::CalculateStartGrid(pts, startPos, startRot);
        TrackManager::GenerateBordersFromCenterLine(dense, 35.0f, walls, checkpoints, startPos);

        int wallCount = walls.size();
        for (int i = 0; i < wallCount; i++) {
            Vector2 a1 = walls[i].first;
            Vector2 a2 = walls[i].second;
            for (int j = i + 2; j < wallCount; j++) {
                if (i == 0 && j == wallCount - 1) continue;
                Vector2 b1 = walls[j].first;
                Vector2 b2 = walls[j].second;
                if (SegmentsStrictlyIntersect(a1, a2, b1, b2)) return false;
            }
        }

        return true;
    }

    inline std::vector<Vector2> SeedWaypoints(int topology, float& outStartAngle) {
        std::vector<Vector2> rawWp;
        float rot = (float)GetRandomValue(0, 360) * (PI_F / 180.0f);
        float cosR = std::cos(rot);
        float sinR = std::sin(rot);

        switch (topology) {
            case 0: {
                float r1 = (float)GetRandomValue(300, 360);
                float r2 = (float)GetRandomValue(280, 340);
                float r3 = (float)GetRandomValue(280, 340);
                rawWp.push_back({ r1, 0.0f });
                rawWp.push_back({ -r2 * 0.5f, r2 * 0.866f });
                rawWp.push_back({ -r3 * 0.5f, -r3 * 0.866f });
                break;
            }
            case 1: {
                for (int i = 0; i < 5; i++) {
                    float a = i * (2.0f * PI_F / 5.0f);
                    float rad = (i == 2) ? (float)GetRandomValue(120, 180) : (float)GetRandomValue(280, 350);
                    rawWp.push_back({ rad * std::cos(a), rad * std::sin(a) });
                }
                break;
            }
            case 2: {
                float w = (float)GetRandomValue(280, 350);
                float h = (float)GetRandomValue(220, 280);
                rawWp.push_back({ w, -h });
                rawWp.push_back({ w * 0.2f, 0.0f });
                rawWp.push_back({ -w, -h });
                rawWp.push_back({ -w, h });
                rawWp.push_back({ w * 0.5f, h });
                break;
            }
            case 3: default: {
                float hw = (float)GetRandomValue(260, 360);
                float hh = (float)GetRandomValue(180, 260);
                rawWp.push_back({ hw, hh });
                rawWp.push_back({ -hw, hh });
                rawWp.push_back({ -hw, -hh });
                rawWp.push_back({ hw, -hh });
                break;
            }
        }

        std::vector<Vector2> worldWp;
        for (const auto& p : rawWp) {
            worldWp.push_back({ 500.0f + p.x * cosR - p.y * sinR, 400.0f + p.x * sinR + p.y * cosR });
        }

        float shoelace = 0.0f;
        int n = worldWp.size();
        for (int i = 0; i < n; i++) {
            Vector2 a = worldWp[i];
            Vector2 b = worldWp[(i + 1) % n];
            shoelace += (b.x - a.x) * (b.y + a.y);
        }
        if (shoelace < 0.0f) std::reverse(worldWp.begin(), worldWp.end());

        int startEdge = GetRandomValue(0, n - 1);
        Vector2 pA = worldWp[startEdge];
        Vector2 pB = worldWp[(startEdge + 1) % n];
        outStartAngle = std::atan2(pB.y - pA.y, pB.x - pA.x);

        std::vector<Vector2> reordered;
        for (int i = 1; i <= n; i++) reordered.push_back(worldWp[(startEdge + i) % n]);
        return reordered;
    }

    inline std::vector<Vector2> GenerateFallbackLoop() {
        std::vector<Vector2> loop;
        int n = 16;
        for (int i = 0; i < n; i++) {
            float a = (float)i * (2.0f * PI_F / (float)n);
            float rx = 340.0f + 35.0f * std::cos(2.0f * a);
            float ry = 230.0f + 25.0f * std::sin(2.0f * a);
            loop.push_back({ 500.0f + rx * std::cos(a), 400.0f + ry * std::sin(a) });
        }
        return loop;
    }

    /**
     * @brief Synthesizes a high-variety modular racing circuit with 360-degree start pose and multi-topology.
     */
    inline std::vector<Vector2> GenerateProceduralCenterPoints() {
        auto catalog = BuildCatalog();

        for (int run = 0; run < 400; run++) {
            int topology = GetRandomValue(0, 3);
            float startHeading = 0.0f;
            std::vector<Vector2> waypoints = SeedWaypoints(topology, startHeading);
            int numWP = (int)waypoints.size();

            Vector2 lastWP = waypoints.back();
            Vector2 firstWP = waypoints.front();
            Port startPort = { { (lastWP.x + firstWP.x) * 0.5f, (lastWP.y + firstWP.y) * 0.5f }, startHeading };
            Port curPort = startPort;

            std::vector<Vector2> trackPoints;
            trackPoints.push_back(startPort.pos);

            int wpIdx = 0;
            bool closed = false;

            for (int step = 0; step < 28; step++) {
                Vector2 targetWP = (wpIdx < numWP) ? waypoints[wpIdx] : startPort.pos;
                float distToTarget = Dist(curPort.pos, targetWP);

                if (distToTarget < 150.0f && wpIdx < numWP) {
                    wpIdx++;
                    if (wpIdx < numWP) targetWP = waypoints[wpIdx];
                }

                if (wpIdx >= numWP) {
                    std::vector<Vector2> bridge;
                    if (TryHermiteBridge(curPort, startPort, bridge)) {
                        bool bridgeClear = true;
                        int headIgnore = 4;
                        int tailIgnore = 4;
                        int limit = (int)trackPoints.size() - tailIgnore;
                        for (const auto& bp : bridge) {
                            for (int i = headIgnore; i < limit; i++) {
                                if (DistSq(bp, trackPoints[i]) < MIN_CLEARANCE * MIN_CLEARANCE) {
                                    bridgeClear = false;
                                    break;
                                }
                            }
                            if (!bridgeClear) break;
                        }

                        if (bridgeClear) {
                            for (const auto& bp : bridge) trackPoints.push_back(bp);
                            closed = true;
                            break;
                        }
                    }
                }

                struct ScoredMod {
                    int catalogIdx;
                    float score;
                };
                std::vector<ScoredMod> scored;

                for (size_t i = 0; i < catalog.size(); i++) {
                    const auto& mod = catalog[i];
                    Port testExit;
                    std::vector<Vector2> testPts;
                    AttachModule(mod, curPort, testExit, testPts);

                    if (!CheckClearance(testPts, trackPoints)) continue;

                    float desiredAngle = std::atan2(targetWP.y - testExit.pos.y, targetWP.x - testExit.pos.x);
                    float headingDiff = std::abs(NormalizeAngle(desiredAngle - testExit.heading));
                    float distScore = Dist(testExit.pos, targetWP);
                    // Balanced heading penalty (reduced from 115.0f to 45.0f) with diversity incentive for medium/pronounced curves
                    float headingPenalty = headingDiff * 45.0f;
                    float curveBonus = (std::abs(mod.deltaHeading) >= (PI_F / 4.0f)) ? -35.0f : 0.0f;
                    float totalScore = distScore + headingPenalty + curveBonus + (float)GetRandomValue(0, 20);

                    scored.push_back({ (int)i, totalScore });
                }

                if (scored.empty()) break;

                std::sort(scored.begin(), scored.end(), [](const ScoredMod& a, const ScoredMod& b) {
                    return a.score < b.score;
                });

                int pickIdx = 0;
                if (scored.size() > 1 && GetRandomValue(0, 100) < 35) pickIdx = 1;

                const auto& bestMod = catalog[scored[pickIdx].catalogIdx];
                Port nextPort;
                std::vector<Vector2> newPts;
                AttachModule(bestMod, curPort, nextPort, newPts);

                for (const auto& np : newPts) trackPoints.push_back(np);
                curPort = nextPort;
            }

            if (closed && trackPoints.size() >= 12) {
                float minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
                for (const auto& p : trackPoints) {
                    if (p.x < minX) minX = p.x;
                    if (p.x > maxX) maxX = p.x;
                    if (p.y < minY) minY = p.y;
                    if (p.y > maxY) maxY = p.y;
                }

                float tw = maxX - minX;
                float th = maxY - minY;
                float scale = std::min(820.0f / std::max(1.0f, tw), 640.0f / std::max(1.0f, th)) * 0.90f;

                float cx = (minX + maxX) * 0.5f;
                float cy = (minY + maxY) * 0.5f;

                std::vector<Vector2> fitted;
                for (const auto& p : trackPoints) {
                    fitted.push_back({ 500.0f + (p.x - cx) * scale, 400.0f + (p.y - cy) * scale });
                }

                float shoelace = 0.0f;
                int n = fitted.size();
                for (int i = 0; i < n; i++) {
                    Vector2 a = fitted[i];
                    Vector2 b = fitted[(i + 1) % n];
                    shoelace += (b.x - a.x) * (b.y + a.y);
                }
                if (shoelace < 0.0f) {
                    std::reverse(fitted.begin(), fitted.end());
                }

                // Enforce zero-defect pre-flight validation
                if (ValidateTrackCandidate(fitted)) {
                    return fitted;
                }
            }
        }
        return GenerateFallbackLoop();
    }

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
