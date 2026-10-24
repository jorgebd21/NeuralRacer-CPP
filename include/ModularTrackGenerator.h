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
 * @brief Professional Modular Track Synthesis Engine (SE(2) Tile Coupling).
 * 
 * Assembles Grand Prix racing circuits from an expanded discrete catalog of 12 module families:
 * - Micro & Super Straights (STR_50, STR_90, STR_150, STR_240)
 * - Ultra High-Speed Kinks 30° (CRV_30_R260_L/R)
 * - High-Downforce Arcs 45° (CRV_45_R160_L/R)
 * - Open V-Corners 60° (CRV_60_R150_L/R)
 * - Medium Apex Corners 90° (CRV_90_R110_L/R)
 * - Progressive Double-Apex Carousels 135° (CRV_DBL_APEX_L/R)
 * - Heavy Braking Hairpins 180° (HRP_180_R60_L/R)
 * - Fluid S-Chicanes (CHK_S_L40/R40)
 * - Violent Bus-Stop Chicanes (CHK_BUS_STOP_L/R)
 * - Technical Snake Esses (ESS_SNAKE_L/R)
 * 
 * Features Multi-Topology Waypoint Guidance (Triangular, Pentagonal Infield, Dog-Leg, and Rotated Quad)
 * with unconstrained 360-degree start pose rotation φ ∈ [0, 2π) and C1 Hermite loop closure.
 */
namespace ModularTrackGenerator {
    constexpr float PI_F = 3.14159265358979323846f;
    constexpr float MIN_CLEARANCE = 38.0f;

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
        int steps = std::max(2, (int)(length / 25.0f));
        for (int i = 0; i <= steps; i++) {
            float t = (float)i / steps;
            m.localPoints.push_back({ t * length, 0.0f });
        }
        return m;
    }

    inline ModuleDef MakeArc(const std::string& id, float radius, float angleRad, bool turnLeft) {
        ModuleDef m;
        m.id = id;
        m.deltaHeading = turnLeft ? angleRad : -angleRad;
        int steps = std::max(3, (int)(radius * angleRad / 20.0f));

        if (turnLeft) {
            m.localExitPos = { radius * std::sin(angleRad), radius * (1.0f - std::cos(angleRad)) };
            for (int i = 0; i <= steps; i++) {
                float a = -PI_F * 0.5f + (float)i / steps * angleRad;
                m.localPoints.push_back({ radius * std::cos(a), radius + radius * std::sin(a) });
            }
        } else {
            m.localExitPos = { radius * std::sin(angleRad), -radius * (1.0f - std::cos(angleRad)) };
            for (int i = 0; i <= steps; i++) {
                float a = PI_F * 0.5f - (float)i / steps * angleRad;
                m.localPoints.push_back({ radius * std::cos(a), -radius + radius * std::sin(a) });
            }
        }
        return m;
    }

    inline ModuleDef MakeDoubleApex(const std::string& id, float r1, float r2, bool turnLeft) {
        ModuleDef m;
        m.id = id;
        float a1 = PI_F * 0.25f;
        float a2 = PI_F * 0.50f;
        m.deltaHeading = turnLeft ? (a1 + a2) : -(a1 + a2);

        std::vector<Vector2> pts;
        int steps1 = 4;
        for (int i = 0; i <= steps1; i++) {
            float t = (float)i / steps1;
            float a = -PI_F * 0.5f + t * a1;
            pts.push_back({ r1 * std::cos(a), r1 + r1 * std::sin(a) });
        }
        Vector2 p1 = pts.back();
        Vector2 c2 = { p1.x - r2 * std::sin(a1), p1.y + r2 * std::cos(a1) };
        int steps2 = 6;
        for (int i = 1; i <= steps2; i++) {
            float t = (float)i / steps2;
            float a = a1 + t * a2;
            pts.push_back({ c2.x + r2 * std::sin(a), c2.y - r2 * std::cos(a) });
        }

        if (!turnLeft) {
            for (auto& pt : pts) pt.y = -pt.y;
        }

        m.localPoints = pts;
        m.localExitPos = pts.back();
        return m;
    }

    inline ModuleDef MakeChicane(const std::string& id, float length, float lateralOffset) {
        ModuleDef m;
        m.id = id;
        m.deltaHeading = 0.0f;
        m.localExitPos = { length, lateralOffset };
        int steps = 8;
        for (int i = 0; i <= steps; i++) {
            float t = (float)i / steps;
            float x = t * length;
            float y = (lateralOffset * 0.5f) * (1.0f - std::cos(PI_F * t));
            m.localPoints.push_back({ x, y });
        }
        return m;
    }

    inline ModuleDef MakeSnakeEsses(const std::string& id, float length, float amplitude) {
        ModuleDef m;
        m.id = id;
        m.deltaHeading = 0.0f;
        m.localExitPos = { length, 0.0f };
        int steps = 12;
        float factor = amplitude / 0.75f;
        for (int i = 0; i <= steps; i++) {
            float t = (float)i / steps;
            float x = t * length;
            float env = 4.0f * t * (1.0f - t);
            float y = factor * std::sin(2.0f * PI_F * t) * env;
            m.localPoints.push_back({ x, y });
        }
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

        cat.push_back(MakeArc("CRV_90_R110_L", 110.0f, PI_F / 2.0f, true));
        cat.push_back(MakeArc("CRV_90_R110_R", 110.0f, PI_F / 2.0f, false));

        cat.push_back(MakeDoubleApex("CRV_DBL_APEX_L", 150.0f, 90.0f, true));
        cat.push_back(MakeDoubleApex("CRV_DBL_APEX_R", 150.0f, 90.0f, false));

        cat.push_back(MakeArc("HRP_180_R60_L", 60.0f, PI_F, true));
        cat.push_back(MakeArc("HRP_180_R60_R", 60.0f, PI_F, false));

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
        for (size_t i = 1; i < def.localPoints.size(); i++) {
            Vector2 lp = def.localPoints[i];
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
        if (d < 50.0f || d > 480.0f) return false;

        float toStartX = startPort.pos.x - curPort.pos.x;
        float toStartY = startPort.pos.y - curPort.pos.y;
        float dirAngle = std::atan2(toStartY, toStartX);
        float angleDiff = std::abs(NormalizeAngle(dirAngle - curPort.heading));
        if (angleDiff > PI_F * 0.72f) return false;

        float tangentLen = d * 0.5f;
        Vector2 m0 = { std::cos(curPort.heading) * tangentLen, std::sin(curPort.heading) * tangentLen };
        Vector2 m1 = { std::cos(startPort.heading) * tangentLen, std::sin(startPort.heading) * tangentLen };

        outBridgePoints.clear();
        int steps = std::max(4, (int)(d / 20.0f));
        for (int i = 1; i < steps; i++) {
            float t = (float)i / steps;
            float t2 = t * t;
            float t3 = t2 * t;

            float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
            float h10 = t3 - 2.0f * t2 + t;
            float h01 = -2.0f * t3 + 3.0f * t2;
            float h11 = t3 - t2;

            Vector2 pt = {
                h00 * curPort.pos.x + h10 * m0.x + h01 * startPort.pos.x + h11 * m1.x,
                h00 * curPort.pos.y + h10 * m0.y + h01 * startPort.pos.y + h11 * m1.y
            };
            outBridgePoints.push_back(pt);
        }
        return true;
    }

    inline bool CheckClearance(const std::vector<Vector2>& newPts, const std::vector<Vector2>& allPts) {
        if (allPts.size() < 10) return true;
        int ignoreCount = 8;
        int safeLimit = (int)allPts.size() - ignoreCount;

        for (const auto& np : newPts) {
            for (int i = 0; i < safeLimit; i++) {
                if (DistSq(np, allPts[i]) < MIN_CLEARANCE * MIN_CLEARANCE) {
                    return false;
                }
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
            case 0: { // Triangular Delta (3 waypoints)
                float r1 = (float)GetRandomValue(300, 360);
                float r2 = (float)GetRandomValue(280, 340);
                float r3 = (float)GetRandomValue(280, 340);
                rawWp.push_back({ r1 * 1.0f, 0.0f });
                rawWp.push_back({ -r2 * 0.5f, r2 * 0.866f });
                rawWp.push_back({ -r3 * 0.5f, -r3 * 0.866f });
                break;
            }
            case 1: { // Pentagonal with Infield Dip (5 waypoints)
                for (int i = 0; i < 5; i++) {
                    float a = i * (2.0f * PI_F / 5.0f);
                    float rad = (i == 2) ? (float)GetRandomValue(120, 180) : (float)GetRandomValue(280, 350);
                    rawWp.push_back({ rad * std::cos(a), rad * std::sin(a) });
                }
                break;
            }
            case 2: { // Dog-Leg / L-Shape (5 waypoints)
                float w = (float)GetRandomValue(280, 350);
                float h = (float)GetRandomValue(220, 280);
                rawWp.push_back({ w, -h });
                rawWp.push_back({ w * 0.2f, 0.0f });
                rawWp.push_back({ -w, -h });
                rawWp.push_back({ -w, h });
                rawWp.push_back({ w * 0.5f, h });
                break;
            }
            case 3: default: { // Rotated Quad Box (4 waypoints)
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
            float wx = 500.0f + p.x * cosR - p.y * sinR;
            float wy = 400.0f + p.x * sinR + p.y * cosR;
            worldWp.push_back({ wx, wy });
        }

        float shoelace = 0.0f;
        int n = worldWp.size();
        for (int i = 0; i < n; i++) {
            Vector2 a = worldWp[i];
            Vector2 b = worldWp[(i + 1) % n];
            shoelace += (b.x - a.x) * (b.y + a.y);
        }
        if (shoelace < 0.0f) {
            std::reverse(worldWp.begin(), worldWp.end());
        }

        int startEdge = GetRandomValue(0, n - 1);
        Vector2 pA = worldWp[startEdge];
        Vector2 pB = worldWp[(startEdge + 1) % n];
        outStartAngle = std::atan2(pB.y - pA.y, pB.x - pA.x);

        std::vector<Vector2> reordered;
        for (int i = 1; i <= n; i++) {
            reordered.push_back(worldWp[(startEdge + i) % n]);
        }
        return reordered;
    }

    /**
     * @brief Synthesizes a high-variety modular racing circuit with 360-degree start pose and multi-topology.
     */
    inline std::vector<Vector2> GenerateProceduralCenterPoints() {
        auto catalog = BuildCatalog();

        for (int run = 0; run < 120; run++) {
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
                        int headIgnore = 8;
                        int tailIgnore = 8;
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
                    float totalScore = distScore + headingDiff * 115.0f + (float)GetRandomValue(0, 30);

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

                return fitted;
            }
        }
        return {};
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
