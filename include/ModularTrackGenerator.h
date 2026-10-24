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
 * @brief Discrete Modular Track Generator by Tile/Segment Coupling.
 * 
 * Synthesizes racing circuits by assembling discrete track modules (straights,
 * 90-deg corners, 45-deg arcs, 180-deg hairpins, and chicanes) through kinematically
 * continuous SE(2) ports, enforcing clearance and solving loop closure with an elastic Hermite bridge.
 */
namespace ModularTrackGenerator {
    constexpr float PI_F = 3.14159265358979323846f;
    constexpr float MIN_CLEARANCE = 40.0f;

    struct Port {
        Vector2 pos;
        float heading; // in radians
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

    inline ModuleDef MakeCurve90(const std::string& id, float radius, bool turnLeft) {
        ModuleDef m;
        m.id = id;
        m.deltaHeading = turnLeft ? (PI_F * 0.5f) : (-PI_F * 0.5f);
        int steps = std::max(4, (int)(radius * (PI_F * 0.5f) / 20.0f));

        if (turnLeft) {
            m.localExitPos = { radius, radius };
            for (int i = 0; i <= steps; i++) {
                float alpha = -PI_F * 0.5f + (float)i / steps * (PI_F * 0.5f);
                m.localPoints.push_back({ radius * std::cos(alpha), radius + radius * std::sin(alpha) });
            }
        } else {
            m.localExitPos = { radius, -radius };
            for (int i = 0; i <= steps; i++) {
                float alpha = PI_F * 0.5f - (float)i / steps * (PI_F * 0.5f);
                m.localPoints.push_back({ radius * std::cos(alpha), -radius + radius * std::sin(alpha) });
            }
        }
        return m;
    }

    inline ModuleDef MakeCurve45(const std::string& id, float radius, bool turnLeft) {
        ModuleDef m;
        m.id = id;
        m.deltaHeading = turnLeft ? (PI_F * 0.25f) : (-PI_F * 0.25f);
        int steps = std::max(3, (int)(radius * (PI_F * 0.25f) / 20.0f));

        if (turnLeft) {
            m.localExitPos = { radius * std::sin(PI_F * 0.25f), radius * (1.0f - std::cos(PI_F * 0.25f)) };
            for (int i = 0; i <= steps; i++) {
                float alpha = -PI_F * 0.5f + (float)i / steps * (PI_F * 0.25f);
                m.localPoints.push_back({ radius * std::cos(alpha), radius + radius * std::sin(alpha) });
            }
        } else {
            m.localExitPos = { radius * std::sin(PI_F * 0.25f), -radius * (1.0f - std::cos(PI_F * 0.25f)) };
            for (int i = 0; i <= steps; i++) {
                float alpha = PI_F * 0.5f - (float)i / steps * (PI_F * 0.25f);
                m.localPoints.push_back({ radius * std::cos(alpha), -radius + radius * std::sin(alpha) });
            }
        }
        return m;
    }

    inline ModuleDef MakeHairpin180(const std::string& id, float radius, bool turnLeft) {
        ModuleDef m;
        m.id = id;
        m.deltaHeading = turnLeft ? PI_F : -PI_F;
        int steps = std::max(6, (int)(radius * PI_F / 20.0f));

        if (turnLeft) {
            m.localExitPos = { 0.0f, 2.0f * radius };
            for (int i = 0; i <= steps; i++) {
                float alpha = -PI_F * 0.5f + (float)i / steps * PI_F;
                m.localPoints.push_back({ radius * std::cos(alpha), radius + radius * std::sin(alpha) });
            }
        } else {
            m.localExitPos = { 0.0f, -2.0f * radius };
            for (int i = 0; i <= steps; i++) {
                float alpha = PI_F * 0.5f - (float)i / steps * PI_F;
                m.localPoints.push_back({ radius * std::cos(alpha), -radius + radius * std::sin(alpha) });
            }
        }
        return m;
    }

    inline ModuleDef MakeChicane(const std::string& id, float length, float lateralOffset) {
        ModuleDef m;
        m.id = id;
        m.deltaHeading = 0.0f;
        m.localExitPos = { length, lateralOffset };
        int steps = 6;
        for (int i = 0; i <= steps; i++) {
            float t = (float)i / steps;
            float x = t * length;
            float y = (lateralOffset * 0.5f) * (1.0f - std::cos(PI_F * t));
            m.localPoints.push_back({ x, y });
        }
        return m;
    }

    inline std::vector<ModuleDef> BuildCatalog() {
        std::vector<ModuleDef> cat;
        cat.push_back(MakeStraight("STR_80", 80.0f));
        cat.push_back(MakeStraight("STR_150", 150.0f));
        cat.push_back(MakeCurve90("CRV_90_R110_L", 110.0f, true));
        cat.push_back(MakeCurve90("CRV_90_R110_R", 110.0f, false));
        cat.push_back(MakeCurve45("CRV_45_R160_L", 160.0f, true));
        cat.push_back(MakeCurve45("CRV_45_R160_R", 160.0f, false));
        cat.push_back(MakeHairpin180("HRP_180_R60_L", 60.0f, true));
        cat.push_back(MakeHairpin180("HRP_180_R60_R", 60.0f, false));
        cat.push_back(MakeChicane("CHK_S_L40", 130.0f, 40.0f));
        cat.push_back(MakeChicane("CHK_S_R40", 130.0f, -40.0f));
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
        if (d < 60.0f || d > 450.0f) return false;

        float toStartX = startPort.pos.x - curPort.pos.x;
        float toStartY = startPort.pos.y - curPort.pos.y;
        float dirAngle = std::atan2(toStartY, toStartX);
        float angleDiff = std::abs(NormalizeAngle(dirAngle - curPort.heading));
        if (angleDiff > PI_F * 0.70f) return false;

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

    /**
     * @brief Generates a closed modular circuit assembled from the discrete tile catalog.
     */
    inline std::vector<Vector2> GenerateProceduralCenterPoints() {
        auto catalog = BuildCatalog();

        for (int run = 0; run < 120; run++) {
            float xR = (float)GetRandomValue(720, 860);
            float xL = (float)GetRandomValue(140, 280);
            float yB = (float)GetRandomValue(620, 700);
            float yT = (float)GetRandomValue(140, 220);

            bool hasInfield = (GetRandomValue(0, 100) < 45);
            float xMid = (xR + xL) * 0.5f + (float)GetRandomValue(-40, 40);
            float yMid = (yB + yT) * 0.5f + (float)GetRandomValue(-40, 40);

            std::vector<Vector2> waypoints;
            waypoints.push_back({ xR, yB });
            waypoints.push_back({ xR, yT });
            if (hasInfield) {
                waypoints.push_back({ xMid, yMid });
            }
            waypoints.push_back({ xL, yT });
            waypoints.push_back({ xL, yB });

            Port startPort = { { (xL + xR) * 0.5f, yB }, 0.0f };
            Port curPort = startPort;

            std::vector<Vector2> trackPoints;
            trackPoints.push_back(startPort.pos);

            int wpIdx = 0;
            int numWP = (int)waypoints.size();
            bool closed = false;

            for (int step = 0; step < 26; step++) {
                Vector2 targetWP = (wpIdx < numWP) ? waypoints[wpIdx] : startPort.pos;
                float distToTarget = Dist(curPort.pos, targetWP);

                if (distToTarget < 140.0f && wpIdx < numWP) {
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
                    float totalScore = distScore + headingDiff * 110.0f + (float)GetRandomValue(0, 35);

                    scored.push_back({ (int)i, totalScore });
                }

                if (scored.empty()) break;

                std::sort(scored.begin(), scored.end(), [](const ScoredMod& a, const ScoredMod& b) {
                    return a.score < b.score;
                });

                int pickIdx = 0;
                if (scored.size() > 1 && GetRandomValue(0, 100) < 30) pickIdx = 1;

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
