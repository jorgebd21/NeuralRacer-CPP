#pragma once
#include <vector>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "raylib.h"

/**
 * @brief Professional Formula 1 Multi-Archetype Procedural Track Generator.
 * 
 * Synthesizes flowing, realistic racing circuits inspired by 6 Grand Prix paradigms:
 * - Archetype 0: Out-and-Back Elongated Corridor (Monza, Montreal, Baku, Red Bull Ring).
 * - Archetype 1: Infield Loop Complex (Bahrain/Sakhir, Shanghai, Silverstone).
 * - Archetype 2: Triangular & Asymmetric Diagonal (Spa-Francorchamps, Paul Ricard, Jeddah).
 * - Archetype 3: Asymmetric L-Shape & Carousel (Suzuka, Catalunya, Portimao, Zandvoort).
 * - Archetype 4: Dual-Lobe Horseshoe & Omega Waist (Interlagos, Hungaroring, Lusail).
 * - Archetype 5: Urban Street Grid & 90-Degree Complexes (Singapore Marina Bay, Albert Park, Las Vegas).
 * 
 * Features parametric random variations, orientation mirroring, guaranteed CCW winding,
 * and curvature bounds designed for proportional F1 car scales (track width 35px, car 14x7px).
 */
namespace TrackGenerator {
    constexpr float PI_F = 3.14159265358979323846f;
    constexpr float CENTER_X = 500.0f;
    constexpr float CENTER_Y = 400.0f;
    constexpr float BOUNDS_PAD_X = 80.0f;
    constexpr float BOUNDS_PAD_Y = 70.0f;

    inline float DistanceSq(Vector2 a, Vector2 b) {
        return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
    }

    /**
     * @brief Archetype 0: Out-and-Back (Monza / Montreal / Red Bull Ring / Baku).
     */
    inline std::vector<Vector2> GenerateOutAndBack() {
        std::vector<Vector2> nodes;
        float yBottom = (float)GetRandomValue(580, 640);
        float yTop = (float)GetRandomValue(160, 240);
        float xLeft = (float)GetRandomValue(120, 160);
        float xRight = (float)GetRandomValue(840, 890);

        // Bottom Main Straight
        nodes.push_back({xRight, yBottom});
        nodes.push_back({xRight * 0.65f + xLeft * 0.35f, yBottom});
        nodes.push_back({xRight * 0.35f + xLeft * 0.65f, yBottom});
        nodes.push_back({xLeft + 90.0f, yBottom});

        // Left Hairpin (Turns 1 & 2)
        nodes.push_back({xLeft + 20.0f, yBottom - 45.0f});
        nodes.push_back({xLeft - 25.0f, (yTop + yBottom) * 0.5f});
        nodes.push_back({xLeft + 20.0f, yTop + 45.0f});

        // Top Straight with chicane
        nodes.push_back({xLeft + 120.0f, yTop});
        float chicaneX = (float)GetRandomValue(380, 520);
        float chicaneDir = (GetRandomValue(0, 1) == 0) ? -1.0f : 1.0f;
        float chicaneAmp = (float)GetRandomValue(35, 55);

        nodes.push_back({chicaneX - 70.0f, yTop + chicaneDir * chicaneAmp});
        nodes.push_back({chicaneX + 70.0f, yTop - chicaneDir * chicaneAmp});
        nodes.push_back({xRight - 120.0f, yTop});

        // Right Hairpin
        nodes.push_back({xRight - 20.0f, yTop + 45.0f});
        nodes.push_back({xRight + 30.0f, (yTop + yBottom) * 0.5f});
        nodes.push_back({xRight - 20.0f, yBottom - 45.0f});

        return nodes;
    }

    /**
     * @brief Archetype 1: Infield Loop (Bahrain / Shanghai / Silverstone).
     */
    inline std::vector<Vector2> GenerateInfieldLoop() {
        std::vector<Vector2> nodes;
        float yBase = (float)GetRandomValue(620, 660);
        float yTop = (float)GetRandomValue(140, 180);
        float xLeft = (float)GetRandomValue(130, 170);
        float xRight = (float)GetRandomValue(820, 880);

        // Main straight at bottom
        nodes.push_back({xRight - 80.0f, yBase});
        nodes.push_back({(xLeft + xRight) * 0.5f, yBase});
        nodes.push_back({xLeft + 80.0f, yBase});

        // Left upward sweeper
        nodes.push_back({xLeft - 10.0f, yBase - 80.0f});
        nodes.push_back({xLeft, (yBase + yTop) * 0.5f});
        nodes.push_back({xLeft + 60.0f, yTop + 60.0f});

        // Top straight
        nodes.push_back({xLeft + 180.0f, yTop});
        nodes.push_back({(xLeft + xRight) * 0.5f, yTop});
        nodes.push_back({xRight - 100.0f, yTop});

        // Outer curve entering Infield
        nodes.push_back({xRight, yTop + 80.0f});
        nodes.push_back({xRight + 15.0f, yTop + 200.0f});

        // Deep Infield loop dipping towards center
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

        // Main diagonal acceleration straight
        nodes.push_back({xStart, yStart});
        nodes.push_back({xStart * 0.65f + xApex * 0.35f, yStart * 0.65f + yApex * 0.35f});
        nodes.push_back({xStart * 0.35f + xApex * 0.65f, yStart * 0.35f + yApex * 0.65f});
        nodes.push_back({xApex - 40.0f, yApex + 20.0f});

        // Chicane complex at high point
        nodes.push_back({xApex + 30.0f, yApex + 40.0f});
        nodes.push_back({xApex + 40.0f, yApex + 130.0f});
        nodes.push_back({xApex - 50.0f, yApex + 210.0f});

        // Technical S-curves descending
        float midX = (xApex + xStart) * 0.5f + (float)GetRandomValue(40, 80);
        float midY = (yApex + yStart) * 0.5f;
        nodes.push_back({midX + 70.0f, midY - 20.0f});
        nodes.push_back({midX, midY + 40.0f});
        nodes.push_back({midX - 60.0f, midY + 100.0f});

        // Return sweeper connecting back cleanly
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

        // Main horizontal straight
        nodes.push_back({xMax - 80.0f, yBase});
        nodes.push_back({(xBase + xMax) * 0.5f, yBase});
        nodes.push_back({xBase + 120.0f, yBase});

        // 90-degree left sweeper into vertical straight
        nodes.push_back({xBase, yBase - 70.0f});
        nodes.push_back({xBase - 15.0f, (yBase + yTop) * 0.5f + 50.0f});
        nodes.push_back({xBase + 10.0f, (yBase + yTop) * 0.5f - 50.0f});
        nodes.push_back({xBase + 60.0f, yTop + 60.0f});

        // Top technical sector (Esses / chicane)
        nodes.push_back({xBase + 180.0f, yTop});
        nodes.push_back({xBase + 340.0f, yTop + 60.0f});
        nodes.push_back({xBase + 460.0f, yTop - 10.0f});

        // Descending carousel sweeper
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

        // Bottom main straight (Reta Oposta)
        nodes.push_back({xRight - 40.0f, 640.0f});
        nodes.push_back({500.0f, 650.0f});
        nodes.push_back({xLeft + 100.0f, 640.0f});

        // Lower-left Horseshoe (Senna S & Curva do Sol)
        nodes.push_back({xLeft, 560.0f});
        nodes.push_back({xLeft - 20.0f, 440.0f});
        nodes.push_back({xLeft + 80.0f, 380.0f});
        nodes.push_back({xLeft + 220.0f, 430.0f}); // Waist transition

        // Upper sweeper climbing into Ferradura
        nodes.push_back({380.0f, 330.0f});
        nodes.push_back({320.0f, 190.0f});

        // Top Horseshoe Bowl
        nodes.push_back({480.0f, 140.0f});
        nodes.push_back({680.0f, 150.0f});
        nodes.push_back({xRight, 240.0f});
        nodes.push_back({xRight + 20.0f, 380.0f});

        // Technical descending S-curves back to straight
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

        // Long Urban High-Speed Boulevard
        nodes.push_back({xRight - 60.0f, yBottom});
        nodes.push_back({(xLeft + xRight) * 0.5f, yBottom});
        nodes.push_back({xLeft + 90.0f, yBottom});

        // 90-degree hard braking turn 1 & 2
        nodes.push_back({xLeft, yBottom - 80.0f});
        nodes.push_back({xLeft + 20.0f, yBottom - 200.0f});

        // Castle / Marina sector (sharp right-left staircase complex)
        nodes.push_back({xLeft + 140.0f, yBottom - 230.0f});
        nodes.push_back({xLeft + 160.0f, yTop + 200.0f});
        nodes.push_back({xLeft + 60.0f, yTop + 140.0f});
        nodes.push_back({xLeft + 120.0f, yTop + 40.0f});

        // Top Boulevard Straight
        nodes.push_back({350.0f, yTop});
        nodes.push_back({620.0f, yTop});

        // Harbor chicane
        nodes.push_back({740.0f, yTop + 70.0f});
        nodes.push_back({xRight, yTop + 130.0f});

        // 90-degree stadium turns
        nodes.push_back({xRight + 20.0f, 360.0f});
        nodes.push_back({xRight - 100.0f, 420.0f});
        nodes.push_back({xRight - 100.0f, 530.0f});
        nodes.push_back({xRight + 10.0f, 570.0f});

        return nodes;
    }

    /**
     * @brief Generates a procedural Formula 1 racing circuit using one of the 6 Grand Prix archetypes.
     * 
     * @return std::vector<Vector2> Ordered centerline nodes.
     */
    inline std::vector<Vector2> GenerateProceduralCenterPoints() {
        int archetype = GetRandomValue(0, 5);
        std::vector<Vector2> nodes;

        switch (archetype) {
            case 0: nodes = GenerateOutAndBack(); break;
            case 1: nodes = GenerateInfieldLoop(); break;
            case 2: nodes = GenerateTriangularDiagonal(); break;
            case 3: nodes = GenerateLShape(); break;
            case 4: nodes = GenerateDualLobeHorseshoe(); break;
            case 5: default: nodes = GenerateUrbanStreet(); break;
        }

        // Random horizontal flip (50% probability) for layout variety
        if (GetRandomValue(0, 1) == 1) {
            for (auto& pt : nodes) {
                pt.x = (2.0f * CENTER_X) - pt.x;
            }
        }

        // Ensure Counter-Clockwise (CCW) winding order via Shoelace formula
        float shoelace = 0.0f;
        int n = nodes.size();
        for (int i = 0; i < n; i++) {
            Vector2 a = nodes[i];
            Vector2 b = nodes[(i + 1) % n];
            shoelace += (b.x - a.x) * (b.y + a.y);
        }
        if (shoelace < 0.0f) {
            std::reverse(nodes.begin(), nodes.end());
        }

        // Clamp nodes inside safe screen margins
        for (auto& pt : nodes) {
            pt.x = std::clamp(pt.x, BOUNDS_PAD_X, 1000.0f - BOUNDS_PAD_X);
            pt.y = std::clamp(pt.y, BOUNDS_PAD_Y, 800.0f - BOUNDS_PAD_Y);
        }

        return nodes;
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
