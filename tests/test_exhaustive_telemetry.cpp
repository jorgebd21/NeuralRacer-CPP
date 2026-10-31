#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <cfloat>
#include "Config.h"
#include "Brain.h"
#include "Car.h"
#include "FlatSpatialGrid.h"
#include "TrackManager.h"
#include "ModularTrackGenerator.h"
#include "Evolution.h"
#include "ExhaustiveTelemetry.h"

int main(int argc, char* argv[]) {
    std::string brainPath = "data/best.json";
    std::string trackType = "monza";
    int maxTicks = 2000;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--brain" && i + 1 < argc) brainPath = argv[++i];
        else if (arg == "--track" && i + 1 < argc) trackType = argv[++i];
        else if (arg == "--ticks" && i + 1 < argc) maxTicks = std::stoi(argv[++i]);
    }

    std::cout << "===============================================================\n";
    std::cout << "       EXHAUSTIVE TELEMETRY HARNESS & BEHAVIORAL AUDITOR       \n";
    std::cout << "===============================================================\n";
    std::cout << " Brain Path:   " << brainPath << "\n";
    std::cout << " Track:        " << trackType << "\n";
    std::cout << " Max Ticks:    " << maxTicks << "\n";

    std::vector<Segment> walls;
    std::vector<Vector2> checkpoints;
    std::vector<Vector2> denseLine;
    Vector2 startPos{0, 0};
    float startRot = 0.0f;
    float trackWidth = 35.0f;

    if (trackType == "monza" || trackType == "0") {
        std::vector<Vector2> pts = ModularTrackGenerator::GenerateProceduralCenterPoints(0);
        if (pts.size() < 12) pts = ModularTrackGenerator::GenerateArchetypeFallback(0);
        constexpr int PROCEDURAL_SPLINE_SEGMENTS = 24;
        denseLine = TrackManager::GenerateSplinePoints(pts, PROCEDURAL_SPLINE_SEGMENTS);
        TrackManager::CalculateStartGrid(pts, startPos, startRot);

        int snapIdx = 0;
        float minDist = FLT_MAX;
        for (int i = 0; i < (int)denseLine.size(); i++) {
            float dx = denseLine[i].x - startPos.x;
            float dy = denseLine[i].y - startPos.y;
            float d = dx*dx + dy*dy;
            if (d < minDist) { minDist = d; snapIdx = i; }
        }
        startPos = denseLine[snapIdx];
        int denseN = (int)denseLine.size();
        Vector2 prev = denseLine[(snapIdx - 1 + denseN) % denseN];
        Vector2 next = denseLine[(snapIdx + 1) % denseN];
        startRot = atan2(next.y - prev.y, next.x - prev.x) * (180.0f / 3.14159265f);
        TrackManager::GenerateBordersFromCenterLine(denseLine, trackWidth, walls, checkpoints, startPos);
    } else {
        // Load from JSON file path
        std::string path = (trackType == "pista_dificil") ? "data/tracks/pista_dificil.json" : trackType;
        TrackManager::LoadTrackFromFile(path, walls, startPos, startRot, checkpoints);
        denseLine = checkpoints;
        trackWidth = 50.0f;
    }

    FlatSpatialGrid grid(Config::WORLD_WIDTH, Config::WORLD_HEIGHT, TrackManager::GRID_CELL_SIZE);
    grid.Build(walls);
    if (!denseLine.empty()) {
        TrackManager::GenerateSurfaceZones(denseLine, trackWidth, grid);
    }

    std::vector<Car> pop;
    pop.emplace_back(startPos.x, startPos.y, startRot);
    int genLoaded = 0;
    if (!Evolution::LoadBestBrains(pop, genLoaded, brainPath)) {
        std::cerr << "Warning: Could not load brain from '" << brainPath << "', using default initialization.\n";
    } else {
        std::cout << "Loaded brain from generation " << genLoaded << ".\n";
    }

    Car& car = pop[0];
    car.Reset(startPos.x, startPos.y, startRot);

    ExhaustiveTelemetryRecorder recorder(maxTicks);

    bool crashed = false;
    std::string crashReason = "";

    for (int t = 0; t < maxTicks; ++t) {
        float inAccel = 0.0f, inTurn = 0.0f;
        car.Think(inAccel, inTurn);
        car.UpdatePhysics(inAccel, inTurn, grid, t, checkpoints);

        recorder.RecordTick(car, grid, t, checkpoints, denseLine, trackWidth);

        if (car.isCrashed) {
            crashed = true;
            if (car.timeSinceLastCheckpoint > Config::CAR_MAX_TIME_WITHOUT_CHECKPOINT) {
                crashReason = "TIMEOUT_NO_CHECKPOINT";
            } else if (car.longitudinalVelocity < Config::CAR_STALL_SPEED_THRESHOLD) {
                crashReason = "REVERSE_STALL_SPEED";
            } else if (car.stallTicks > Config::CAR_STALL_TIME_THRESHOLD) {
                crashReason = "STAGNANT_STALL_TICKS";
            } else {
                crashReason = "WALL_OBB_COLLISION";
            }
            break;
        }
    }

    recorder.Finalize(crashed, crashReason);

    recorder.PrintSummaryTable(std::cout);
    recorder.PrintAnomalyTimeline(std::cout, 30);

    recorder.ExportCSV("data/exhaustive_telemetry.csv");
    recorder.ExportJSON("data/exhaustive_telemetry.json");

    std::cout << "\n[SUCCESS] Exhaustive time-series saved to 'data/exhaustive_telemetry.csv'\n";
    std::cout << "[SUCCESS] JSON Summary saved to 'data/exhaustive_telemetry.json'\n";

    return 0;
}
