#include "Simulation.h"
#include "Config.h"
#include "Evolution.h"
#include "TrackManager.h"
#include "TrackGenerator.h"
#include "ModularTrackGenerator.h"
#include "Telemetry.h"
#include <algorithm>
#include <cfloat>
#include <iostream>
#include <fstream>
#include <string>
#include <execution>
#include <atomic>
#include <csignal>
#include <omp.h>

static volatile sig_atomic_t stopRequested = 0;
static void signalHandler(int) { stopRequested = 1; }

Simulation::Simulation(bool isHeadless) : 
    isHeadless(isHeadless),
    showCheckpoints(false),
    showTelemetry(false),
    currentState(MENU),
    currentMapIndex(0),
    startPosition{Config::SIM_START_POS_X, Config::SIM_START_POS_Y},
    startRotation(0.0f),
    generationTimer(0),
    generationCount(0),
    currentEvaluationTrack(0),
    simSpeed(1),
    playerCar(startPosition.x, startPosition.y, startRotation),
    aiCar(startPosition.x, startPosition.y, startRotation),
    exhibitionResult(0),
    generationTotalLaps(0),
    benchmarkTopFitness(0.0f),
    cameraMode(CAM_FOLLOW_LEADER),
    worldCamera{},
    targetZoom(1.0f),
    cameraDragStart{0.0f, 0.0f},
    isDraggingCamera(false)
{
}

constexpr int SIM_TARGET_FPS = 60;
constexpr int PROCEDURAL_SPLINE_SEGMENTS = 16;
constexpr float PROCEDURAL_TRACK_WIDTH = 35.0f;

void Simulation::Init() {
    if(!isHeadless){
        InitWindow(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, "Genetic Simulator - Autonomous AI");
        SetTargetFPS(SIM_TARGET_FPS);
        worldCamera.offset = { (Config::SCREEN_WIDTH - 250.0f) * 0.5f, Config::SCREEN_HEIGHT * 0.5f };
        worldCamera.target = startPosition;
        worldCamera.rotation = 0.0f;
        worldCamera.zoom = 1.0f;
        targetZoom = 1.0f;
    }
    
    Telemetry::Init();

    mapFiles = TrackManager::ScanMapFiles();
    if (mapFiles.empty()) {
        std::cerr << "Warning: No track files found in data/tracks/. Generating a procedural track." << std::endl;
        proceduralPoints = ModularTrackGenerator::GenerateProceduralCenterPoints();
        std::vector<Vector2> denseCenterLine = TrackManager::GenerateSplinePoints(proceduralPoints, PROCEDURAL_SPLINE_SEGMENTS);
        TrackManager::CalculateStartGrid(proceduralPoints, startPosition, startRotation);

        // Snap to nearest dense center line point (same logic as LoadTrackFromFile)
        int snapIdx = 0;
        float minDist = FLT_MAX;
        for (int i = 0; i < (int)denseCenterLine.size(); i++) {
            float dx = denseCenterLine[i].x - startPosition.x;
            float dy = denseCenterLine[i].y - startPosition.y;
            float d = dx*dx + dy*dy;
            if (d < minDist) { minDist = d; snapIdx = i; }
        }
        startPosition = denseCenterLine[snapIdx];
        int denseN = (int)denseCenterLine.size();
        Vector2 prev = denseCenterLine[(snapIdx - 1 + denseN) % denseN];
        Vector2 next = denseCenterLine[(snapIdx + 1) % denseN];
        startRotation = atan2(next.y - prev.y, next.x - prev.x) * (180.0f / PI);

        TrackManager::GenerateBordersFromCenterLine(denseCenterLine, PROCEDURAL_TRACK_WIDTH, trackWalls, trackCheckpoints, startPosition);
    } else {
        trackFile = mapFiles[currentMapIndex];
        TrackManager::LoadTrackFromFile(trackFile, trackWalls, startPosition, startRotation, trackCheckpoints);
    }
    BuildSpatialGrid();
    if (trackWalls.empty()) trackWalls.push_back({{100, 100}, {900, 100}});

    InitTrainingTracks();

    for (int i = 0; i < Config::POPULATION_SIZE; i++) {
        population.push_back(Car(startPosition.x, startPosition.y, startRotation));
    }
    Evolution::LoadBestBrains(population, generationCount);
    SetupCurriculumForGeneration(generationCount);
    SetTrainingTrack(0);
    
    playerCar = Car(startPosition.x, startPosition.y, startRotation);
    aiCar = population.empty() ? Car(startPosition.x, startPosition.y, startRotation) : population[0];
    aiCar.Reset(startPosition.x, startPosition.y, startRotation);
}

void Simulation::Run() {
    Init();
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    if(isHeadless){
        currentState = TRAINING;
        while(!stopRequested){
            Update();
        }
        std::cout << "Training stopped gracefully after " << generationCount << " generations." << std::endl;
    }else{
        while (!WindowShouldClose() && !stopRequested) {
            Update();
            Draw();
        }
        CloseWindow();
    }
    Telemetry::Flush();
}

void Simulation::Update() {
    UpdateCameraControl();

    // Main state machine: delegates logical update according to the current mode
    if (currentState == MENU) UpdateMenu();
    else if (currentState == TRAINING) UpdateTraining();
    else if (currentState == EXHIBITION) UpdateExhibition();
    else if (currentState == TEST_AI) UpdateTestAI();
}

void Simulation::UpdateMenu() {
    if (aiCar.isCrashed) {
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
    }
    if (!aiCar.isCrashed) {
        float aiAccelerate = 0.0f, aiTurn = 0.0f;
        aiCar.Think(aiAccelerate, aiTurn);
        aiCar.UpdatePhysics(aiAccelerate, aiTurn, spatialGrid, 0, trackCheckpoints);
    }

    if (IsKeyPressed(KEY_T)) {
        currentEvaluationTrack = 0;
        SetupCurriculumForGeneration(generationCount);
        SetTrainingTrack(0);
        for (auto& car : population) car.Reset(startPosition.x, startPosition.y, startRotation);
        currentState = TRAINING;
    }
    if (IsKeyPressed(KEY_E)) {
        playerCar.Reset(startPosition.x, startPosition.y, startRotation);
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
        
        std::vector<Car> temp(1, Car(startPosition.x, startPosition.y, startRotation));
        if (Evolution::LoadBestBrains(temp, generationCount)) {
            aiCar = temp[0];
        }
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
        exhibitionResult = 0;
        currentState = EXHIBITION;
    }
    if (IsKeyPressed(KEY_A)) {
        std::vector<Car> temp(1, Car(startPosition.x, startPosition.y, startRotation));
        if (Evolution::LoadBestBrains(temp, generationCount)) {
            aiCar = temp[0];
        }
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
        exhibitionResult = 0;
        currentState = TEST_AI;
    }
    if (IsKeyPressed(KEY_P)) {
        proceduralPoints = ModularTrackGenerator::GenerateProceduralCenterPoints();
        if (!proceduralPoints.empty()) {
            std::vector<Vector2> denseCenterLine = TrackManager::GenerateSplinePoints(proceduralPoints, PROCEDURAL_SPLINE_SEGMENTS);
            TrackManager::CalculateStartGrid(proceduralPoints, startPosition, startRotation);
            trackWalls.clear();
            trackCheckpoints.clear();
            Telemetry::ResetHeatmap();
            TrackManager::GenerateBordersFromCenterLine(denseCenterLine, PROCEDURAL_TRACK_WIDTH, trackWalls, trackCheckpoints, startPosition);
            BuildSpatialGrid();
            InitTrainingTracks();
            
            for (auto& car : population) car.Reset(startPosition.x, startPosition.y, startRotation);
            playerCar.Reset(startPosition.x, startPosition.y, startRotation);
            aiCar.Reset(startPosition.x, startPosition.y, startRotation);
        }
    }
    if (IsKeyPressed(KEY_LEFT) && !mapFiles.empty()) {
        currentMapIndex--;
        if (currentMapIndex < 0) currentMapIndex = (int)mapFiles.size() - 1;
        trackFile = mapFiles[currentMapIndex];
        trackWalls.clear();
        trackCheckpoints.clear();
        Telemetry::ResetHeatmap();
        TrackManager::LoadTrackFromFile(trackFile, trackWalls, startPosition, startRotation, trackCheckpoints);
        BuildSpatialGrid();
        InitTrainingTracks();
        for (auto& car : population) car.Reset(startPosition.x, startPosition.y, startRotation);
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
    }
    if (IsKeyPressed(KEY_RIGHT) && !mapFiles.empty()) {
        currentMapIndex++;
        if (currentMapIndex >= (int)mapFiles.size()) currentMapIndex = 0;
        trackFile = mapFiles[currentMapIndex];
        trackWalls.clear();
        trackCheckpoints.clear();
        Telemetry::ResetHeatmap();
        TrackManager::LoadTrackFromFile(trackFile, trackWalls, startPosition, startRotation, trackCheckpoints);
        BuildSpatialGrid();
        InitTrainingTracks();
        for (auto& car : population) car.Reset(startPosition.x, startPosition.y, startRotation);
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
    }
    if (IsKeyPressed(KEY_G)) {
        if (!proceduralPoints.empty()) {
            int counter = 1;
            std::string filename;
            while (true) {
                filename = "data/tracks/procedural_track_" + std::to_string(counter) + ".json";
                std::ifstream f(filename.c_str());
                if (!f.good()) break;
                counter++;
            }
            ModularTrackGenerator::SaveTrackToFile(proceduralPoints, filename);
        }
    }
}

constexpr int FAST_FORWARD_MULTIPLIER = 50;

void Simulation::UpdateTraining() {
    // Allows accelerating the simulation to train faster
    if(!isHeadless){
        if (IsKeyPressed(KEY_SPACE)) simSpeed = (simSpeed == 1) ? FAST_FORWARD_MULTIPLIER : 1;
        if (IsKeyPressed(KEY_M)) currentState = MENU;
        if (IsKeyPressed(KEY_C)) showCheckpoints = !showCheckpoints;
        if (IsKeyPressed(KEY_H)) showTelemetry = !showTelemetry;
    }

    // Execute logic multiple times per frame if in fast forward mode
    for (int s = 0; s < simSpeed; s++) {
        std::atomic<int> carsAlive{0};

        // High-performance OpenMP multi-core parallelization across all available CPU threads
        #pragma omp parallel for schedule(static)
        for (int i = 0; i < (int)population.size(); i++) {
            Car& car = population[i];
            if (!car.isCrashed) {
                carsAlive++;

                // 1. The brain decides what to do based on the 9-dimensional state space
                float inputAccelerate = 0.0f, inputTurn = 0.0f;
                car.Think(inputAccelerate, inputTurn);

                // 2. The car moves according to the decision and checks if it crashed
                car.UpdatePhysics(inputAccelerate, inputTurn, spatialGrid, generationTimer, trackCheckpoints);
            }
        }

        // Sequential post-step: record newly crashed cars only on the tick they impact
        for (auto& car : population) {
            if (car.justCrashed) {
                Telemetry::RecordCrash(car.position);
                car.justCrashed = false;
            }
        }
        
        bool allCrashed = (carsAlive == 0);
        generationTimer++;
        
        // If all cars died or the maximum time per generation is up
        if (allCrashed || generationTimer >= Config::MAX_GENERATION_TIME) {
            std::cout << "Generation " << generationCount << " (Track " << currentEvaluationTrack + 1 << "/" << trainingTracks.size() 
                      << ") finished in " << generationTimer << " ticks (" << carsAlive << " alive)" << std::endl;

            // Accumulate completed laps across the population on this track
            int numCp = (int)trackCheckpoints.size();
            if (numCp > 0) {
                for (const auto& car : population) {
                    generationTotalLaps += car.totalCheckPointsCrossed / numCp;
                }
            }
            if (currentEvaluationTrack == 0) {
                float maxFit0 = 0.0f;
                for (const auto& car : population) {
                    if (car.fitness > maxFit0) maxFit0 = car.fitness;
                }
                benchmarkTopFitness = maxFit0;
            }

            currentEvaluationTrack++;
            if (currentEvaluationTrack >= (int)trainingTracks.size()) {
                std::vector<Car*> sortedPop;
                sortedPop.reserve(population.size());
                for (auto& car : population) {
                    sortedPop.push_back(&car);
                }
                std::sort(sortedPop.begin(), sortedPop.end(), [](const Car* a, const Car* b) { return a->fitness > b->fitness; });
                std::cout << "Best fitness: " << sortedPop[0]->fitness << std::endl;
                
                int phase = 3; // Pure procedural multi-circuit evaluation from Generation 0
                float totalTopFit = sortedPop[0]->fitness;
                float procFit = std::max(0.0f, totalTopFit - benchmarkTopFitness);
                float eliteAvgSpeed = sortedPop[0]->GetAverageSpeed();
                float eliteSmoothness = std::clamp(1.0f - (std::abs(sortedPop[0]->angularVelocity) / Config::CAR_TURN_SPEED), 0.0f, 1.0f);

                int fastestLapTicks = 0;
                for (const auto& car : population) {
                    if (car.bestLapTicks > 0) {
                        if (fastestLapTicks == 0 || car.bestLapTicks < fastestLapTicks) {
                            fastestLapTicks = car.bestLapTicks;
                        }
                    }
                }
                float bestLapTimeSeconds = (fastestLapTicks > 0) ? ((float)fastestLapTicks / 60.0f) : 0.0f;

                float eliteKamm = sortedPop[0]->kammUtilization;
                float eliteSlip = std::abs(sortedPop[0]->slipAngle);
                float eliteJerk = std::sqrt(sortedPop[0]->jerkLong * sortedPop[0]->jerkLong + sortedPop[0]->jerkLat * sortedPop[0]->jerkLat);
                float eliteTrackUtil = (sortedPop[0]->trackOffsetSamples > 0) ? (sortedPop[0]->sumTrackOffset / sortedPop[0]->trackOffsetSamples) : 0.0f;
                float eliteSaturation = sortedPop[0]->brain.ComputeHiddenSaturation();

                // Find fastest lap buffer to export as time-series
                const Car* fastestCar = nullptr;
                if (fastestLapTicks > 0) {
                    for (const auto& car : population) {
                        if (car.bestLapTicks == fastestLapTicks && !car.bestLapSamples.empty()) {
                            fastestCar = &car;
                            break;
                        }
                    }
                }
                if (!fastestCar && !sortedPop.empty()) {
                    if (!sortedPop[0]->bestLapSamples.empty()) fastestCar = sortedPop[0];
                    else if (!sortedPop[0]->currentLapSamples.empty()) fastestCar = sortedPop[0];
                }
                if (fastestCar) {
                    const auto& samples = !fastestCar->bestLapSamples.empty() ? fastestCar->bestLapSamples : fastestCar->currentLapSamples;
                    Telemetry::RecordFastestLap(samples);
                    Telemetry::ExportFastestLapAsync();
                }

                Telemetry::RecordGeneration(generationCount, population, Config::MAX_GENERATION_TIME,
                                            phase, benchmarkTopFitness, procFit, generationTotalLaps,
                                            eliteAvgSpeed, eliteSmoothness, Config::MUTATION,
                                            bestLapTimeSeconds,
                                            eliteKamm, eliteSlip, eliteJerk, eliteTrackUtil, eliteSaturation);
                Telemetry::ExportDataAsync();
                
                generationTotalLaps = 0;
                benchmarkTopFitness = 0.0f;
                generationTimer = 0;
                generationCount++;
                currentEvaluationTrack = 0;

                // Setup adaptive curriculum tracks for upcoming generation
                SetupCurriculumForGeneration(generationCount);
                Evolution::EvolvePopulation(population, trainingTracks[0].startPos, trainingTracks[0].startRot, generationCount);
                SetTrainingTrack(0);
            } else {
                for (auto& car : population) {
                    car.accumulatedFitness = car.fitness;
                }
                
                SetTrainingTrack(currentEvaluationTrack);
                
                for (auto& car : population) {
                    car.Reset(startPosition.x, startPosition.y, startRotation, false);
                }
                generationTimer = 0;
            }
        }
    }
}

void Simulation::UpdateExhibition() {
    if (IsKeyPressed(KEY_M)) currentState = MENU;
    if (IsKeyPressed(KEY_R) && exhibitionResult != 0) {
        playerCar.Reset(startPosition.x, startPosition.y, startRotation);
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
        exhibitionResult = 0;
    }

    if (exhibitionResult == 0) {
        float playerAccelerate = 0.0f, playerTurn = 0.0f;
        if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) playerAccelerate = 1.0f;
        if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) playerAccelerate = -1.0f;
        if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) playerTurn = 1.0f;
        if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) playerTurn = -1.0f;
        playerCar.UpdatePhysics(playerAccelerate, playerTurn, spatialGrid, 0, trackCheckpoints);

        float aiAccelerate = 0.0f, aiTurn = 0.0f;
        aiCar.Think(aiAccelerate, aiTurn);
        aiCar.UpdatePhysics(aiAccelerate, aiTurn, spatialGrid, 0, trackCheckpoints);

        if (playerCar.isCrashed && !aiCar.isCrashed) exhibitionResult = 2;
        else if (aiCar.isCrashed && !playerCar.isCrashed) exhibitionResult = 1;
        else if (aiCar.isCrashed && playerCar.isCrashed) exhibitionResult = 2; 
    }
}

void Simulation::Draw() {
    BeginDrawing();
    ClearBackground(DARKGRAY);
    
    if (currentState == MENU) DrawMenu();
    else if (currentState == TRAINING) DrawTraining();
    else if (currentState == EXHIBITION) DrawExhibition();
    else if (currentState == TEST_AI) DrawTestAI();
    
    EndDrawing();
}

constexpr int FINISH_LINE_BLOCK_SIZE = 10;
constexpr int FINISH_LINE_BLOCK_COUNT = 10;

void Simulation::DrawCheckpoints(float alpha) {
    for (size_t i = 0; i < trackCheckpoints.size(); i++) {
        auto& cp = trackCheckpoints[i];
        DrawCircleLines(cp.x, cp.y, Config::CHECKPOINT_RADIUS, Fade(YELLOW, alpha));
        DrawText(TextFormat("%zu", i + 1), cp.x - 5, cp.y - 10, 16, Fade(ORANGE, alpha));
    }
}

void Simulation::DrawFinishLine(float alpha) {
    float rad = startRotation * DEG2RAD;
    float cosR = cos(rad);
    float sinR = sin(rad);
    
    float halfWidth = (FINISH_LINE_BLOCK_COUNT * FINISH_LINE_BLOCK_SIZE) / 2.0f;

    for (int i = 0; i < FINISH_LINE_BLOCK_COUNT; i++) {
        float localY = i * FINISH_LINE_BLOCK_SIZE - halfWidth;
        
        float localX1 = 0;
        Vector2 pos1 = {
            startPosition.x + localX1 * cosR - localY * sinR,
            startPosition.y + localX1 * sinR + localY * cosR
        };
        Rectangle rec1 = { pos1.x, pos1.y, Config::CAR_HALF_WIDTH * 2, Config::CAR_HALF_LENGTH * 2 };
        DrawRectanglePro(rec1, {0, 0}, startRotation, Fade((i % 2 == 0) ? WHITE : BLACK, alpha));
        
        float localX2 = FINISH_LINE_BLOCK_SIZE;
        Vector2 pos2 = {
            startPosition.x + localX2 * cosR - localY * sinR,
            startPosition.y + localX2 * sinR + localY * cosR
        };
        Rectangle rec2 = { pos2.x, pos2.y, (float)FINISH_LINE_BLOCK_SIZE, (float)FINISH_LINE_BLOCK_SIZE };
        DrawRectanglePro(rec2, {0, 0}, startRotation, Fade((i % 2 == 0) ? BLACK : WHITE, alpha));
    }
}

void Simulation::UpdateCameraControl() {
    if (isHeadless) return;

    // Toggle camera mode with [V]
    if (IsKeyPressed(KEY_V)) {
        if (cameraMode == CAM_FOLLOW_LEADER) {
            cameraMode = CAM_OVERVIEW;
        } else {
            cameraMode = CAM_FOLLOW_LEADER;
        }
    }

    // Zoom control with mouse wheel
    float wheel = GetMouseWheelMove();
    if (wheel != 0.0f) {
        targetZoom = std::clamp(targetZoom + wheel * 0.08f, 0.20f, 2.5f);
    }

    // Manual camera pan with right mouse button or middle mouse button
    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) || IsMouseButtonPressed(MOUSE_BUTTON_MIDDLE)) {
        isDraggingCamera = true;
        cameraDragStart = GetMousePosition();
        cameraMode = CAM_MANUAL;
    }
    if (isDraggingCamera) {
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) || IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
            Vector2 delta = GetMouseDelta();
            worldCamera.target.x -= delta.x / worldCamera.zoom;
            worldCamera.target.y -= delta.y / worldCamera.zoom;
        } else {
            isDraggingCamera = false;
        }
    }

    // Determine target entity to follow
    Vector2 followPos = startPosition;
    if (currentState == TRAINING) {
        Car* leader = nullptr;
        float maxFit = -1.0f;
        for (auto& car : population) {
            if (!car.isCrashed && car.fitness > maxFit) {
                maxFit = car.fitness;
                leader = &car;
            }
        }
        if (!leader && !population.empty()) leader = &population[0];
        if (leader) followPos = leader->position;
    } else if (currentState == EXHIBITION) {
        followPos = playerCar.isCrashed ? aiCar.position : playerCar.position;
    } else if (currentState == TEST_AI || currentState == MENU) {
        followPos = aiCar.position;
    }

    if (cameraMode == CAM_FOLLOW_LEADER) {
        worldCamera.target.x += (followPos.x - worldCamera.target.x) * 0.08f;
        worldCamera.target.y += (followPos.y - worldCamera.target.y) * 0.08f;
    } else if (cameraMode == CAM_OVERVIEW) {
        // Calculate track center and bounding box to fit the entire circuit
        float minX = 1e9f, maxX = -1e9f, minY = 1e9f, maxY = -1e9f;
        for (const auto& wall : trackWalls) {
            minX = std::min({minX, wall.first.x, wall.second.x});
            maxX = std::max({maxX, wall.first.x, wall.second.x});
            minY = std::min({minY, wall.first.y, wall.second.y});
            maxY = std::max({maxY, wall.first.y, wall.second.y});
        }
        if (maxX > minX && maxY > minY) {
            Vector2 center = { (minX + maxX) * 0.5f, (minY + maxY) * 0.5f };
            worldCamera.target.x += (center.x - worldCamera.target.x) * 0.08f;
            worldCamera.target.y += (center.y - worldCamera.target.y) * 0.08f;
            float viewW = (float)(Config::SCREEN_WIDTH - 250);
            float viewH = (float)Config::SCREEN_HEIGHT;
            float fitZoom = std::min((viewW - 80.0f) / (maxX - minX), (viewH - 80.0f) / (maxY - minY));
            targetZoom = std::clamp(fitZoom, 0.20f, 1.2f);
        }
    }

    // Smooth zoom dampening
    worldCamera.zoom += (targetZoom - worldCamera.zoom) * 0.1f;
}

void Simulation::DrawMinimap(const std::vector<Car*>& sortedCars) {
    if (trackWalls.empty()) return;

    // Tactical F1 Minimap HUD in lower-left viewport
    float mmX = 20.0f;
    float mmY = Config::SCREEN_HEIGHT - 175.0f;
    float mmW = 230.0f;
    float mmH = 155.0f;

    DrawRectangleRounded({ mmX, mmY, mmW, mmH }, 0.08f, 4, Fade(BLACK, 0.80f));
    DrawRectangleRoundedLinesEx({ mmX, mmY, mmW, mmH }, 0.08f, 4, 1.5f, Fade(SKYBLUE, 0.7f));
    DrawText("F1 CIRCUIT MAP", mmX + 10, mmY + 6, 11, SKYBLUE);

    float minX = 1e9f, maxX = -1e9f, minY = 1e9f, maxY = -1e9f;
    for (const auto& wall : trackWalls) {
        minX = std::min({minX, wall.first.x, wall.second.x});
        maxX = std::max({maxX, wall.first.x, wall.second.x});
        minY = std::min({minY, wall.first.y, wall.second.y});
        maxY = std::max({maxY, wall.first.y, wall.second.y});
    }

    float trackW = std::max(10.0f, maxX - minX);
    float trackH = std::max(10.0f, maxY - minY);
    float pad = 12.0f;
    float innerW = mmW - 2.0f * pad;
    float innerH = mmH - 24.0f - pad;
    float scale = std::min(innerW / trackW, innerH / trackH);

    float offsetX = mmX + pad + (innerW - trackW * scale) * 0.5f - minX * scale;
    float offsetY = mmY + 22.0f + (innerH - trackH * scale) * 0.5f - minY * scale;

    auto WorldToMinimap = [&](Vector2 p) -> Vector2 {
        return { p.x * scale + offsetX, p.y * scale + offsetY };
    };

    // Draw track walls miniaturized
    for (const auto& wall : trackWalls) {
        Vector2 p1 = WorldToMinimap(wall.first);
        Vector2 p2 = WorldToMinimap(wall.second);
        DrawLineV(p1, p2, Fade(WHITE, 0.6f));
    }

    // Start/finish line marker
    Vector2 sf = WorldToMinimap(startPosition);
    DrawCircleV(sf, 3.5f, YELLOW);

    // Cars on track
    if (!sortedCars.empty()) {
        for (int i = (int)sortedCars.size() - 1; i >= 0; i--) {
            const Car* car = sortedCars[i];
            if (car->isCrashed) continue;
            Vector2 mp = WorldToMinimap(car->position);
            if (i == 0) {
                DrawCircleV(mp, 4.0f, GOLD);
                DrawCircleLines(mp.x, mp.y, 4.0f, WHITE);
            } else if (i < 3) {
                DrawCircleV(mp, 2.5f, SKYBLUE);
            } else {
                DrawCircleV(mp, 1.5f, Fade(RED, 0.5f));
            }
        }
    } else if (!aiCar.isCrashed) {
        Vector2 aiP = WorldToMinimap(aiCar.position);
        DrawCircleV(aiP, 3.5f, RED);
        if (currentState == EXHIBITION && !playerCar.isCrashed) {
            Vector2 plP = WorldToMinimap(playerCar.position);
            DrawCircleV(plP, 3.5f, BLUE);
        }
    }

    // Current camera viewport indicator
    Vector2 camCenter = WorldToMinimap(worldCamera.target);
    float viewBoxW = ((Config::SCREEN_WIDTH - 250.0f) / worldCamera.zoom) * scale;
    float viewBoxH = (Config::SCREEN_HEIGHT / worldCamera.zoom) * scale;
    DrawRectangleLines(camCenter.x - viewBoxW * 0.5f, camCenter.y - viewBoxH * 0.5f, viewBoxW, viewBoxH, Fade(YELLOW, 0.45f));
}

void Simulation::DrawMenu() {
    BeginMode2D(worldCamera);
    DrawFinishLine(0.3f);
    DrawCheckpoints(0.3f);
    
    for (const auto& wall : trackWalls) {
        DrawLineEx(wall.first, wall.second, 6.0f, Fade(WHITE, 0.3f));
    }

    if (!aiCar.isCrashed) {
        DrawRectanglePro({ aiCar.position.x, aiCar.position.y, Config::CAR_HALF_LENGTH * 2.0f, Config::CAR_HALF_WIDTH * 2.0f }, { Config::CAR_HALF_LENGTH, Config::CAR_HALF_WIDTH }, aiCar.rotation, Fade(RED, 0.3f));
        for (int i = 0; i < 5; i++) {
            float rayAngle = (aiCar.rotation + Config::SENSOR_ANGLES[i]) * DEG2RAD;
            Vector2 actualRayEnd = { aiCar.position.x + cos(rayAngle) * aiCar.sensorDistances[i], aiCar.position.y + sin(rayAngle) * aiCar.sensorDistances[i] };
            DrawLineV(aiCar.position, actualRayEnd, Fade(GREEN, 0.15f));
            DrawCircleV(actualRayEnd, 3.0f, Fade(GREEN, 0.15f));
        }
    }
    EndMode2D();

    DrawRectangle(0, 0, Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, Fade(BLACK, 0.7f));

    std::string displayName = mapFiles[currentMapIndex];
    size_t pos = displayName.find_last_of('/');
    if(pos != std::string::npos) displayName = displayName.substr(pos+1);
    
    DrawText("GENETIC SIMULATOR", Config::SCREEN_WIDTH/2 - 250, 200, 40, WHITE);
    DrawText(TextFormat("Current Track: < %s >", displayName.c_str()), Config::SCREEN_WIDTH/2 - 200, 300, 25, SKYBLUE);
    DrawText("[ T ] TRAINING MODE (AI vs AI)", Config::SCREEN_WIDTH/2 - 200, 350, 20, LIGHTGRAY);
    DrawText("[ E ] EXHIBITION MODE (Player vs Best AI)", Config::SCREEN_WIDTH/2 - 200, 400, 20, LIGHTGRAY);
    DrawText("[ A ] TEST AI (Only watch best AI)", Config::SCREEN_WIDTH/2 - 200, 450, 20, LIGHTGRAY);
    DrawText("[ P ] GENERATE PROCEDURAL TRACK (Modular Tiles)", Config::SCREEN_WIDTH/2 - 200, 500, 20, YELLOW);
    if (!proceduralPoints.empty()) {
        DrawText("[ G ] SAVE CURRENT TRACK", Config::SCREEN_WIDTH/2 - 200, 550, 20, GREEN);
    }
}

void Simulation::DrawTraining() {
    BeginMode2D(worldCamera);
    DrawFinishLine(1.0f);
    if (showCheckpoints) {
        DrawCheckpoints(0.5f);
    }
    
    if (showTelemetry) {
        Telemetry::DrawHeatmap();
    }
    
    for (const auto& wall : trackWalls) {
        DrawLineEx(wall.first, wall.second, 6.0f, WHITE);
        Vector2 dir = {wall.second.x - wall.first.x, wall.second.y - wall.first.y};
        float length = sqrt(dir.x*dir.x + dir.y*dir.y);
        dir.x /= length; dir.y /= length;
        for(float d = 0; d < length; d += 20.0f) {
            if (((int)(d / 20.0f)) % 2 == 0) {
                Vector2 startP = {wall.first.x + dir.x * d, wall.first.y + dir.y * d};
                Vector2 endP = {wall.first.x + dir.x * fmin(d + 20.0f, length), wall.first.y + dir.y * fmin(d + 20.0f, length)};
                DrawLineEx(startP, endP, 6.0f, RED);
            }
        }
    }

    int aliveCount = 0;
    for (auto& car : population) {
        if (!car.isCrashed) {
            aliveCount++;
            Color carColor = (car.totalCheckPointsCrossed >= (int)trackCheckpoints.size()) ? GOLD : Fade(RED, 0.5f);
            DrawRectanglePro({ car.position.x, car.position.y, Config::CAR_HALF_LENGTH * 2.0f, Config::CAR_HALF_WIDTH * 2.0f }, { Config::CAR_HALF_LENGTH, Config::CAR_HALF_WIDTH }, car.rotation, carColor);
            if (aliveCount == 1) {
                for (int i = 0; i < 5; i++) {
                    float rayAngle = (car.rotation + Config::SENSOR_ANGLES[i]) * DEG2RAD;
                    Vector2 actualRayEnd = { car.position.x + cos(rayAngle) * car.sensorDistances[i], car.position.y + sin(rayAngle) * car.sensorDistances[i] };
                    DrawLineV(car.position, actualRayEnd, Fade(GREEN, 0.5f));
                    DrawCircleV(actualRayEnd, 3.0f, Fade(GREEN, 0.5f));
                }
            }
        }
    }
    EndMode2D();

    std::vector<Car*> sortedPop;
    sortedPop.reserve(population.size());
    for (auto& car : population) {
        sortedPop.push_back(&car);
    }
    std::sort(sortedPop.begin(), sortedPop.end(), [](const Car* a, const Car* b) { return a->fitness > b->fitness; });

    constexpr int UI_PANEL_WIDTH = 250;
    DrawRectangle(Config::SCREEN_WIDTH - UI_PANEL_WIDTH, 0, UI_PANEL_WIDTH, Config::SCREEN_HEIGHT, Fade(BLACK, 0.85f));
    DrawText(TextFormat("Generation: %d (Track %d/%d)", generationCount, currentEvaluationTrack + 1, (int)trainingTracks.size()), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 12, 17, WHITE);
    
    const char* phaseStr = "PURE PROCEDURAL F1 (6 CIRCUITS)";
    Color phaseColor = GREEN;
    DrawText(phaseStr, Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 32, 11, phaseColor);

    DrawText(TextFormat("Time: %d / %d", generationTimer, Config::MAX_GENERATION_TIME), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 48, 16, WHITE);
    DrawText(TextFormat("Alive: %d / %d", aliveCount, Config::POPULATION_SIZE), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 68, 16, WHITE);
    DrawText(TextFormat("Speed: %s", (simSpeed == 1) ? "NORMAL" : "MAX (x50)"), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 88, 14, (simSpeed == 1) ? GREEN : RED);
    DrawText(TextFormat("Mutation: %d %% (step: %.2f)", Config::MUTATION, Config::MUTATION_SIGMA), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 106, 14, YELLOW);
    
    if (!sortedPop.empty()) {
        float leaderBestSec = (sortedPop[0]->bestLapTicks > 0) ? ((float)sortedPop[0]->bestLapTicks / 60.0f) : 0.0f;
        DrawText(TextFormat("Laps: %d | Best: %.2fs", sortedPop[0]->lapsCompleted, leaderBestSec), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 124, 13, (sortedPop[0]->bestLapTicks > 0) ? GREEN : LIGHTGRAY);
    }

    DrawText("[SPACE] Change speed", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 140, 10, LIGHTGRAY);
    DrawText("[M] Return to Menu", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 152, 10, LIGHTGRAY);
    DrawText("[C] Toggle Checkpoints", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 164, 10, LIGHTGRAY);
    DrawText("[H] Telemetry & Heatmap", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 176, 10, LIGHTGRAY);
    const char* camName = (cameraMode == CAM_FOLLOW_LEADER) ? "LEADER CHASE" : (cameraMode == CAM_OVERVIEW ? "OVERVIEW" : "MANUAL");
    DrawText(TextFormat("[V] Cam: %s", camName), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 188, 10, SKYBLUE);

    DrawText("TOP 10 FITNESS", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 203, 17, YELLOW);
    for (int i = 0; i < 10 && i < (int)sortedPop.size(); i++) {
        Color rowColor = (sortedPop[i]->isCrashed) ? GRAY : WHITE;
        if (i == 0) rowColor = GOLD;
        DrawText(TextFormat("%d. Fit: %.0f (L:%d)", i + 1, sortedPop[i]->fitness, sortedPop[i]->lapsCompleted), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 224 + (i * 24), 16, rowColor);
    }

    if (!sortedPop.empty()) {
        DrawText("NEURAL NETWORK (Leader)", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 490, 15, SKYBLUE);
        
        Brain& bestBrain = sortedPop[0]->brain;
        int startY = 510;
        int endY = 750;
        int layerX[3] = {
            Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 30,
            Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 125,
            Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 220
        };
        int nodesPerLayer[3] = { INPUT_NODES, HIDDEN_NODES, OUTPUT_NODES };
        
        std::vector<Vector2> nodePos[3];
        for (int l = 0; l < 3; l++) {
            int nodes = nodesPerLayer[l];
            float spacing = (endY - startY) / (float)(nodes + 1);
            for (int n = 0; n < nodes; n++) {
                nodePos[l].push_back({ (float)layerX[l], startY + spacing * (n + 1) });
            }
        }
        
        for (int i = 0; i < INPUT_NODES; i++) {
            for (int j = 0; j < HIDDEN_NODES; j++) {
                float weight = bestBrain.weights_input_hidden[j][i];
                float alpha = fmin(fabs(weight), 1.0f);
                Color edgeColor = (weight > 0) ? Fade(GREEN, alpha) : Fade(RED, alpha);
                DrawLineV(nodePos[0][i], nodePos[1][j], edgeColor);
            }
        }
        
        for (int i = 0; i < HIDDEN_NODES; i++) {
            for (int j = 0; j < OUTPUT_NODES; j++) {
                float weight = bestBrain.weights_hidden_output[j][i];
                float alpha = fmin(fabs(weight), 1.0f);
                Color edgeColor = (weight > 0) ? Fade(GREEN, alpha) : Fade(RED, alpha);
                DrawLineV(nodePos[1][i], nodePos[2][j], edgeColor);
            }
        }
        
        for (int l = 0; l < 3; l++) {
            for (int n = 0; n < nodesPerLayer[l]; n++) {
                float val = 0.0f;
                if (l == 0) {
                    val = bestBrain.last_input[n];
                } else if (l == 1) {
                    val = bestBrain.last_hidden[n];
                } else if (l == 2) {
                    val = bestBrain.last_output[n];
                }
                
                val = fmax(-1.0f, fmin(1.0f, val));
                Color nodeColor = (val > 0) ? Fade(GREEN, val) : Fade(RED, -val);
                if (fabs(val) < 0.1f) nodeColor = DARKGRAY;
                
                DrawCircleV(nodePos[l][n], 6.0f, nodeColor);
                DrawCircleLines(nodePos[l][n].x, nodePos[l][n].y, 6.0f, LIGHTGRAY);
            }
        }
    }
    
    DrawMinimap(sortedPop);

    if (showTelemetry) {
        Telemetry::DrawDashboard(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT);
    }
}

void Simulation::DrawExhibition() {
    BeginMode2D(worldCamera);
    DrawFinishLine(1.0f);
    DrawCheckpoints(0.5f);
    
    for (const auto& wall : trackWalls) {
        DrawLineEx(wall.first, wall.second, 6.0f, WHITE);
    }

    if (!aiCar.isCrashed) DrawRectanglePro({ aiCar.position.x, aiCar.position.y, Config::CAR_HALF_LENGTH * 2.0f, Config::CAR_HALF_WIDTH * 2.0f }, { Config::CAR_HALF_LENGTH, Config::CAR_HALF_WIDTH }, aiCar.rotation, RED);
    if (!playerCar.isCrashed) DrawRectanglePro({ playerCar.position.x, playerCar.position.y, Config::CAR_HALF_LENGTH * 2.0f, Config::CAR_HALF_WIDTH * 2.0f }, { Config::CAR_HALF_LENGTH, Config::CAR_HALF_WIDTH }, playerCar.rotation, BLUE);
    EndMode2D();

    DrawMinimap({});

    if (exhibitionResult == 1) {
        DrawText("YOU BEAT THE AI!", Config::SCREEN_WIDTH/2 - 200, 200, 40, GREEN);
        DrawText("Press R for revenge", Config::SCREEN_WIDTH/2 - 150, 250, 20, WHITE);
    } else if (exhibitionResult == 2) {
        DrawText("THE AI DESTROYED YOU", Config::SCREEN_WIDTH/2 - 200, 200, 40, RED);
        DrawText("Press R to retry", Config::SCREEN_WIDTH/2 - 150, 250, 20, WHITE);
    }

    DrawText("[M] Return to Main Menu", 20, 20, 20, LIGHTGRAY);
}

void Simulation::UpdateTestAI() {
    if (IsKeyPressed(KEY_M)) currentState = MENU;
    if (IsKeyPressed(KEY_R) && aiCar.isCrashed) {
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
    }

    if (!aiCar.isCrashed) {
        float aiAccelerate = 0.0f, aiTurn = 0.0f;
        aiCar.Think(aiAccelerate, aiTurn);
        aiCar.UpdatePhysics(aiAccelerate, aiTurn, spatialGrid, 0, trackCheckpoints);
    }
}

void Simulation::DrawTestAI() {
    BeginMode2D(worldCamera);
    DrawFinishLine(1.0f);
    DrawCheckpoints(0.5f);
    
    for (const auto& wall : trackWalls) {
        DrawLineEx(wall.first, wall.second, 6.0f, WHITE);
    }

    if (!aiCar.isCrashed) {
        DrawRectanglePro({ aiCar.position.x, aiCar.position.y, Config::CAR_HALF_LENGTH * 2.0f, Config::CAR_HALF_WIDTH * 2.0f }, { Config::CAR_HALF_LENGTH, Config::CAR_HALF_WIDTH }, aiCar.rotation, RED);
        for (int i = 0; i < 5; i++) {
            float rayAngle = (aiCar.rotation + Config::SENSOR_ANGLES[i]) * DEG2RAD;
            Vector2 actualRayEnd = { aiCar.position.x + cos(rayAngle) * aiCar.sensorDistances[i], aiCar.position.y + sin(rayAngle) * aiCar.sensorDistances[i] };
            DrawLineV(aiCar.position, actualRayEnd, Fade(GREEN, 0.5f));
            DrawCircleV(actualRayEnd, 3.0f, Fade(GREEN, 0.5f));
        }
    }
    EndMode2D();

    DrawMinimap({});

    if (aiCar.isCrashed) {
        DrawText("THE AI HAS CRASHED", Config::SCREEN_WIDTH/2 - 150, 200, 30, RED);
        DrawText("Press R to restart", Config::SCREEN_WIDTH/2 - 100, 250, 20, WHITE);
    }

    DrawText("[M] Return to Main Menu", 20, 20, 20, LIGHTGRAY);
}

void Simulation::BuildSpatialGrid(){
    spatialGrid.clear();

    for(int i = 0; i < trackWalls.size(); i++){
        int grid1X = (int)std::floor(trackWalls[i].first.x  / TrackManager::GRID_CELL_SIZE);
        int grid1Y = (int)std::floor(trackWalls[i].first.y  / TrackManager::GRID_CELL_SIZE);
        int grid2X = (int)std::floor(trackWalls[i].second.x / TrackManager::GRID_CELL_SIZE);
        int grid2Y = (int)std::floor(trackWalls[i].second.y / TrackManager::GRID_CELL_SIZE);
        
        int startX = std::min(grid1X, grid2X);
        int startY = std::min(grid1Y, grid2Y);
        int endX = std::max(grid1X, grid2X);
        int endY = std::max(grid1Y, grid2Y);
        
        for(int x = startX; x <= endX; x++){
            for(int y = startY; y <= endY; y++){
                uint64_t key = TrackManager::GetGridKey(x, y);
                spatialGrid[key].push_back(trackWalls[i]);
            }
        }
    }
}

void Simulation::InitTrainingTracks() {
    // Generate authentic F1-scale benchmark tracks for the curriculum phases (2400x1600 canvas)
    baseTrackFacil = GenerateProceduralTrainingTrack(0);   // Archetype 0: Monza Cathedral (Fast, open sweepers)
    baseTrackMedio = GenerateProceduralTrainingTrack(1);   // Archetype 1: Silverstone / Spa (Multi-sector undulating flow)
    baseTrackDificil = GenerateProceduralTrainingTrack(2); // Archetype 2: Interlagos / Spielberg (Technical horseshoe)
}

Simulation::TrainingTrack Simulation::GenerateProceduralTrainingTrack(int fixedArchetype) {
    std::vector<Vector2> pts = ModularTrackGenerator::GenerateProceduralCenterPoints(fixedArchetype);
    for (int retry = 0; retry < 5 && pts.empty(); retry++) {
        pts = ModularTrackGenerator::GenerateProceduralCenterPoints(fixedArchetype);
    }
    if (pts.size() < 12) {
        pts = ModularTrackGenerator::GenerateArchetypeFallback(fixedArchetype);
    }

    std::vector<Vector2> denseLine = TrackManager::GenerateSplinePoints(pts, PROCEDURAL_SPLINE_SEGMENTS);
    if (denseLine.size() < 10) {
        pts = ModularTrackGenerator::GenerateArchetypeFallback(fixedArchetype);
        denseLine = TrackManager::GenerateSplinePoints(pts, PROCEDURAL_SPLINE_SEGMENTS);
    }

    TrainingTrack tt;
    TrackManager::CalculateStartGrid(pts, tt.startPos, tt.startRot);

    // Snap startPos to nearest dense center line point
    int snapIdx = 0;
    float minDist = FLT_MAX;
    for (int i = 0; i < (int)denseLine.size(); i++) {
        float dx = denseLine[i].x - tt.startPos.x;
        float dy = denseLine[i].y - tt.startPos.y;
        float d = dx*dx + dy*dy;
        if (d < minDist) { minDist = d; snapIdx = i; }
    }
    tt.startPos = denseLine[snapIdx];
    int denseN = (int)denseLine.size();
    Vector2 prev = denseLine[(snapIdx - 1 + denseN) % denseN];
    Vector2 next = denseLine[(snapIdx + 1) % denseN];
    tt.startRot = atan2(next.y - prev.y, next.x - prev.x) * (180.0f / PI);

    TrackManager::GenerateBordersFromCenterLine(denseLine, PROCEDURAL_TRACK_WIDTH, tt.walls, tt.checkpoints, tt.startPos);
    return tt;
}

void Simulation::SetupCurriculumForGeneration(int gen) {
    (void)gen;
    trainingTracks.clear();
    static const char* ARCHETYPE_NAMES[6] = {
        "Monza Cathedral (High-Speed Straights & Chicanes)",
        "Silverstone / Spa (Tri-Sector Infield Sweepers)",
        "Interlagos Horseshoe (Technical Amphitheater)",
        "Suzuka / COTA (Linked Rhythm Esses & Carousel)",
        "Montreal / Albert Park (Perimeter Straights & Chicanes)",
        "Stochastic Modular Hybrid (High-Entropy Geometric Assembly)"
    };
    std::cout << "[Curriculum Gen " << generationCount << "] Generating 6 Distinct Procedural Circuits:" << std::endl;
    // Pure procedural multi-circuit evaluation from Generation 0 (6 modular tracks per generation)
    // Tracks 0..4 cover varied F1 macro-skeletons (Monza, Silverstone, Interlagos, Suzuka, Montreal)
    // and Track 5 evaluates a stochastic macro-topology.
    for (int t = 0; t < Config::TRACKS_PER_GENERATION; t++) {
        int topology = (t < 5) ? t : -1;
        TrainingTrack tt = GenerateProceduralTrainingTrack(topology);
        trainingTracks.push_back(tt);
        int nameIdx = (topology >= 0 && topology < 5) ? topology : 5;
        std::cout << "  Track " << t + 1 << "/6: " << ARCHETYPE_NAMES[nameIdx] 
                  << " (" << tt.checkpoints.size() << " checkpoints, " << tt.walls.size() << " wall segments)" << std::endl;
    }
}

void Simulation::SetTrainingTrack(int trackIndex) {
    if (trackIndex >= 0 && trackIndex < (int)trainingTracks.size()) {
        trackWalls = trainingTracks[trackIndex].walls;
        trackCheckpoints = trainingTracks[trackIndex].checkpoints;
        startPosition = trainingTracks[trackIndex].startPos;
        startRotation = trainingTracks[trackIndex].startRot;
        BuildSpatialGrid();
        Telemetry::ResetHeatmap();
    }
}