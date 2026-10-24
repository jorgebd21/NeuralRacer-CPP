#include "Simulation.h"
#include "Config.h"
#include "Evolution.h"
#include "TrackManager.h"
#include "TrackGenerator.h"
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
    benchmarkTopFitness(0.0f)
{
}

constexpr int SIM_TARGET_FPS = 60;
constexpr int PROCEDURAL_SPLINE_SEGMENTS = 50;
constexpr float PROCEDURAL_TRACK_WIDTH = 35.0f;

void Simulation::Init() {
    if(!isHeadless){
        InitWindow(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, "Genetic Simulator - Autonomous AI");
        SetTargetFPS(SIM_TARGET_FPS);
    }
    
    Telemetry::Init();

    mapFiles = TrackManager::ScanMapFiles();
    if (mapFiles.empty()) {
        std::cerr << "Warning: No track files found in data/tracks/. Generating a procedural track." << std::endl;
        proceduralPoints = TrackGenerator::GenerateProceduralCenterPoints();
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
        proceduralPoints = TrackGenerator::GenerateProceduralCenterPoints();
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
            TrackGenerator::SaveTrackToFile(proceduralPoints, filename);
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
                
                int phase = (generationCount < Config::CURRICULUM_PHASE_1_END) ? 1 :
                            (generationCount < Config::CURRICULUM_PHASE_2_END) ? 2 : 3;
                float totalTopFit = sortedPop[0]->fitness;
                float procFit = (phase == 1) ? 0.0f : std::max(0.0f, totalTopFit - benchmarkTopFitness);
                float eliteAvgSpeed = sortedPop[0]->GetAverageSpeed();
                float eliteSmoothness = std::clamp(1.0f - (std::abs(sortedPop[0]->angularVelocity) / Config::CAR_TURN_SPEED), 0.0f, 1.0f);

                Telemetry::RecordGeneration(generationCount, population, Config::MAX_GENERATION_TIME,
                                            phase, benchmarkTopFitness, procFit, generationTotalLaps,
                                            eliteAvgSpeed, eliteSmoothness, Config::MUTATION);
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

void Simulation::DrawMenu() {
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

    DrawRectangle(0, 0, Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, Fade(BLACK, 0.7f));

    std::string displayName = mapFiles[currentMapIndex];
    size_t pos = displayName.find_last_of('/');
    if(pos != std::string::npos) displayName = displayName.substr(pos+1);
    
    DrawText("GENETIC SIMULATOR", Config::SCREEN_WIDTH/2 - 250, 200, 40, WHITE);
    DrawText(TextFormat("Current Track: < %s >", displayName.c_str()), Config::SCREEN_WIDTH/2 - 200, 300, 25, SKYBLUE);
    DrawText("[ T ] TRAINING MODE (AI vs AI)", Config::SCREEN_WIDTH/2 - 200, 350, 20, LIGHTGRAY);
    DrawText("[ E ] EXHIBITION MODE (Player vs Best AI)", Config::SCREEN_WIDTH/2 - 200, 400, 20, LIGHTGRAY);
    DrawText("[ A ] TEST AI (Only watch best AI)", Config::SCREEN_WIDTH/2 - 200, 450, 20, LIGHTGRAY);
    DrawText("[ P ] GENERATE PROCEDURAL TRACK", Config::SCREEN_WIDTH/2 - 200, 500, 20, YELLOW);
    if (!proceduralPoints.empty()) {
        DrawText("[ G ] SAVE CURRENT TRACK", Config::SCREEN_WIDTH/2 - 200, 550, 20, GREEN);
    }
}

void Simulation::DrawTraining() {
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

    constexpr int UI_PANEL_WIDTH = 250;
    DrawRectangle(Config::SCREEN_WIDTH - UI_PANEL_WIDTH, 0, UI_PANEL_WIDTH, Config::SCREEN_HEIGHT, Fade(BLACK, 0.85f));
    DrawText(TextFormat("Generation: %d (Track %d/%d)", generationCount, currentEvaluationTrack + 1, (int)trainingTracks.size()), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 12, 17, WHITE);
    
    const char* phaseStr = (generationCount < Config::CURRICULUM_PHASE_1_END) ? "PHASE 1: INDUCTION" :
                           (generationCount < Config::CURRICULUM_PHASE_2_END) ? "PHASE 2: CONSOLIDATION" :
                           "PHASE 3: PROCEDURAL GEN";
    Color phaseColor = (generationCount < Config::CURRICULUM_PHASE_1_END) ? YELLOW :
                       (generationCount < Config::CURRICULUM_PHASE_2_END) ? SKYBLUE : GREEN;
    DrawText(phaseStr, Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 32, 11, phaseColor);

    DrawText(TextFormat("Time: %d / %d", generationTimer, Config::MAX_GENERATION_TIME), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 48, 16, WHITE);
    DrawText(TextFormat("Alive: %d / %d", aliveCount, Config::POPULATION_SIZE), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 68, 16, WHITE);
    DrawText(TextFormat("Speed: %s", (simSpeed == 1) ? "NORMAL" : "MAX (x50)"), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 90, 14, (simSpeed == 1) ? GREEN : RED);
    DrawText(TextFormat("Mutation: %d %%", Config::MUTATION), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 110, 14, YELLOW);
    DrawText("[SPACE] Change speed", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 130, 10, LIGHTGRAY);
    DrawText("[M] Return to Menu", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 145, 10, LIGHTGRAY);
    DrawText("[C] Toggle Checkpoints", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 160, 10, LIGHTGRAY);
    DrawText("[H] Telemetry & Heatmap", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 175, 10, LIGHTGRAY);

    DrawText("TOP 10 FITNESS", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 195, 18, YELLOW);
    std::vector<Car*> sortedPop;
    sortedPop.reserve(population.size());
    for (auto& car : population) {
        sortedPop.push_back(&car);
    }
    std::sort(sortedPop.begin(), sortedPop.end(), [](const Car* a, const Car* b) { return a->fitness > b->fitness; });
    for (int i = 0; i < 10 && i < (int)sortedPop.size(); i++) {
        Color rowColor = (sortedPop[i]->isCrashed) ? GRAY : WHITE;
        if (i == 0) rowColor = GOLD;
        DrawText(TextFormat("%d. Fit: %.1f", i + 1, sortedPop[i]->fitness), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 220 + (i * 25), 18, rowColor);
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
    
    if (showTelemetry) {
        Telemetry::DrawDashboard(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT);
    }
}

void Simulation::DrawExhibition() {
    DrawFinishLine(1.0f);
    DrawCheckpoints(0.5f);
    
    for (const auto& wall : trackWalls) {
        DrawLineEx(wall.first, wall.second, 6.0f, WHITE);
    }

    if (!aiCar.isCrashed) DrawRectanglePro({ aiCar.position.x, aiCar.position.y, Config::CAR_HALF_LENGTH * 2.0f, Config::CAR_HALF_WIDTH * 2.0f }, { Config::CAR_HALF_LENGTH, Config::CAR_HALF_WIDTH }, aiCar.rotation, RED);
    if (!playerCar.isCrashed) DrawRectanglePro({ playerCar.position.x, playerCar.position.y, Config::CAR_HALF_LENGTH * 2.0f, Config::CAR_HALF_WIDTH * 2.0f }, { Config::CAR_HALF_LENGTH, Config::CAR_HALF_WIDTH }, playerCar.rotation, BLUE);

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
    } else {
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
    // 1. Load baseTrackFacil (Phase 1 induction & Phase 2 consolidation)
    std::string path0 = "data/tracks/pista_facil.json";
    if (std::ifstream(path0.c_str()).good()) {
        TrackManager::LoadTrackFromFile(path0, baseTrackFacil.walls, baseTrackFacil.startPos, baseTrackFacil.startRot, baseTrackFacil.checkpoints);
    } else if (!trackWalls.empty()) {
        baseTrackFacil.walls = trackWalls;
        baseTrackFacil.checkpoints = trackCheckpoints;
        baseTrackFacil.startPos = startPosition;
        baseTrackFacil.startRot = startRotation;
    } else {
        TrackManager::LoadTrackFromFile(trackFile, baseTrackFacil.walls, baseTrackFacil.startPos, baseTrackFacil.startRot, baseTrackFacil.checkpoints);
    }

    // 2. Load baseTrackMedio (Phase 2 consolidation & Phase 3 benchmark)
    std::string path1 = "data/tracks/pista_medio.json";
    if (std::ifstream(path1.c_str()).good()) {
        TrackManager::LoadTrackFromFile(path1, baseTrackMedio.walls, baseTrackMedio.startPos, baseTrackMedio.startRot, baseTrackMedio.checkpoints);
    } else {
        baseTrackMedio = baseTrackFacil;
    }

    // 3. Load baseTrackDificil (Phase 2 advanced consolidation)
    std::string path2 = "data/tracks/pista_dificil.json";
    if (std::ifstream(path2.c_str()).good()) {
        TrackManager::LoadTrackFromFile(path2, baseTrackDificil.walls, baseTrackDificil.startPos, baseTrackDificil.startRot, baseTrackDificil.checkpoints);
    } else {
        baseTrackDificil = baseTrackMedio;
    }
}

Simulation::TrainingTrack Simulation::GenerateProceduralTrainingTrack() {
    TrainingTrack tt;
    std::vector<Vector2> pts = TrackGenerator::GenerateProceduralCenterPoints();
    std::vector<Vector2> denseLine = TrackManager::GenerateSplinePoints(pts, PROCEDURAL_SPLINE_SEGMENTS);
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
    trainingTracks.clear();
    if (gen < Config::CURRICULUM_PHASE_1_END) {
        // Phase 1: Inducción (pista_facil exclusivamente)
        trainingTracks.push_back(baseTrackFacil);
    } else if (gen < Config::CURRICULUM_PHASE_2_END) {
        // Phase 2: Consolidación multi-pista determinista (Fácil -> Media -> Difícil)
        trainingTracks.push_back(baseTrackFacil);
        trainingTracks.push_back(baseTrackMedio);
        trainingTracks.push_back(baseTrackDificil);
    } else {
        // Phase 3: Generalización continua (1 Benchmark de control + 2 Pistas procedurales dinámicas)
        trainingTracks.push_back(baseTrackMedio);
        trainingTracks.push_back(GenerateProceduralTrainingTrack());
        trainingTracks.push_back(GenerateProceduralTrainingTrack());
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