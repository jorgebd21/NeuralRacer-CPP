#include "Simulation.h"
#include "Config.h"
#include "Evolution.h"
#include "TrackManager.h"
#include "TrackGenerator.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>

Simulation::Simulation() : 
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
    exhibitionResult(0)
{
}

constexpr int SIM_TARGET_FPS = 60;
constexpr int PROCEDURAL_SPLINE_SEGMENTS = 50;
constexpr float PROCEDURAL_TRACK_WIDTH = 65.0f;

void Simulation::Init() {
    InitWindow(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, "Simulador Genético - IA Autónoma");
    SetTargetFPS(SIM_TARGET_FPS);

    mapFiles = TrackManager::ScanMapFiles();
    trackFile = mapFiles[currentMapIndex];
    
    TrackManager::LoadTrackFromFile(trackFile, trackWalls, startPosition, startRotation);
    if (trackWalls.empty()) trackWalls.push_back({{100, 100}, {900, 100}});

    for (int i = 0; i < Config::POPULATION_SIZE; i++) {
        population.push_back(Car(startPosition.x, startPosition.y, startRotation));
    }
    Evolution::CargarMejoresCerebros(population);
    
    playerCar = Car(startPosition.x, startPosition.y, startRotation);
    aiCar = Car(startPosition.x, startPosition.y, startRotation);
}

void Simulation::Run() {
    Init();
    while (!WindowShouldClose()) {
        Update();
        Draw();
    }
    CloseWindow();
}

void Simulation::Update() {
    // Máquina de estados principal: delega la actualización lógica según el modo actual
    if (currentState == MENU) UpdateMenu();
    else if (currentState == TRAINING) UpdateTraining();
    else if (currentState == EXHIBITION) UpdateExhibition();
    else if (currentState == TEST_AI) UpdateTestAI();
}

void Simulation::UpdateMenu() {
    if (IsKeyPressed(KEY_T)) currentState = TRAINING;
    if (IsKeyPressed(KEY_E)) {
        playerCar.Reset(startPosition.x, startPosition.y, startRotation);
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
        
        std::vector<Car> temp(1, Car(startPosition.x, startPosition.y, startRotation));
        if (Evolution::CargarMejoresCerebros(temp)) {
            aiCar = temp[0];
        }
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
        exhibitionResult = 0;
        currentState = EXHIBITION;
    }
    if (IsKeyPressed(KEY_A)) {
        std::vector<Car> temp(1, Car(startPosition.x, startPosition.y, startRotation));
        if (Evolution::CargarMejoresCerebros(temp)) {
            aiCar = temp[0];
        }
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
        exhibitionResult = 0;
        currentState = TEST_AI;
    }
    if (IsKeyPressed(KEY_P)) {
        puntosProcedurales = TrackGenerator::GenerateProceduralCenterPoints();
        std::vector<Vector2> denseCenterLine = TrackManager::GenerateSplinePoints(puntosProcedurales, PROCEDURAL_SPLINE_SEGMENTS);
        trackWalls.clear();
        TrackManager::GenerateBordersFromCenterLine(denseCenterLine, PROCEDURAL_TRACK_WIDTH, trackWalls);
        TrackManager::CalculateStartGrid(puntosProcedurales, startPosition, startRotation);
        
        for (auto& car : population) car.Reset(startPosition.x, startPosition.y, startRotation);
        playerCar.Reset(startPosition.x, startPosition.y, startRotation);
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
    }
    if (IsKeyPressed(KEY_LEFT)) {
        currentMapIndex--;
        if (currentMapIndex < 0) currentMapIndex = mapFiles.size() - 1;
        trackFile = mapFiles[currentMapIndex];
        trackWalls.clear();
        TrackManager::LoadTrackFromFile(trackFile, trackWalls, startPosition, startRotation);
        for (auto& car : population) car.Reset(startPosition.x, startPosition.y, startRotation);
    }
    if (IsKeyPressed(KEY_RIGHT)) {
        currentMapIndex++;
        if (currentMapIndex >= (int)mapFiles.size()) currentMapIndex = 0;
        trackFile = mapFiles[currentMapIndex];
        trackWalls.clear();
        TrackManager::LoadTrackFromFile(trackFile, trackWalls, startPosition, startRotation);
        for (auto& car : population) car.Reset(startPosition.x, startPosition.y, startRotation);
    }
    if (IsKeyPressed(KEY_G)) {
        if (!puntosProcedurales.empty()) {
            int counter = 1;
            std::string filename;
            while (true) {
                filename = "data/tracks/pista_procedural" + std::to_string(counter) + ".txt";
                std::ifstream f(filename.c_str());
                if (!f.good()) break;
                counter++;
            }
            TrackGenerator::SaveTrackToFile(puntosProcedurales, filename);
        }
    }
}

constexpr int FAST_FORWARD_MULTIPLIER = 50;

void Simulation::UpdateTraining() {
    // Permite acelerar la simulación para entrenar más rápido
    if (IsKeyPressed(KEY_SPACE)) simSpeed = (simSpeed == 1) ? FAST_FORWARD_MULTIPLIER : 1;
    if (IsKeyPressed(KEY_M)) currentState = MENU;

    // Ejecutamos la lógica varias veces por frame si estamos en modo cámara rápida
    for (int s = 0; s < simSpeed; s++) {
        bool allCrashed = true;
        for (auto& car : population) {
            if (!car.isCrashed) {
                allCrashed = false;
                
                // 1. El cerebro decide qué hacer basándose en los sensores
                float inputAcelerar = 0.0f, inputGiro = 0.0f;
                car.brain.Evaluate(car.sensorDistances, car.speed, inputAcelerar, inputGiro);
                
                // 2. El coche se mueve según la decisión y comprueba si ha chocado
                car.UpdatePhysics(inputAcelerar, inputGiro, trackWalls, generationTimer);
            }
        }
        
        generationTimer++;
        
        // Si todos los coches han muerto o se acabó el tiempo máximo por generación
        if (allCrashed || generationTimer >= Config::MAX_GENERATION_TIME) {
            currentEvaluationTrack++;
            if (currentEvaluationTrack >= 3) {
                Evolution::EvolvePopulation(population, startPosition, startRotation);
                generationTimer = 0;
                generationCount++;
                currentEvaluationTrack = 0;
            } else {
                for (auto& car : population) {
                    car.accumulatedFitness = car.fitness;
                }
                
                puntosProcedurales = TrackGenerator::GenerateProceduralCenterPoints();
                std::vector<Vector2> denseCenterLine = TrackManager::GenerateSplinePoints(puntosProcedurales, PROCEDURAL_SPLINE_SEGMENTS);
                trackWalls.clear();
                TrackManager::GenerateBordersFromCenterLine(denseCenterLine, PROCEDURAL_TRACK_WIDTH, trackWalls);
                TrackManager::CalculateStartGrid(puntosProcedurales, startPosition, startRotation);
                
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
        float playerAcelerar = 0.0f, playerGiro = 0.0f;
        if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) playerAcelerar = 1.0f;
        if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) playerAcelerar = -1.0f;
        if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) playerGiro = 1.0f;
        if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) playerGiro = -1.0f;
        playerCar.UpdatePhysics(playerAcelerar, playerGiro, trackWalls, 0);

        float aiAcelerar = 0.0f, aiGiro = 0.0f;
        aiCar.brain.Evaluate(aiCar.sensorDistances, aiCar.speed, aiAcelerar, aiGiro);
        aiCar.UpdatePhysics(aiAcelerar, aiGiro, trackWalls, 0);

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

void Simulation::DrawMenu() {
    std::string displayName = mapFiles[currentMapIndex];
    size_t pos = displayName.find_last_of('/');
    if(pos != std::string::npos) displayName = displayName.substr(pos+1);
    
    DrawText("SIMULADOR GENETICO", Config::SCREEN_WIDTH/2 - 250, 200, 40, WHITE);
    DrawText(TextFormat("Pista Actual: < %s >", displayName.c_str()), Config::SCREEN_WIDTH/2 - 200, 300, 25, SKYBLUE);
    DrawText("[ T ] MODO ENTRENAMIENTO (IA vs IA)", Config::SCREEN_WIDTH/2 - 200, 350, 20, LIGHTGRAY);
    DrawText("[ E ] MODO EXHIBICION (Jugador vs Mejor IA)", Config::SCREEN_WIDTH/2 - 200, 400, 20, LIGHTGRAY);
    DrawText("[ A ] PROBAR IA (Solo ver mejor IA)", Config::SCREEN_WIDTH/2 - 200, 450, 20, LIGHTGRAY);
    DrawText("[ P ] GENERAR PISTA PROCEDURAL", Config::SCREEN_WIDTH/2 - 200, 500, 20, YELLOW);
    if (!puntosProcedurales.empty()) {
        DrawText("[ G ] GUARDAR PISTA ACTUAL", Config::SCREEN_WIDTH/2 - 200, 550, 20, GREEN);
    }
}

constexpr int FINISH_LINE_X = 450;
constexpr int FINISH_LINE_START_Y = 600;
constexpr int FINISH_LINE_BLOCK_SIZE = 10;
constexpr int FINISH_LINE_BLOCK_COUNT = 10;

void Simulation::DrawTraining() {
    for(int i=0; i<FINISH_LINE_BLOCK_COUNT; i++) {
        DrawRectangle(FINISH_LINE_X, FINISH_LINE_START_Y + i * FINISH_LINE_BLOCK_SIZE, FINISH_LINE_BLOCK_SIZE, FINISH_LINE_BLOCK_SIZE, (i % 2 == 0) ? WHITE : BLACK);
        DrawRectangle(FINISH_LINE_X + FINISH_LINE_BLOCK_SIZE, FINISH_LINE_START_Y + i * FINISH_LINE_BLOCK_SIZE, FINISH_LINE_BLOCK_SIZE, FINISH_LINE_BLOCK_SIZE, (i % 2 == 0) ? BLACK : WHITE);
    }
    
    for (auto wall : trackWalls) {
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
            DrawRectanglePro({ car.position.x, car.position.y, 20.0f, 10.0f }, { 10.0f, 5.0f }, car.rotation, Fade(RED, 0.5f));
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
    DrawText(TextFormat("Generacion: %d (Pista %d/3)", generationCount, currentEvaluationTrack + 1), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 20, 20, WHITE);
    DrawText(TextFormat("Tiempo: %d / %d", generationTimer, Config::MAX_GENERATION_TIME), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 50, 20, WHITE);
    DrawText(TextFormat("Vivos: %d / %d", aliveCount, Config::POPULATION_SIZE), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 80, 20, WHITE);
    DrawText(TextFormat("Velocidad: %s", (simSpeed == 1) ? "NORMAL" : "MAX (x50)"), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 110, 15, (simSpeed == 1) ? GREEN : RED);
    DrawText("[ESPACIO] Cambiar vel", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 130, 10, LIGHTGRAY);
    DrawText("[M] Volver al Menu", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 145, 10, LIGHTGRAY);

    DrawText("TOP 10 FITNESS", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 180, 20, YELLOW);
    std::vector<Car> sortedPop = population;
    std::sort(sortedPop.begin(), sortedPop.end(), [](const Car& a, const Car& b) { return a.fitness > b.fitness; });
    for (int i = 0; i < 10 && i < (int)sortedPop.size(); i++) {
        Color rowColor = (sortedPop[i].isCrashed) ? GRAY : WHITE;
        if (i == 0) rowColor = GOLD;
        DrawText(TextFormat("%d. Fit: %.1f", i + 1, sortedPop[i].fitness), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 210 + (i * 25), 18, rowColor);
    }
}

void Simulation::DrawExhibition() {
    for(int i=0; i<FINISH_LINE_BLOCK_COUNT; i++) {
        DrawRectangle(FINISH_LINE_X, FINISH_LINE_START_Y + i * FINISH_LINE_BLOCK_SIZE, FINISH_LINE_BLOCK_SIZE, FINISH_LINE_BLOCK_SIZE, (i % 2 == 0) ? WHITE : BLACK);
        DrawRectangle(FINISH_LINE_X + FINISH_LINE_BLOCK_SIZE, FINISH_LINE_START_Y + i * FINISH_LINE_BLOCK_SIZE, FINISH_LINE_BLOCK_SIZE, FINISH_LINE_BLOCK_SIZE, (i % 2 == 0) ? BLACK : WHITE);
    }
    
    for (auto wall : trackWalls) {
        DrawLineEx(wall.first, wall.second, 6.0f, WHITE);
    }

    if (!aiCar.isCrashed) DrawRectanglePro({ aiCar.position.x, aiCar.position.y, 20.0f, 10.0f }, { 10.0f, 5.0f }, aiCar.rotation, RED);
    if (!playerCar.isCrashed) DrawRectanglePro({ playerCar.position.x, playerCar.position.y, 20.0f, 10.0f }, { 10.0f, 5.0f }, playerCar.rotation, BLUE);

    if (exhibitionResult == 1) {
        DrawText("¡HAS GANADO A LA IA!", Config::SCREEN_WIDTH/2 - 200, 200, 40, GREEN);
        DrawText("Pulsa R para revancha", Config::SCREEN_WIDTH/2 - 150, 250, 20, WHITE);
    } else if (exhibitionResult == 2) {
        DrawText("LA IA TE HA DESTRUIDO", Config::SCREEN_WIDTH/2 - 200, 200, 40, RED);
        DrawText("Pulsa R para reintentar", Config::SCREEN_WIDTH/2 - 150, 250, 20, WHITE);
    }

    DrawText("[M] Volver al Menú Principal", 20, 20, 20, LIGHTGRAY);
}

void Simulation::UpdateTestAI() {
    if (IsKeyPressed(KEY_M)) currentState = MENU;
    if (IsKeyPressed(KEY_R) && aiCar.isCrashed) {
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
    }

    if (!aiCar.isCrashed) {
        float aiAcelerar = 0.0f, aiGiro = 0.0f;
        aiCar.brain.Evaluate(aiCar.sensorDistances, aiCar.speed, aiAcelerar, aiGiro);
        aiCar.UpdatePhysics(aiAcelerar, aiGiro, trackWalls, 0);
    }
}

void Simulation::DrawTestAI() {
    for(int i=0; i<FINISH_LINE_BLOCK_COUNT; i++) {
        DrawRectangle(FINISH_LINE_X, FINISH_LINE_START_Y + i * FINISH_LINE_BLOCK_SIZE, FINISH_LINE_BLOCK_SIZE, FINISH_LINE_BLOCK_SIZE, (i % 2 == 0) ? WHITE : BLACK);
        DrawRectangle(FINISH_LINE_X + FINISH_LINE_BLOCK_SIZE, FINISH_LINE_START_Y + i * FINISH_LINE_BLOCK_SIZE, FINISH_LINE_BLOCK_SIZE, FINISH_LINE_BLOCK_SIZE, (i % 2 == 0) ? BLACK : WHITE);
    }
    
    for (auto wall : trackWalls) {
        DrawLineEx(wall.first, wall.second, 6.0f, WHITE);
    }

    if (!aiCar.isCrashed) {
        DrawRectanglePro({ aiCar.position.x, aiCar.position.y, 20.0f, 10.0f }, { 10.0f, 5.0f }, aiCar.rotation, RED);
        for (int i = 0; i < 5; i++) {
            float rayAngle = (aiCar.rotation + Config::SENSOR_ANGLES[i]) * DEG2RAD;
            Vector2 actualRayEnd = { aiCar.position.x + cos(rayAngle) * aiCar.sensorDistances[i], aiCar.position.y + sin(rayAngle) * aiCar.sensorDistances[i] };
            DrawLineV(aiCar.position, actualRayEnd, Fade(GREEN, 0.5f));
            DrawCircleV(actualRayEnd, 3.0f, Fade(GREEN, 0.5f));
        }
    } else {
        DrawText("LA IA HA CHOCADO", Config::SCREEN_WIDTH/2 - 150, 200, 30, RED);
        DrawText("Pulsa R para reiniciar", Config::SCREEN_WIDTH/2 - 100, 250, 20, WHITE);
    }

    DrawText("[M] Volver al Menú Principal", 20, 20, 20, LIGHTGRAY);
}
