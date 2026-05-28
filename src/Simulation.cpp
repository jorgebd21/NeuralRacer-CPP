#include "Simulation.h"
#include "Config.h"
#include "Evolution.h"
#include "TrackManager.h"
#include "TrackGenerator.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <execution>
#include <atomic>

Simulation::Simulation(bool isHeadless) : 
    isHeadless(isHeadless),
    showCheckpoints(false),
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
    if(!isHeadless){
        InitWindow(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, "Simulador Genético - IA Autónoma");
        SetTargetFPS(SIM_TARGET_FPS);
    }

    mapFiles = TrackManager::ScanMapFiles();
    trackFile = mapFiles[currentMapIndex];
    
    TrackManager::LoadTrackFromFile(trackFile, trackWalls, startPosition, startRotation, trackCheckpoints);
    BuildSpacialGrid();
    if (trackWalls.empty()) trackWalls.push_back({{100, 100}, {900, 100}});

    for (int i = 0; i < Config::POPULATION_SIZE; i++) {
        population.push_back(Car(startPosition.x, startPosition.y, startRotation));
    }
    Evolution::CargarMejoresCerebros(population, generationCount);
    
    playerCar = Car(startPosition.x, startPosition.y, startRotation);
    aiCar = population.empty() ? Car(startPosition.x, startPosition.y, startRotation) : population[0];
    aiCar.Reset(startPosition.x, startPosition.y, startRotation);
}

void Simulation::Run() {
    Init();
    if(isHeadless){
        currentState = TRAINING;
        while(true){
            Update();
        }
    }else{
        while (!WindowShouldClose()) {
            Update();
            Draw();
        }
        CloseWindow();
}   
}

void Simulation::Update() {
    // Máquina de estados principal: delega la actualización lógica según el modo actual
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
        float aiAcelerar = 0.0f, aiGiro = 0.0f;
        aiCar.brain.Evaluate(aiCar.sensorDistances, aiCar.GetSpeed(), aiAcelerar, aiGiro);
        aiCar.UpdatePhysics(aiAcelerar, aiGiro, spatialGrid, 0, trackCheckpoints);
    }

    if (IsKeyPressed(KEY_T)) currentState = TRAINING;
    if (IsKeyPressed(KEY_E)) {
        playerCar.Reset(startPosition.x, startPosition.y, startRotation);
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
        
        std::vector<Car> temp(1, Car(startPosition.x, startPosition.y, startRotation));
        if (Evolution::CargarMejoresCerebros(temp, generationCount)) {
            aiCar = temp[0];
        }
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
        exhibitionResult = 0;
        currentState = EXHIBITION;
    }
    if (IsKeyPressed(KEY_A)) {
        std::vector<Car> temp(1, Car(startPosition.x, startPosition.y, startRotation));
        if (Evolution::CargarMejoresCerebros(temp, generationCount)) {
            aiCar = temp[0];
        }
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
        exhibitionResult = 0;
        currentState = TEST_AI;
    }
    if (IsKeyPressed(KEY_P)) {
        puntosProcedurales = TrackGenerator::GenerateProceduralCenterPoints();
        std::vector<Vector2> denseCenterLine = TrackManager::GenerateSplinePoints(puntosProcedurales, PROCEDURAL_SPLINE_SEGMENTS);
        TrackManager::CalculateStartGrid(puntosProcedurales, startPosition, startRotation);
        trackWalls.clear();
        trackCheckpoints.clear();
        TrackManager::GenerateBordersFromCenterLine(denseCenterLine, PROCEDURAL_TRACK_WIDTH, trackWalls, trackCheckpoints, startPosition);
        BuildSpacialGrid();
        
        for (auto& car : population) car.Reset(startPosition.x, startPosition.y, startRotation);
        playerCar.Reset(startPosition.x, startPosition.y, startRotation);
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
    }
    if (IsKeyPressed(KEY_LEFT)) {
        currentMapIndex--;
        if (currentMapIndex < 0) currentMapIndex = mapFiles.size() - 1;
        trackFile = mapFiles[currentMapIndex];
        trackWalls.clear();
        trackCheckpoints.clear();
        TrackManager::LoadTrackFromFile(trackFile, trackWalls, startPosition, startRotation, trackCheckpoints);
        BuildSpacialGrid();
        for (auto& car : population) car.Reset(startPosition.x, startPosition.y, startRotation);
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
    }
    if (IsKeyPressed(KEY_RIGHT)) {
        currentMapIndex++;
        if (currentMapIndex >= (int)mapFiles.size()) currentMapIndex = 0;
        trackFile = mapFiles[currentMapIndex];
        trackWalls.clear();
        trackCheckpoints.clear();
        TrackManager::LoadTrackFromFile(trackFile, trackWalls, startPosition, startRotation, trackCheckpoints);
        BuildSpacialGrid();
        for (auto& car : population) car.Reset(startPosition.x, startPosition.y, startRotation);
        aiCar.Reset(startPosition.x, startPosition.y, startRotation);
    }
    if (IsKeyPressed(KEY_G)) {
        if (!puntosProcedurales.empty()) {
            int counter = 1;
            std::string filename;
            while (true) {
                filename = "data/tracks/pista_procedural" + std::to_string(counter) + ".json";
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
    if(!isHeadless){
        if (IsKeyPressed(KEY_SPACE)) simSpeed = (simSpeed == 1) ? FAST_FORWARD_MULTIPLIER : 1;
        if (IsKeyPressed(KEY_M)) currentState = MENU;
        if (IsKeyPressed(KEY_C)) showCheckpoints = !showCheckpoints;
    }

    // Ejecutamos la lógica varias veces por frame si estamos en modo cámara rápida
    for (int s = 0; s < simSpeed; s++) {
        std::atomic<int> carsAlive{0};
        std::for_each(std::execution::par_unseq, population.begin(), population.end(), [&](Car& car) {
            if (!car.isCrashed) {
                carsAlive++;
                
                // 1. El cerebro decide qué hacer basándose en los sensores
                float inputAcelerar = 0.0f, inputGiro = 0.0f;
                car.brain.Evaluate(car.sensorDistances, car.GetSpeed(), inputAcelerar, inputGiro);
                
                // 2. El coche se mueve según la decisión y comprueba si ha chocado
                car.UpdatePhysics(inputAcelerar, inputGiro, spatialGrid, generationTimer, trackCheckpoints);
            }
        });
        
        bool allCrashed = (carsAlive == 0);
        generationTimer++;
        
        // Si todos los coches han muerto o se acabó el tiempo máximo por generación
        if (allCrashed || generationTimer >= Config::MAX_GENERATION_TIME) {
            std::cout << "Generacion " << generationCount << " (Pista " << currentEvaluationTrack + 1 << "/3)" << std::endl;
            currentEvaluationTrack++;
            if (currentEvaluationTrack >= 3) {
                std::vector<Car*> sortedPop;
                sortedPop.reserve(population.size());
                for (auto& car : population) {
                    sortedPop.push_back(&car);
                }
                std::sort(sortedPop.begin(), sortedPop.end(), [](const Car* a, const Car* b) { return a->fitness > b->fitness; });
                std::cout << "Mejor fitness: " << sortedPop[0]->fitness << std::endl;
                Evolution::EvolvePopulation(population, startPosition, startRotation, generationCount);
                generationTimer = 0;
                generationCount++;
                currentEvaluationTrack = 0;
            } else {
                for (auto& car : population) {
                    car.accumulatedFitness = car.fitness;
                }
                
                puntosProcedurales = TrackGenerator::GenerateProceduralCenterPoints();
                std::vector<Vector2> denseCenterLine = TrackManager::GenerateSplinePoints(puntosProcedurales, PROCEDURAL_SPLINE_SEGMENTS);
                TrackManager::CalculateStartGrid(puntosProcedurales, startPosition, startRotation);
                trackWalls.clear();
                trackCheckpoints.clear();
                TrackManager::GenerateBordersFromCenterLine(denseCenterLine, PROCEDURAL_TRACK_WIDTH, trackWalls, trackCheckpoints, startPosition);
                BuildSpacialGrid();
                
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
        playerCar.UpdatePhysics(playerAcelerar, playerGiro, spatialGrid, 0, trackCheckpoints);

        float aiAcelerar = 0.0f, aiGiro = 0.0f;
        aiCar.brain.Evaluate(aiCar.sensorDistances, aiCar.GetSpeed(), aiAcelerar, aiGiro);
        aiCar.UpdatePhysics(aiAcelerar, aiGiro, spatialGrid, 0, trackCheckpoints);

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
        DrawCircleLines(cp.x, cp.y, 65.0f, Fade(YELLOW, alpha));
        
        DrawText(TextFormat("%zu", i + 1), cp.x - 5, cp.y - 10, 20, Fade(ORANGE, alpha));
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
    
    for (auto wall : trackWalls) {
        DrawLineEx(wall.first, wall.second, 6.0f, Fade(WHITE, 0.3f));
    }

    if (!aiCar.isCrashed) {
        DrawRectanglePro({ aiCar.position.x, aiCar.position.y, 20.0f, 10.0f }, { 10.0f, 5.0f }, aiCar.rotation, Fade(RED, 0.3f));
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

void Simulation::DrawTraining() {
    DrawFinishLine(1.0f);
    if (showCheckpoints) {
        DrawCheckpoints(0.5f);
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
    DrawText(TextFormat("Mutacion: %d %%", Config::MUTACION), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 130, 15, YELLOW);
    DrawText("[ESPACIO] Cambiar vel", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 150, 10, LIGHTGRAY);
    DrawText("[M] Volver al Menu", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 165, 10, LIGHTGRAY);
    DrawText("[C] Ver/Ocultar Checkpoints", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 180, 10, LIGHTGRAY);

    DrawText("TOP 10 FITNESS", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 200, 20, YELLOW);
    std::vector<Car*> sortedPop;
    sortedPop.reserve(population.size());
    for (auto& car : population) {
        sortedPop.push_back(&car);
    }
    std::sort(sortedPop.begin(), sortedPop.end(), [](const Car* a, const Car* b) { return a->fitness > b->fitness; });
    for (int i = 0; i < 10 && i < (int)sortedPop.size(); i++) {
        Color rowColor = (sortedPop[i]->isCrashed) ? GRAY : WHITE;
        if (i == 0) rowColor = GOLD;
        DrawText(TextFormat("%d. Fit: %.1f", i + 1, sortedPop[i]->fitness), Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 230 + (i * 25), 18, rowColor);
    }

    if (!sortedPop.empty()) {
        DrawText("RED NEURONAL (Lider)", Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 15, 490, 15, SKYBLUE);
        
        Brain& bestBrain = sortedPop[0]->brain;
        int startY = 510;
        int endY = 750;
        int layerX[3] = {
            Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 30,
            Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 125,
            Config::SCREEN_WIDTH - UI_PANEL_WIDTH + 220
        };
        int nodesPerLayer[3] = { NODOS_ENTRADA, NODOS_OCULTOS, NODOS_SALIDA };
        
        std::vector<Vector2> nodePos[3];
        for (int l = 0; l < 3; l++) {
            int nodes = nodesPerLayer[l];
            float spacing = (endY - startY) / (float)(nodes + 1);
            for (int n = 0; n < nodes; n++) {
                nodePos[l].push_back({ (float)layerX[l], startY + spacing * (n + 1) });
            }
        }
        
        for (int i = 0; i < NODOS_ENTRADA; i++) {
            for (int j = 0; j < NODOS_OCULTOS; j++) {
                float weight = bestBrain.peso_entrada_oculta[j][i];
                float alpha = fmin(fabs(weight), 1.0f);
                Color edgeColor = (weight > 0) ? Fade(GREEN, alpha) : Fade(RED, alpha);
                DrawLineV(nodePos[0][i], nodePos[1][j], edgeColor);
            }
        }
        
        for (int i = 0; i < NODOS_OCULTOS; i++) {
            for (int j = 0; j < NODOS_SALIDA; j++) {
                float weight = bestBrain.peso_oculta_salida[j][i];
                float alpha = fmin(fabs(weight), 1.0f);
                Color edgeColor = (weight > 0) ? Fade(GREEN, alpha) : Fade(RED, alpha);
                DrawLineV(nodePos[1][i], nodePos[2][j], edgeColor);
            }
        }
        
        for (int l = 0; l < 3; l++) {
            for (int n = 0; n < nodesPerLayer[l]; n++) {
                float val = 0.0f;
                if (l == 0) {
                    if (n < 5) val = bestBrain.last_entrada[n] / 250.0f; // SENSOR_MAX_DIST aprox
                    else val = bestBrain.last_entrada[n] / 10.0f; // max speed aprox
                } else if (l == 1) {
                    val = bestBrain.last_ocultos[n];
                } else if (l == 2) {
                    val = bestBrain.last_salida[n];
                }
                
                val = fmax(-1.0f, fmin(1.0f, val));
                Color nodeColor = (val > 0) ? Fade(GREEN, val) : Fade(RED, -val);
                if (fabs(val) < 0.1f) nodeColor = DARKGRAY;
                
                DrawCircleV(nodePos[l][n], 6.0f, nodeColor);
                DrawCircleLines(nodePos[l][n].x, nodePos[l][n].y, 6.0f, LIGHTGRAY);
            }
        }
    }
}

void Simulation::DrawExhibition() {
    DrawFinishLine(1.0f);
    DrawCheckpoints(0.5f);
    
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
        aiCar.brain.Evaluate(aiCar.sensorDistances, aiCar.GetSpeed(), aiAcelerar, aiGiro);
        aiCar.UpdatePhysics(aiAcelerar, aiGiro, spatialGrid, 0, trackCheckpoints);
    }
}

void Simulation::DrawTestAI() {
    DrawFinishLine(1.0f);
    DrawCheckpoints(0.5f);
    
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

void Simulation::BuildSpacialGrid(){
    spatialGrid.clear();

    for(int i = 0; i < trackWalls.size(); i++){
        int grid1X = trackWalls[i].first.x / TrackManager::GRID_CELL_SIZE;
        int grid1Y = trackWalls[i].first.y / TrackManager::GRID_CELL_SIZE;
        int grid2X = trackWalls[i].second.x / TrackManager::GRID_CELL_SIZE;
        int grid2Y = trackWalls[i].second.y / TrackManager::GRID_CELL_SIZE;
        
        int iniX = std::min(grid1X, grid2X);
        int iniY = std::min(grid1Y, grid2Y);
        int endX = std::max(grid1X, grid2X);
        int endY = std::max(grid1Y, grid2Y);
        
        for(int x = iniX; x <= endX; x++){
            for(int y = iniY; y <= endY; y++){
                uint64_t key = TrackManager::GetGridKey(x, y);
                spatialGrid[key].push_back(trackWalls[i]);
            }
        }
    }
}