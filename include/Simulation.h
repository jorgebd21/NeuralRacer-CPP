#pragma once

#include "raylib.h"
#include "Car.h"
#include <vector>
#include <string>
#include <utility>

enum GameState { MENU, TRAINING, EXHIBITION };

class Simulation {
public:
    Simulation();
    void Run();

private:
    void Init();
    void Update();
    void Draw();

    void UpdateMenu();
    void UpdateTraining();
    void UpdateExhibition();

    void DrawMenu();
    void DrawTraining();
    void DrawExhibition();

    GameState currentState;

    std::vector<std::string> mapFiles;
    int currentMapIndex;
    std::string trackFile;

    Vector2 startPosition;
    float startRotation;
    std::vector<std::pair<Vector2, Vector2>> trackWalls;
    std::vector<Vector2> puntosProcedurales;

    std::vector<Car> population;
    int generationTimer;
    int generationCount;
    int simSpeed;

    Car playerCar;
    Car aiCar;
    int exhibitionResult; // 0=jugando, 1=player gana, 2=ai gana
};
