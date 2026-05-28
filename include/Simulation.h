#pragma once

#include "raylib.h"
#include "Car.h"
#include <vector>
#include <string>
#include <utility>
#include <unordered_map>

/**
 * @brief Representa el estado global de la aplicación (máquina de estados finitos).
 */
enum GameState { MENU, TRAINING, EXHIBITION, TEST_AI };

/**
 * @brief Controlador principal de la simulación gráfica y lógica.
 * 
 * Orquesta los menús, las fases de entrenamiento y las carreras de exhibición.
 * Es responsable de mantener el bucle principal de la aplicación y la gestión de estado.
 */
class Simulation {
public:
    /**
     * @brief Constructor de la clase Simulation.
     */
    Simulation(bool isHeadless = false);

    /**
     * @brief Inicia y mantiene el ciclo de vida principal de la simulación.
     */
    void Run();

private:
    /**
     * @brief Inicializa los recursos visuales y las lógicas base necesarias.
     */
    void Init();

    /**
     * @brief Actualiza la lógica de negocio dependiendo del GameState actual.
     */
    void Update();

    /**
     * @brief Dibuja por pantalla según el estado activo de GameState.
     */
    void Draw();

    void UpdateMenu();
    void UpdateTraining();
    void UpdateExhibition();
    void UpdateTestAI();

    void DrawMenu();
    void DrawTraining();
    void DrawExhibition();
    void DrawTestAI();

    void DrawFinishLine(float alpha = 1.0f);
    void DrawCheckpoints(float alpha = 1.0f);

    void BuildSpacialGrid();

    GameState currentState;

    bool isHeadless;
    bool showCheckpoints;
    bool showTelemetry;

    std::vector<std::string> mapFiles;
    int currentMapIndex;
    std::string trackFile;

    Vector2 startPosition;
    float startRotation;
    std::vector<std::pair<Vector2, Vector2>> trackWalls;
    std::vector<Vector2> trackCheckpoints;
    std::unordered_map<uint64_t, std::vector<std::pair<Vector2, Vector2>>> spatialGrid;
    std::vector<Vector2> puntosProcedurales;

    std::vector<Car> population;
    int generationTimer;
    int generationCount;
    int currentEvaluationTrack;
    int simSpeed;

    Car playerCar;
    Car aiCar;
    int exhibitionResult;
};
