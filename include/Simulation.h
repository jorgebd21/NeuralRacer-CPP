#pragma once

#include "raylib.h"
#include "Car.h"
#include <vector>
#include <string>
#include <utility>
#include <unordered_map>

/**
 * @brief Represents the global state of the application (finite state machine).
 */
enum GameState { MENU, TRAINING, EXHIBITION, TEST_AI };

/**
 * @brief Main controller for graphical and logical simulation.
 * 
 * Orchestrates menus, training phases, and exhibition races.
 * Responsible for maintaining the main application loop and state management.
 */
class Simulation {
public:
    /**
     * @brief Constructor for the Simulation class.
     */
    Simulation(bool isHeadless = false);

    /**
     * @brief Starts and maintains the main lifecycle of the simulation.
     */
    void Run();

private:
    /**
     * @brief Initializes visual resources and necessary base logic.
     */
    void Init();

    /**
     * @brief Updates business logic depending on the current GameState.
     */
    void Update();

    /**
     * @brief Draws to the screen according to the active GameState.
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
    void DrawMinimap(const std::vector<Car*>& sortedCars);
    void UpdateCameraControl();

    void BuildSpatialGrid();
    void InitTrainingTracks();
    void SetupCurriculumForGeneration(int gen);
    void SetTrainingTrack(int trackIndex);

    struct TrainingTrack {
        std::vector<std::pair<Vector2, Vector2>> walls;
        std::vector<Vector2> checkpoints;
        Vector2 startPos;
        float startRot;
    };
    TrainingTrack GenerateProceduralTrainingTrack(int fixedArchetype = -1);

    TrainingTrack baseTrackFacil;
    TrainingTrack baseTrackMedio;
    TrainingTrack baseTrackDificil;
    std::vector<TrainingTrack> trainingTracks;

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
    std::vector<Vector2> proceduralPoints;

    std::vector<Car> population;
    int generationTimer;
    int generationCount;
    int currentEvaluationTrack;
    int simSpeed;

    Car playerCar;
    Car aiCar;
    int exhibitionResult;

    int generationTotalLaps;
    float benchmarkTopFitness;

    // Dynamic 2D camera & multi-scale viewport navigation
    enum CameraMode { CAM_FOLLOW_LEADER = 0, CAM_OVERVIEW = 1, CAM_MANUAL = 2 };
    CameraMode cameraMode;
    Camera2D worldCamera;
    float targetZoom;
    Vector2 cameraDragStart;
    bool isDraggingCamera;
};
