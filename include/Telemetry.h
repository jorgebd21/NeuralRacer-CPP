#pragma once

#include <vector>
#include <string>
#include <future>
#include <mutex>
#include "raylib.h"
#include "Car.h"

struct GenerationMetrics {
    int generation;
    float maxFitness;
    float minFitness;
    float avgFitness;
    float survivalRate;      // Percentage that survived without crashing
    float avgLifeTime;       // Average lifetime in ticks
    float weightVariance;    // Measure of genetic diversity
    int curriculumPhase;     // Phase 1 (Induction), Phase 2 (Consolidation), Phase 3 (Procedural)
    float benchmarkFitness;  // Fitness achieved strictly on fixed control benchmark track
    float proceduralFitness; // Fitness achieved on procedural / non-control tracks
    int totalLapsCompleted;  // Total full laps completed by the population in this generation
    float topSpeedAvg;       // Average cruising speed of elite individual (px/tick)
    float steeringSmoothness;// Steering rate of change stability [0.0 - 1.0]
    int activeMutationRate;  // Active mutation percentage
    float bestLapTime;       // Fastest lap time recorded in seconds (0.0f if no lap completed)
};

class Telemetry {
public:
    static void Init();
    static void RecordCrash(Vector2 position);
    static void RecordGeneration(int generation,
                                 const std::vector<Car>& population,
                                 int maxGenerationTime,
                                 int curriculumPhase = 1,
                                 float benchmarkFitness = 0.0f,
                                 float proceduralFitness = 0.0f,
                                 int totalLaps = 0,
                                 float topSpeed = 0.0f,
                                 float smoothness = 1.0f,
                                 int mutationRate = 15,
                                 float bestLapTime = 0.0f);
    static void DrawDashboard(int screenWidth, int screenHeight);
    static void DrawHeatmap();
    static void ExportDataAsync(const std::string& filename = "data/telemetry_log.csv");
    static void Flush(const std::string& filename = "data/telemetry_log.csv");
    static void ResetHeatmap();

private:
    static std::vector<GenerationMetrics> history;
    static std::vector<Vector2> crashPoints;
    static std::future<void> exportFuture;
    static std::mutex dataMutex;
    
    static float CalculateWeightVariance(const std::vector<Car>& population);
};
