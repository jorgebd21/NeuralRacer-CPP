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
};

class Telemetry {
public:
    static void Init();
    static void RecordCrash(Vector2 position);
    static void RecordGeneration(int generation, const std::vector<Car>& population, int maxGenerationTime);
    static void DrawDashboard(int screenWidth, int screenHeight);
    static void DrawHeatmap();
    static void ExportDataAsync(const std::string& filename = "data/telemetry_log.csv");
    static void ResetHeatmap();

private:
    static std::vector<GenerationMetrics> history;
    static std::vector<Vector2> crashPoints;
    static std::future<void> exportFuture;
    static std::mutex dataMutex;
    
    static float CalculateWeightVariance(const std::vector<Car>& population);
};
