#include "Telemetry.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>

std::vector<GenerationMetrics> Telemetry::history;
std::vector<Vector2> Telemetry::crashPoints;
std::future<void> Telemetry::exportFuture;
std::mutex Telemetry::dataMutex;

void Telemetry::Init() {
    std::lock_guard<std::mutex> lock(dataMutex);
    history.clear();
    crashPoints.clear();
}

void Telemetry::RecordCrash(Vector2 position) {
    std::lock_guard<std::mutex> lock(dataMutex);
    crashPoints.push_back(position);
}

void Telemetry::ResetHeatmap() {
    std::lock_guard<std::mutex> lock(dataMutex);
    crashPoints.clear();
}

float Telemetry::CalculateWeightVariance(const std::vector<Car>& population) {
    if (population.empty()) return 0.0f;

    double sum = 0.0;
    int count = 0;

    for (const auto& car : population) {
        for (int i = 0; i < HIDDEN_NODES; i++) {
            for (int j = 0; j < INPUT_NODES; j++) {
                sum += car.brain.weights_input_hidden[i][j];
                count++;
            }
            sum += car.brain.biases_hidden[i];
            count++;
        }
        for (int i = 0; i < OUTPUT_NODES; i++) {
            for (int j = 0; j < HIDDEN_NODES; j++) {
                sum += car.brain.weights_hidden_output[i][j];
                count++;
            }
            sum += car.brain.biases_output[i];
            count++;
        }
    }

    double mean = sum / count;
    double varSum = 0.0;

    for (const auto& car : population) {
        for (int i = 0; i < HIDDEN_NODES; i++) {
            for (int j = 0; j < INPUT_NODES; j++) {
                double diff = car.brain.weights_input_hidden[i][j] - mean;
                varSum += diff * diff;
            }
            double diff = car.brain.biases_hidden[i] - mean;
            varSum += diff * diff;
        }
        for (int i = 0; i < OUTPUT_NODES; i++) {
            for (int j = 0; j < HIDDEN_NODES; j++) {
                double diff = car.brain.weights_hidden_output[i][j] - mean;
                varSum += diff * diff;
            }
            double diff = car.brain.biases_output[i] - mean;
            varSum += diff * diff;
        }
    }

    return (float)(varSum / count);
}

void Telemetry::RecordGeneration(int generation, const std::vector<Car>& population, int maxGenerationTime) {
    if (population.empty()) return;

    GenerationMetrics metrics;
    metrics.generation = generation;
    
    metrics.maxFitness = population[0].fitness;
    metrics.minFitness = population[0].fitness;
    double sumFitness = 0.0;
    double sumLifeTime = 0.0;
    int survivedCount = 0;

    for (const auto& car : population) {
        if (car.fitness > metrics.maxFitness) metrics.maxFitness = car.fitness;
        if (car.fitness < metrics.minFitness) metrics.minFitness = car.fitness;
        sumFitness += car.fitness;
        sumLifeTime += car.timeAlive;
        if (!car.isCrashed) {
            survivedCount++;
        }
    }

    metrics.avgFitness = (float)(sumFitness / population.size());
    metrics.survivalRate = (float)survivedCount / population.size();
    metrics.avgLifeTime = (float)(sumLifeTime / population.size());
    metrics.weightVariance = CalculateWeightVariance(population);

    history.push_back(metrics);
}

void Telemetry::ExportDataAsync(const std::string& filename) {
    // Copy the history so the asynchronous thread doesn't have concurrency issues
    std::vector<GenerationMetrics> historyCopy = history;
    
    exportFuture = std::async(std::launch::async, [historyCopy, filename]() {
        std::ofstream file(filename, std::ios::out);
        if (file.is_open()) {
            file << "Generation,MaxFitness,MinFitness,AvgFitness,SurvivalRate,AvgLifeTime,WeightVariance\n";
            for (const auto& m : historyCopy) {
                file << m.generation << ","
                     << m.maxFitness << ","
                     << m.minFitness << ","
                     << m.avgFitness << ","
                     << m.survivalRate << ","
                     << m.avgLifeTime << ","
                     << m.weightVariance << "\n";
            }
            file.close();
        }
    });
}

void Telemetry::DrawDashboard(int screenWidth, int screenHeight) {
    if (history.empty()) return;

    int panelWidth = 400;
    int panelHeight = 300;
    int padding = 20;
    int x = padding;
    int y = screenHeight - panelHeight - padding;

    // Dashboard background
    DrawRectangle(x, y, panelWidth, panelHeight, Fade(BLACK, 0.85f));
    DrawRectangleLines(x, y, panelWidth, panelHeight, Fade(WHITE, 0.5f));

    DrawText("TELEMETRY AND CONVERGENCE", x + 10, y + 10, 20, SKYBLUE);

    // Find maximums to scale the graph
    float maxFit = 0.0001f; // Avoid division by zero
    for (const auto& m : history) {
        if (m.maxFitness > maxFit) maxFit = m.maxFitness;
    }

    int graphX = x + 40;
    int graphY = y + 50;
    int graphW = panelWidth - 60;
    int graphH = 150;

    // Graph axes
    DrawLine(graphX, graphY, graphX, graphY + graphH, WHITE);
    DrawLine(graphX, graphY + graphH, graphX + graphW, graphY + graphH, WHITE);

    // Draw fitness lines
    if (history.size() > 1) {
        float stepX = (float)graphW / (history.size() - 1);
        for (size_t i = 1; i < history.size(); i++) {
            int px1 = graphX + (i - 1) * stepX;
            int px2 = graphX + i * stepX;

            // Max Fitness (Green)
            int pyMax1 = graphY + graphH - (int)((history[i-1].maxFitness / maxFit) * graphH);
            int pyMax2 = graphY + graphH - (int)((history[i].maxFitness / maxFit) * graphH);
            DrawLineEx({(float)px1, (float)pyMax1}, {(float)px2, (float)pyMax2}, 2.0f, GREEN);

            // Avg Fitness (Yellow)
            int pyAvg1 = graphY + graphH - (int)((history[i-1].avgFitness / maxFit) * graphH);
            int pyAvg2 = graphY + graphH - (int)((history[i].avgFitness / maxFit) * graphH);
            DrawLineEx({(float)px1, (float)pyAvg1}, {(float)px2, (float)pyAvg2}, 2.0f, YELLOW);
            
            // Min Fitness (Red)
            int pyMin1 = graphY + graphH - (int)((history[i-1].minFitness / maxFit) * graphH);
            int pyMin2 = graphY + graphH - (int)((history[i].minFitness / maxFit) * graphH);
            DrawLineEx({(float)px1, (float)pyMin1}, {(float)px2, (float)pyMin2}, 2.0f, RED);
        }
    }

    // Graph legend
    DrawText(TextFormat("Max Fit: %.1f", history.back().maxFitness), x + 10, graphY + graphH + 10, 15, GREEN);
    DrawText(TextFormat("Avg Fit: %.1f", history.back().avgFitness), x + 10, graphY + graphH + 30, 15, YELLOW);
    DrawText(TextFormat("Min Fit: %.1f", history.back().minFitness), x + 10, graphY + graphH + 50, 15, RED);

    // Other metrics
    int rightCol = x + 200;
    DrawText(TextFormat("Survival: %.1f%%", history.back().survivalRate * 100.0f), rightCol, graphY + graphH + 10, 15, WHITE);
    DrawText(TextFormat("Avg Life: %.0f t", history.back().avgLifeTime), rightCol, graphY + graphH + 30, 15, WHITE);
    DrawText(TextFormat("Gen. Var.: %.4f", history.back().weightVariance), rightCol, graphY + graphH + 50, 15, ORANGE);
}

void Telemetry::DrawHeatmap() {
    // Draw the points where cars have crashed
    for (const auto& cp : crashPoints) {
        DrawCircleV(cp, 4.0f, Fade(RED, 0.2f)); // Semi-transparent so accumulation looks brighter
    }
}
