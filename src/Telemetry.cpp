#include "Telemetry.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cmath>

std::vector<GenerationMetrics> Telemetry::history;
std::vector<Vector2> Telemetry::crashPoints;
std::vector<TelemetrySample> Telemetry::fastestLapBuffer;
std::future<void> Telemetry::exportFuture;
std::future<void> Telemetry::fastestLapFuture;
std::mutex Telemetry::dataMutex;

void Telemetry::Init() {
    std::lock_guard<std::mutex> lock(dataMutex);
    history.clear();
    crashPoints.clear();
    fastestLapBuffer.clear();

    // Load existing telemetry CSV if present to persist longitudinal progress across sessions
    std::ifstream file("data/telemetry_log.csv");
    if (file.is_open()) {
        std::string header;
        std::getline(file, header); // Skip header line
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string token;
            GenerationMetrics m{};
            try {
                if (std::getline(ss, token, ',')) m.generation = std::stoi(token);
                if (std::getline(ss, token, ',')) m.maxFitness = std::stof(token);
                if (std::getline(ss, token, ',')) m.minFitness = std::stof(token);
                if (std::getline(ss, token, ',')) m.avgFitness = std::stof(token);
                if (std::getline(ss, token, ',')) m.survivalRate = std::stof(token);
                if (std::getline(ss, token, ',')) m.avgLifeTime = std::stof(token);
                if (std::getline(ss, token, ',')) m.weightVariance = std::stof(token);
                if (std::getline(ss, token, ',')) m.curriculumPhase = std::stoi(token);
                else m.curriculumPhase = (m.generation < 15) ? 1 : (m.generation < 50) ? 2 : 3;
                if (std::getline(ss, token, ',')) m.benchmarkFitness = std::stof(token);
                else m.benchmarkFitness = m.maxFitness;
                if (std::getline(ss, token, ',')) m.proceduralFitness = std::stof(token);
                else m.proceduralFitness = 0.0f;
                if (std::getline(ss, token, ',')) m.totalLapsCompleted = std::stoi(token);
                else m.totalLapsCompleted = 0;
                if (std::getline(ss, token, ',')) m.topSpeedAvg = std::stof(token);
                else m.topSpeedAvg = 0.0f;
                if (std::getline(ss, token, ',')) m.steeringSmoothness = std::stof(token);
                else m.steeringSmoothness = 1.0f;
                if (std::getline(ss, token, ',')) m.activeMutationRate = std::stoi(token);
                else m.activeMutationRate = 15;
                if (std::getline(ss, token, ',')) m.bestLapTime = std::stof(token);
                else m.bestLapTime = 0.0f;
                if (std::getline(ss, token, ',')) m.kammUtilizationAvg = std::stof(token);
                else m.kammUtilizationAvg = 0.0f;
                if (std::getline(ss, token, ',')) m.slipAngleAvg = std::stof(token);
                else m.slipAngleAvg = 0.0f;
                if (std::getline(ss, token, ',')) m.controlJerkAvg = std::stof(token);
                else m.controlJerkAvg = 0.0f;
                if (std::getline(ss, token, ',')) m.trackWidthUtil = std::stof(token);
                else m.trackWidthUtil = 0.0f;
                if (std::getline(ss, token, ',')) m.neuralSaturation = std::stof(token);
                else m.neuralSaturation = 0.0f;
                history.push_back(m);
            } catch (...) {
                // Ignore any malformed lines
            }
        }
        file.close();
    }
}

void Telemetry::RecordCrash(Vector2 position) {
    std::lock_guard<std::mutex> lock(dataMutex);
    crashPoints.push_back(position);
}

void Telemetry::RecordFastestLap(const std::vector<TelemetrySample>& lapSamples) {
    std::lock_guard<std::mutex> lock(dataMutex);
    fastestLapBuffer = lapSamples;
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

void Telemetry::RecordGeneration(int generation,
                                 const std::vector<Car>& population,
                                 int maxGenerationTime,
                                 int curriculumPhase,
                                 float benchmarkFitness,
                                 float proceduralFitness,
                                 int totalLaps,
                                 float topSpeed,
                                 float smoothness,
                                 int mutationRate,
                                 float bestLapTime,
                                 float kammUtil,
                                 float slipAngle,
                                 float jerk,
                                 float trackUtil,
                                 float saturation) {
    if (population.empty()) return;

    GenerationMetrics metrics{};
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
    metrics.curriculumPhase = curriculumPhase;
    metrics.benchmarkFitness = benchmarkFitness;
    metrics.proceduralFitness = proceduralFitness;
    metrics.totalLapsCompleted = totalLaps;
    metrics.topSpeedAvg = topSpeed;
    metrics.steeringSmoothness = smoothness;
    metrics.activeMutationRate = mutationRate;
    metrics.bestLapTime = bestLapTime;
    metrics.kammUtilizationAvg = kammUtil;
    metrics.slipAngleAvg = slipAngle;
    metrics.controlJerkAvg = jerk;
    metrics.trackWidthUtil = trackUtil;
    metrics.neuralSaturation = saturation;

    auto it = std::find_if(history.begin(), history.end(), [generation](const GenerationMetrics& m) {
        return m.generation == generation;
    });
    if (it != history.end()) {
        *it = metrics;
    } else {
        history.push_back(metrics);
    }
}

void Telemetry::ExportFastestLapAsync(const std::string& filename) {
    std::lock_guard<std::mutex> lock(dataMutex);
    if (fastestLapBuffer.empty()) return;
    std::vector<TelemetrySample> bufferCopy = fastestLapBuffer;
    fastestLapFuture = std::async(std::launch::async, [bufferCopy, filename]() {
        std::ofstream file(filename, std::ios::out);
        if (file.is_open()) {
            file << "tick,lap_tick,distance,x,y,speed,v_long,v_lat,angular_vel,throttle_input,steer_input,"
                 << "long_accel,lat_accel,kamm_util,slip_angle,jerk_long,jerk_lat,track_offset,"
                 << "lidar_0,lidar_1,lidar_2,lidar_3,lidar_4,sens_steer_front,sens_steer_lat\n";
            for (const auto& s : bufferCopy) {
                file << s.tick << ","
                     << s.lapTick << ","
                     << s.distance << ","
                     << s.position.x << ","
                     << s.position.y << ","
                     << s.speed << ","
                     << s.vLong << ","
                     << s.vLat << ","
                     << s.angularVel << ","
                     << s.throttleInput << ","
                     << s.steerInput << ","
                     << s.longAccel << ","
                     << s.latAccel << ","
                     << s.kammUtil << ","
                     << s.slipAngle << ","
                     << s.jerkLong << ","
                     << s.jerkLat << ","
                     << s.trackOffset << ","
                     << s.sensorDistances[0] << ","
                     << s.sensorDistances[1] << ","
                     << s.sensorDistances[2] << ","
                     << s.sensorDistances[3] << ","
                     << s.sensorDistances[4] << ","
                     << s.sensSteerFront << ","
                     << s.sensSteerLat << "\n";
            }
            file.close();
        }
    });
}

void Telemetry::ExportDataAsync(const std::string& filename) {
    // Copy the history so the asynchronous thread doesn't have concurrency issues
    std::vector<GenerationMetrics> historyCopy = history;
    
    exportFuture = std::async(std::launch::async, [historyCopy, filename]() {
        std::ofstream file(filename, std::ios::out);
        if (file.is_open()) {
            file << "Generation,MaxFitness,MinFitness,AvgFitness,SurvivalRate,AvgLifeTime,WeightVariance,CurriculumPhase,BenchmarkFitness,ProceduralFitness,TotalLapsCompleted,TopSpeedAvg,SteeringSmoothness,ActiveMutationRate,BestLapTime,KammUtilAvg,SlipAngleAvg,ControlJerkAvg,TrackWidthUtil,NeuralSaturation\n";
            for (const auto& m : historyCopy) {
                file << m.generation << ","
                     << m.maxFitness << ","
                     << m.minFitness << ","
                     << m.avgFitness << ","
                     << m.survivalRate << ","
                     << m.avgLifeTime << ","
                     << m.weightVariance << ","
                     << m.curriculumPhase << ","
                     << m.benchmarkFitness << ","
                     << m.proceduralFitness << ","
                     << m.totalLapsCompleted << ","
                     << m.topSpeedAvg << ","
                     << m.steeringSmoothness << ","
                     << m.activeMutationRate << ","
                     << m.bestLapTime << ","
                     << m.kammUtilizationAvg << ","
                     << m.slipAngleAvg << ","
                     << m.controlJerkAvg << ","
                     << m.trackWidthUtil << ","
                     << m.neuralSaturation << "\n";
            }
            file.close();
        }
    });
}

void Telemetry::Flush(const std::string& filename) {
    if (exportFuture.valid()) {
        exportFuture.wait();
    }
    if (fastestLapFuture.valid()) {
        fastestLapFuture.wait();
    }
    std::lock_guard<std::mutex> lock(dataMutex);
    std::ofstream file(filename, std::ios::out);
    if (file.is_open()) {
        file << "Generation,MaxFitness,MinFitness,AvgFitness,SurvivalRate,AvgLifeTime,WeightVariance,CurriculumPhase,BenchmarkFitness,ProceduralFitness,TotalLapsCompleted,TopSpeedAvg,SteeringSmoothness,ActiveMutationRate,BestLapTime,KammUtilAvg,SlipAngleAvg,ControlJerkAvg,TrackWidthUtil,NeuralSaturation\n";
        for (const auto& m : history) {
            file << m.generation << ","
                 << m.maxFitness << ","
                 << m.minFitness << ","
                 << m.avgFitness << ","
                 << m.survivalRate << ","
                 << m.avgLifeTime << ","
                 << m.weightVariance << ","
                 << m.curriculumPhase << ","
                 << m.benchmarkFitness << ","
                 << m.proceduralFitness << ","
                 << m.totalLapsCompleted << ","
                 << m.topSpeedAvg << ","
                 << m.steeringSmoothness << ","
                 << m.activeMutationRate << ","
                 << m.bestLapTime << ","
                 << m.kammUtilizationAvg << ","
                 << m.slipAngleAvg << ","
                 << m.controlJerkAvg << ","
                 << m.trackWidthUtil << ","
                 << m.neuralSaturation << "\n";
        }
        file.close();
    }
}

void Telemetry::DrawDashboard(int screenWidth, int screenHeight) {
    if (history.empty()) return;

    int panelWidth = 510;
    int panelHeight = 310;
    int padding = 20;
    int x = padding;
    int y = screenHeight - panelHeight - padding;

    // Dashboard background
    DrawRectangle(x, y, panelWidth, panelHeight, Fade(BLACK, 0.88f));
    DrawRectangleLines(x, y, panelWidth, panelHeight, Fade(WHITE, 0.6f));

    DrawText("TELEMETRY AND ADVANCED ANALYTICS (F1 GRADE)", x + 10, y + 10, 17, SKYBLUE);

    // Find maximums to scale the graph
    float maxFit = 0.0001f; // Avoid division by zero
    for (const auto& m : history) {
        if (m.maxFitness > maxFit) maxFit = m.maxFitness;
    }

    int graphX = x + 35;
    int graphY = y + 42;
    int graphW = panelWidth - 55;
    int graphH = 145;

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

    // Column 1: Fitness & Lap Time
    DrawText(TextFormat("Max Fit: %.0f", history.back().maxFitness), x + 10, graphY + graphH + 8, 13, GREEN);
    DrawText(TextFormat("Avg Fit: %.0f", history.back().avgFitness), x + 10, graphY + graphH + 24, 13, YELLOW);
    DrawText(TextFormat("Min Fit: %.0f", history.back().minFitness), x + 10, graphY + graphH + 40, 13, RED);
    DrawText(TextFormat("Best Lap: %.2fs", history.back().bestLapTime), x + 10, graphY + graphH + 56, 13, SKYBLUE);

    // Column 2: Population & Evolution
    int col2 = x + 165;
    DrawText(TextFormat("Survival: %.1f%%", history.back().survivalRate * 100.0f), col2, graphY + graphH + 8, 13, WHITE);
    DrawText(TextFormat("Avg Life: %.0f t", history.back().avgLifeTime), col2, graphY + graphH + 24, 13, WHITE);
    DrawText(TextFormat("Gen. Var.: %.3f", history.back().weightVariance), col2, graphY + graphH + 40, 13, ORANGE);
    DrawText(TextFormat("Laps: %d", history.back().totalLapsCompleted), col2, graphY + graphH + 56, 13, GOLD);

    // Column 3: Vehicle Dynamics & XAI
    int col3 = x + 325;
    Color kammColor = (history.back().kammUtilizationAvg > 0.85f) ? RED : GREEN;
    DrawText(TextFormat("Kamm Util: %.1f%%", history.back().kammUtilizationAvg * 100.0f), col3, graphY + graphH + 8, 13, kammColor);
    DrawText(TextFormat("Slip: %.1f deg", history.back().slipAngleAvg), col3, graphY + graphH + 24, 13, SKYBLUE);
    DrawText(TextFormat("Jerk: %.3f", history.back().controlJerkAvg), col3, graphY + graphH + 40, 13, LIGHTGRAY);
    DrawText(TextFormat("Track Util: %.1f%%", history.back().trackWidthUtil * 100.0f), col3, graphY + graphH + 56, 13, YELLOW);
    DrawText(TextFormat("Neural Sat: %.1f%%", history.back().neuralSaturation * 100.0f), col3, graphY + graphH + 72, 13, (history.back().neuralSaturation > 0.5f ? RED : GREEN));
}

void Telemetry::DrawHeatmap() {
    // Draw the points where cars have crashed
    for (const auto& cp : crashPoints) {
        DrawCircleV(cp, 4.0f, Fade(RED, 0.2f)); // Semi-transparent so accumulation looks brighter
    }
}
