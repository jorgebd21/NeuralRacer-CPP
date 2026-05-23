#include "./include/raylib.h"
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <ctime>
#include <algorithm>
#include <random>
#include "./include/TrackGenerator.h"

// --- CONFIGURACIÓN BÁSICA ---
const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 768;
const int POPULATION_SIZE = 100; // Número de coches por generación
const int NUM_MEJORES = 10; // Número de coches mejores para la siguiente generación
const int MUTACION = 10; // %10 de mutacion
const int NODOS_OCULTOS = 8;
const int NODOS_ENTRADA = 6;
const int NODOS_SALIDA = 2;
const int MAX_GENERATION_TIME = 3000;

// ====================================================================
// ESPACIO DE TRABAJO ML: RED NEURONAL Y MUTACIÓN
// ====================================================================

struct Brain {
    float peso_entrada_oculta[NODOS_OCULTOS][NODOS_ENTRADA];
    float sesgos_oculta[NODOS_OCULTOS];
    float peso_oculta_salida[NODOS_SALIDA][NODOS_OCULTOS];
    float sesgos_salida[NODOS_SALIDA];
    
    Brain() {
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            sesgos_oculta[i] = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
            for(int j = 0; j < NODOS_ENTRADA; j++) {
                peso_entrada_oculta[i][j] = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
            }
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            sesgos_salida[i] = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
            for(int j = 0; j < NODOS_OCULTOS; j++) {
                peso_oculta_salida[i][j] = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
            }
        }
    }

    // Función de feedforward
    void Evaluate(float sensorDistances[5], float velocidad, float &outAcelerar, float &outGiro) {
        float entrada[6] = {sensorDistances[0], sensorDistances[1], sensorDistances[2], sensorDistances[3], sensorDistances[4], velocidad};
        
        float valores_ocultos[NODOS_OCULTOS];
        for(int i=0; i<NODOS_OCULTOS; i++) {
            valores_ocultos[i] = 0.0f;
            for(int j=0; j<NODOS_ENTRADA; j++) {
                valores_ocultos[i] += entrada[j] * peso_entrada_oculta[i][j];
            }
            valores_ocultos[i] += sesgos_oculta[i];
            valores_ocultos[i] = tanh(valores_ocultos[i]);
        }

        float valores_salida[NODOS_SALIDA];
        for(int i=0; i<NODOS_SALIDA; i++) {
            valores_salida[i] = 0.0f;
            for(int j=0; j<NODOS_OCULTOS; j++) {
                valores_salida[i] += valores_ocultos[j] * peso_oculta_salida[i][j];
            }
            valores_salida[i] += sesgos_salida[i];
            valores_salida[i] = tanh(valores_salida[i]);
        }

        outAcelerar = valores_salida[0];
        outGiro = valores_salida[1]; 
    }
    
    float MutateGaussian() {
        static std::random_device rd; 
        static std::mt19937 generador(rd()); 
        std::normal_distribution<float> distribucion(0.0f, 0.1f);
        return distribucion(generador);
    }
};

// ====================================================================
// --- ESTRUCTURA DEL COCHE ---
struct Car {
    Vector2 position;
    float rotation; // En grados
    float speed;
    float sensorDistances[5]; // Lo que leerá tu Red Neuronal
    bool isCrashed;
    
    // Métricas para tu algoritmo genético
    float fitness;
    int timeAlive;
    float distanceTraveled;
    
    Brain brain; // Cada coche tiene su propio cerebro

    Car(float startX, float startY, float startRot = 0.0f) {
        Reset(startX, startY, startRot);
    }
    
    void Reset(float startX, float startY, float startRot = 0.0f) {
        position = {startX, startY};
        rotation = startRot;
        speed = 0.0f;
        isCrashed = false;
        fitness = 0.0f;
        timeAlive = 0;
        distanceTraveled = 0.0f;
        for(int i=0; i<5; i++) sensorDistances[i] = 100.0f;
    }
};

// ====================================================================
// ESPACIO DE TRABAJO ML: EVOLUCIÓN DE LA POBLACIÓN
// ====================================================================

void GuardarMejoresCerebros(const std::vector<Car>& population) {
    std::ofstream mejoresFile("mejores.txt", std::ios::out);
    for(int n = 0; n < NUM_MEJORES; n++) {
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            for(int j = 0; j < NODOS_ENTRADA; j++) {
                mejoresFile << population[n].brain.peso_entrada_oculta[i][j] << " ";
            }
        }
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            mejoresFile << population[n].brain.sesgos_oculta[i] << " ";
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            for(int j = 0; j < NODOS_OCULTOS; j++) {
                mejoresFile << population[n].brain.peso_oculta_salida[i][j] << " ";
            }
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            mejoresFile << population[n].brain.sesgos_salida[i] << " ";
        }
        mejoresFile << std::endl;
    }
}

bool CargarMejoresCerebros(std::vector<Car>& population) {
    std::ifstream file("mejores.txt");
    if (!file.is_open()) return false;

    // 1. Cargamos los 10 mejores directamente del txt
    for(int n = 0; n < NUM_MEJORES && n < population.size(); n++) {
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            for(int j = 0; j < NODOS_ENTRADA; j++) {
                if (!(file >> population[n].brain.peso_entrada_oculta[i][j])) return false;
            }
        }
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            if (!(file >> population[n].brain.sesgos_oculta[i])) return false;
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            for(int j = 0; j < NODOS_OCULTOS; j++) {
                if (!(file >> population[n].brain.peso_oculta_salida[i][j])) return false;
            }
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            if (!(file >> population[n].brain.sesgos_salida[i])) return false;
        }
    }

    // 2. Generamos el "Frankenstein" para los 40 restantes basándonos en los 10 que acabamos de leer.
    for(int n = NUM_MEJORES; n < POPULATION_SIZE && n < population.size(); n++) {
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            for(int j = 0; j < NODOS_ENTRADA; j++) {
                population[n].brain.peso_entrada_oculta[i][j] = population[rand()%10].brain.peso_entrada_oculta[i][j];
                if(rand()%100<MUTACION) population[n].brain.peso_entrada_oculta[i][j] += population[n].brain.MutateGaussian();
            }
        }
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            population[n].brain.sesgos_oculta[i] = population[rand()%10].brain.sesgos_oculta[i];
            if(rand()%100<MUTACION) population[n].brain.sesgos_oculta[i] += population[n].brain.MutateGaussian();
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            for(int j = 0; j < NODOS_OCULTOS; j++) {
                population[n].brain.peso_oculta_salida[i][j] = population[rand()%10].brain.peso_oculta_salida[i][j];
                if(rand()%100<MUTACION) population[n].brain.peso_oculta_salida[i][j] += population[n].brain.MutateGaussian();
            }
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            population[n].brain.sesgos_salida[i] = population[rand()%10].brain.sesgos_salida[i];
            if(rand()%100<MUTACION) population[n].brain.sesgos_salida[i] += population[n].brain.MutateGaussian();
        }
    }
    
    return true;
}

void EvolvePopulation(std::vector<Car>& population, Vector2 startPosition, float startRotation) {

    std::sort(population.begin(), population.end(), [](const Car& a, const Car& b) {
        return a.fitness > b.fitness;
    });

    GuardarMejoresCerebros(population);
    
    for(int n = NUM_MEJORES; n < POPULATION_SIZE; n++) {
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            for(int j = 0; j < NODOS_ENTRADA; j++) {
                population[n].brain.peso_entrada_oculta[i][j] = population[rand()%10].brain.peso_entrada_oculta[i][j];
                if(rand()%100<MUTACION){
                    population[n].brain.peso_entrada_oculta[i][j] += population[n].brain.MutateGaussian();
                }
            }
        }
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            population[n].brain.sesgos_oculta[i] = population[rand()%10].brain.sesgos_oculta[i];
            if(rand()%100<MUTACION){
                population[n].brain.sesgos_oculta[i] += population[n].brain.MutateGaussian();
            }
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            for(int j = 0; j < NODOS_OCULTOS; j++) {
                population[n].brain.peso_oculta_salida[i][j] = population[rand()%10].brain.peso_oculta_salida[i][j];
                if(rand()%100<MUTACION){
                    population[n].brain.peso_oculta_salida[i][j] += population[n].brain.MutateGaussian();
                }
            }
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            population[n].brain.sesgos_salida[i] = population[rand()%10].brain.sesgos_salida[i];
            if(rand()%100<MUTACION){
                population[n].brain.sesgos_salida[i] += population[n].brain.MutateGaussian();
            }
        }
    }

    for (auto& car : population) {
        car.Reset(startPosition.x, startPosition.y, startRotation);
    }
    std::cout << "Generación terminada. ¡Evolucionando población!" << std::endl;
}

// ====================================================================
// FUNCIONES AUXILIARES FÍSICAS Y DIBUJO
// ====================================================================

bool GetLineIntersectionDist(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, float &outDist) {
    float den = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
    if (den == 0) return false;
    float t = ((p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)) / den;
    float u = -((p1.x - p2.x) * (p1.y - p3.y) - (p1.y - p2.y) * (p1.x - p3.x)) / den;
    if (t > 0 && t < 1 && u > 0 && u < 1) {
        Vector2 pt = { p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y) };
        outDist = sqrt(pow(pt.x - p1.x, 2) + pow(pt.y - p1.y, 2));
        return true;
    }
    return false;
}

Vector2 GetCatmullRomPoint(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3, float t) {
    float t2 = t * t;
    float t3 = t2 * t;
    Vector2 result;
    result.x = 0.5f * ((2.0f * p1.x) + (-p0.x + p2.x) * t + (2.0f * p0.x - 5.0f * p1.x + 4.0f * p2.x - p3.x) * t2 + (-p0.x + 3.0f * p1.x - 3.0f * p2.x + p3.x) * t3);
    result.y = 0.5f * ((2.0f * p1.y) + (-p0.y + p2.y) * t + (2.0f * p0.y - 5.0f * p1.y + 4.0f * p2.y - p3.y) * t2 + (-p0.y + 3.0f * p1.y - 3.0f * p2.y + p3.y) * t3);
    return result;
}

std::vector<Vector2> GenerateSplinePoints(const std::vector<Vector2>& points, int segmentsPerCurve = 15) {
    std::vector<Vector2> densePoints;
    if (points.size() < 3) return densePoints;
    int n = points.size();
    for (int i = 0; i < n; i++) {
        Vector2 p0 = points[(i - 1 + n) % n];
        Vector2 p1 = points[i];
        Vector2 p2 = points[(i + 1) % n];
        Vector2 p3 = points[(i + 2) % n];
        for (int j = 0; j < segmentsPerCurve; j++) {
            float t = (float)j / segmentsPerCurve;
            Vector2 pt = GetCatmullRomPoint(p0, p1, p2, p3, t);
            densePoints.push_back(pt);
        }
    }
    return densePoints;
}

void SmoothPoints(std::vector<Vector2>& pts, int radius, int passes) {
    int n = pts.size();
    for (int pass = 0; pass < passes; pass++) {
        std::vector<Vector2> tmp(n);
        for (int i = 0; i < n; i++) {
            Vector2 acc = {0, 0};
            int count = 0;
            for (int j = -radius; j <= radius; j++) {
                int idx = (i + j + n) % n;
                acc.x += pts[idx].x;
                acc.y += pts[idx].y;
                count++;
            }
            tmp[i] = {acc.x / count, acc.y / count};
        }
        pts = tmp;
    }
}

bool GetSegmentIntersection(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, Vector2& outIntersection) {
    float den = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
    if (std::abs(den) < 1e-6f) return false;
    float t = ((p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)) / den;
    float u = -((p1.x - p2.x) * (p1.y - p3.y) - (p1.y - p2.y) * (p1.x - p3.x)) / den;
    if (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f) {
        outIntersection = { p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y) };
        return true;
    }
    return false;
}

void RemoveSelfIntersections(std::vector<Vector2>& pts) {
    int n = pts.size();
    if (n < 4) return;
    bool foundIntersection = true;
    int maxIterations = 5000;
    int iterations = 0;
    while (foundIntersection && iterations++ < maxIterations) {
        foundIntersection = false;
        n = pts.size();
        for (int i = 0; i < n; i++) {
            Vector2 p1 = pts[i];
            Vector2 p2 = pts[(i + 1) % n];
            int maxSearch = std::min(n - 1, 150); 
            for (int k = 2; k < maxSearch; k++) {
                int j = (i + k) % n;
                Vector2 p3 = pts[j];
                Vector2 p4 = pts[(j + 1) % n];
                Vector2 intersectPt;
                if (GetSegmentIntersection(p1, p2, p3, p4, intersectPt)) {
                    std::vector<Vector2> newPts;
                    newPts.reserve(n);
                    if (i < j) {
                        for (int idx = 0; idx <= i; idx++) newPts.push_back(pts[idx]);
                        newPts.push_back(intersectPt);
                        for (int idx = j + 1; idx < n; idx++) newPts.push_back(pts[idx]);
                    } else {
                        for (int idx = j + 1; idx <= i; idx++) newPts.push_back(pts[idx]);
                        newPts.push_back(intersectPt);
                    }
                    pts = newPts;
                    foundIntersection = true;
                    break;
                }
            }
            if (foundIntersection) break;
        }
    }
}

void GenerateBordersFromCenterLine(const std::vector<Vector2>& centerPoints, float trackWidth, std::vector<std::pair<Vector2, Vector2>>& outWalls) {
    int n = centerPoints.size();
    if (n < 2) return;
    float halfWidth = trackWidth / 2.0f;
    std::vector<Vector2> outerPoints(n);
    std::vector<Vector2> innerPoints(n);
    int step = 8;
    for (int i = 0; i < n; i++) {
        Vector2 prev = centerPoints[(i - step + n) % n];
        Vector2 next = centerPoints[(i + step) % n];
        Vector2 dir = {next.x - prev.x, next.y - prev.y};
        float length = sqrt(dir.x * dir.x + dir.y * dir.y);
        if (length < 0.0001f) dir = {1.0f, 0.0f};
        else { dir.x /= length; dir.y /= length; }
        Vector2 normal = {-dir.y, dir.x};
        outerPoints[i] = {centerPoints[i].x + normal.x * halfWidth, centerPoints[i].y + normal.y * halfWidth};
        innerPoints[i] = {centerPoints[i].x - normal.x * halfWidth, centerPoints[i].y - normal.y * halfWidth};
    }
    RemoveSelfIntersections(outerPoints);
    RemoveSelfIntersections(innerPoints);
    SmoothPoints(outerPoints, 4, 5);
    SmoothPoints(innerPoints, 4, 5);
    RemoveSelfIntersections(outerPoints);
    RemoveSelfIntersections(innerPoints);
    for (size_t i = 0; i < outerPoints.size(); i++) {
        size_t nextIdx = (i + 1) % outerPoints.size();
        outWalls.push_back({outerPoints[i], outerPoints[nextIdx]});
    }
    for (size_t i = 0; i < innerPoints.size(); i++) {
        size_t nextIdx = (i + 1) % innerPoints.size();
        outWalls.push_back({innerPoints[i], innerPoints[nextIdx]});
    }
}

void LoadTrackFromFile(const std::string& filename, std::vector<std::pair<Vector2, Vector2>>& outWalls, Vector2& outStartPos) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: No se pudo abrir " << filename << std::endl;
        return;
    }
    std::vector<Vector2> centerPoints;
    std::string line;
    bool isFirstLine = true;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        float x, y;
        if (ss >> x >> y) {
            if (isFirstLine) { outStartPos = {x, y}; isFirstLine = false; }
            else { centerPoints.push_back({x, y}); }
        }
    }
    if (centerPoints.size() >= 3) {
        std::vector<Vector2> denseCenterLine = GenerateSplinePoints(centerPoints, 50);
        GenerateBordersFromCenterLine(denseCenterLine, 65.0f, outWalls);
    }
}

// ----------------------------------------------------------------------------------
// HELPER PARA ACTUALIZAR FÍSICA DE CUALQUIER COCHE
// ----------------------------------------------------------------------------------
void UpdateCarPhysics(Car& car, float inputAcelerar, float inputGiro, const std::vector<std::pair<Vector2, Vector2>>& trackWalls, float sensorAngles[5], int timer) {
    if (car.isCrashed) return;
    car.timeAlive++;
    
    float maxSpeedForward = 4.0f;
    float maxSpeedBackward = -1.5f;
    float accelRate = 0.04f;
    float brakeRate = 0.1f;
    float friction = 0.015f;

    if (inputAcelerar > 0) car.speed += accelRate;
    else if (inputAcelerar < 0) car.speed -= brakeRate;
    else {
        if (car.speed > 0) { car.speed -= friction; if (car.speed < 0) car.speed = 0; }
        else if (car.speed < 0) { car.speed += friction; if (car.speed > 0) car.speed = 0; }
    }

    if (car.speed > maxSpeedForward) car.speed = maxSpeedForward;
    if (car.speed < maxSpeedBackward) car.speed = maxSpeedBackward;

    float turnSpeed = 3.5f;
    if (car.speed != 0) {
        float direction = (car.speed > 0) ? 1.0f : -1.0f;
        car.rotation += inputGiro * turnSpeed * direction * (std::abs(car.speed) / maxSpeedForward); 
    } 

    car.position.x += cos(car.rotation * DEG2RAD) * car.speed;
    car.position.y += sin(car.rotation * DEG2RAD) * car.speed;

    if (car.speed > 0) {
        car.distanceTraveled += car.speed - (std::abs(inputGiro) * 0.5f); 
    }

    float maxSensorDist = 150.0f;
    for (int i = 0; i < 5; i++) {
        car.sensorDistances[i] = maxSensorDist;
        float rayAngle = (car.rotation + sensorAngles[i]) * DEG2RAD;
        Vector2 rayEnd = { car.position.x + cos(rayAngle) * maxSensorDist, car.position.y + sin(rayAngle) * maxSensorDist };

        for (auto wall : trackWalls) {
            float dist;
            if (GetLineIntersectionDist(car.position, rayEnd, wall.first, wall.second, dist)) {
                if (dist < car.sensorDistances[i]) car.sensorDistances[i] = dist;
            }
        }
        
        car.fitness = car.distanceTraveled - car.timeAlive;
        if (car.sensorDistances[i] < 5.0f || (timer > 100 && car.fitness < 0) || car.speed < -0.2f) {
            car.isCrashed = true; 
        }
    }
}

// ----------------------------------------------------------------------------------
// ESTADOS DEL JUEGO
// ----------------------------------------------------------------------------------
enum GameState { MENU, TRAINING, EXHIBITION };

int main(int argc, char* argv[]) {
    srand(time(NULL));
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Simulador Genético - IA Autónoma");
    SetTargetFPS(60);

    std::string trackFile = "track1_facil.txt";
    if (argc > 1) trackFile = argv[1];

    Vector2 startPosition = {400.0f, 650.0f};
    float startRotation = 0.0f;
    std::vector<std::pair<Vector2, Vector2>> trackWalls;
    LoadTrackFromFile(trackFile, trackWalls, startPosition);
    if (trackWalls.empty()) trackWalls.push_back({{100, 100}, {900, 100}});

    float sensorAngles[5] = {-90.0f, -45.0f, 0.0f, 45.0f, 90.0f};
    
    std::vector<Vector2> puntosProcedurales;
    
    // --- Variables MODO TRAINING ---
    std::vector<Car> population;
    for (int i = 0; i < POPULATION_SIZE; i++) population.push_back(Car(startPosition.x, startPosition.y, startRotation));
    CargarMejoresCerebros(population);
    int generationTimer = 0;
    int generationCount = 0;
    int simSpeed = 1;

    // --- Variables MODO EXHIBICION ---
    Car playerCar(startPosition.x, startPosition.y, startRotation);
    Car aiCar(startPosition.x, startPosition.y, startRotation);
    int exhibitionResult = 0; // 0=jugando, 1=player gana, 2=ai gana

    GameState currentState = MENU;

    while (!WindowShouldClose()) {
        
        // =========================================================
        // ESTADO 1: MENU PRINCIPAL
        // =========================================================
        if (currentState == MENU) {
            if (IsKeyPressed(KEY_T)) currentState = TRAINING;
            if (IsKeyPressed(KEY_E)) {
                playerCar.Reset(startPosition.x, startPosition.y, startRotation);
                aiCar.Reset(startPosition.x, startPosition.y, startRotation);
                // Cargar mejor IA
                std::vector<Car> temp(1, Car(startPosition.x, startPosition.y, startRotation));
                if (CargarMejoresCerebros(temp)) {
                    aiCar = temp[0];
                }
                aiCar.Reset(startPosition.x, startPosition.y, startRotation);
                exhibitionResult = 0;
                currentState = EXHIBITION;
            }
            if (IsKeyPressed(KEY_P)) {
                puntosProcedurales = TrackGenerator::GenerateProceduralCenterPoints();
                std::vector<Vector2> denseCenterLine = GenerateSplinePoints(puntosProcedurales, 50);
                trackWalls.clear();
                GenerateBordersFromCenterLine(denseCenterLine, 65.0f, trackWalls);
                if (!puntosProcedurales.empty()) {
                    int n = puntosProcedurales.size();
                    float maxDist = 0;
                    int bestIdx = 0;
                    for (int i = 0; i < n; i++) {
                        Vector2 p1 = puntosProcedurales[i];
                        Vector2 p2 = puntosProcedurales[(i+1)%n];
                        float d = (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y);
                        if (d > maxDist) {
                            maxDist = d;
                            bestIdx = i;
                        }
                    }
                    Vector2 p1 = puntosProcedurales[bestIdx];
                    Vector2 p2 = puntosProcedurales[(bestIdx+1)%n];
                    startPosition.x = (p1.x + p2.x) / 2.0f;
                    startPosition.y = (p1.y + p2.y) / 2.0f;
                    startRotation = atan2(p2.y - p1.y, p2.x - p1.x) * (180.0f / PI);
                }
                for (auto& car : population) car.Reset(startPosition.x, startPosition.y, startRotation);
                playerCar.Reset(startPosition.x, startPosition.y, startRotation);
                aiCar.Reset(startPosition.x, startPosition.y, startRotation);
            }
            if (IsKeyPressed(KEY_G)) {
                if (!puntosProcedurales.empty()) {
                    int counter = 1;
                    std::string filename;
                    while (true) {
                        filename = "pista_procedural" + std::to_string(counter) + ".txt";
                        std::ifstream f(filename.c_str());
                        if (!f.good()) {
                            break;
                        }
                        counter++;
                    }
                    TrackGenerator::SaveTrackToFile(puntosProcedurales, filename);
                }
            }
            
            BeginDrawing();
            ClearBackground(DARKGRAY);
            DrawText("SIMULADOR GENETICO", SCREEN_WIDTH/2 - 250, 200, 40, WHITE);
            DrawText("[ T ] MODO ENTRENAMIENTO (IA vs IA)", SCREEN_WIDTH/2 - 200, 350, 20, LIGHTGRAY);
            DrawText("[ E ] MODO EXHIBICION (Jugador vs Mejor IA)", SCREEN_WIDTH/2 - 200, 400, 20, LIGHTGRAY);
            DrawText("[ P ] GENERAR PISTA PROCEDURAL", SCREEN_WIDTH/2 - 200, 450, 20, YELLOW);
            if (!puntosProcedurales.empty()) {
                DrawText("[ G ] GUARDAR PISTA ACTUAL", SCREEN_WIDTH/2 - 200, 500, 20, GREEN);
            }
            EndDrawing();
        }
        
        // =========================================================
        // ESTADO 2: MODO ENTRENAMIENTO (El clásico)
        // =========================================================
        else if (currentState == TRAINING) {
            if (IsKeyPressed(KEY_SPACE)) simSpeed = (simSpeed == 1) ? 50 : 1;
            if (IsKeyPressed(KEY_M)) currentState = MENU;

            for (int s = 0; s < simSpeed; s++) {
                bool allCrashed = true;
                for (auto& car : population) {
                    if (!car.isCrashed) {
                        allCrashed = false;
                        float inputAcelerar = 0.0f, inputGiro = 0.0f;
                        car.brain.Evaluate(car.sensorDistances, car.speed, inputAcelerar, inputGiro);
                        UpdateCarPhysics(car, inputAcelerar, inputGiro, trackWalls, sensorAngles, generationTimer);
                    }
                }
                generationTimer++;
                if (allCrashed || generationTimer >= MAX_GENERATION_TIME) {
                    EvolvePopulation(population, startPosition, startRotation);
                    generationTimer = 0;
                    generationCount++;
                }
            }

            BeginDrawing();
            ClearBackground(DARKGRAY);
            for(int i=0; i<10; i++) {
                DrawRectangle(450, 600 + i * 10, 10, 10, (i % 2 == 0) ? WHITE : BLACK);
                DrawRectangle(460, 600 + i * 10, 10, 10, (i % 2 == 0) ? BLACK : WHITE);
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
                            float rayAngle = (car.rotation + sensorAngles[i]) * DEG2RAD;
                            Vector2 actualRayEnd = { car.position.x + cos(rayAngle) * car.sensorDistances[i], car.position.y + sin(rayAngle) * car.sensorDistances[i] };
                            DrawLineV(car.position, actualRayEnd, Fade(GREEN, 0.5f));
                            DrawCircleV(actualRayEnd, 3.0f, Fade(GREEN, 0.5f));
                        }
                    }
                }
            }

            int panelWidth = 250;
            DrawRectangle(SCREEN_WIDTH - panelWidth, 0, panelWidth, SCREEN_HEIGHT, Fade(BLACK, 0.85f));
            DrawText(TextFormat("Generacion: %d", generationCount), SCREEN_WIDTH - panelWidth + 15, 20, 20, WHITE);
            DrawText(TextFormat("Tiempo: %d / %d", generationTimer, MAX_GENERATION_TIME), SCREEN_WIDTH - panelWidth + 15, 50, 20, WHITE);
            DrawText(TextFormat("Vivos: %d / %d", aliveCount, POPULATION_SIZE), SCREEN_WIDTH - panelWidth + 15, 80, 20, WHITE);
            DrawText(TextFormat("Velocidad: %s", (simSpeed == 1) ? "NORMAL" : "MAX (x50)"), SCREEN_WIDTH - panelWidth + 15, 110, 15, (simSpeed == 1) ? GREEN : RED);
            DrawText("[ESPACIO] Cambiar vel", SCREEN_WIDTH - panelWidth + 15, 130, 10, LIGHTGRAY);
            DrawText("[M] Volver al Menu", SCREEN_WIDTH - panelWidth + 15, 145, 10, LIGHTGRAY);

            DrawText("TOP 10 FITNESS", SCREEN_WIDTH - panelWidth + 15, 180, 20, YELLOW);
            std::vector<Car> sortedPop = population;
            std::sort(sortedPop.begin(), sortedPop.end(), [](const Car& a, const Car& b) { return a.fitness > b.fitness; });
            for (int i = 0; i < 10; i++) {
                Color rowColor = (sortedPop[i].isCrashed) ? GRAY : WHITE;
                if (i == 0) rowColor = GOLD;
                DrawText(TextFormat("%d. Fit: %.1f", i + 1, sortedPop[i].fitness), SCREEN_WIDTH - panelWidth + 15, 210 + (i * 25), 18, rowColor);
            }
            EndDrawing();
        }

        // =========================================================
        // ESTADO 3: MODO EXHIBICIÓN (JUGADOR vs IA)
        // =========================================================
        else if (currentState == EXHIBITION) {
            if (IsKeyPressed(KEY_M)) currentState = MENU;
            if (IsKeyPressed(KEY_R) && exhibitionResult != 0) {
                playerCar.Reset(startPosition.x, startPosition.y, startRotation);
                aiCar.Reset(startPosition.x, startPosition.y, startRotation);
                exhibitionResult = 0;
            }

            if (exhibitionResult == 0) {
                // Actualizar Player
                float playerAcelerar = 0.0f, playerGiro = 0.0f;
                if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) playerAcelerar = 1.0f;
                if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) playerAcelerar = -1.0f;
                if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) playerGiro = 1.0f;
                if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) playerGiro = -1.0f;
                UpdateCarPhysics(playerCar, playerAcelerar, playerGiro, trackWalls, sensorAngles, 0);

                // Actualizar AI
                float aiAcelerar = 0.0f, aiGiro = 0.0f;
                aiCar.brain.Evaluate(aiCar.sensorDistances, aiCar.speed, aiAcelerar, aiGiro);
                UpdateCarPhysics(aiCar, aiAcelerar, aiGiro, trackWalls, sensorAngles, 0);

                // Check win condition
                if (playerCar.isCrashed && !aiCar.isCrashed) exhibitionResult = 2;
                else if (aiCar.isCrashed && !playerCar.isCrashed) exhibitionResult = 1;
                else if (aiCar.isCrashed && playerCar.isCrashed) exhibitionResult = 2; // Empate pierde player
            }

            BeginDrawing();
            ClearBackground(DARKGRAY);
            for(int i=0; i<10; i++) {
                DrawRectangle(450, 600 + i * 10, 10, 10, (i % 2 == 0) ? WHITE : BLACK);
                DrawRectangle(460, 600 + i * 10, 10, 10, (i % 2 == 0) ? BLACK : WHITE);
            }
            for (auto wall : trackWalls) {
                DrawLineEx(wall.first, wall.second, 6.0f, WHITE);
            }

            // Dibujar IA (Rojo)
            if (!aiCar.isCrashed) DrawRectanglePro({ aiCar.position.x, aiCar.position.y, 20.0f, 10.0f }, { 10.0f, 5.0f }, aiCar.rotation, RED);
            // Dibujar Player (Azul)
            if (!playerCar.isCrashed) DrawRectanglePro({ playerCar.position.x, playerCar.position.y, 20.0f, 10.0f }, { 10.0f, 5.0f }, playerCar.rotation, BLUE);

            // Mensajes de resultado
            if (exhibitionResult == 1) {
                DrawText("¡HAS GANADO A LA IA!", SCREEN_WIDTH/2 - 200, 200, 40, GREEN);
                DrawText("Pulsa R para revancha", SCREEN_WIDTH/2 - 150, 250, 20, WHITE);
            } else if (exhibitionResult == 2) {
                DrawText("LA IA TE HA DESTRUIDO", SCREEN_WIDTH/2 - 200, 200, 40, RED);
                DrawText("Pulsa R para reintentar", SCREEN_WIDTH/2 - 150, 250, 20, WHITE);
            }

            DrawText("[M] Volver al Menú Principal", 20, 20, 20, LIGHTGRAY);
            EndDrawing();
        }
    }

    CloseWindow();
    return 0;
}
