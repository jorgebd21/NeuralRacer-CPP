#pragma once

#include "Brain.h"
#include "raylib.h"
#include <vector>
#include <utility>

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

    Car(float startX, float startY, float startRot = 0.0f);
    
    void Reset(float startX, float startY, float startRot = 0.0f);
    
    void UpdatePhysics(float inputAcelerar, float inputGiro, const std::vector<std::pair<Vector2, Vector2>>& trackWalls, int timer);
};