#pragma once

#include "Brain.h"
#include "raylib.h"
#include <vector>
#include <utility>

/**
 * @brief Estructura que representa la entidad de un coche en la simulación.
 * 
 * Contiene tanto los componentes físicos (posición, velocidad) como 
 * los elementos necesarios para el algoritmo genético (brain, fitness).
 */
struct Car {
    Vector2 position;
    float rotation;
    float speed;
    float sensorDistances[5];
    bool isCrashed;
    
    float fitness;
    int timeAlive;
    float distanceTraveled;
    
    Brain brain;

    /**
     * @brief Constructor que inicializa el coche en una posición y rotación inicial.
     * 
     * @param startX Posición inicial en el eje X.
     * @param startY Posición inicial en el eje Y.
     * @param startRot Rotación inicial en grados (por defecto 0.0f).
     */
    Car(float startX, float startY, float startRot = 0.0f);
    
    /**
     * @brief Reinicia el estado físico del coche y resetea las métricas genéticas.
     * 
     * @param startX Nueva posición inicial en el eje X.
     * @param startY Nueva posición inicial en el eje Y.
     * @param startRot Nueva rotación inicial en grados (por defecto 0.0f).
     */
    void Reset(float startX, float startY, float startRot = 0.0f);
    
    /**
     * @brief Actualiza la física del coche según sus entradas y calcula las colisiones.
     * 
     * @param inputAcelerar Valor de aceleración (-1.0 a 1.0).
     * @param inputGiro Valor de giro (-1.0 a 1.0).
     * @param trackWalls Vector con las paredes del circuito para detección de colisión y sensores.
     * @param timer El tiempo de vida actual del coche en la simulación.
     */
    void UpdatePhysics(float inputAcelerar, float inputGiro, const std::vector<std::pair<Vector2, Vector2>>& trackWalls, int timer);
};