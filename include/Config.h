#pragma once

/**
 * @brief Contiene todos los parámetros de configuración globales de la aplicación.
 * 
 * Agrupa constantes que ajustan la física, la simulación genética y las 
 * configuraciones de ventana. Se encapsulan en un namespace para evitar polución.
 */
namespace Config {
    constexpr int SCREEN_WIDTH = 1280;
    constexpr int SCREEN_HEIGHT = 768;
    constexpr int POPULATION_SIZE = 100;
    constexpr int NUM_MEJORES = 10;
    constexpr int MAX_MUTACION = 50;
    inline int MUTACION = 50;
    constexpr float TASA_CAIDA = 0.005f;
    constexpr int MAX_GENERATION_TIME = 2000;
    constexpr float SENSOR_ANGLES[5] = {-90.0f, -45.0f, 0.0f, 45.0f, 90.0f};

    constexpr float CAR_TURN_SPEED = 1.5f;
    constexpr float CAR_TURN_PENALTY = 0.05f;
    constexpr float CAR_MAX_SENSOR_DIST = 400.0f;
    constexpr int CAR_STALL_TIME_THRESHOLD = 100;
    constexpr float CAR_STALL_SPEED_THRESHOLD = -0.2f;
    constexpr int CAR_MAX_TIME_WITHOUT_CHECKPOINT = 800;

    constexpr float CAR_MASS = 1200.0f;
    constexpr float ENGINE_POWER = 35.0f;
    constexpr float BRAKING_POWER = 45.0f;
    constexpr float DRAG_MULTIPLIER = 0.35f;
    constexpr float CORNERING_STIFFNESS = 1000.0f;
    constexpr float WEIGHT_TRANSFER_FACTOR = 0.15f;

    constexpr float CAR_HALF_WIDTH = 10.0f;
    constexpr float CAR_HALF_LENGTH = 20.0f;
    
    constexpr float SIM_START_POS_X = 400.0f;
    constexpr float SIM_START_POS_Y = 650.0f;
}
