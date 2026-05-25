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
    constexpr int MUTACION = 10;
    constexpr int MAX_GENERATION_TIME = 2000;
    constexpr float SENSOR_ANGLES[5] = {-90.0f, -45.0f, 0.0f, 45.0f, 90.0f};

    constexpr float CAR_MAX_SPEED_FORWARD = 4.0f;
    constexpr float CAR_MAX_SPEED_BACKWARD = -1.5f;
    constexpr float CAR_ACCEL_RATE = 0.04f;
    constexpr float CAR_BRAKE_RATE = 0.2f;
    constexpr float CAR_FRICTION = 0.015f;
    constexpr float CAR_TURN_SPEED = 4.5f;
    constexpr float CAR_TURN_PENALTY = 0.05f;
    constexpr float CAR_MAX_SENSOR_DIST = 250.0f;
    constexpr float CAR_CRASH_DIST_THRESHOLD = 5.0f;
    constexpr int CAR_STALL_TIME_THRESHOLD = 100;
    constexpr float CAR_STALL_SPEED_THRESHOLD = -0.2f;
    
    constexpr float SIM_START_POS_X = 400.0f;
    constexpr float SIM_START_POS_Y = 650.0f;
}
