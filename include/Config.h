#pragma once

namespace Config {
    constexpr int SCREEN_WIDTH = 1280;
    constexpr int SCREEN_HEIGHT = 768;
    constexpr int POPULATION_SIZE = 100; // Número de coches por generación
    constexpr int NUM_MEJORES = 10; // Número de coches mejores para la siguiente generación
    constexpr int MUTACION = 10; // %10 de mutacion
    constexpr int MAX_GENERATION_TIME = 3000;
    constexpr float SENSOR_ANGLES[5] = {-90.0f, -45.0f, 0.0f, 45.0f, 90.0f};
}
