#pragma once

/**
 * @brief Contains all global configuration parameters for the application.
 * 
 * Groups constants that adjust physics, genetic simulation, and 
 * window settings. Encapsulated in a namespace to avoid global scope pollution.
 */
namespace Config {
    constexpr int SCREEN_WIDTH = 1280;
    constexpr int SCREEN_HEIGHT = 768;
    constexpr int POPULATION_SIZE = 100;
    constexpr int NUM_BEST = 10;
    constexpr int MAX_MUTATION = 50;
    constexpr int MIN_MUTATION = 15;
    inline int MUTATION = 50;
    constexpr float DROP_RATE = 0.0005f;
    constexpr int MAX_GENERATION_TIME = 2000;
    constexpr int CURRICULUM_PHASE_1_END = 15;
    constexpr int CURRICULUM_PHASE_2_END = 50;
    constexpr float SENSOR_ANGLES[5] = {-90.0f, -45.0f, 0.0f, 45.0f, 90.0f};

    constexpr float CAR_TURN_SPEED = 3.5f;
    constexpr float CAR_TURN_PENALTY = 0.05f;
    constexpr float CAR_MAX_SENSOR_DIST = 500.0f;
    constexpr float CAR_FRONT_SENSOR_DIST = 600.0f;
    constexpr int CAR_STALL_TIME_THRESHOLD = 100;
    constexpr float CAR_STALL_SPEED_THRESHOLD = -1.5f;
    constexpr int CAR_MAX_TIME_WITHOUT_CHECKPOINT = 150;

    constexpr float CAR_MASS = 1200.0f;
    constexpr float ENGINE_POWER = 35.0f;
    constexpr float BRAKING_POWER = 45.0f;
    constexpr float DRAG_MULTIPLIER = 0.35f;
    constexpr float CORNERING_STIFFNESS = 1000.0f;
    constexpr float WEIGHT_TRANSFER_FACTOR = 0.05f;
    constexpr float TIRE_GRIP_LIMIT = 0.55f;
    constexpr float TIRE_SCRUB_FACTOR = 3.5f;

    constexpr float CAR_HALF_WIDTH = 6.0f;
    constexpr float CAR_HALF_LENGTH = 13.0f;
    constexpr float CHECKPOINT_RADIUS = 45.0f;
    
    constexpr float SIM_START_POS_X = 400.0f;
    constexpr float SIM_START_POS_Y = 650.0f;
}
