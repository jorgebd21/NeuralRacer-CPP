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

    // Dual-Parameter Decremental Mutation Schedule:
    // Controls both gene mutation probability (rate %) and Gaussian perturbation magnitude (sigma).
    constexpr int MAX_MUTATION = 50;           // Initial per-gene mutation probability (50% in Phase 1)
    constexpr int MIN_MUTATION = 25;           // Sustained exploration floor for dynamic procedural tracks (25% in Phase 3)
    inline int MUTATION = 50;                  // Active generational mutation rate (%)
    constexpr float MAX_MUTATION_SIGMA = 0.12f;// Initial Gaussian step size for macro-exploration
    constexpr float MIN_MUTATION_SIGMA = 0.08f;// Floor Gaussian step size for adaptive exploitation
    inline float MUTATION_SIGMA = 0.12f;       // Active generational Gaussian mutation step size (sigma)
    constexpr float DROP_RATE = 0.015f;        // Decay coefficient calibrated to Curriculum horizon (0-100 gens)

    constexpr int MAX_GENERATION_TIME = 2000;
    constexpr int CURRICULUM_PHASE_1_END = 15;
    constexpr int CURRICULUM_PHASE_2_END = 50;
    constexpr float SENSOR_ANGLES[5] = {-90.0f, -45.0f, 0.0f, 45.0f, 90.0f};

    constexpr float CAR_TURN_SPEED = 3.5f;
    constexpr float CAR_TURN_PENALTY = 0.05f;
    constexpr float CAR_MAX_SENSOR_DIST = 350.0f;
    constexpr float CAR_FRONT_SENSOR_DIST = 450.0f;
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

    constexpr float CAR_HALF_WIDTH = 4.0f;
    constexpr float CAR_HALF_LENGTH = 8.5f;
    constexpr float CHECKPOINT_RADIUS = 28.0f;
    
    constexpr float SIM_START_POS_X = 400.0f;
    constexpr float SIM_START_POS_Y = 650.0f;
}
