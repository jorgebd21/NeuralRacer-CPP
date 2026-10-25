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
    constexpr float WORLD_WIDTH = 2400.0f;
    constexpr float WORLD_HEIGHT = 1600.0f;
    constexpr int POPULATION_SIZE = 100;
    constexpr int NUM_BEST = 10;

    // Two-Stage Decremental Mutation Schedule:
    // Controls both gene mutation probability (rate %) and Gaussian perturbation magnitude (sigma).
    // Stage 1 (Gens 0-200): Macro-exploration & procedural plasticity (50% -> 25%, sigma 0.12 -> 0.08)
    // Stage 2 (Gens 200-400): Deep exploitation & deleterious load reduction (25% -> 3%, sigma 0.08 -> 0.035)
    // Stage 3 (Gens 400+): Sustained ultra-fine exploitation floor (3%, sigma 0.035)
    constexpr int STAGE1_MUTATION_MAX = 50;
    constexpr int STAGE1_MUTATION_MIN = 25;
    constexpr float STAGE1_SIGMA_MAX = 0.12f;
    constexpr float STAGE1_SIGMA_MIN = 0.08f;
    constexpr int MUTATION_HORIZON_STAGE1 = 200;
    constexpr float DROP_RATE_STAGE1 = 0.015f;

    constexpr int STAGE2_MUTATION_MAX = 25;
    constexpr int STAGE2_MUTATION_MIN = 3;
    constexpr float STAGE2_SIGMA_MAX = 0.08f;
    constexpr float STAGE2_SIGMA_MIN = 0.035f;
    constexpr int MUTATION_HORIZON_STAGE2 = 400;
    constexpr float DROP_RATE_STAGE2 = 0.010f;

    inline int MUTATION = 50;                  // Active generational mutation rate (%)
    inline float MUTATION_SIGMA = 0.12f;       // Active generational Gaussian mutation step size (sigma)

    constexpr int MAX_GENERATION_TIME = 2000;
    constexpr int TRACKS_PER_GENERATION = 6; // Pure procedural multi-circuit evaluation from Gen 0
    constexpr int CURRICULUM_PHASE_1_END = 0;
    constexpr int CURRICULUM_PHASE_2_END = 0;
    constexpr float SENSOR_ANGLES[5] = {-90.0f, -45.0f, 0.0f, 45.0f, 90.0f};

    constexpr float CAR_TURN_SPEED = 3.5f;
    constexpr float CAR_TURN_PENALTY = 0.08f;  // Induced steering deflection drag factor
    constexpr float ROLLING_RESISTANCE = 3.5f; // Constant Coulomb rolling resistance (N)
    constexpr float CAR_MAX_SENSOR_DIST = 350.0f;
    constexpr float CAR_FRONT_SENSOR_DIST = 450.0f;
    constexpr int CAR_STALL_TIME_THRESHOLD = 100;
    constexpr float CAR_STALL_SPEED_THRESHOLD = -1.5f;
    constexpr int CAR_MAX_TIME_WITHOUT_CHECKPOINT = 320; // Calibrated for 2400x1600 F1 track checkpoint pacing

    constexpr float CAR_MASS = 1200.0f;
    constexpr float ENGINE_POWER = 35.0f;
    constexpr float BRAKING_POWER = 45.0f;
    constexpr float DRAG_MULTIPLIER = 0.35f;
    constexpr float CORNERING_STIFFNESS = 1000.0f;
    constexpr float WEIGHT_TRANSFER_FACTOR = 0.05f;
    constexpr float TIRE_GRIP_LIMIT = 0.55f;
    constexpr float TIRE_SCRUB_FACTOR = 55.0f;  // Lateral tire scrub deceleration force scale

    constexpr float CAR_HALF_WIDTH = 4.0f;
    constexpr float CAR_HALF_LENGTH = 8.5f;
    constexpr float CHECKPOINT_RADIUS = 28.0f;
    
    constexpr float SIM_START_POS_X = 400.0f;
    constexpr float SIM_START_POS_Y = 650.0f;
}
