#pragma once

/**
 * @brief Contains all global configuration parameters for the application.
 * 
 * Groups constants that adjust physics, genetic simulation, and 
 * window settings. Encapsulated in a namespace to avoid global scope pollution.
 */
namespace Config {
    constexpr const char* ENGINE_VERSION = "2.5.0";
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
    constexpr int TRACKS_PER_GENERATION = 6; // Maximum procedural multi-circuit evaluation limit
    constexpr int CURRICULUM_PHASE_1_END = 40;   // Phase 1 Induction: single benchmark track (Gens 0-39)
    constexpr int CURRICULUM_PHASE_2_END = 100;  // Phase 2 Consolidation: 3 benchmark circuits (Gens 40-99)
    constexpr int PHASE_1_TRACK_CHANGE_GENS = 10; // In Phase 1, rotate induction track every N generations
    constexpr float SENSOR_ANGLES[5] = {-75.0f, -20.0f, 0.0f, 20.0f, 75.0f};

    constexpr float CAR_TURN_SPEED = 3.5f;
    constexpr float CAR_TURN_PENALTY = 0.08f;  // Induced steering deflection drag factor
    constexpr float ROLLING_RESISTANCE = 3.5f; // Constant Coulomb rolling resistance (N)
    constexpr float CAR_MAX_SENSOR_DIST = 350.0f;
    constexpr float CAR_LATERAL_SENSOR_MAX = 80.0f;
    constexpr float CAR_FRONT_SENSOR_DIST = 500.0f;
    constexpr int CAR_STALL_TIME_THRESHOLD = 90;
    constexpr float CAR_STALL_SPEED_THRESHOLD = -0.3f;
    constexpr int CAR_MAX_TIME_WITHOUT_CHECKPOINT = 180; // Anti-crawl sector timeout forcing aggressive race pacing

    constexpr float CAR_MASS = 1200.0f;
    constexpr float ENGINE_POWER = 68.0f;   // F1-grade powertrain propulsion (+70% longitudinal acceleration)
    constexpr float BRAKING_POWER = 210.0f; // High-authority F1 carbon-ceramic threshold braking (3.1:1 brake-to-engine ratio)
    constexpr float DRAG_MULTIPLIER = 0.44f; // High aerodynamic downforce drag (v_term ~ 12.1 px/tick, with drafting ~ 15.6 px/tick)
    constexpr float CORNERING_STIFFNESS = 1000.0f;
    constexpr float WEIGHT_TRANSFER_FACTOR = 0.05f;
    constexpr float TIRE_GRIP_LIMIT = 0.62f; // High-downforce F1 slick tire lateral grip envelope
    constexpr float TIRE_SCRUB_FACTOR = 55.0f;  // Lateral tire scrub deceleration force scale

    // Multi-Agent Aerodynamic Drafting & Slipstream Wake
    constexpr float DRAFTING_MAX_DISTANCE = 160.0f;
    constexpr float DRAFTING_MAX_WIDTH = 20.0f;
    constexpr float DRAFTING_SIGMA_LONG = 60.0f;
    constexpr float DRAFTING_SIGMA_LAT = 10.0f;
    constexpr float DRAFTING_MAX_DRAG_REDUCTION = 0.40f;
    constexpr float DRAFTING_DIRTY_AIR_PENALTY = 0.15f;

    // Surface friction coefficients (\mu)
    constexpr float FRICTION_ASPHALT = 1.00f;
    constexpr float FRICTION_APEX_CURBS = 0.85f;
    constexpr float FRICTION_RUNOFF_TURF = 0.65f;
    constexpr float FRICTION_GRAVEL_TRAP = 0.25f;

    // Speciation & Recurrent Neuroevolution (v2.3.0)
    constexpr float SPECIATION_C1 = 1.0f;           // Weight of excess genes E / N
    constexpr float SPECIATION_C2 = 1.0f;           // Weight of disjoint genes D / N
    constexpr float SPECIATION_C3 = 0.4f;           // Weight of average synaptic weight divergence W_diff
    constexpr float SPECIATION_THRESHOLD = 0.18f;   // Compatibility distance threshold delta_t (calibrated for 3-5 species)

    constexpr float CAR_HALF_WIDTH = 4.0f;
    constexpr float CAR_HALF_LENGTH = 8.5f;
    constexpr float CHECKPOINT_RADIUS = 28.0f;
    
    constexpr float SIM_START_POS_X = 400.0f;
    constexpr float SIM_START_POS_Y = 650.0f;
}
