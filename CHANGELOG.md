# Changelog

All notable changes to the **NeuralRacer-CPP** engine are documented in this file.
The project adheres strictly to [Semantic Versioning 2.0.0](https://semver.org/) and [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/).

---

## [v2.6.0] — 2026-10-31
### Formula 1 Powertrain Dynamics, High-G Threshold Braking & Generational Curriculum

#### Summary
Upgrades the vehicle physics to true Formula 1 powertrain dynamics (+70% engine thrust, +75% braking power, increased aerodynamic drag and tire grip), implements heavy straight-line threshold braking and corner-entry lift-off incentives, enforces dynamic understeer safety corridors and universal barrier throttle suppression, decouples evolutionary curriculum learning by generation count across 6 Grand Prix circuit archetypes, and establishes test harness genome integrity.

#### Features & Improvements
- **Formula 1 Powertrain & Aerodynamic Envelope (`Config.h`):**
  - Scaled `ENGINE_POWER` from 40.0 N to 68.0 N (+70% longitudinal acceleration) and `BRAKING_POWER` from 120.0 N to 210.0 N (+75% high-G deceleration authority).
  - Elevated `DRAG_MULTIPLIER` to 0.44 for realistic high-speed aerodynamic resistance and coasting deceleration.
  - Upgraded `TIRE_GRIP_LIMIT` from 0.55 to 0.62 (+12.7% lateral cornering grip), raising cornering limit from ~2.7 px/t to ~4.2 px/t.
  - Extended forward perception range (`CAR_FRONT_SENSOR_DIST`) from 450 px to 500 px for long-range corner anticipation at speeds > 7.5 px/t.
- **High-G Threshold Braking & Deceleration Scale (`Car.cpp`):**
  - Kinetic dissipation reward (15.0 · |u_a| · v_long) enhanced with a 1.5× amplification factor when v_long > 4.5 px/t into corner entry zones.
  - Progressive reward for lifting off throttle on corner entry (u_a <= 0.10 while turning).
  - Suppression of power understeer: penalizes full throttle when lateral slip exceeds 0.35 px/t.
- **Dynamic Corner Safety Corridor & Barrier Penalty Hardening (`Car.cpp`):**
  - Dynamic stopping corridor `d_stop_margin = max(95.0 px, 1.25 · d_stopping)` dynamically calculates deceleration requirements based on current velocity, eliminating corner overshoot.
  - Universal barrier throttle penalty: removed turning angle threshold loophole, ensuring any vehicle heading into a wall within 95 px is severely penalized for applying positive throttle.
- **Generation-Driven Curriculum & Circuit Rotation (`Simulation.cpp`, `Telemetry.cpp`):**
  - Curriculum phases now advance predictably based on generation count: Phase 1 (Single Track Induction) for generations 0 <= g < 40; Phase 2 (Multi-Archetype Generalization) for 40 <= g < 100; Phase 3 (Pure Stochastic Procedural Generation) for g >= 100.
  - Periodic 10-generation circuit rotation during Phase 1 across 6 canonical Grand Prix archetypes (Silverstone, Interlagos, Suzuka, Montreal, Baku, Monza).
- **Multi-Archetype Exhaustive Telemetry Test Harness (`test_exhaustive_telemetry.cpp`):**
  - Enhanced CLI support with `--track <0-5>`, `--ticks <N>`, and `--brain <path>` for automated multi-circuit validation.
  - Automated behavioral coherence auditing across all Formula 1 archetypes.
- **Test Harness Genome Preservation (`Evolution.h`, `Evolution.cpp`, `test_speciation_diversity.cpp`):**
  - Added optional `saveDisk = true` parameter to `Evolution::EvolvePopulation` and set `false` in test suites to prevent unit tests from corrupting trained production genomes in `data/best.json`.

---

## [v2.5.0] — 2026-10-31
### Exhaustive Telemetry & Behavioral Coherence Engine

#### Summary
Introduces a high-resolution 60 Hz 45-parameter exhaustive telemetry recorder, automated real-time behavioral incoherence auditor, lookahead sensor geometry re-architecting, physical steering rack rate-limiting, and species-allotted evolutionary crossover. These improvements eliminate corridor blindness and premature genetic convergence, unlocking multi-lap survival across Formula 1 Grand Prix archetypes and dynamic procedural tracks.

#### Features & Improvements
- **60 Hz Exhaustive Telemetry & Behavioral Incoherence Auditor (`ExhaustiveTelemetry`):**
  - Continuous 45-field synchronized physics and neurocognitive telemetry recording (`data/exhaustive_telemetry.csv` and `.json`).
  - Automated behavioral auditor detecting cognitive contradictions:
    - *Steering into immediate walls* (anomalous negative obstacle avoidance).
    - *Full-throttle wall impacts* (failure to trail-brake or lift off).
    - *Braking on open straights* (phantom obstacle deceleration).
    - *Bang-bang steering chatter* (|Δsteer| > 1.2 in a single tick).
    - *Tire adhesion collapses* (slip angle > 35° or continuous spinning).
    - *Lateral Jacobian inversions* (∂Steer / ∂Lidar_lat · error < 0).
  - Computes formal Steering, Throttle, and Grip Coherence Indexes (% coherent ticks).
- **Adaptive Lookahead Sensor Geometry & Rescaled Corridor Perception:**
  - Reshaped 5-ray LIDAR sweep to `{-75°, -20°, 0°, 20°, 75°}`.
  - Wide lateral rays (±75°) normalized to 80.0 px prevent dynamic range collapse in narrow 35 px corridors.
  - Narrow lookahead rays (±20°) normalized to 350.0 px detect approaching bends > 250 px in advance, enabling decisive corner anticipation.
  - Front Time-to-Collision (ΔTTC) sensor signal normalized to [-1.0, 1.0].
- **Physical Steering Slew Rate Limiter & Anti-Chatter Damping:**
  - Implements realistic steering rack mechanical latency with a maximum slew rate limit of 0.20/tick.
  - Penalizes steering actuator chatter (0.35 · |Δsteer| per tick), eliminating high-frequency bang-bang oscillation exploits.
- **True Species-Allotted Breeding & Diversity Niches:**
  - Fully connects `ComputeSpeciesAllotments` into `CrossoverAndMutate`, ensuring offspring allocation is proportionally distributed across active species niches.
  - Eliminates the genetic sinkhole where straight-line speed records in early generations wiped out cornering and braking exploratory phenotypes.
- **Chassis Operating Bounds & In-Bounds Neural Input Scaling:**
  - Rescales longitudinal velocity to `v_long / 4.0f` and lateral slip to `v_lat / 1.5f` in the neural perception vector, balancing synaptic weight activations.
- **Longitudinal Training CLI Options:**
  - Added `--headless` mode for ultra-fast OpenMP multi-core training without Raylib window overhead.
  - Added `--fresh` / `--new` flag to initialize clean generation 0 populations from scratch.
  - Added `--generations <N>` limiter to execute deterministic bounded evolutionary training runs.

---

## [v2.4.0] — 2026-10-29
### Advanced Telemetry Visualizer & Replay Engine

#### Summary
Delivers a MoTeC i2-grade analytical visualization suite and interactive 60 Hz replay engine, allowing deep diagnostic inspection of autonomous agent decision-making and lap telemetry.

#### Features & Improvements
- **Semi-Transparent Ghost Vehicle Renderer (`GhostRenderer`):**
  - Renders a translucent ghost vehicle projecting historical personal best laps loaded from `data/best.json`.
  - Employs smooth Catmull-Rom temporal position and yaw angle interpolation between recorded discrete 60 Hz samples.
- **Dynamic Surface Speed & Grip Heatmap (`HeatmapRenderer`):**
  - Projects continuous color-coded velocity and scrub gradients directly onto the circuit asphalt:
    - **Green:** High velocity & optimal grip utilization.
    - **Amber:** Trail braking & longitudinal transition zones.
    - **Red:** Excessive tire scrub, lateral slides, and off-track excursions.
- **Interactive 60 Hz Replay Scrubbing Tool (`ReplayScrubber`):**
  - Comprehensive replay mode (`[ R ]` key) parsing recorded `telemetry_best_lap.csv`.
  - Supports multi-speed playback (0.25× slow-motion, 1.0× real-time, 4.0× fast-forward), frame-by-frame stepping (`[ LEFT ] / [ RIGHT ]`), pause/resume (`[ SPACE ]`), and direct mouse timeline scrubbing.
- **Cockpit XAI Jacobian Directional Vectors:**
  - Real-time HUD vector arrows overlaid directly above the vehicle chassis indicating instantaneous sensitivity gradients:
    - **Magenta Arrow:** Steering response to frontal obstacle proximity (∂Steer / ∂Front).
    - **Orange Arrow:** Steering response to lateral differential balance (∂Steer / ∂Left - ∂Steer / ∂Right).

---

## [v2.3.0] — 2026-10-29
### Temporal Recurrent Neuroevolution & Topological Speciation

#### Summary
Upgrades the agent cognitive architecture from reactive memoryless controllers to temporal recurrent neural networks with working memory, coupled with NEAT-style speciation and explicit fitness sharing.

#### Features & Improvements
- **Minimal Gated Unit (MGU) Recurrent Cell:**
  - Implements an 8-dimensional recurrent hidden state vector `h_t` integrated directly into the `Brain` pipeline:
    - Reset/Update Gate: `f_t = σ(W_f · x_t + U_f · h_{t-1} + b_f)`
    - Candidate Hidden State: `h̃_t = tanh(W_h · x_t + U_h · (f_t ⊙ h_{t-1}) + b_h)`
    - Hidden State Transition: `h_t = (1 - f_t) ⊙ h_{t-1} + f_t ⊙ h̃_t`
  - Enables agents to anticipate turn sequences and remember track curvature trends across successive frames.
- **Genomic Compatibility Distance & Speciation Clustering:**
  - Dynamic speciation metric `δ = (c_1 · E)/N + (c_2 · D)/N + c_3 · W̄_diff` clustering agents based on structural and synaptic divergence.
  - Dynamically adjusted compatibility threshold maintaining population species diversity over hundreds of generations.
- **Explicit Fitness Sharing:**
  - Divides raw fitness by species niche counts (`f'_i = f_i / |S_k|`), preventing premature population takeover by dominant suboptimal strategies and preserving innovative driving approaches.
- **Modernized Genome Serialization & Memory Optimization:**
  - Extended JSON genome schema preserving recurrent weight matrices (`W_f, U_f, W_h, U_h`).
  - Cache-aligned memory layout ensuring vectorizable, contiguous evaluation of recurrent state transitions.

---

## [v2.2.0] — 2026-10-29
### Multi-Agent Perception & Interactive Racecraft

#### Summary
Transitions the simulation from isolated ghost time trials to authentic multi-car wheel-to-wheel competitive racing, incorporating adversary perception, aerodynamic drafting, and physical collision resolution.

#### Features & Improvements
- **Dynamic Oriented Bounding Box (OBB) Spatial Registration:**
  - Dynamically registers all active competitor bounding boxes into a per-frame secondary spatial grid for fast O(1) neighborhood queries.
- **Dynamic Multi-Agent LIDAR Raycasting:**
  - Extends vehicle 5-ray LIDAR sensors to detect dynamic opponent chassis alongside static track barriers.
  - Appends relative closing velocities `Δv` to the neural state input vector, enabling defensive blocking and tactical overtaking.
- **Aerodynamic Slipstream & Drafting Wake:**
  - Trailing wake modeling: `C_{d,eff} = C_d · (1.0 - 0.40 · exp(-Δx² / (2 · σ_wake²)))`.
  - Rewards tactical trailing on high-speed straights with higher top speeds while requiring earlier braking into technical corners.
- **Momentum-Conserving Elastic Collision Resolution:**
  - 2D rigid-body impulse restitution for non-terminal side-by-side wheel rubs and glancing collisions without immediate disqualification.
  - Total crash disqualification preserved strictly for violent barrier impacts or high-speed perpendicular T-bone accidents.

---

## [v2.1.0] — 2026-10-29
### Advanced Surface Dynamics & Tire Thermodynamics

#### Summary
Introduces track surface spatial friction partitioning (μ) and a lumped-capacitance dynamic tire thermal model, heavily penalizing power-sliding and rewarding line precision.

#### Features & Improvements
- **Multi-Surface Spatial Friction Grid (μ Partitioning):**
  - Asphalt track surface: μ = 1.00
  - FIA Apex Curbs (Rumble Strips): μ = 0.85 with high-frequency chassis oscillation impulse.
  - Runoff Astroturf: μ = 0.65
  - Gravel Traps: μ = 0.25 with aggressive longitudinal rolling resistance.
- **Lumped-Parameter Tire Thermal Degradation Model:**
  - Thermodynamic model governing tire bulk and surface temperatures:
    `T_t = T_{t-1} + (Q_slip + Q_deflection - Q_cooling) · Δt`
  - Optimal thermal operating window (85°C – 100°C) delivering maximum grip, with steep thermal degradation above 115°C.
- **Coupled Kamm Friction Ellipse & Scrub Resistance:**
  - Dynamic modulation of `F_max_grip` and `F_tireScrub` according to both the instantaneous surface friction coefficient and tire temperature.
- **Extended 60 Hz Telemetry Export:**
  - Adds tire bulk temperature, surface temperature, thermal grip factor, and active surface type to the time-series telemetry logger.

---

## [v2.0.0] — 2026-10-29
### Low-Latency High-Performance Computing (HPC) & Memory Pipeline

#### Summary
Achieves zero dynamic heap allocations in the high-frequency evaluation loop and vectorizes neural forward inference using AVX2 and FMA SIMD intrinsics.

#### Features & Improvements
- **Preallocated Fixed-Capacity Ring Buffers:**
  - Replaced dynamic `std::vector::push_back` in telemetry logging with contiguous `FixedRingBuffer<TelemetrySample, Config::MAX_GENERATION_TIME>` per agent.
  - Guaranteed 0 runtime heap allocations across 100,000 continuous simulation ticks.
- **Contiguous 1D Flat Spatial Grid (`FlatSpatialGrid`):**
  - Deprecated node-allocated `std::unordered_map` spatial hash.
  - Introduced 1D row-major array indexing (`idx = y · COLS + x`) backed by a centralized segment pool arena (`std::vector<uint32_t>`).
  - Drastically increased CPU L1/L3 cache hit rates during raycasting and narrow-phase collision passes.
- **AVX2 / FMA Vectorized Neural Forward Pass:**
  - Vectorized dense matrix-vector products `W·x + b` using `_mm256_fmadd_ps` and 8-wide float vector registers.
  - High-precision [3/3] rational Padé polynomial approximation of `tanh(x)` replacing slow standard math library calls.
- **High-Frequency Zero-Allocation Benchmark Suite:**
  - Built dedicated allocation-tracking test suite asserting 0 calls to `malloc`/`operator new` during real-time multi-agent execution.

---

## [v1.3.0] — 2026-10-25
### Grand Prix Scale Expansion, Procedural Curriculum & XAI Telemetry

- Expanded world canvas to 2400 × 1600 px arena (6.4× track area) supporting genuine Grand Prix aspect ratios.
- Pure procedural curriculum with 6 dynamic F1 circuits per generation from Generation 0.
- Decoupled `Camera2D` dynamic chase drone (`CAM_FOLLOW_LEADER`) and tactical minimap HUD.
- Advanced vehicle dynamics with Kamm friction ellipse and weight transfer.
- Analytical explainable AI (XAI) Jacobian sensitivity derivatives.
- High-frequency 60 Hz CSV telemetry export.

---

## [v1.2.0] — 2026-10-24
### Dynamic Lateral Kinematics & Scientific Telemetry

- Real-time chassis slip angle and lateral velocity decomposition.
- Cornering scrub drag and tire slip restoring forces.
- 14-parameter scientific telemetry dataset export.

---

## [v1.1.0] — 2026-05-25
### Concurrency & Performance Hardening

- OpenMP multi-threading acceleration across agent population.
- Thread-safe crash handling and telemetry isolation.

---

## [v1.0.0] — 2026-05-24
### Initial Release

- Initial 2D autonomous driving engine in Modern C++17.
- Custom 9-10-2 feedforward neural network and genetic algorithm.
- Procedural track generation and Raylib graphics.
