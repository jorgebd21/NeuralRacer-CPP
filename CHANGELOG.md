# Changelog

All notable changes to the **NeuralRacer-CPP** engine are documented in this file.
The project adheres strictly to [Semantic Versioning 2.0.0](https://semver.org/) and [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/).

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
  - Supports multi-speed playback ($0.25\times$ slow-motion, $1.0\times$ real-time, $4.0\times$ fast-forward), frame-by-frame stepping (`[ LEFT ] / [ RIGHT ]`), pause/resume (`[ SPACE ]`), and direct mouse timeline scrubbing.
- **Cockpit XAI Jacobian Directional Vectors:**
  - Real-time HUD vector arrows overlaid directly above the vehicle chassis indicating instantaneous sensitivity gradients:
    - **Magenta Arrow:** Steering response to frontal obstacle proximity ($\partial \text{Steer} / \partial \text{Front}$).
    - **Orange Arrow:** Steering response to lateral differential balance ($\partial \text{Steer} / \partial \text{Left} - \partial \text{Steer} / \partial \text{Right}$).

---

## [v2.3.0] — 2026-10-29
### Temporal Recurrent Neuroevolution & Topological Speciation

#### Summary
Upgrades the agent cognitive architecture from reactive memoryless controllers to temporal recurrent neural networks with working memory, coupled with NEAT-style speciation and explicit fitness sharing.

#### Features & Improvements
- **Minimal Gated Unit (MGU) Recurrent Cell:**
  - Implements an 8-dimensional recurrent hidden state vector $\mathbf{h}_t$ integrated directly into the `Brain` pipeline:
    - Reset/Update Gate: $\mathbf{f}_t = \sigma(\mathbf{W}_f \mathbf{x}_t + \mathbf{U}_f \mathbf{h}_{t-1} + \mathbf{b}_f)$
    - Candidate Hidden State: $\tilde{\mathbf{h}}_t = \tanh(\mathbf{W}_h \mathbf{x}_t + \mathbf{U}_h (\mathbf{f}_t \odot \mathbf{h}_{t-1}) + \mathbf{b}_h)$
    - Hidden State Transition: $\mathbf{h}_t = (1 - \mathbf{f}_t) \odot \mathbf{h}_{t-1} + \mathbf{f}_t \odot \tilde{\mathbf{h}}_t$
  - Enables agents to anticipate turn sequences and remember track curvature trends across successive frames.
- **Genomic Compatibility Distance & Speciation Clustering:**
  - Dynamic speciation metric $\delta = \frac{c_1 E}{N} + \frac{c_2 D}{N} + c_3 \cdot \bar{W}_{\text{diff}}$ clustering agents based on structural and synaptic divergence.
  - Dynamically adjusted compatibility threshold $\delta_t$ maintaining population species diversity over hundreds of generations.
- **Explicit Fitness Sharing:**
  - Divides raw fitness by species niche counts ($f'_i = f_i / |S_k|$), preventing premature population takeover by dominant suboptimal strategies and preserving innovative driving approaches.
- **Modernized Genome Serialization & Memory Optimization:**
  - Extended JSON genome schema preserving recurrent weight matrices ($\mathbf{W}_f, \mathbf{U}_f, \mathbf{W}_h, \mathbf{U}_h$).
  - Cache-aligned memory layout ensuring vectorizable, contiguous evaluation of recurrent state transitions.

---

## [v2.2.0] — 2026-10-29
### Multi-Agent Perception & Interactive Racecraft

#### Summary
Transitions the simulation from isolated ghost time trials to authentic multi-car wheel-to-wheel competitive racing, incorporating adversary perception, aerodynamic drafting, and physical collision resolution.

#### Features & Improvements
- **Dynamic Oriented Bounding Box (OBB) Spatial Registration:**
  - Dynamically registers all active competitor bounding boxes into a per-frame secondary spatial grid for fast $O(1)$ neighborhood queries.
- **Dynamic Multi-Agent LIDAR Raycasting:**
  - Extends vehicle 5-ray LIDAR sensors to detect dynamic opponent chassis alongside static track barriers.
  - Appends relative closing velocities $\Delta v$ to the neural state input vector, enabling defensive blocking and tactical overtaking.
- **Aerodynamic Slipstream & Drafting Wake:**
  - Trailing wake modeling: $C_{d,\text{eff}} = C_d \cdot (1.0 - 0.40 \cdot \exp(-\Delta x^2 / 2\sigma_{\text{wake}}^2))$.
  - Rewards tactical trailing on high-speed straights with higher top speeds while requiring earlier braking into technical corners.
- **Momentum-Conserving Elastic Collision Resolution:**
  - 2D rigid-body impulse restitution for non-terminal side-by-side wheel rubs and glancing collisions without immediate disqualification.
  - Total crash disqualification preserved strictly for violent barrier impacts or high-speed perpendicular T-bone accidents.

---

## [v2.1.0] — 2026-10-29
### Advanced Surface Dynamics & Tire Thermodynamics

#### Summary
Introduces track surface spatial friction partitioning ($\mu$) and a lumped-capacitance dynamic tire thermal model, heavily penalizing power-sliding and rewarding line precision.

#### Features & Improvements
- **Multi-Surface Spatial Friction Grid ($\mu$ Partitioning):**
  - Asphalt track surface: $\mu = 1.00$
  - FIA Apex Curbs (Rumble Strips): $\mu = 0.85$ with high-frequency chassis oscillation impulse.
  - Runoff Astroturf: $\mu = 0.65$
  - Gravel Traps: $\mu = 0.25$ with aggressive longitudinal rolling resistance.
- **Lumped-Parameter Tire Thermal Degradation Model:**
  - Thermodynamic ODE governing tire bulk and surface temperatures:
    $$T_t = T_{t-1} + (Q_{\text{slip}} + Q_{\text{deflection}} - Q_{\text{cooling}}) \Delta t$$
  - Optimal thermal operating window ($85^\circ\text{C}\text{--}100^\circ\text{C}$) delivering maximum grip, with steep thermal degradation factor $\eta_{\text{thermal}}(T)$ above $115^\circ\text{C}$.
- **Coupled Kamm Friction Ellipse & Scrub Resistance:**
  - Dynamic modulation of $F_{\text{max\_grip}}$ and $F_{\text{tireScrub}}$ according to both the instantaneous surface friction coefficient and tire temperature.
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
  - Guaranteed $0$ runtime heap allocations across 100,000 continuous simulation ticks.
- **Contiguous 1D Flat Spatial Grid (`FlatSpatialGrid`):**
  - Deprecated node-allocated `std::unordered_map` spatial hash.
  - Introduced 1D row-major array indexing ($\text{idx} = y \times \text{COLS} + x$) backed by a centralized segment pool arena (`std::vector<uint32_t>`).
  - Drastically increased CPU L1/L3 cache hit rates during raycasting and narrow-phase collision passes.
- **AVX2 / FMA Vectorized Neural Forward Pass:**
  - Vectorized dense matrix-vector products $\mathbf{W}\mathbf{x} + \mathbf{b}$ using `_mm256_fmadd_ps` and 8-wide float vector registers.
  - High-precision [3/3] rational Padé polynomial approximation of $\tanh(x)$ replacing slow standard math library calls.
- **High-Frequency Zero-Allocation Benchmark Suite:**
  - Built dedicated allocation-tracking test suite asserting 0 calls to `malloc`/`operator new` during real-time multi-agent execution.

---

## [v1.3.0] — 2026-10-25
### Grand Prix Scale Expansion, Procedural Curriculum & XAI Telemetry

- Expanded world canvas to $2400 \times 1600\text{ px}$ arena ($6.4\times$ track area) supporting genuine Grand Prix aspect ratios.
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
