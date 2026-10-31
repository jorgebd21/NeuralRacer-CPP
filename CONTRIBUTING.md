# Contributing to NeuralRacer-CPP

Thank you for your interest in contributing to **NeuralRacer-CPP**! We welcome contributions that improve physics fidelity, neuroevolutionary algorithms, high-performance computing (HPC) optimizations, telemetry analysis, and documentation.

---

## Code of Conduct

All contributors and maintainers are expected to adhere to our [Code of Conduct](CODE_OF_CONDUCT.md). Please read it before participating in discussions or submitting code.

---

## Development Prerequisites

Ensure your development environment meets the following minimum requirements:

* **Language Standard:** Modern C++17 compatible compiler (GCC 8+, Clang 7+, or MSVC 2019+).
* **Build System:** CMake 3.10 or newer.
* **Core Libraries:**
  * [Raylib](https://www.raylib.com/) 4.0+ (Windowing, 2D hardware-accelerated rendering, input polling).
  * OpenMP (`libomp-dev`) for multi-core population parallelization.
  * Intel OneTBB (`libtbb-dev`) for task scheduling.
  * [nlohmann/json](https://github.com/nlohmann/json) (included in `include/json.hpp`).

### Debian / Ubuntu Setup:
```bash
sudo apt update
sudo apt install build-essential cmake libraylib-dev libomp-dev libtbb-dev
```

---

## Architectural & Design Guidelines

To maintain simulation stability and deterministic performance, contributions must adhere to these foundational principles:

### 1. Zero-Allocation Hot Path
* **No Dynamic Memory in Real-Time Loops:** Never invoke `malloc`, `free`, `new`, `delete`, or resize heap-allocated containers (`std::vector::push_back`) within vehicle updates, physics loops, or per-tick rendering routines.
* **Preallocated Contiguous Structures:** Use `FixedRingBuffer` or preallocated arrays sized by `Config::` constants.
* Verify compliance with `./bin/bench_zero_alloc`.

### 2. Thread Safety & OpenMP Parallelism
* Physics and neural inferences are evaluated concurrently across threads using `#pragma omp parallel for`.
* Avoid shared mutable state or use `thread_local` generators for random number generation (`std::mt19937`).
* Do not call thread-unsafe Raylib or OpenGL routines inside parallel regions.

### 3. Kinematic & Physical Realism
* Maintain dimensional scaling consistency (track width $35\text{ px}$, vehicle dimensions $17 \times 8\text{ px}$).
* Respect friction ellipse constraints (Kamm circle) and dynamic load transfer invariants.

---

## Commit Guidelines

We enforce the [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) standard. Commit messages must follow this structure:

```
<type>(<scope>): <short imperative summary>

[optional body describing technical context and rationale]

[optional footer(s) referencing issue numbers]
```

### Allowed Types:
* `feat`: A new feature or capability (e.g., `feat(physics): add power-understeer suppression`).
* `fix`: A bug fix or kinematic correction (e.g., `fix(brain): normalize lateral velocity in input vector`).
* `perf`: Performance improvement (e.g., `perf(simd): vectorize Padé tanh approximation`).
* `test`: Adding or updating test suites (e.g., `test(telemetry): add multi-circuit coherence audit`).
* `docs`: Documentation updates (e.g., `docs: update README with CLI flags`).
* `refactor`: Code change that neither fixes a bug nor adds a feature.
* `chore`: Build scripts, CI maintenance, or auxiliary tools.

---

## Verification & Testing Workflow

Before opening a pull request, verify all automated test targets pass locally:

```bash
# 1. Create build directory and configure
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release

# 2. Compile all targets
cmake --build . -j2

# 3. Run automated CTest test suite
ctest --output-on-failure

# 4. Run zero-allocation benchmark
./bin/bench_zero_alloc

# 5. Run headless sanity check
./bin/app --headless --generations 5

# 6. Run multi-circuit telemetry coherence check
./bin/test_exhaustive_telemetry --brain ../data/best.json --track 0 --ticks 1000
```

> [!IMPORTANT]
> When adding new features or physics models, provide corresponding unit tests in `tests/` and register them in `CMakeLists.txt` via `add_test()`.

---

## Pull Request Submission Process

1. **Fork and Branch:** Create a feature branch off `main` (e.g., `feat/slipstream-wake-recalibration`).
2. **Atomic Commits:** Structure changes into logical, well-tested commits.
3. **Fill PR Template:** Complete all sections of the [Pull Request Template](.github/PULL_REQUEST_TEMPLATE.md).
4. **Review & Rebase:** Ensure your branch is rebased on current `origin/main` without merge conflicts.
