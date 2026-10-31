---
name: Bug Report
about: Create a report to help us improve NeuralRacer-CPP
title: "[BUG]: "
labels: ["bug"]
assignees: ''
---

## Description
A clear and concise description of the bug encountered.

## Environment & Toolchain
- **OS:** (e.g., Ubuntu 24.04 LTS, Arch Linux, macOS Sonoma, Windows 11)
- **Compiler:** (e.g., GCC 13.2.0, Clang 18.1, MSVC 2022)
- **CMake Version:** (e.g., 3.28.3)
- **Raylib Version:** (e.g., 4.5, 5.0)
- **CPU Architecture / Flags:** (e.g., x86_64 with AVX2/FMA, ARM64)
- **Engine Version / Git Commit:** (e.g., `v2.6.0` or commit `e555faa`)

## Steps to Reproduce
1. Configure and build: `cmake -B build && cmake --build build`
2. Run target command: `./bin/app ...` or `./bin/test_...`
3. Execute action: (e.g., press `[ T ]` then toggle fast-forward `[ SPACE ]`)
4. Observe failure.

## Expected Behavior
A concise description of what you expected to happen.

## Actual Behavior / Error Logs
Include terminal output, CTest crash logs, or backtraces (e.g., from `gdb` or `valgrind`):
```text
(Paste error logs here)
```

## Additional Context / Screenshots
Add any other context, screenshots, or telemetry graphs (`data/exhaustive_telemetry.csv`) about the problem here.
