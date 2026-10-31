---
name: Feature Request
about: Suggest an idea or physical/algorithmic enhancement
title: "[FEAT]: "
labels: ["enhancement"]
assignees: ''
---

## Summary
A clear and concise description of the suggested feature or optimization.

## Problem or Research Motivation
Is your feature request related to a specific limitation, research goal, or physics discrepancy? Please explain the motivation (e.g., "Tire temperature decay during slow corners is too rapid under wet conditions").

## Proposed Solution & Mathematical Foundation
Describe the solution and mathematical formulations you'd like to see implemented:
- Governing equations or algorithms (e.g., Pacejka tire model, Runge-Kutta 4th order integrator, NEAT topological mutation).
- Anticipated data structures (e.g., zero-allocation static arena, SIMD kernel).
- Affected subsystems (`include/Car.h`, `src/TrackManager.cpp`, etc.).

## Alternatives Considered
Describe any alternative solutions or features you considered.

## Impact on Performance & Telemetry
Will this impact real-time execution (>60 FPS, OpenMP scaling) or require new parameters in `data/exhaustive_telemetry.csv`?
