# 🏎️ NeuralRacer-CPP

> **🚧 WORK IN PROGRESS:** Actively refactoring architecture and memory optimizations.

A high-performance autonomous driving simulation built entirely in **Modern C++ (C++17)**. This project implements a custom neural network and genetic algorithm from scratch—without relying on heavy ML frameworks—to train virtual vehicles to navigate complex racing circuits.

Graphical rendering and 2D physics are handled via the lightweight [raylib](https://www.raylib.com/) library, keeping the focus entirely on algorithmic efficiency, low-level memory management, and artificial intelligence.

## ✨ Key Technical Features

* **From-Scratch Neural Network:** A Feedforward Neural Network (`Brain` struct) with a 6-8-2 architecture, evaluating sensor distances and speed to output steering and acceleration.
* **Genetic Algorithm (Neuroevolution):** Trains populations of 100 cars per generation. Features fitness sorting, elitism (saving the top 10 brains), and Gaussian mutation strategies to evolve network weights and biases.
* **Procedural Track Generation:** Implements mathematical splines (Catmull-Rom) and solves the Traveling Salesperson Problem (TSP) using Nearest Neighbor and 2-Opt algorithms to generate infinite, non-intersecting random circuits.
* **Raycasting Physics:** Vehicles utilize 5 simulated directional sensors calculating line-segment intersections against track boundaries in real-time.

## 🛠️ Tech Stack

* **Core Language:** C++17
* **Graphics & Windowing:** raylib
* **Build System:** Makefile / GCC

## 🚀 Getting Started

### Prerequisites
* A C++17 compatible compiler (GCC / Clang).
* [raylib](https://github.com/raysan5/raylib) installed and configured on your system.

### Building the Project
```bash
# Clone the repository
git clone [https://github.com/jorgebd21/NeuralRacer-CPP.git](https://github.com/jorgebd21/NeuralRacer-CPP.git)
cd NeuralRacer-CPP

# Build using the provided Makefile
make

# Run the simulation
./app
