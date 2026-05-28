#pragma once

#include "Car.h"
#include <vector>

/**
 * @brief Genetic algorithms and persistence for neural network evolution.
 * 
 * Manages natural selection, mutations, and crossovers of the car population,
 * as well as loading and saving genomes (weights and biases).
 */
namespace Evolution {
    /**
     * @brief Saves the brains of the best cars from the current generation to disk.
     * 
     * @param population The complete evaluated population of cars.
     * @param generation The current generation number.
     */
    void SaveBestBrains(const std::vector<Car>& population, int generation);

    /**
     * @brief Loads the best previously saved brains from disk for the new generation.
     * 
     * @param population The population into which the loaded brains will be injected.
     * @param generation Reference to store the loaded generation number.
     * @return true If loading was successful.
     * @return false If an error occurred during loading or the file was not found.
     */
    bool LoadBestBrains(std::vector<Car>& population, int &generation);

    /**
     * @brief Evaluates the current population and applies genetic algorithms to generate the next generation.
     * 
     * @param population Reference to the current population, which will be mutated and replaced.
     * @param startPosition Initial position to reset the cars.
     * @param startRotation Initial rotation to reset the cars.
     * @param genCount The current generation count.
     */
    void EvolvePopulation(std::vector<Car>& population, Vector2 startPosition, float startRotation, int genCount);
}
