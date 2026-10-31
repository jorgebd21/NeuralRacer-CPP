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
     * @brief Dynamic species niche representing a cluster of topologically/synaptically compatible brains.
     */
    struct Species {
        int id{0};
        Brain representative;
        std::vector<int> memberIndices;
        float maxFitness{0.0f};
        float averageFitness{0.0f};
        float adjustedFitnessSum{0.0f};
        int staleness{0};
    };

    /**
     * @brief Computes genomic compatibility distance delta between two brains:
     *        delta = (c1 * E) / N + (c2 * D) / N + c3 * W_diff
     * 
     * @param a First individual genome.
     * @param b Second individual genome.
     * @param c1 Weight coefficient for excess genes.
     * @param c2 Weight coefficient for disjoint genes.
     * @param c3 Weight coefficient for average synaptic weight divergence.
     * @return float Genomic compatibility distance delta.
     */
    float ComputeCompatibilityDistance(const Brain& a, const Brain& b,
                                       float c1 = Config::SPECIATION_C1,
                                       float c2 = Config::SPECIATION_C2,
                                       float c3 = Config::SPECIATION_C3);

    /**
     * @brief Clusters the population into dynamic species based on genomic compatibility threshold.
     * 
     * @param population The evaluated agent population.
     * @param speciesList Reference to active species list, modified in-place.
     * @param threshold Compatibility distance threshold delta_t.
     */
    void ClusterSpecies(const std::vector<Car>& population, std::vector<Species>& speciesList,
                        float threshold = Config::SPECIATION_THRESHOLD);

    /**
     * @brief Retrieves the current active species list.
     */
    const std::vector<Species>& GetActiveSpecies();

    /**
     * @brief Computes explicit niche-adjusted fitness for every individual in the population:
     *        f'_i = f_i / |Species(i)|
     * 
     * @param population The evaluated agent population.
     * @param speciesList Active species list containing member groupings.
     */
    void ApplyFitnessSharing(std::vector<Car>& population, std::vector<Species>& speciesList);

    /**
     * @brief Computes reproduction quotas per species proportional to their summed adjusted fitness:
     *        quota_s = (F_s / F_total) * totalSlots
     * 
     * @param speciesList Active species list.
     * @param totalSlots Total offspring slots to allocate across the population.
     * @return std::vector<int> Integer offspring allotment per species.
     */
    std::vector<int> ComputeSpeciesAllotments(const std::vector<Species>& speciesList, int totalSlots);

    /**
     * @brief Saves the brains of the best cars from the current generation to disk.
     * 
     * @param population The complete evaluated population of cars.
     * @param generation The current generation number.
     * @param filepath Target JSON file path (default "data/best.json").
     */
    void SaveBestBrains(const std::vector<Car>& population, int generation, const std::string& filepath = "data/best.json");

    /**
     * @brief Loads the best previously saved brains from disk for the new generation.
     * 
     * @param population The population into which the loaded brains will be injected.
     * @param generation Reference to store the loaded generation number.
     * @param filepath Source JSON file path (default "data/best.json").
     * @return true If loading was successful.
     * @return false If an error occurred during loading or the file was not found.
     */
    bool LoadBestBrains(std::vector<Car>& population, int &generation, const std::string& filepath = "data/best.json");

    /**
     * @brief Evaluates the current population and applies genetic algorithms to generate the next generation.
     * 
     * @param population Reference to the current population, which will be mutated and replaced.
     * @param startPosition Initial position to reset the cars.
     * @param startRotation Initial rotation to reset the cars.
     * @param genCount The current generation count.
     */
    void EvolvePopulation(std::vector<Car>& population, Vector2 startPosition, float startRotation, int genCount, bool saveDisk = true);

    /**
     * @brief Updates global mutation rate and Gaussian perturbation step size (sigma) based on curriculum decay.
     * 
     * @param generation Current generational milestone.
     */
    void UpdateMutationParameters(int generation);
}
