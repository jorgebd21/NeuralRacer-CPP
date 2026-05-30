#include "Evolution.h"
#include "Config.h"
#include "Simulation.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <random>
#include "json.hpp"

namespace Evolution {

/**
 * @brief Applies two-parent crossover and Gaussian mutation to all non-elite individuals.
 *
 * Selects two distinct parents exclusively from the elite group (indices 0..NUM_BEST-1),
 * blends their genomes with a 50/50 gene-level coin flip, and applies decaying
 * Gaussian mutation based on the current Config::MUTATION rate.
 *
 * @param population The full population. Indices 0..NUM_BEST-1 are treated as elite
 *                   (read-only parents). Indices NUM_BEST..POPULATION_SIZE-1 are overwritten.
 */
static void CrossoverAndMutate(std::vector<Car>& population) {
    thread_local static std::random_device rd;
    thread_local static std::mt19937 generator(rd());
    std::uniform_int_distribution<int> distBest(0, Config::NUM_BEST - 1);
    std::uniform_int_distribution<int> distMutation(0, 99);

    for (int n = Config::NUM_BEST; n < Config::POPULATION_SIZE && n < (int)population.size(); n++) {
        int father = distBest(generator);
        int mother;
        do {
            mother = distBest(generator);
        } while (mother == father);

        for (int i = 0; i < HIDDEN_NODES; i++) {
            for (int j = 0; j < INPUT_NODES; j++) {
                int ref = distMutation(generator) < 50 ? father : mother;
                population[n].brain.weights_input_hidden[i][j] = population[ref].brain.weights_input_hidden[i][j];
                if (distMutation(generator) < Config::MUTATION)
                    population[n].brain.weights_input_hidden[i][j] += population[n].brain.MutateGaussian();
            }
        }
        for (int i = 0; i < HIDDEN_NODES; i++) {
            int ref = distMutation(generator) < 50 ? father : mother;
            population[n].brain.biases_hidden[i] = population[ref].brain.biases_hidden[i];
            if (distMutation(generator) < Config::MUTATION)
                population[n].brain.biases_hidden[i] += population[n].brain.MutateGaussian();
        }
        for (int i = 0; i < OUTPUT_NODES; i++) {
            for (int j = 0; j < HIDDEN_NODES; j++) {
                int ref = distMutation(generator) < 50 ? father : mother;
                population[n].brain.weights_hidden_output[i][j] = population[ref].brain.weights_hidden_output[i][j];
                if (distMutation(generator) < Config::MUTATION)
                    population[n].brain.weights_hidden_output[i][j] += population[n].brain.MutateGaussian();
            }
        }
        for (int i = 0; i < OUTPUT_NODES; i++) {
            int ref = distMutation(generator) < 50 ? father : mother;
            population[n].brain.biases_output[i] = population[ref].brain.biases_output[i];
            if (distMutation(generator) < Config::MUTATION)
                population[n].brain.biases_output[i] += population[n].brain.MutateGaussian();
        }
    }
}

void SaveBestBrains(const std::vector<Car>& population, int generation) {
    std::ofstream bestFile("data/best.json", std::ios::out);
    nlohmann::json data;
    data["generation"] = generation;
    for (int n = 0; n < Config::NUM_BEST; n++) {
        nlohmann::json car;
        car["id"] = n;
        car["weights_input_hidden"] = population[n].brain.weights_input_hidden;
        car["biases_hidden"] = population[n].brain.biases_hidden;
        car["weights_hidden_output"] = population[n].brain.weights_hidden_output;
        car["biases_output"] = population[n].brain.biases_output;
        data["elite"].push_back(car);
    }
    bestFile << data.dump(4);
}

bool LoadBestBrains(std::vector<Car>& population, int& generation) {
    std::ifstream file("data/best.json");
    // Fallback to old name if best.json doesn't exist yet, but only read it, we will save to best.json later.
    if (!file.is_open()) {
        file.open("data/mejores.json");
        if (!file.is_open()) return false;
    }

    nlohmann::json data;
    try {
        data = nlohmann::json::parse(file);
    } catch (const nlohmann::json::parse_error& e) {
        std::cerr << "Error: Failed to parse brain data file: " << e.what() << std::endl;
        return false;
    }

    // Support for old json keys for backward compatibility initially, though saving uses new ones
    if (data.contains("generacion")) generation = data["generacion"];
    else if (data.contains("generation")) generation = data["generation"];

    Config::MUTATION = std::max(1, (int)(Config::MAX_MUTATION / (1.0f + (Config::DROP_RATE * generation))));

    // The first phase of loading directly injects the elite brains from the previous generation.
    // This ensures we do not lose progress ("elitism").
    for (int n = 0; n < Config::NUM_BEST && n < (int)population.size(); n++) {
        for (int i = 0; i < HIDDEN_NODES; i++) {
            for (int j = 0; j < INPUT_NODES; j++) {
                if (data["elite"][n].contains("pesos_entrada_oculta"))
                    population[n].brain.weights_input_hidden[i][j] = data["elite"][n]["pesos_entrada_oculta"][i][j];
                else
                    population[n].brain.weights_input_hidden[i][j] = data["elite"][n]["weights_input_hidden"][i][j];
            }
        }
        for (int i = 0; i < HIDDEN_NODES; i++) {
            if (data["elite"][n].contains("sesgos_oculta"))
                population[n].brain.biases_hidden[i] = data["elite"][n]["sesgos_oculta"][i];
            else
                population[n].brain.biases_hidden[i] = data["elite"][n]["biases_hidden"][i];
        }
        for (int i = 0; i < OUTPUT_NODES; i++) {
            for (int j = 0; j < HIDDEN_NODES; j++) {
                if (data["elite"][n].contains("pesos_oculta_salida"))
                    population[n].brain.weights_hidden_output[i][j] = data["elite"][n]["pesos_oculta_salida"][i][j];
                else
                    population[n].brain.weights_hidden_output[i][j] = data["elite"][n]["weights_hidden_output"][i][j];
            }
        }
        for (int i = 0; i < OUTPUT_NODES; i++) {
            if (data["elite"][n].contains("sesgos_salida"))
                population[n].brain.biases_output[i] = data["elite"][n]["sesgos_salida"][i];
            else
                population[n].brain.biases_output[i] = data["elite"][n]["biases_output"][i];
        }
    }

    // The rest of the population is generated by randomly copying genomes from the elites and applying mutation.
    // This introduces genetic diversity while relying on proven successful traits.
    CrossoverAndMutate(population);

    return true;
}

void EvolvePopulation(std::vector<Car>& population, Vector2 startPosition, float startRotation, int genCount) {
    Config::MUTATION = std::max(1, (int)(Config::MAX_MUTATION / (1.0f + (Config::DROP_RATE * genCount))));

    std::sort(population.begin(), population.end(), [](const Car& a, const Car& b) {
        return a.fitness > b.fitness;
    });

    SaveBestBrains(population, genCount);

    // Breed the next generation using crossover and mutation on the elite pool.
    CrossoverAndMutate(population);

    for (auto& car : population) {
        car.Reset(startPosition.x, startPosition.y, startRotation);
    }
    std::cout << "Generation finished. Evolving population!" << std::endl;
}

} // namespace Evolution
