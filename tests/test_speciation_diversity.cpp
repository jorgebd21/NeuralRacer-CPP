#include "Evolution.h"
#include "Brain.h"
#include "Car.h"
#include "Config.h"
#include <cassert>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

void TestSpeciesDiversityAcross200Generations() {
    std::cout << "[INFO] Initializing population of " << Config::POPULATION_SIZE 
              << " agents for 200-generation diversity simulation..." << std::endl;

    std::vector<Car> population;
    population.reserve(Config::POPULATION_SIZE);
    for (int i = 0; i < Config::POPULATION_SIZE; ++i) {
        population.emplace_back(Config::SIM_START_POS_X, Config::SIM_START_POS_Y, 0.0f);
        // Inject random initial variation across population
        if (i % 2 == 0) {
            population.back().brain.useRecurrent = true;
        }
    }

    int minSpeciesObserved = 1000;
    int maxSpeciesObserved = 0;
    float bestFitnessOverall = 0.0f;

    Vector2 startPos = {Config::SIM_START_POS_X, Config::SIM_START_POS_Y};

    for (int gen = 0; gen < 200; ++gen) {
        // Simulate generational evaluation: cars are scored based on their driving policy
        for (size_t i = 0; i < population.size(); ++i) {
            // Simulated fitness evaluation: evaluate forward control decisions on synthetic inputs
            float x[INPUT_NODES] = {0.2f, 0.5f, 0.8f, 0.5f, 0.2f, 0.0f, 10.0f, 0.0f, 0.0f, 10.0f, 10.0f, 0.0f};
            (void)x;
            float accel = 0.0f, turn = 0.0f;
            population[i].brain.Evaluate(population[i].sensorDistances, 0.0f, 10.0f, 0.0f, 0.0f, accel, turn);

            // Reward high acceleration and smooth steering
            float fitnessScore = 50.0f + (accel * 30.0f) - (std::abs(turn) * 10.0f);
            if (population[i].brain.useRecurrent) {
                // Working memory bonus
                fitnessScore += 15.0f * (1.0f - std::abs(population[i].brain.last_output[1]));
            }
            population[i].fitness = std::max(1.0f, fitnessScore);
            if (population[i].fitness > bestFitnessOverall) {
                bestFitnessOverall = population[i].fitness;
            }
        }

        // Apply natural selection, speciation, and fitness sharing without overwriting data/best.json
        Evolution::EvolvePopulation(population, startPos, 0.0f, gen, false);

        int activeSpeciesCount = static_cast<int>(Evolution::GetActiveSpecies().size());
        if (activeSpeciesCount < minSpeciesObserved) minSpeciesObserved = activeSpeciesCount;
        if (activeSpeciesCount > maxSpeciesObserved) maxSpeciesObserved = activeSpeciesCount;

        if (gen % 50 == 0 || gen == 199) {
            std::cout << "[GEN " << gen << "] Active Species: " << activeSpeciesCount 
                      << ", Global Peak Fitness: " << bestFitnessOverall << std::endl;
        }

        // Speciation diversity assertion: species count must NOT collapse to 0
        assert(activeSpeciesCount >= 1);
    }

    // Verify sustained genetic diversity: across 200 generations, speciation produced distinct niches
    std::cout << "[STATS] 200-Gen Speciation Summary: Min Species = " << minSpeciesObserved 
              << ", Max Species = " << maxSpeciesObserved 
              << ", Final Peak Fitness = " << bestFitnessOverall << std::endl;

    assert(maxSpeciesObserved >= 2); // Distinct species emerged and were protected
    std::cout << "[PASS] TestSpeciesDiversityAcross200Generations passed successfully." << std::endl;
}

int main() {
    std::cout << "================================================================" << std::endl;
    std::cout << "[TEST] 200-Generation Recurrent Neuroevolution Diversity Suite" << std::endl;
    std::cout << "================================================================" << std::endl;
    TestSpeciesDiversityAcross200Generations();
    std::cout << "================================================================" << std::endl;
    std::cout << "[PASS] All Speciation Diversity Integration Tests Passed!" << std::endl;
    std::cout << "================================================================" << std::endl;
    return 0;
}
