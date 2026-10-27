#include "Evolution.h"
#include "Brain.h"
#include "Car.h"
#include "Config.h"
#include <cassert>
#include <iostream>
#include <cmath>
#include <vector>

void TestExplicitFitnessSharingCalculation() {
    std::vector<Car> population;
    population.reserve(12);

    // Species 1: 10 members, raw fitness 100.0 each
    for (int i = 0; i < 10; ++i) {
        population.emplace_back(100.0f, 100.0f, 0.0f);
        population.back().fitness = 100.0f;
    }

    // Species 2: 2 members (innovative niche), raw fitness 60.0 each
    for (int i = 0; i < 2; ++i) {
        population.emplace_back(100.0f, 100.0f, 0.0f);
        population.back().fitness = 60.0f;
    }

    std::vector<Evolution::Species> speciesList(2);
    speciesList[0].id = 1;
    for (int i = 0; i < 10; ++i) speciesList[0].memberIndices.push_back(i);
    speciesList[0].staleness = 0;

    speciesList[1].id = 2;
    speciesList[1].memberIndices.push_back(10);
    speciesList[1].memberIndices.push_back(11);
    speciesList[1].staleness = 0;

    Evolution::ApplyFitnessSharing(population, speciesList);

    // Each member of species 1 should have adjusted fitness = 100.0 / 10 = 10.0
    for (int i = 0; i < 10; ++i) {
        assert(std::abs(population[i].adjustedFitness - 10.0f) < 1e-4f);
        assert(population[i].speciesId == 1);
    }
    assert(std::abs(speciesList[0].adjustedFitnessSum - 100.0f) < 1e-4f);

    // Each member of species 2 should have adjusted fitness = 60.0 / 2 = 30.0
    assert(std::abs(population[10].adjustedFitness - 30.0f) < 1e-4f);
    assert(std::abs(population[11].adjustedFitness - 30.0f) < 1e-4f);
    assert(population[10].speciesId == 2);
    assert(std::abs(speciesList[1].adjustedFitnessSum - 60.0f) < 1e-4f);

    std::cout << "[PASS] TestExplicitFitnessSharingCalculation passed." << std::endl;
}

void TestSpeciesAllotments() {
    std::vector<Evolution::Species> speciesList(2);
    speciesList[0].id = 1;
    speciesList[0].memberIndices = {0, 1, 2, 3};
    speciesList[0].adjustedFitnessSum = 75.0f;

    speciesList[1].id = 2;
    speciesList[1].memberIndices = {4, 5};
    speciesList[1].adjustedFitnessSum = 25.0f;

    int totalSlots = 80;
    std::vector<int> allotments = Evolution::ComputeSpeciesAllotments(speciesList, totalSlots);

    assert(allotments.size() == 2);
    assert(allotments[0] + allotments[1] == totalSlots);
    // 75% of 80 = 60, 25% of 80 = 20
    assert(allotments[0] == 60);
    assert(allotments[1] == 20);

    std::cout << "[PASS] TestSpeciesAllotments passed (Allotments: S1=" 
              << allotments[0] << ", S2=" << allotments[1] << ")." << std::endl;
}

void TestStaleSpeciesPenalty() {
    std::vector<Car> population;
    population.emplace_back(100.0f, 100.0f, 0.0f);
    population.back().fitness = 100.0f;

    std::vector<Evolution::Species> speciesList(1);
    speciesList[0].id = 1;
    speciesList[0].memberIndices = {0};
    speciesList[0].staleness = 20; // > 15 generations stale

    Evolution::ApplyFitnessSharing(population, speciesList);

    // Stale penalty should scale adjustedFitnessSum down to 0.20 * 100.0 = 20.0
    assert(std::abs(speciesList[0].adjustedFitnessSum - 20.0f) < 1e-4f);

    std::cout << "[PASS] TestStaleSpeciesPenalty passed (Stale sum penalized to " 
              << speciesList[0].adjustedFitnessSum << ")." << std::endl;
}

int main() {
    std::cout << "Running Explicit Fitness Sharing & Reproduction Quota Tests..." << std::endl;
    TestExplicitFitnessSharingCalculation();
    TestSpeciesAllotments();
    TestStaleSpeciesPenalty();
    std::cout << "All Explicit Fitness Sharing tests completed successfully!" << std::endl;
    return 0;
}
