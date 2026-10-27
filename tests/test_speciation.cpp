#include "Evolution.h"
#include "Brain.h"
#include "Car.h"
#include "Config.h"
#include <cassert>
#include <iostream>
#include <cmath>
#include <vector>

void TestIdenticalBrainsDistance() {
    Brain b1;
    Brain b2 = b1;

    float dist = Evolution::ComputeCompatibilityDistance(b1, b2);
    assert(std::abs(dist) < 1e-6f);
    std::cout << "[PASS] TestIdenticalBrainsDistance passed (dist = " << dist << ")." << std::endl;
}

void TestExcessGenesDistance() {
    Brain ffBrain;
    ffBrain.useRecurrent = false;

    Brain recBrain = ffBrain;
    recBrain.useRecurrent = true;

    // Same feedforward weights, but recBrain has 354 excess genes
    float dist = Evolution::ComputeCompatibilityDistance(ffBrain, recBrain);
    
    // Theoretical distance: (c1 * 354) / 506 + c3 * 0 = 354 / 506 ≈ 0.6996
    float expectedExcess = (Config::SPECIATION_C1 * 354.0f) / 506.0f;
    assert(std::abs(dist - expectedExcess) < 1e-4f);

    std::cout << "[PASS] TestExcessGenesDistance passed (dist = " << dist 
              << ", expected = " << expectedExcess << ")." << std::endl;
}

void TestWeightDivergenceDistance() {
    Brain b1;
    Brain b2 = b1;

    // Perturb all 152 feedforward weights by exactly +0.10f
    for (int i = 0; i < HIDDEN_NODES; ++i) {
        for (int j = 0; j < INPUT_NODES; ++j) {
            b2.weights_input_hidden[i][j] += 0.10f;
        }
        b2.biases_hidden[i] += 0.10f;
    }
    for (int i = 0; i < OUTPUT_NODES; ++i) {
        for (int j = 0; j < HIDDEN_NODES; ++j) {
            b2.weights_hidden_output[i][j] += 0.10f;
        }
        b2.biases_output[i] += 0.10f;
    }

    float dist = Evolution::ComputeCompatibilityDistance(b1, b2);
    // Expected: c3 * 0.10f = 0.4 * 0.10 = 0.04f
    float expected = Config::SPECIATION_C3 * 0.10f;
    assert(std::abs(dist - expected) < 1e-4f);

    std::cout << "[PASS] TestWeightDivergenceDistance passed (dist = " << dist 
              << ", expected = " << expected << ")." << std::endl;
}

void TestSpeciesClustering() {
    std::vector<Car> population;
    population.reserve(20);

    // Create 10 identical cars of Cluster Alpha
    Brain alphaBrain;
    alphaBrain.useRecurrent = false;
    for (int i = 0; i < 10; ++i) {
        population.emplace_back(100.0f, 100.0f, 0.0f);
        population.back().brain = alphaBrain;
        population.back().fitness = 100.0f + static_cast<float>(i) * 5.0f;
    }

    // Create 10 distinct cars of Cluster Beta (divergent weights)
    Brain betaBrain = alphaBrain;
    for (int i = 0; i < HIDDEN_NODES; ++i) {
        for (int j = 0; j < INPUT_NODES; ++j) {
            betaBrain.weights_input_hidden[i][j] += 3.0f; // Significant divergence
        }
    }
    for (int i = 0; i < 10; ++i) {
        population.emplace_back(100.0f, 100.0f, 0.0f);
        population.back().brain = betaBrain;
        population.back().fitness = 200.0f + static_cast<float>(i) * 2.0f;
    }

    std::vector<Evolution::Species> speciesList;
    Evolution::ClusterSpecies(population, speciesList, Config::SPECIATION_THRESHOLD);

    assert(speciesList.size() == 2);
    assert(speciesList[0].memberIndices.size() == 10);
    assert(speciesList[1].memberIndices.size() == 10);
    assert(speciesList[0].maxFitness == 145.0f);
    assert(speciesList[1].maxFitness == 218.0f);

    std::cout << "[PASS] TestSpeciesClustering passed (2 distinct species clustered correctly)." << std::endl;
}

int main() {
    std::cout << "Running Genomic Compatibility & Speciation Clustering Tests..." << std::endl;
    TestIdenticalBrainsDistance();
    TestExcessGenesDistance();
    TestWeightDivergenceDistance();
    TestSpeciesClustering();
    std::cout << "All Speciation Clustering tests completed successfully!" << std::endl;
    return 0;
}
