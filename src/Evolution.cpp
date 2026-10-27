#include "Evolution.h"
#include "Config.h"
#include "Simulation.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <random>
#include "json.hpp"

namespace Evolution {

static std::vector<Species> activeSpecies;
static int nextSpeciesId = 1;

static float SumFeedforwardDiff(const Brain& a, const Brain& b) noexcept {
    float diff = 0.0f;
    for (int i = 0; i < HIDDEN_NODES; ++i) {
        for (int j = 0; j < INPUT_NODES; ++j) {
            diff += std::abs(a.weights_input_hidden[i][j] - b.weights_input_hidden[i][j]);
        }
        diff += std::abs(a.biases_hidden[i] - b.biases_hidden[i]);
    }
    for (int i = 0; i < OUTPUT_NODES; ++i) {
        for (int j = 0; j < HIDDEN_NODES; ++j) {
            diff += std::abs(a.weights_hidden_output[i][j] - b.weights_hidden_output[i][j]);
        }
        diff += std::abs(a.biases_output[i] - b.biases_output[i]);
    }
    return diff;
}

static float SumRecurrentDiff(const Brain& a, const Brain& b) noexcept {
    float diff = 0.0f;
    for (int i = 0; i < RECURRENT_NODES; ++i) {
        for (int j = 0; j < INPUT_NODES; ++j) {
            diff += std::abs(a.W_f[i][j] - b.W_f[i][j]);
            diff += std::abs(a.W_h[i][j] - b.W_h[i][j]);
        }
        for (int j = 0; j < RECURRENT_NODES; ++j) {
            diff += std::abs(a.U_f[i][j] - b.U_f[i][j]);
            diff += std::abs(a.U_h[i][j] - b.U_h[i][j]);
        }
        diff += std::abs(a.b_f[i] - b.b_f[i]);
        diff += std::abs(a.b_h[i] - b.b_h[i]);
    }
    for (int o = 0; o < OUTPUT_NODES; ++o) {
        for (int j = 0; j < RECURRENT_NODES; ++j) {
            diff += std::abs(a.W_out[o][j] - b.W_out[o][j]);
        }
        diff += std::abs(a.b_out[o] - b.b_out[o]);
    }
    return diff;
}

float ComputeCompatibilityDistance(const Brain& a, const Brain& b,
                                   float c1, float c2, float c3) {
    constexpr int FEEDFORWARD_GENES = HIDDEN_NODES * INPUT_NODES + HIDDEN_NODES +
                                      OUTPUT_NODES * HIDDEN_NODES + OUTPUT_NODES; // 152
    constexpr int RECURRENT_GENES = RECURRENT_NODES * INPUT_NODES * 2 +
                                    RECURRENT_NODES * RECURRENT_NODES * 2 +
                                    RECURRENT_NODES * 2 +
                                    OUTPUT_NODES * RECURRENT_NODES + OUTPUT_NODES; // 354
    constexpr int TOTAL_RECURRENT_GENES = FEEDFORWARD_GENES + RECURRENT_GENES; // 506

    int E = 0;
    int D = 0;
    int N = FEEDFORWARD_GENES;
    float avgWDiff = 0.0f;

    if (a.useRecurrent == b.useRecurrent) {
        if (!a.useRecurrent) {
            float diff = SumFeedforwardDiff(a, b);
            avgWDiff = diff / static_cast<float>(FEEDFORWARD_GENES);
            N = FEEDFORWARD_GENES;
        } else {
            float diff = SumFeedforwardDiff(a, b) + SumRecurrentDiff(a, b);
            avgWDiff = diff / static_cast<float>(TOTAL_RECURRENT_GENES);
            N = TOTAL_RECURRENT_GENES;
        }
    } else {
        E = RECURRENT_GENES;
        N = TOTAL_RECURRENT_GENES;
        float diff = SumFeedforwardDiff(a, b);
        avgWDiff = diff / static_cast<float>(FEEDFORWARD_GENES);
    }

    float delta = (c1 * static_cast<float>(E) + c2 * static_cast<float>(D)) / static_cast<float>(N) + c3 * avgWDiff;
    return delta;
}

void ClusterSpecies(const std::vector<Car>& population, std::vector<Species>& speciesList,
                    float threshold) {
    for (auto& s : speciesList) {
        s.memberIndices.clear();
    }

    for (size_t i = 0; i < population.size(); ++i) {
        bool found = false;
        for (auto& s : speciesList) {
            float dist = ComputeCompatibilityDistance(population[i].brain, s.representative,
                                                    Config::SPECIATION_C1,
                                                    Config::SPECIATION_C2,
                                                    Config::SPECIATION_C3);
            if (dist < threshold) {
                s.memberIndices.push_back(static_cast<int>(i));
                found = true;
                break;
            }
        }

        if (!found) {
            Species newSpecies;
            newSpecies.id = nextSpeciesId++;
            newSpecies.representative = population[i].brain;
            newSpecies.memberIndices.push_back(static_cast<int>(i));
            newSpecies.maxFitness = population[i].fitness;
            newSpecies.averageFitness = population[i].fitness;
            newSpecies.staleness = 0;
            speciesList.push_back(newSpecies);
        }
    }

    speciesList.erase(std::remove_if(speciesList.begin(), speciesList.end(),
                                     [](const Species& s) { return s.memberIndices.empty(); }),
                      speciesList.end());

    for (auto& s : speciesList) {
        float sumFit = 0.0f;
        float maxFit = -1e9f;
        int bestMemberIdx = s.memberIndices[0];
        for (int idx : s.memberIndices) {
            float f = population[idx].fitness;
            sumFit += f;
            if (f > maxFit) {
                maxFit = f;
                bestMemberIdx = idx;
            }
        }
        s.averageFitness = sumFit / static_cast<float>(s.memberIndices.size());
        if (maxFit > s.maxFitness) {
            s.maxFitness = maxFit;
            s.staleness = 0;
        } else {
            s.staleness++;
        }
        s.representative = population[bestMemberIdx].brain;
    }
}

const std::vector<Species>& GetActiveSpecies() {
    return activeSpecies;
}

void ApplyFitnessSharing(std::vector<Car>& population, std::vector<Species>& speciesList) {
    for (auto& s : speciesList) {
        float nicheSize = static_cast<float>(std::max(1, static_cast<int>(s.memberIndices.size())));
        float sumAdjusted = 0.0f;
        for (int idx : s.memberIndices) {
            float rawFit = std::max(0.0f, population[idx].fitness);
            population[idx].adjustedFitness = rawFit / nicheSize;
            population[idx].speciesId = s.id;
            sumAdjusted += population[idx].adjustedFitness;
        }
        if (s.staleness > 15) {
            sumAdjusted *= 0.2f; // Penalize stagnant lineages
        }
        s.adjustedFitnessSum = sumAdjusted;
    }
}

std::vector<int> ComputeSpeciesAllotments(const std::vector<Species>& speciesList, int totalSlots) {
    std::vector<int> allotments(speciesList.size(), 0);
    if (speciesList.empty() || totalSlots <= 0) return allotments;

    float totalAdjustedFitness = 0.0f;
    for (const auto& s : speciesList) {
        totalAdjustedFitness += s.adjustedFitnessSum;
    }

    if (totalAdjustedFitness <= 1e-6f) {
        int base = totalSlots / static_cast<int>(speciesList.size());
        int remainder = totalSlots % static_cast<int>(speciesList.size());
        for (size_t i = 0; i < speciesList.size(); ++i) {
            allotments[i] = base + (static_cast<int>(i) < remainder ? 1 : 0);
        }
        return allotments;
    }

    int allocated = 0;
    for (size_t i = 0; i < speciesList.size(); ++i) {
        float ratio = speciesList[i].adjustedFitnessSum / totalAdjustedFitness;
        int count = static_cast<int>(std::round(ratio * static_cast<float>(totalSlots)));
        if (count == 0 && !speciesList[i].memberIndices.empty() && allocated < totalSlots) {
            count = 1;
        }
        allotments[i] = count;
        allocated += count;
    }

    int diff = totalSlots - allocated;
    while (diff != 0) {
        if (diff > 0) {
            size_t bestIdx = 0;
            float maxFit = -1.0f;
            for (size_t i = 0; i < speciesList.size(); ++i) {
                if (speciesList[i].adjustedFitnessSum > maxFit) {
                    maxFit = speciesList[i].adjustedFitnessSum;
                    bestIdx = i;
                }
            }
            allotments[bestIdx]++;
            diff--;
        } else {
            size_t maxAllocIdx = 0;
            int maxAlloc = 0;
            for (size_t i = 0; i < speciesList.size(); ++i) {
                if (allotments[i] > maxAlloc) {
                    maxAlloc = allotments[i];
                    maxAllocIdx = i;
                }
            }
            if (allotments[maxAllocIdx] > 1) {
                allotments[maxAllocIdx]--;
                diff++;
            } else {
                break;
            }
        }
    }

    return allotments;
}

static void CrossAndMutateOffspring(Car& child, const Car& parentA, const Car& parentB) {
    thread_local static std::random_device rd;
    thread_local static std::mt19937 generator(rd());
    std::uniform_int_distribution<int> distMutation(0, 99);

    for (int i = 0; i < HIDDEN_NODES; i++) {
        for (int j = 0; j < INPUT_NODES; j++) {
            const Car& ref = (distMutation(generator) < 50) ? parentA : parentB;
            child.brain.weights_input_hidden[i][j] = ref.brain.weights_input_hidden[i][j];
            if (distMutation(generator) < Config::MUTATION)
                child.brain.weights_input_hidden[i][j] += child.brain.MutateGaussian(Config::MUTATION_SIGMA);
        }
        const Car& ref = (distMutation(generator) < 50) ? parentA : parentB;
        child.brain.biases_hidden[i] = ref.brain.biases_hidden[i];
        if (distMutation(generator) < Config::MUTATION)
            child.brain.biases_hidden[i] += child.brain.MutateGaussian(Config::MUTATION_SIGMA);
    }
    for (int i = 0; i < OUTPUT_NODES; i++) {
        for (int j = 0; j < HIDDEN_NODES; j++) {
            const Car& ref = (distMutation(generator) < 50) ? parentA : parentB;
            child.brain.weights_hidden_output[i][j] = ref.brain.weights_hidden_output[i][j];
            if (distMutation(generator) < Config::MUTATION)
                child.brain.weights_hidden_output[i][j] += child.brain.MutateGaussian(Config::MUTATION_SIGMA);
        }
        const Car& ref = (distMutation(generator) < 50) ? parentA : parentB;
        child.brain.biases_output[i] = ref.brain.biases_output[i];
        if (distMutation(generator) < Config::MUTATION)
            child.brain.biases_output[i] += child.brain.MutateGaussian(Config::MUTATION_SIGMA);
    }

    child.brain.useRecurrent = (distMutation(generator) < 50) ? parentA.brain.useRecurrent : parentB.brain.useRecurrent;
    for (int i = 0; i < RECURRENT_NODES; ++i) {
        for (int j = 0; j < INPUT_NODES; ++j) {
            const Car& ref = (distMutation(generator) < 50) ? parentA : parentB;
            child.brain.W_f[i][j] = ref.brain.W_f[i][j];
            child.brain.W_h[i][j] = ref.brain.W_h[i][j];
            if (distMutation(generator) < Config::MUTATION) {
                child.brain.W_f[i][j] += child.brain.MutateGaussian(Config::MUTATION_SIGMA);
                child.brain.W_h[i][j] += child.brain.MutateGaussian(Config::MUTATION_SIGMA);
            }
        }
        for (int j = 0; j < RECURRENT_NODES; ++j) {
            const Car& ref = (distMutation(generator) < 50) ? parentA : parentB;
            child.brain.U_f[i][j] = ref.brain.U_f[i][j];
            child.brain.U_h[i][j] = ref.brain.U_h[i][j];
            if (distMutation(generator) < Config::MUTATION) {
                child.brain.U_f[i][j] += child.brain.MutateGaussian(Config::MUTATION_SIGMA);
                child.brain.U_h[i][j] += child.brain.MutateGaussian(Config::MUTATION_SIGMA);
            }
        }
        const Car& ref = (distMutation(generator) < 50) ? parentA : parentB;
        child.brain.b_f[i] = ref.brain.b_f[i];
        child.brain.b_h[i] = ref.brain.b_h[i];
        if (distMutation(generator) < Config::MUTATION) {
            child.brain.b_f[i] += child.brain.MutateGaussian(Config::MUTATION_SIGMA);
            child.brain.b_h[i] += child.brain.MutateGaussian(Config::MUTATION_SIGMA);
        }
    }
    for (int o = 0; o < OUTPUT_NODES; ++o) {
        for (int j = 0; j < RECURRENT_NODES; ++j) {
            const Car& ref = (distMutation(generator) < 50) ? parentA : parentB;
            child.brain.W_out[o][j] = ref.brain.W_out[o][j];
            if (distMutation(generator) < Config::MUTATION)
                child.brain.W_out[o][j] += child.brain.MutateGaussian(Config::MUTATION_SIGMA);
        }
        const Car& ref = (distMutation(generator) < 50) ? parentA : parentB;
        child.brain.b_out[o] = ref.brain.b_out[o];
        if (distMutation(generator) < Config::MUTATION)
            child.brain.b_out[o] += child.brain.MutateGaussian(Config::MUTATION_SIGMA);
    }
}

/**
 * @brief Applies two-parent crossover and Gaussian mutation to all non-elite individuals.
 *
 * Selects parents utilizing explicit species niche allotments and fitness sharing,
 * preserving top elites and breeding intra-species offspring.
 *
 * @param population The full population. Indices 0..NUM_BEST-1 are treated as elite
 *                   (read-only parents). Indices NUM_BEST..POPULATION_SIZE-1 are overwritten.
 */
static void CrossoverAndMutate(std::vector<Car>& population) {
    thread_local static std::random_device rd;
    thread_local static std::mt19937 generator(rd());

    int totalNonElite = Config::POPULATION_SIZE - Config::NUM_BEST;
    if (totalNonElite <= 0) return;

    if (!activeSpecies.empty()) {
        std::vector<Car> oldPopulation = population;
        std::vector<int> allotments = ComputeSpeciesAllotments(activeSpecies, totalNonElite);
        int writeIdx = Config::NUM_BEST;

        for (size_t sIdx = 0; sIdx < activeSpecies.size() && writeIdx < Config::POPULATION_SIZE; ++sIdx) {
            const auto& sp = activeSpecies[sIdx];
            int count = allotments[sIdx];
            if (count <= 0 || sp.memberIndices.empty()) continue;

            std::uniform_int_distribution<int> distMember(0, static_cast<int>(sp.memberIndices.size()) - 1);
            for (int k = 0; k < count && writeIdx < Config::POPULATION_SIZE; ++k) {
                int p1Idx = sp.memberIndices[distMember(generator)];
                int p2Idx = sp.memberIndices[distMember(generator)];
                CrossAndMutateOffspring(population[writeIdx], oldPopulation[p1Idx], oldPopulation[p2Idx]);
                writeIdx++;
            }
        }

        // Fill any remaining slots by breeding from top elites
        std::uniform_int_distribution<int> distBest(0, Config::NUM_BEST - 1);
        while (writeIdx < Config::POPULATION_SIZE && writeIdx < static_cast<int>(population.size())) {
            int p1 = distBest(generator);
            int p2 = distBest(generator);
            CrossAndMutateOffspring(population[writeIdx], oldPopulation[p1], oldPopulation[p2]);
            writeIdx++;
        }
        return;
    }

    std::uniform_int_distribution<int> distBest(0, Config::NUM_BEST - 1);
    std::uniform_int_distribution<int> distMutation(0, 99);

    for (int n = Config::NUM_BEST; n < Config::POPULATION_SIZE && n < (int)population.size(); n++) {
        int father = distBest(generator);
        int mother;
        do {
            mother = distBest(generator);
        } while (mother == father);
        CrossAndMutateOffspring(population[n], population[father], population[mother]);
    }
}

void SaveBestBrains(const std::vector<Car>& population, int generation, const std::string& filepath) {
    std::ofstream bestFile(filepath, std::ios::out);
    nlohmann::json data;
    data["generation"] = generation;
    for (int n = 0; n < Config::NUM_BEST && n < static_cast<int>(population.size()); n++) {
        nlohmann::json car;
        car["id"] = n;
        car["weights_input_hidden"] = population[n].brain.weights_input_hidden;
        car["biases_hidden"] = population[n].brain.biases_hidden;
        car["weights_hidden_output"] = population[n].brain.weights_hidden_output;
        car["biases_output"] = population[n].brain.biases_output;

        // Recurrent MGU matrices
        car["useRecurrent"] = population[n].brain.useRecurrent;
        car["W_f"] = population[n].brain.W_f;
        car["U_f"] = population[n].brain.U_f;
        car["b_f"] = population[n].brain.b_f;
        car["W_h"] = population[n].brain.W_h;
        car["U_h"] = population[n].brain.U_h;
        car["b_h"] = population[n].brain.b_h;
        car["W_out"] = population[n].brain.W_out;
        car["b_out"] = population[n].brain.b_out;

        data["elite"].push_back(car);
    }
    bestFile << data.dump(4);
}

bool LoadBestBrains(std::vector<Car>& population, int& generation, const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        if (filepath == "data/best.json") {
            file.open("data/mejores.json");
            if (!file.is_open()) return false;
        } else {
            return false;
        }
    }

    nlohmann::json data;
    try {
        data = nlohmann::json::parse(file);
    } catch (const nlohmann::json::parse_error& e) {
        std::cerr << "Error: Failed to parse brain data file: " << e.what() << std::endl;
        return false;
    }

    // Validate network topology compatibility
    if (!data.contains("elite") || data["elite"].empty() ||
        (data["elite"][0].contains("biases_hidden") && data["elite"][0]["biases_hidden"].size() != HIDDEN_NODES) ||
        (data["elite"][0].contains("weights_input_hidden") && data["elite"][0]["weights_input_hidden"][0].size() != INPUT_NODES)) {
        generation = 0;
        std::cout << "Notice: '" << filepath << "' contains legacy topology. Starting population with fresh network." << std::endl;
        return false;
    }

    if (data.contains("generacion")) generation = data["generacion"];
    else if (data.contains("generation")) generation = data["generation"];

    UpdateMutationParameters(generation);

    for (int n = 0; n < Config::NUM_BEST && n < static_cast<int>(population.size()) && n < static_cast<int>(data["elite"].size()); n++) {
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

        // Deserialize recurrent MGU matrices if present
        if (data["elite"][n].contains("useRecurrent")) {
            population[n].brain.useRecurrent = data["elite"][n]["useRecurrent"].get<bool>();
        } else {
            population[n].brain.useRecurrent = false;
        }

        if (data["elite"][n].contains("W_f")) {
            for (int i = 0; i < RECURRENT_NODES; ++i) {
                for (int j = 0; j < INPUT_NODES; ++j) {
                    population[n].brain.W_f[i][j] = data["elite"][n]["W_f"][i][j];
                    population[n].brain.W_h[i][j] = data["elite"][n]["W_h"][i][j];
                }
                for (int j = 0; j < RECURRENT_NODES; ++j) {
                    population[n].brain.U_f[i][j] = data["elite"][n]["U_f"][i][j];
                    population[n].brain.U_h[i][j] = data["elite"][n]["U_h"][i][j];
                }
                population[n].brain.b_f[i] = data["elite"][n]["b_f"][i];
                population[n].brain.b_h[i] = data["elite"][n]["b_h"][i];
            }
            for (int o = 0; o < OUTPUT_NODES; ++o) {
                for (int j = 0; j < RECURRENT_NODES; ++j) {
                    population[n].brain.W_out[o][j] = data["elite"][n]["W_out"][o][j];
                }
                population[n].brain.b_out[o] = data["elite"][n]["b_out"][o];
            }
        }
    }

    CrossoverAndMutate(population);
    return true;
}

/**
 * @brief Updates generational mutation rate and Gaussian perturbation step size.
 *
 * Implements a Two-Stage Normalized Hyperbolic Schedule:
 * - Stage 1 (Gens 0-200): Decays smoothly from 50% down to 25% (sigma 0.12 to 0.08),
 *   providing abundant phenotypic plasticity for Curriculum and Procedural generalization.
 *   Touches exactly 25% and sigma 0.08 at generation 200.
 * - Stage 2 (Gens 200-400): Gently continues descending from 25% down to 3% (sigma 0.08 to 0.035),
 *   progressively eliminating deleterious mutation load on competent driving policies.
 *   Touches exactly 3% and sigma 0.035 at generation 400.
 * - Stage 3 (Gens 400+): Operates indefinitely at the ultra-fine floor (3%, sigma 0.035).
 *
 * @param generation Current simulation generation index.
 */
void UpdateMutationParameters(int generation) {
    if (generation < Config::MUTATION_HORIZON_STAGE1) {
        float rawCurrent = 1.0f / (1.0f + (Config::DROP_RATE_STAGE1 * (float)generation));
        float rawHorizon = 1.0f / (1.0f + (Config::DROP_RATE_STAGE1 * (float)Config::MUTATION_HORIZON_STAGE1));
        float decayFactor = std::max(0.0f, (rawCurrent - rawHorizon) / (1.0f - rawHorizon));
        Config::MUTATION = Config::STAGE1_MUTATION_MIN + (int)std::round((Config::STAGE1_MUTATION_MAX - Config::STAGE1_MUTATION_MIN) * decayFactor);
        Config::MUTATION_SIGMA = Config::STAGE1_SIGMA_MIN + (Config::STAGE1_SIGMA_MAX - Config::STAGE1_SIGMA_MIN) * decayFactor;
    } else if (generation < Config::MUTATION_HORIZON_STAGE2) {
        float genOffset = (float)(generation - Config::MUTATION_HORIZON_STAGE1);
        float horizonSpan = (float)(Config::MUTATION_HORIZON_STAGE2 - Config::MUTATION_HORIZON_STAGE1);
        float rawCurrent = 1.0f / (1.0f + (Config::DROP_RATE_STAGE2 * genOffset));
        float rawHorizon = 1.0f / (1.0f + (Config::DROP_RATE_STAGE2 * horizonSpan));
        float decayFactor = std::max(0.0f, (rawCurrent - rawHorizon) / (1.0f - rawHorizon));
        Config::MUTATION = Config::STAGE2_MUTATION_MIN + (int)std::round((Config::STAGE2_MUTATION_MAX - Config::STAGE2_MUTATION_MIN) * decayFactor);
        Config::MUTATION_SIGMA = Config::STAGE2_SIGMA_MIN + (Config::STAGE2_SIGMA_MAX - Config::STAGE2_SIGMA_MIN) * decayFactor;
    } else {
        Config::MUTATION = Config::STAGE2_MUTATION_MIN;
        Config::MUTATION_SIGMA = Config::STAGE2_SIGMA_MIN;
    }
}

void EvolvePopulation(std::vector<Car>& population, Vector2 startPosition, float startRotation, int genCount) {
    UpdateMutationParameters(genCount);

    std::sort(population.begin(), population.end(), [](const Car& a, const Car& b) {
        return a.fitness > b.fitness;
    });

    ClusterSpecies(population, activeSpecies, Config::SPECIATION_THRESHOLD);
    ApplyFitnessSharing(population, activeSpecies);

    SaveBestBrains(population, genCount);

    // Breed the next generation using crossover and mutation on the elite pool.
    CrossoverAndMutate(population);

    for (auto& car : population) {
        car.Reset(startPosition.x, startPosition.y, startRotation);
    }
    std::cout << "Generation finished. Evolving population! (" << activeSpecies.size() << " active species)" << std::endl;
}

} // namespace Evolution
