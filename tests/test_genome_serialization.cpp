#include "Evolution.h"
#include "Brain.h"
#include "Car.h"
#include "Config.h"
#include <cassert>
#include <iostream>
#include <cmath>
#include <vector>
#include <cstdio>

void TestFeedforwardSerializationRoundtrip() {
    std::string testPath = "build/test_ff_brain.json";
    std::vector<Car> popSave;
    popSave.reserve(Config::NUM_BEST);

    for (int i = 0; i < Config::NUM_BEST; ++i) {
        popSave.emplace_back(100.0f, 100.0f, 0.0f);
        popSave.back().brain.useRecurrent = false;
        popSave.back().brain.weights_input_hidden[0][0] = 0.42f + static_cast<float>(i) * 0.01f;
        popSave.back().brain.biases_hidden[1] = -0.15f;
    }

    Evolution::SaveBestBrains(popSave, 42, testPath);

    std::vector<Car> popLoad(Config::POPULATION_SIZE, Car(100.0f, 100.0f, 0.0f));
    int loadedGen = 0;
    bool success = Evolution::LoadBestBrains(popLoad, loadedGen, testPath);

    assert(success);
    assert(loadedGen == 42);
    for (int i = 0; i < Config::NUM_BEST; ++i) {
        assert(!popLoad[i].brain.useRecurrent);
        float expectedW = 0.42f + static_cast<float>(i) * 0.01f;
        assert(std::abs(popLoad[i].brain.weights_input_hidden[0][0] - expectedW) < 1e-5f);
        assert(std::abs(popLoad[i].brain.biases_hidden[1] - (-0.15f)) < 1e-5f);
    }

    std::remove(testPath.c_str());
    std::cout << "[PASS] TestFeedforwardSerializationRoundtrip passed." << std::endl;
}

void TestRecurrentMGUSerializationRoundtrip() {
    std::string testPath = "build/test_mgu_brain.json";
    std::vector<Car> popSave;
    popSave.reserve(Config::NUM_BEST);

    for (int i = 0; i < Config::NUM_BEST; ++i) {
        popSave.emplace_back(100.0f, 100.0f, 0.0f);
        popSave.back().brain.useRecurrent = true;
        popSave.back().brain.W_f[2][3] = 0.888f;
        popSave.back().brain.U_f[1][1] = -0.777f;
        popSave.back().brain.b_f[4] = 0.555f;
        popSave.back().brain.W_h[0][5] = 0.333f;
        popSave.back().brain.U_h[3][2] = -0.222f;
        popSave.back().brain.b_h[1] = 0.111f;
        popSave.back().brain.W_out[1][7] = 0.999f;
        popSave.back().brain.b_out[0] = -0.444f;
    }

    Evolution::SaveBestBrains(popSave, 105, testPath);

    std::vector<Car> popLoad(Config::POPULATION_SIZE, Car(100.0f, 100.0f, 0.0f));
    int loadedGen = 0;
    bool success = Evolution::LoadBestBrains(popLoad, loadedGen, testPath);

    assert(success);
    assert(loadedGen == 105);
    for (int i = 0; i < Config::NUM_BEST; ++i) {
        assert(popLoad[i].brain.useRecurrent == true);
        assert(std::abs(popLoad[i].brain.W_f[2][3] - 0.888f) < 1e-5f);
        assert(std::abs(popLoad[i].brain.U_f[1][1] - (-0.777f)) < 1e-5f);
        assert(std::abs(popLoad[i].brain.b_f[4] - 0.555f) < 1e-5f);
        assert(std::abs(popLoad[i].brain.W_h[0][5] - 0.333f) < 1e-5f);
        assert(std::abs(popLoad[i].brain.U_h[3][2] - (-0.222f)) < 1e-5f);
        assert(std::abs(popLoad[i].brain.b_h[1] - 0.111f) < 1e-5f);
        assert(std::abs(popLoad[i].brain.W_out[1][7] - 0.999f) < 1e-5f);
        assert(std::abs(popLoad[i].brain.b_out[0] - (-0.444f)) < 1e-5f);
    }

    std::remove(testPath.c_str());
    std::cout << "[PASS] TestRecurrentMGUSerializationRoundtrip passed." << std::endl;
}

int main() {
    std::cout << "Running Genome Serialization & Deserialization Tests..." << std::endl;
    TestFeedforwardSerializationRoundtrip();
    TestRecurrentMGUSerializationRoundtrip();
    std::cout << "All Genome Serialization tests completed successfully!" << std::endl;
    return 0;
}
