#pragma once

#include "Car.h"
#include <vector>

namespace Evolution {
    void GuardarMejoresCerebros(const std::vector<Car>& population);
    bool CargarMejoresCerebros(std::vector<Car>& population);
    void EvolvePopulation(std::vector<Car>& population, Vector2 startPosition, float startRotation);
}
