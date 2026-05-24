#include "Evolution.h"
#include "Config.h"
#include <fstream>
#include <iostream>
#include <algorithm>

namespace Evolution {

void GuardarMejoresCerebros(const std::vector<Car>& population) {
    std::ofstream mejoresFile("data/mejores.txt", std::ios::out);
    for(int n = 0; n < Config::NUM_MEJORES; n++) {
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            for(int j = 0; j < NODOS_ENTRADA; j++) {
                mejoresFile << population[n].brain.peso_entrada_oculta[i][j] << " ";
            }
        }
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            mejoresFile << population[n].brain.sesgos_oculta[i] << " ";
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            for(int j = 0; j < NODOS_OCULTOS; j++) {
                mejoresFile << population[n].brain.peso_oculta_salida[i][j] << " ";
            }
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            mejoresFile << population[n].brain.sesgos_salida[i] << " ";
        }
        mejoresFile << std::endl;
    }
}

bool CargarMejoresCerebros(std::vector<Car>& population) {
    std::ifstream file("data/mejores.txt");
    if (!file.is_open()) return false;

    // 1. Cargamos los mejores directamente del txt
    for(int n = 0; n < Config::NUM_MEJORES && n < (int)population.size(); n++) {
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            for(int j = 0; j < NODOS_ENTRADA; j++) {
                if (!(file >> population[n].brain.peso_entrada_oculta[i][j])) return false;
            }
        }
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            if (!(file >> population[n].brain.sesgos_oculta[i])) return false;
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            for(int j = 0; j < NODOS_OCULTOS; j++) {
                if (!(file >> population[n].brain.peso_oculta_salida[i][j])) return false;
            }
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            if (!(file >> population[n].brain.sesgos_salida[i])) return false;
        }
    }

    // 2. Generamos el "Frankenstein" para los restantes basándonos en los que acabamos de leer.
    for(int n = Config::NUM_MEJORES; n < Config::POPULATION_SIZE && n < (int)population.size(); n++) {
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            for(int j = 0; j < NODOS_ENTRADA; j++) {
                population[n].brain.peso_entrada_oculta[i][j] = population[rand()%Config::NUM_MEJORES].brain.peso_entrada_oculta[i][j];
                if(rand()%100 < Config::MUTACION) population[n].brain.peso_entrada_oculta[i][j] += population[n].brain.MutateGaussian();
            }
        }
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            population[n].brain.sesgos_oculta[i] = population[rand()%Config::NUM_MEJORES].brain.sesgos_oculta[i];
            if(rand()%100 < Config::MUTACION) population[n].brain.sesgos_oculta[i] += population[n].brain.MutateGaussian();
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            for(int j = 0; j < NODOS_OCULTOS; j++) {
                population[n].brain.peso_oculta_salida[i][j] = population[rand()%Config::NUM_MEJORES].brain.peso_oculta_salida[i][j];
                if(rand()%100 < Config::MUTACION) population[n].brain.peso_oculta_salida[i][j] += population[n].brain.MutateGaussian();
            }
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            population[n].brain.sesgos_salida[i] = population[rand()%Config::NUM_MEJORES].brain.sesgos_salida[i];
            if(rand()%100 < Config::MUTACION) population[n].brain.sesgos_salida[i] += population[n].brain.MutateGaussian();
        }
    }
    
    return true;
}

void EvolvePopulation(std::vector<Car>& population, Vector2 startPosition, float startRotation) {
    std::sort(population.begin(), population.end(), [](const Car& a, const Car& b) {
        return a.fitness > b.fitness;
    });

    GuardarMejoresCerebros(population);
    
    for(int n = Config::NUM_MEJORES; n < Config::POPULATION_SIZE; n++) {
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            for(int j = 0; j < NODOS_ENTRADA; j++) {
                population[n].brain.peso_entrada_oculta[i][j] = population[rand()%Config::NUM_MEJORES].brain.peso_entrada_oculta[i][j];
                if(rand()%100 < Config::MUTACION){
                    population[n].brain.peso_entrada_oculta[i][j] += population[n].brain.MutateGaussian();
                }
            }
        }
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            population[n].brain.sesgos_oculta[i] = population[rand()%Config::NUM_MEJORES].brain.sesgos_oculta[i];
            if(rand()%100 < Config::MUTACION){
                population[n].brain.sesgos_oculta[i] += population[n].brain.MutateGaussian();
            }
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            for(int j = 0; j < NODOS_OCULTOS; j++) {
                population[n].brain.peso_oculta_salida[i][j] = population[rand()%Config::NUM_MEJORES].brain.peso_oculta_salida[i][j];
                if(rand()%100 < Config::MUTACION){
                    population[n].brain.peso_oculta_salida[i][j] += population[n].brain.MutateGaussian();
                }
            }
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            population[n].brain.sesgos_salida[i] = population[rand()%Config::NUM_MEJORES].brain.sesgos_salida[i];
            if(rand()%100 < Config::MUTACION){
                population[n].brain.sesgos_salida[i] += population[n].brain.MutateGaussian();
            }
        }
    }

    for (auto& car : population) {
        car.Reset(startPosition.x, startPosition.y, startRotation);
    }
    std::cout << "Generación terminada. ¡Evolucionando población!" << std::endl;
}

} // namespace Evolution
