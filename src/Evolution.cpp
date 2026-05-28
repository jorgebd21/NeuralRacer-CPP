#include "Evolution.h"
#include "Config.h"
#include "Simulation.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <random>

namespace Evolution {

void GuardarMejoresCerebros(const std::vector<Car>& population, int generacion) {
    std::ofstream mejoresFile("data/mejores.txt", std::ios::out);
    mejoresFile << generacion << " ";
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

bool CargarMejoresCerebros(std::vector<Car>& population, int &generacion) {
    std::ifstream file("data/mejores.txt");
    if (!file.is_open()) return false;

    if (!(file >> generacion)) return false;
    Config::MUTACION = std::max(1, (int)(Config::MAX_MUTACION / (1.0f + (Config::TASA_CAIDA * generacion))));

    // La primera fase de la carga inyecta directamente los cerebros élite de la generacion anterior.
    // Esto asegura que no perdemos el progreso ("elitismo").
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

    // El resto de la población se genera copiando aleatoriamente genomas de los élites y aplicando mutación.
    // Esto introduce diversidad genética mientras se apoya en características exitosas probadas.
    static std::random_device rd;
    static std::mt19937 generador(rd());
    std::uniform_int_distribution<int> distMejores(0, Config::NUM_MEJORES - 1);
    std::uniform_int_distribution<int> distMutacion(0, 99);

    int referente;
    int padre, madre;
    std::pair<int, int> padres;
    
    for(int n = Config::NUM_MEJORES; n < Config::POPULATION_SIZE && n < (int)population.size(); n++) {
        padre = distMejores(generador);
        do{
            madre = distMejores(generador);
        }while(madre == padre);
        padres = {padre, madre};
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            for(int j = 0; j < NODOS_ENTRADA; j++) {
                referente = distMutacion(generador) < 50 ? padres.first : padres.second;
                population[n].brain.peso_entrada_oculta[i][j] = population[referente].brain.peso_entrada_oculta[i][j];
                if(distMutacion(generador) < Config::MUTACION) population[n].brain.peso_entrada_oculta[i][j] += population[n].brain.MutateGaussian();
            }
        }
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            referente = distMutacion(generador) < 50 ? padres.first : padres.second;
            population[n].brain.sesgos_oculta[i] = population[referente].brain.sesgos_oculta[i];
            if(distMutacion(generador) < Config::MUTACION) population[n].brain.sesgos_oculta[i] += population[n].brain.MutateGaussian();
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            for(int j = 0; j < NODOS_OCULTOS; j++) {
                referente = distMutacion(generador) < 50 ? padres.first : padres.second;
                population[n].brain.peso_oculta_salida[i][j] = population[referente].brain.peso_oculta_salida[i][j];
                if(distMutacion(generador) < Config::MUTACION) population[n].brain.peso_oculta_salida[i][j] += population[n].brain.MutateGaussian();
            }
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            referente = distMutacion(generador) < 50 ? padres.first : padres.second;
            population[n].brain.sesgos_salida[i] = population[referente].brain.sesgos_salida[i];
            if(distMutacion(generador) < Config::MUTACION) population[n].brain.sesgos_salida[i] += population[n].brain.MutateGaussian();
        }
    }
    
    return true;
}

void EvolvePopulation(std::vector<Car>& population, Vector2 startPosition, float startRotation, int genCount) {
    Config::MUTACION = std::max(1, (int)(Config::MAX_MUTACION / (1.0f + (Config::TASA_CAIDA * genCount))));
    
    std::sort(population.begin(), population.end(), [](const Car& a, const Car& b) {
        return a.fitness > b.fitness;
    });

    GuardarMejoresCerebros(population, genCount);
    
    static std::random_device rd;
    static std::mt19937 generador(rd());
    std::uniform_int_distribution<int> distMejores(0, Config::NUM_MEJORES - 1);
    std::uniform_int_distribution<int> distMutacion(0, 99);

    int referente;
    int padre, madre;
    std::pair<int, int> padres;
    for(int n = Config::NUM_MEJORES; n < Config::POPULATION_SIZE && n < (int)population.size(); n++) {
        padre = distMejores(generador);
        do{
            madre = distMejores(generador);
        }while(madre == padre);
        padres = {padre, madre};
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            for(int j = 0; j < NODOS_ENTRADA; j++) {
                referente = distMutacion(generador) < 50 ? padres.first : padres.second;
                population[n].brain.peso_entrada_oculta[i][j] = population[referente].brain.peso_entrada_oculta[i][j];
                if(distMutacion(generador) < Config::MUTACION){
                    population[n].brain.peso_entrada_oculta[i][j] += population[n].brain.MutateGaussian();
                }
            }
        }
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            referente = distMutacion(generador) < 50 ? padres.first : padres.second;
            population[n].brain.sesgos_oculta[i] = population[referente].brain.sesgos_oculta[i];
            if(distMutacion(generador) < Config::MUTACION){
                population[n].brain.sesgos_oculta[i] += population[n].brain.MutateGaussian();
            }
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            for(int j = 0; j < NODOS_OCULTOS; j++) {
                referente = distMutacion(generador) < 50 ? padres.first : padres.second;
                population[n].brain.peso_oculta_salida[i][j] = population[referente].brain.peso_oculta_salida[i][j];
                if(distMutacion(generador) < Config::MUTACION){
                    population[n].brain.peso_oculta_salida[i][j] += population[n].brain.MutateGaussian();
                }
            }
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            referente = distMutacion(generador) < 50 ? padres.first : padres.second;
            population[n].brain.sesgos_salida[i] = population[referente].brain.sesgos_salida[i];
            if(distMutacion(generador) < Config::MUTACION){
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
