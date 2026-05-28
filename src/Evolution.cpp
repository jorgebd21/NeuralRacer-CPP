#include "Evolution.h"
#include "Config.h"
#include "Simulation.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <random>
#include "json.hpp"

namespace Evolution {

void GuardarMejoresCerebros(const std::vector<Car>& population, int generacion) {
    std::ofstream mejoresFile("data/mejores.json", std::ios::out);
    nlohmann::json datos;
    datos["generacion"] = generacion;
    for(int n = 0; n < Config::NUM_MEJORES; n++) {
        nlohmann::json coche;
        coche["id"] = n;
        coche["pesos_entrada_oculta"] = population[n].brain.peso_entrada_oculta;
        coche["sesgos_oculta"] = population[n].brain.sesgos_oculta;
        coche["pesos_oculta_salida"] = population[n].brain.peso_oculta_salida;
        coche["sesgos_salida"] = population[n].brain.sesgos_salida;
        datos["elite"].push_back(coche);
    }
    mejoresFile << datos.dump(4);
}

bool CargarMejoresCerebros(std::vector<Car>& population, int &generacion) {
    std::ifstream file("data/mejores.json");
    if (!file.is_open()) return false;
    
    nlohmann::json datos = nlohmann::json::parse(file);
    generacion = datos["generacion"];
    Config::MUTACION = std::max(1, (int)(Config::MAX_MUTACION / (1.0f + (Config::TASA_CAIDA * generacion))));

    // La primera fase de la carga inyecta directamente los cerebros élite de la generacion anterior.
    // Esto asegura que no perdemos el progreso ("elitismo").
    for(int n = 0; n < Config::NUM_MEJORES && n < (int)population.size(); n++) {
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            for(int j = 0; j < NODOS_ENTRADA; j++) {
                population[n].brain.peso_entrada_oculta[i][j] = datos["elite"][n]["pesos_entrada_oculta"][i][j];
            }
        }
        for(int i = 0; i < NODOS_OCULTOS; i++) {
            population[n].brain.sesgos_oculta[i] = datos["elite"][n]["sesgos_oculta"][i];
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            for(int j = 0; j < NODOS_OCULTOS; j++) {
                population[n].brain.peso_oculta_salida[i][j] = datos["elite"][n]["pesos_oculta_salida"][i][j];
            }
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            population[n].brain.sesgos_salida[i] = datos["elite"][n]["sesgos_salida"][i];
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
