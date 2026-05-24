#pragma once

#include "Car.h"
#include <vector>

/**
 * @brief Algoritmos genéticos y persistencia para la evolución de la red neuronal.
 * 
 * Gestiona la selección natural, mutaciones y cruces de la población de coches,
 * además de manejar la carga y guardado de los genomas (pesos y sesgos).
 */
namespace Evolution {
    /**
     * @brief Guarda en disco los cerebros de los mejores coches de la generación actual.
     * 
     * @param population La población completa de coches evaluada.
     */
    void GuardarMejoresCerebros(const std::vector<Car>& population);

    /**
     * @brief Carga desde el disco los mejores cerebros previamente guardados para la nueva generación.
     * 
     * @param population La población a la que se le inyectarán los cerebros cargados.
     * @return true Si la carga fue exitosa.
     * @return false Si ocurrió un error en la carga o no se encontró el archivo.
     */
    bool CargarMejoresCerebros(std::vector<Car>& population);

    /**
     * @brief Evalúa la población actual y aplica algoritmos genéticos para generar la próxima generación.
     * 
     * @param population Referencia a la población actual, que será mutada y reemplazada.
     * @param startPosition Posición inicial para resetear los coches.
     * @param startRotation Rotación inicial para resetear los coches.
     */
    void EvolvePopulation(std::vector<Car>& population, Vector2 startPosition, float startRotation);
}
