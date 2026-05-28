#pragma once
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "raylib.h"

/**
 * @brief Algoritmos procedimentales para la generación aleatoria de circuitos.
 * 
 * Este espacio de nombres implementa el pipeline completo de creación de mapas
 * usando generación de puntos aleatorios filtrados, resolución del TSP (Viajante de Comercio) 
 * mediante nearest neighbor, y optimización con el algoritmo 2-Opt para evitar auto-intersecciones.
 */
namespace TrackGenerator {
    constexpr int MAX_INTENTS = 2000;
    constexpr float MIN_DIST_SQ = 22500.0f; // Asegura que los puntos no estén demasiado cerca (150^2)
    constexpr int PROCEDURAL_POINTS = 25;
    constexpr int BOUNDS_MIN_X = 150;
    constexpr int BOUNDS_MAX_X = 800;
    constexpr int BOUNDS_MIN_Y = 150;
    constexpr int BOUNDS_MAX_Y = 600;

    /**
     * @brief Genera un conjunto de puntos aleatorios respetando una distancia mínima entre ellos.
     * 
     * @param count Número deseado de puntos a generar.
     * @param minX Límite izquierdo del área.
     * @param maxX Límite derecho del área.
     * @param minY Límite superior del área.
     * @param maxY Límite inferior del área.
     * @return std::vector<Vector2> Lista de puntos válidos generados.
     */
    inline std::vector<Vector2> GenerateRandomPoints(int count, int minX, int maxX, int minY, int maxY, const std::vector<Vector2>& existingPoints = {}) {
        std::vector<Vector2> points = existingPoints;
        int max_intentos = MAX_INTENTS; 
        
        for(int i = 0; i < count; i++){
            Vector2 punto;
            bool valido = false;
            int intentos = 0;
            
            while(!valido && intentos < max_intentos){
                punto.x = GetRandomValue(minX, maxX);
                punto.y = GetRandomValue(minY, maxY);
                valido = true;
                
                for(size_t j = 0; j < points.size(); j++){
                    float dist_cuadrada = (punto.x - points[j].x)*(punto.x - points[j].x) + (punto.y - points[j].y)*(punto.y - points[j].y);
                    
                    // Utilizamos la distancia al cuadrado para evitar el coste computacional
                    // de calcular la raíz cuadrada con sqrt() repetidas veces.
                    if(dist_cuadrada < MIN_DIST_SQ){
                        valido = false;
                        break;
                    }
                }
                intentos++;
            }
            
            if (valido) {
                points.push_back(punto);
            }
        }
        
        return points;
    }

    /**
     * @brief Calcula el producto cruz (cross product) 2D de tres puntos.
     * 
     * @param p0 Punto origen.
     * @param p1 Primer vector.
     * @param p2 Segundo vector.
     * @return float Valor negativo si p2 está a la derecha de p0->p1, positivo si está a la izquierda.
     */
    inline float CrossProduct(Vector2 p0, Vector2 p1, Vector2 p2) {
        return (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
    }

    /**
     * @brief Calcula la distancia Euclidiana al cuadrado entre dos puntos.
     * 
     * @param a Primer punto.
     * @param b Segundo punto.
     * @return float Distancia al cuadrado (evita uso de sqrt por rendimiento).
     */
    inline float Distance(Vector2 a, Vector2 b) {
        return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
    }

    /**
     * @brief Resuelve heurísticamente el problema del Viajante de Comercio (TSP) usando Nearest Neighbor.
     * 
     * @param points Conjunto de puntos a ordenar.
     * @return std::vector<Vector2> El recorrido inicial aproximado.
     */
    inline std::vector<Vector2> SolveTSPNearestNeighbor(const std::vector<Vector2>& points) {
        int n = points.size();
        if (n==0){return {};}

        std::vector<Vector2> tour;
        tour.reserve(n);

        std::vector<bool> visited(n, false);

        tour.push_back(points[0]);
        visited[0] = true;
        
        if (n > 1) {
            tour.push_back(points[1]);
            visited[1] = true;
        }

        for(int i = 2; i < n; i++){
            Vector2 next_point;
            int min_dist = 0;
            int pos_point = -1;
            for(int j = 0; j < n; j++){
                if(!visited[j]){
                    int dist = Distance(tour[i-1], points[j]);
                    if(pos_point == -1 || dist < min_dist){
                        min_dist = dist;
                        pos_point = j;
                        next_point = points[j];
                    }
                }
            }
            tour.push_back(next_point);
            visited[pos_point] = true;
        }

        return tour;
    }

    /**
     * @brief Optimiza un recorrido TSP mediante el algoritmo 2-Opt.
     * 
     * Deshace cruces de líneas intercambiando aristas, lo que es vital
     * para que la pista resultante no tenga colisiones consigo misma.
     * 
     * @param tour Recorrido actual a optimizar.
     * @return std::vector<Vector2> Recorrido optimizado sin auto-intersecciones evidentes.
     */
    inline std::vector<Vector2> Optimize2Opt(std::vector<Vector2> tour) {
        int n = tour.size();

        bool repetir = true;
        while(repetir){
            repetir = false;

            for(int i = 1; i < n-2; i++){
                for(int j = i+2; j < n; j++){
                    Vector2 A = tour[i];
                    Vector2 B = tour[i+1];
                    Vector2 C = tour[j];
                    Vector2 D = tour[(j+1)%n];
                    
                    float distance_vieja = Distance(A, B) + Distance(C, D);
                    float distance_nueva = Distance(A, C) + Distance(B, D);
                    
                    // Si intercambiar los nodos reduce la distancia total,
                    // estamos deshaciendo un cruce de caminos.
                    if(distance_nueva < distance_vieja){
                        repetir = true;
                        std::reverse(tour.begin() + i + 1, tour.begin() + j + 1);
                        break;
                    }
                }

                if(repetir) break;
            }
        }

        return tour;
    }


    /**
     * @brief Ejecuta el pipeline completo y genera los nodos ordenados de la pista.
     * 
     * Incluye una verificación de Winding Order basada en el área de Gauss
     * para asegurar que la pista siempre se genera en sentido antihorario.
     * 
     * @return std::vector<Vector2> Lista final de nodos centrales procedimentales.
     */
    inline std::vector<Vector2> GenerateProceduralCenterPoints() {
        int prefabType = GetRandomValue(0, 3);
        Vector2 p1, p2;
        int minX = BOUNDS_MIN_X, maxX = BOUNDS_MAX_X;
        int minY = BOUNDS_MIN_Y, maxY = BOUNDS_MAX_Y;

        if (prefabType == 0) { // Top
            p1 = {(float)BOUNDS_MIN_X + 150, (float)BOUNDS_MIN_Y};
            p2 = {(float)BOUNDS_MAX_X - 150, (float)BOUNDS_MIN_Y};
            minY = BOUNDS_MIN_Y + 150;
        } else if (prefabType == 1) { // Bottom
            p1 = {(float)BOUNDS_MAX_X - 150, (float)BOUNDS_MAX_Y};
            p2 = {(float)BOUNDS_MIN_X + 150, (float)BOUNDS_MAX_Y};
            maxY = BOUNDS_MAX_Y - 150;
        } else if (prefabType == 2) { // Left
            p1 = {(float)BOUNDS_MIN_X, (float)BOUNDS_MAX_Y - 150};
            p2 = {(float)BOUNDS_MIN_X, (float)BOUNDS_MIN_Y + 150};
            minX = BOUNDS_MIN_X + 150;
        } else { // Right
            p1 = {(float)BOUNDS_MAX_X, (float)BOUNDS_MIN_Y + 150};
            p2 = {(float)BOUNDS_MAX_X, (float)BOUNDS_MAX_Y - 150};
            maxX = BOUNDS_MAX_X - 150;
        }

        std::vector<Vector2> allPoints;
        allPoints.push_back(p1);
        allPoints.push_back(p2);
        
        allPoints = GenerateRandomPoints(PROCEDURAL_POINTS - 2, minX, maxX, minY, maxY, allPoints);

        auto tour_feo = SolveTSPNearestNeighbor(allPoints);
        auto tour_bonito = Optimize2Opt(tour_feo);
        
        float sum = 0.0f;
        int n = tour_bonito.size();
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                Vector2 p1 = tour_bonito[i];
                Vector2 p2 = tour_bonito[(i + 1) % n];
                sum += (p2.x - p1.x) * (p2.y + p1.y);
            }
            // Mantenemos una convención consistente de giro antihorario
            // para que los algoritmos de generación de bordes no colapsen o inviertan caras.
            if (sum < 0) {
                std::reverse(tour_bonito.begin(), tour_bonito.end());
            }
        }
        
        return tour_bonito;
    }

    /**
     * @brief Serializa y guarda la estructura central de la pista en disco.
     * 
     * @param centerPoints Vector de nodos a guardar.
     * @param filename Ruta y nombre del archivo resultante.
     */
    inline void SaveTrackToFile(const std::vector<Vector2>& centerPoints, const std::string& filename) {
        std::ofstream file(filename);
        file << "{ \"puntos_centrales\": [\n";
        for (size_t i = 0; i < centerPoints.size(); i++) {
            file << "    {\"x\": " << centerPoints[i].x << ", \"y\": " << centerPoints[i].y << "}";
            if (i < centerPoints.size() - 1) file << ",";
            file << "\n";
        }
        file << "  ]\n}\n";
        file.close();
    }
}
