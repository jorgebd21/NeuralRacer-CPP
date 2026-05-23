#pragma once
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "raylib.h"

namespace TrackGenerator {

    // 1. Genera 'count' puntos aleatorios dentro de los limites pasados.
    inline std::vector<Vector2> GenerateRandomPoints(int count, int minX, int maxX, int minY, int maxY) {
        std::vector<Vector2> points;
        int max_intentos = 2000; // Evitar bucles infinitos
        
        for(int i = 0; i < count; i++){
            Vector2 punto;
            bool valido = false;
            int intentos = 0;
            
            while(!valido && intentos < max_intentos){
                punto.x = GetRandomValue(minX, maxX);
                punto.y = GetRandomValue(minY, maxY);
                valido = true;
                
                // Comprobar la distancia contra todos los puntos ya creados
                for(int j = 0; j < points.size(); j++){
                    float dist_cuadrada = (punto.x - points[j].x)*(punto.x - points[j].x) + (punto.y - points[j].y)*(punto.y - points[j].y);
                    // Distancia mínima de 150 píxeles (150 * 150 = 22500)
                    if(dist_cuadrada < 22500.0f){
                        valido = false;
                        break;
                    }
                }
                intentos++;
            }
            
            // Si encontró un punto válido, lo añade. Si se quedó sin intentos, 
            // significa que el mapa ya está "lleno" y no caben más puntos.
            if (valido) {
                points.push_back(punto);
            }
        }
        
        return points;
    }

    // Funcion auxiliar matematica.
    // Devuelve el producto cruz de 3 puntos. 
    // Si el resultado es < 0, p2 esta a la DERECHA de la linea imaginaria p0->p1.
    // Si el resultado es > 0, p2 esta a la IZQUIERDA.
    inline float CrossProduct(Vector2 p0, Vector2 p1, Vector2 p2) {
        return (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
    }

    inline float Distance(Vector2 a, Vector2 b) {
        return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
    }

    inline std::vector<Vector2> SolveTSPNearestNeighbor(const std::vector<Vector2>& points) {
        int n = points.size();
        if (n==0){return {};}

        std::vector<Vector2> tour;
        tour.reserve(n);
        int total_cost = 0;

        std::vector<bool> visited(n, false);

        Vector2 current_point = points[0];
        tour.push_back(current_point);
        visited[0]=true;

        for(int i = 1; i < n; i++){
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
            total_cost += min_dist;
            current_point = next_point;
        }

        return tour;
    }

    inline std::vector<Vector2> Optimize2Opt(std::vector<Vector2> tour) {
        int n = tour.size();

        bool repetir = true;
        while(repetir){
            repetir = false;

            for(int i = 0; i < n-2; i++){
                for(int j = i+2; j < n; j++){
                    Vector2 A = tour[i];
                    Vector2 B = tour[i+1];
                    Vector2 C = tour[j];
                    Vector2 D = tour[(j+1)%n];
                    
                    float distance_vieja = Distance(A, B) + Distance(C, D);
                    float distance_nueva = Distance(A, C) + Distance(B, D);
                    
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


    // 4. Une todo el pipeline procedural
    inline std::vector<Vector2> GenerateProceduralCenterPoints() {
        // Reducimos maxX a 800 para compensar el "overshoot" de la curva Spline y el ancho de la pista (65px)
        auto randomPoints = GenerateRandomPoints(25, 150, 800, 150, 600);
        auto tour_feo = SolveTSPNearestNeighbor(randomPoints);
        auto tour_bonito = Optimize2Opt(tour_feo);
        
        // Comprobar sentido de giro (Winding Order) con la formula del area de Gauss (Shoelace)
        float sum = 0.0f;
        int n = tour_bonito.size();
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                Vector2 p1 = tour_bonito[i];
                Vector2 p2 = tour_bonito[(i + 1) % n];
                sum += (p2.x - p1.x) * (p2.y + p1.y);
            }
            // En coordenadas de pantalla (Y hacia abajo), sum < 0 significa sentido Horario.
            // Si es horario, invertimos el vector para que sea Antihorario.
            if (sum < 0) {
                std::reverse(tour_bonito.begin(), tour_bonito.end());
            }
        }
        
        return tour_bonito;
    }

    // 5. Guarda la pista para usarla despues
    inline void SaveTrackToFile(const std::vector<Vector2>& centerPoints, const std::string& filename) {
        std::ofstream file(filename);
        for (const auto& point : centerPoints) {
            file << point.x << " " << point.y << "\n";
        }
        file.close();
    }
}
