#pragma once
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "raylib.h"

namespace TrackGenerator {

    // 1. Genera 'count' puntos aleatorios dentro de los limites pasados.
    inline std::vector<Vector2> GenerateRandomPoints(int count, int minX, int maxX, int minY, int maxY) {
        std::vector<Vector2> points;

        for(int i = 0; i < count; i++){
            Vector2 punto;
            punto.x = GetRandomValue(minX, maxX);
            punto.y = GetRandomValue(minY, maxY);
            points.push_back(punto);
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

    // 2. Aplica la Marcha de Jarvis para sacar la Envolvente Convexa
    inline std::vector<Vector2> CalculateConvexHull(const std::vector<Vector2>& points) {
        std::vector<Vector2> hull;

        if(points.size() < 4){
            return points;
        }

        Vector2 min = points[0];
        for(int i = 1; i < points.size(); i++){
            if(points[i].x < min.x){
                min = points[i];
            }
        }

        Vector2 ancla = min;
        hull.push_back(ancla);

        Vector2 punto_actual = ancla;
        Vector2 siguiente;

        do {
            siguiente = points[0]; 

            for(int i = 0; i < points.size(); i++) {
                if (siguiente.x == punto_actual.x && siguiente.y == punto_actual.y) {
                    siguiente = points[i];
                }

                if (CrossProduct(punto_actual, siguiente, points[i]) < 0) {
                    siguiente = points[i];
                }
            }
            
            punto_actual = siguiente;
    
            if (punto_actual.x != ancla.x || punto_actual.y != ancla.y) {
                hull.push_back(punto_actual);
            }
        } while (punto_actual.x != ancla.x || punto_actual.y != ancla.y);
        
        return hull;
    }

    // 3. Empuja los puntos medios hacia adentro para hacer pistas reales
    inline std::vector<Vector2> AddDisplacement(const std::vector<Vector2>& hullPoints, float factor) {
        std::vector<Vector2> displaced;
        
        Vector2 centro;
        centro.x = 0.0f;
        centro.y = 0.0f;

        for(int i = 0; i < hullPoints.size(); i++){
            centro.x += hullPoints[i].x;
            centro.y += hullPoints[i].y;
        }
        centro.x /= hullPoints.size();
        centro.y /= hullPoints.size();

        for(int i = 0; i < hullPoints.size(); i++){
            Vector2 p1 = hullPoints[i];
            Vector2 p2 = hullPoints[(i + 1) % hullPoints.size()];
            
            Vector2 p_medio;
            p_medio.x = (p1.x + p2.x) / 2.0f;
            p_medio.y = (p1.y + p2.y) / 2.0f;
            
            float factor_displacement = GetRandomValue(0, static_cast<int>(factor)) / 100.0f;
            Vector2 direccion;
            direccion.x = (centro.x - p_medio.x) * factor_displacement;
            direccion.y = (centro.y - p_medio.y) * factor_displacement;
            
            Vector2 punto_desplazado;
            punto_desplazado.x = p_medio.x + direccion.x;
            punto_desplazado.y = p_medio.y + direccion.y;
            
            displaced.push_back(p1);
            displaced.push_back(punto_desplazado);
        }

        return displaced;
    }

    // 4. Une todo el pipeline procedural
    inline std::vector<Vector2> GenerateProceduralCenterPoints() {
        // Reducimos maxX a 800 para compensar el "overshoot" de la curva Spline y el ancho de la pista (65px)
        auto randomPoints = GenerateRandomPoints(50, 150, 800, 150, 600);
        auto hull = CalculateConvexHull(randomPoints);
        auto displaced = AddDisplacement(hull, 25.0f);

        return displaced;
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
