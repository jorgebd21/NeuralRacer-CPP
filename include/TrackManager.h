#pragma once

#include "raylib.h"
#include <vector>
#include <string>
#include <utility>
#include <cstdint>
#include <unordered_map>

/**
 * @brief Espacio de nombres para utilidades de generación, procesamiento y carga de circuitos.
 * 
 * Contiene funciones matemáticas para interpolación (Catmull-Rom), detección de colisiones, 
 * y lectura/escritura de archivos de mapa.
 */
namespace TrackManager {
    const int GRID_CELL_SIZE = 100;

    inline uint64_t GetGridKey(int x, int y) {
        return ((uint64_t)(uint32_t)x << 32) | (uint32_t)y;
    }

    /**
     * @brief Calcula la distancia a la intersección de dos segmentos de línea.
     * 
     * @param p1 Inicio del primer segmento.
     * @param p2 Fin del primer segmento.
     * @param p3 Inicio del segundo segmento.
     * @param p4 Fin del segundo segmento.
     * @param outDist Distancia resultante si existe intersección.
     * @return true Si existe intersección.
     * @return false Si no hay intersección.
     */
    bool GetLineIntersectionDist(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, float &outDist);

    /**
     * @brief Interpola un punto utilizando una curva spline Catmull-Rom.
     * 
     * @param p0 Punto de control anterior.
     * @param p1 Punto de inicio del segmento.
     * @param p2 Punto de fin del segmento.
     * @param p3 Punto de control siguiente.
     * @param t Valor de interpolación [0.0, 1.0].
     * @return Vector2 El punto interpolado en la curva.
     */
    Vector2 GetCatmullRomPoint(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3, float t);

    /**
     * @brief Genera puntos intermedios a lo largo de una lista de puntos utilizando splines.
     * 
     * @param points Nodos base del circuito.
     * @param segmentsPerCurve Resolución de la curva (cuántos puntos generar por segmento base).
     * @return std::vector<Vector2> El recorrido suavizado.
     */
    std::vector<Vector2> GenerateSplinePoints(const std::vector<Vector2>& points, int segmentsPerCurve = 15);

    /**
     * @brief Aplica un filtro de suavizado a una serie de puntos para redondear las curvas.
     * 
     * @param pts Vector de puntos a suavizar (se modifica in-place).
     * @param radius Alcance del suavizado hacia adelante y hacia atrás.
     * @param passes Número de iteraciones del algoritmo de suavizado.
     */
    void SmoothPoints(std::vector<Vector2>& pts, int radius, int passes);

    /**
     * @brief Comprueba si dos segmentos de línea se cortan y obtiene el punto exacto.
     * 
     * @param p1 Inicio del primer segmento.
     * @param p2 Fin del primer segmento.
     * @param p3 Inicio del segundo segmento.
     * @param p4 Fin del segundo segmento.
     * @param outIntersection Coordenada exacta de la intersección.
     * @return true Si los segmentos se cruzan.
     * @return false Si no se cruzan.
     */
    bool GetSegmentIntersection(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, Vector2& outIntersection);

    /**
     * @brief Detecta y elimina bucles donde el circuito se cruza consigo mismo.
     * 
     * @param pts La lista de puntos que define el circuito.
     */
    void RemoveSelfIntersections(std::vector<Vector2>& pts);

    /**
     * @brief Genera las paredes exteriores e interiores del circuito a partir de su línea central.
     * 
     * @param centerPoints Puntos que definen la ruta óptima del circuito.
     * @param trackWidth Ancho total de la pista.
     * @param outWalls Vector de segmentos donde se almacenarán las paredes calculadas.
     */
    void GenerateBordersFromCenterLine(const std::vector<Vector2>& centerPoints, float trackWidth, std::vector<std::pair<Vector2, Vector2>>& outWalls, std::vector<std::pair<Vector2, Vector2>>& outCheckpoints);

    /**
     * @brief Determina la posición y rotación inicial idóneas para los coches.
     * 
     * @param puntosProcedurales Los nodos centrales del circuito.
     * @param startPosition Parámetro de salida con la posición inicial.
     * @param startRotation Parámetro de salida con la rotación inicial en grados.
     */
    void CalculateStartGrid(const std::vector<Vector2>& puntosProcedurales, Vector2& startPosition, float& startRotation);

    /**
     * @brief Carga un circuito completo (paredes y punto de inicio) desde un archivo de texto.
     * 
     * @param filename Ruta al archivo del circuito.
     * @param outWalls Vector donde se añadirán los bordes del mapa.
     * @param outStartPos Posición de salida leída del mapa.
     * @param outStartRot Rotación de salida calculada u obtenida del mapa.
     */
    void LoadTrackFromFile(const std::string& filename, std::vector<std::pair<Vector2, Vector2>>& outWalls, Vector2& outStartPos, float& outStartRot, std::vector<std::pair<Vector2, Vector2>>& outCheckpoints);

    /**
     * @brief Explora el directorio de mapas y devuelve una lista de los nombres disponibles.
     * 
     * @return std::vector<std::string> Nombres de los archivos encontrados en la carpeta de mapas.
     */
    std::vector<std::string> ScanMapFiles();
}
