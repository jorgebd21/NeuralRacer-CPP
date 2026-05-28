#pragma once
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "raylib.h"

/**
 * @brief Procedural algorithms for random track generation.
 * 
 * This namespace implements the complete pipeline for map creation
 * using random filtered point generation, solving the TSP (Traveling Salesperson Problem) 
 * via nearest neighbor, and optimization with the 2-Opt algorithm to prevent self-intersections.
 */
namespace TrackGenerator {
    constexpr int MAX_ATTEMPTS = 2000;
    constexpr float MIN_DIST_SQ = 22500.0f; // Ensures points are not too close (150^2)
    constexpr int PROCEDURAL_POINTS = 25;
    constexpr int BOUNDS_MIN_X = 150;
    constexpr int BOUNDS_MAX_X = 800;
    constexpr int BOUNDS_MIN_Y = 150;
    constexpr int BOUNDS_MAX_Y = 600;

    /**
     * @brief Generates a set of random points respecting a minimum distance between them.
     * 
     * @param count Desired number of points to generate.
     * @param minX Left boundary of the area.
     * @param maxX Right boundary of the area.
     * @param minY Top boundary of the area.
     * @param maxY Bottom boundary of the area.
     * @return std::vector<Vector2> List of valid generated points.
     */
    inline std::vector<Vector2> GenerateRandomPoints(int count, int minX, int maxX, int minY, int maxY, const std::vector<Vector2>& existingPoints = {}) {
        std::vector<Vector2> points = existingPoints;
        int maxAttempts = MAX_ATTEMPTS; 
        
        for(int i = 0; i < count; i++){
            Vector2 point;
            bool isValid = false;
            int attempts = 0;
            
            while(!isValid && attempts < maxAttempts){
                point.x = GetRandomValue(minX, maxX);
                point.y = GetRandomValue(minY, maxY);
                isValid = true;
                
                for(size_t j = 0; j < points.size(); j++){
                    float sqDist = (point.x - points[j].x)*(point.x - points[j].x) + (point.y - points[j].y)*(point.y - points[j].y);
                    
                    // We use squared distance to avoid the computational cost
                    // of calculating the square root with sqrt() repeatedly.
                    if(sqDist < MIN_DIST_SQ){
                        isValid = false;
                        break;
                    }
                }
                attempts++;
            }
            
            if (isValid) {
                points.push_back(point);
            }
        }
        
        return points;
    }

    /**
     * @brief Calculates the 2D cross product of three points.
     * 
     * @param p0 Origin point.
     * @param p1 First vector.
     * @param p2 Second vector.
     * @return float Negative value if p2 is to the right of p0->p1, positive if it is to the left.
     */
    inline float CrossProduct(Vector2 p0, Vector2 p1, Vector2 p2) {
        return (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
    }

    /**
     * @brief Calculates the squared Euclidean distance between two points.
     * 
     * @param a First point.
     * @param b Second point.
     * @return float Squared distance (avoids sqrt for performance).
     */
    inline float Distance(Vector2 a, Vector2 b) {
        return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
    }

    /**
     * @brief Heuristically solves the Traveling Salesperson Problem (TSP) using Nearest Neighbor.
     * 
     * @param points Set of points to sort.
     * @return std::vector<Vector2> The approximate initial path.
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
            Vector2 nextPoint;
            int minDist = 0;
            int posPoint = -1;
            for(int j = 0; j < n; j++){
                if(!visited[j]){
                    int dist = Distance(tour[i-1], points[j]);
                    if(posPoint == -1 || dist < minDist){
                        minDist = dist;
                        posPoint = j;
                        nextPoint = points[j];
                    }
                }
            }
            tour.push_back(nextPoint);
            visited[posPoint] = true;
        }

        return tour;
    }

    /**
     * @brief Optimizes a TSP path using the 2-Opt algorithm.
     * 
     * Undoes line crossings by swapping edges, which is vital
     * so that the resulting track does not have self-collisions.
     * 
     * @param tour Current path to optimize.
     * @return std::vector<Vector2> Optimized path without obvious self-intersections.
     */
    inline std::vector<Vector2> Optimize2Opt(std::vector<Vector2> tour) {
        int n = tour.size();

        bool repeat = true;
        while(repeat){
            repeat = false;

            for(int i = 1; i < n-2; i++){
                for(int j = i+2; j < n; j++){
                    Vector2 A = tour[i];
                    Vector2 B = tour[i+1];
                    Vector2 C = tour[j];
                    Vector2 D = tour[(j+1)%n];
                    
                    float oldDistance = Distance(A, B) + Distance(C, D);
                    float newDistance = Distance(A, C) + Distance(B, D);
                    
                    // If swapping nodes reduces total distance,
                    // we are undoing a path crossing.
                    if(newDistance < oldDistance){
                        repeat = true;
                        std::reverse(tour.begin() + i + 1, tour.begin() + j + 1);
                        break;
                    }
                }

                if(repeat) break;
            }
        }

        return tour;
    }


    /**
     * @brief Executes the complete pipeline and generates the ordered nodes of the track.
     * 
     * Includes a Winding Order check based on the Gauss area
     * to ensure the track is always generated counter-clockwise.
     * 
     * @return std::vector<Vector2> Final list of procedural center nodes.
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

        auto rawTour = SolveTSPNearestNeighbor(allPoints);
        auto optimizedTour = Optimize2Opt(rawTour);
        
        float sum = 0.0f;
        int n = optimizedTour.size();
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                Vector2 p1 = optimizedTour[i];
                Vector2 p2 = optimizedTour[(i + 1) % n];
                sum += (p2.x - p1.x) * (p2.y + p1.y);
            }
            // Maintain a consistent counter-clockwise turn convention
            // so edge generation algorithms do not collapse or flip faces.
            if (sum < 0) {
                std::reverse(optimizedTour.begin(), optimizedTour.end());
            }
        }
        
        return optimizedTour;
    }

    /**
     * @brief Serializes and saves the center structure of the track to disk.
     * 
     * @param centerPoints Vector of nodes to save.
     * @param filename Path and name of the resulting file.
     */
    inline void SaveTrackToFile(const std::vector<Vector2>& centerPoints, const std::string& filename) {
        std::ofstream file(filename);
        file << "{ \"center_points\": [\n";
        for (size_t i = 0; i < centerPoints.size(); i++) {
            file << "    {\"x\": " << centerPoints[i].x << ", \"y\": " << centerPoints[i].y << "}";
            if (i < centerPoints.size() - 1) file << ",";
            file << "\n";
        }
        file << "  ]\n}\n";
        file.close();
    }
}
