#ifndef BRAIN_H
#define BRAIN_H

#include <random>

const int NODOS_OCULTOS = 8;
const int NODOS_ENTRADA = 6;
const int NODOS_SALIDA = 2;

struct Brain {
    float peso_entrada_oculta[NODOS_OCULTOS][NODOS_ENTRADA];
    float sesgos_oculta[NODOS_OCULTOS];
    float peso_oculta_salida[NODOS_SALIDA][NODOS_OCULTOS];
    float sesgos_salida[NODOS_SALIDA];
    
    Brain() {
        static std::random_device rd; 
        static std::mt19937 generador(rd()); 
        std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

        for(int i = 0; i < NODOS_OCULTOS; i++) {
            sesgos_oculta[i] = dist(generador);
            for(int j = 0; j < NODOS_ENTRADA; j++) {
                peso_entrada_oculta[i][j] = dist(generador);
            }
        }
        for(int i = 0; i < NODOS_SALIDA; i++) {
            sesgos_salida[i] = dist(generador);
            for(int j = 0; j < NODOS_OCULTOS; j++) {
                peso_oculta_salida[i][j] = dist(generador);
            }
        }
    }

    // Función de feedforward
    void Evaluate(float sensorDistances[5], float velocidad, float &outAcelerar, float &outGiro) {
        float entrada[6] = {sensorDistances[0], sensorDistances[1], sensorDistances[2], sensorDistances[3], sensorDistances[4], velocidad};
        
        float valores_ocultos[NODOS_OCULTOS];
        for(int i=0; i<NODOS_OCULTOS; i++) {
            valores_ocultos[i] = 0.0f;
            for(int j=0; j<NODOS_ENTRADA; j++) {
                valores_ocultos[i] += entrada[j] * peso_entrada_oculta[i][j];
            }
            valores_ocultos[i] += sesgos_oculta[i];
            valores_ocultos[i] = tanh(valores_ocultos[i]);
        }

        float valores_salida[NODOS_SALIDA];
        for(int i=0; i<NODOS_SALIDA; i++) {
            valores_salida[i] = 0.0f;
            for(int j=0; j<NODOS_OCULTOS; j++) {
                valores_salida[i] += valores_ocultos[j] * peso_oculta_salida[i][j];
            }
            valores_salida[i] += sesgos_salida[i];
            valores_salida[i] = tanh(valores_salida[i]);
        }

        outAcelerar = valores_salida[0];
        outGiro = valores_salida[1]; 
    }
    
    float MutateGaussian() {
        static std::random_device rd; 
        static std::mt19937 generador(rd()); 
        std::normal_distribution<float> distribucion(0.0f, 0.1f);
        return distribucion(generador);
    }
};

#endif // BRAIN_H