#ifndef BRAIN_H
#define BRAIN_H

#include <random>
#include <algorithm>
#include "Config.h"

const int HIDDEN_NODES = 10;
const int INPUT_NODES = 9;
const int OUTPUT_NODES = 2;

/**
 * @brief Structure representing the brain (Neural Network) of a car.
 * 
 * Implements a simple feedforward neural network with one hidden layer (9-10-2).
 * Used to evaluate sensor readings, temporal delta, velocities and decide acceleration and turn.
 */
struct Brain {
    float weights_input_hidden[HIDDEN_NODES][INPUT_NODES];
    float biases_hidden[HIDDEN_NODES];
    float weights_hidden_output[OUTPUT_NODES][HIDDEN_NODES];
    float biases_output[OUTPUT_NODES];
    
    // Store the last activations to draw them in the graphical interface
    float last_input[INPUT_NODES];
    float last_hidden[HIDDEN_NODES];
    float last_output[OUTPUT_NODES];
    
    /**
     * @brief Default constructor that initializes weights and biases randomly.
     * 
     * Uses a random number generator to assign initial values between -1.0 and 1.0.
     */
    Brain() {
        thread_local static std::random_device rd; 
        thread_local static std::mt19937 generator(rd()); 
        std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

        for(int i = 0; i < HIDDEN_NODES; i++) {
            biases_hidden[i] = dist(generator);
            for(int j = 0; j < INPUT_NODES; j++) {
                weights_input_hidden[i][j] = dist(generator);
            }
        }
        for(int i = 0; i < OUTPUT_NODES; i++) {
            biases_output[i] = dist(generator);
            for(int j = 0; j < HIDDEN_NODES; j++) {
                weights_hidden_output[i][j] = dist(generator);
            }
        }
        for(int i = 0; i < INPUT_NODES; i++) last_input[i] = 0.0f;
        for(int i = 0; i < HIDDEN_NODES; i++) last_hidden[i] = 0.0f;
        for(int i = 0; i < OUTPUT_NODES; i++) last_output[i] = 0.0f;
    }

    /**
     * @brief Evaluates normalized inputs via feedforward to obtain control outputs.
     * 
     * @param sensorDistances Array of 5 sensor distances.
     * @param frontSensorDelta Temporal delta of front sensor (Time-to-Collision).
     * @param vLong Longitudinal velocity.
     * @param vLat Lateral velocity (slip).
     * @param angularVelocity Angular velocity (yaw rate).
     * @param outAccelerate Reference where the calculated acceleration value will be stored.
     * @param outTurn Reference where the calculated turn value will be stored.
     */
    void Evaluate(const float sensorDistances[5], float frontSensorDelta, float vLong, float vLat, float angularVelocity, float &outAccelerate, float &outTurn) {
        float input[INPUT_NODES];
        for(int i = 0; i < 5; i++) {
            float maxDist = (i == 2) ? Config::CAR_FRONT_SENSOR_DIST : Config::CAR_MAX_SENSOR_DIST;
            input[i] = std::clamp(sensorDistances[i] / maxDist, 0.0f, 1.0f);
        }
        input[5] = std::clamp(frontSensorDelta / 200.0f, -1.0f, 1.0f);
        input[6] = std::clamp(vLong / 20.0f, -1.0f, 1.0f);
        input[7] = std::clamp(vLat / 10.0f, -1.0f, 1.0f);
        input[8] = std::clamp(angularVelocity / Config::CAR_TURN_SPEED, -1.0f, 1.0f);

        for(int i = 0; i < INPUT_NODES; i++) last_input[i] = input[i];
        
        float hidden_values[HIDDEN_NODES];
        for(int i = 0; i < HIDDEN_NODES; i++) {
            hidden_values[i] = 0.0f;
            for(int j = 0; j < INPUT_NODES; j++) {
                hidden_values[i] += input[j] * weights_input_hidden[i][j];
            }
            hidden_values[i] += biases_hidden[i];
            
            // Use hyperbolic tangent to normalize values between -1 and 1
            hidden_values[i] = tanh(hidden_values[i]);
            last_hidden[i] = hidden_values[i];
        }

        float output_values[OUTPUT_NODES];
        for(int i = 0; i < OUTPUT_NODES; i++) {
            output_values[i] = 0.0f;
            for(int j = 0; j < HIDDEN_NODES; j++) {
                output_values[i] += hidden_values[j] * weights_hidden_output[i][j];
            }
            output_values[i] += biases_output[i];
            output_values[i] = tanh(output_values[i]);
            last_output[i] = output_values[i];
        }

        outAccelerate = output_values[0];
        outTurn = output_values[1]; 
    }

    /**
     * @brief Backward-compatible overload for evaluation with raw speed only.
     */
    void Evaluate(const float sensorDistances[5], float speed, float &outAccelerate, float &outTurn) {
        Evaluate(sensorDistances, 0.0f, speed, 0.0f, 0.0f, outAccelerate, outTurn);
    }
    
    /**
     * @brief Computes analytical Jacobian matrix J_ki = d(Output_k) / d(Input_i) via backprop chain rule.
     */
    void ComputeJacobian(float jacobian[OUTPUT_NODES][INPUT_NODES]) const {
        for (int k = 0; k < OUTPUT_NODES; k++) {
            float dyk_duk = 1.0f - last_output[k] * last_output[k];
            for (int i = 0; i < INPUT_NODES; i++) {
                float sum = 0.0f;
                for (int j = 0; j < HIDDEN_NODES; j++) {
                    float dhj_dzj = 1.0f - last_hidden[j] * last_hidden[j];
                    sum += weights_hidden_output[k][j] * dhj_dzj * weights_input_hidden[j][i];
                }
                jacobian[k][i] = dyk_duk * sum;
            }
        }
    }

    /**
     * @brief Computes proportion of hidden neurons saturated near boundaries (|h_j| > 0.90).
     */
    float ComputeHiddenSaturation() const {
        int satCount = 0;
        for (int j = 0; j < HIDDEN_NODES; j++) {
            if (std::abs(last_hidden[j]) > 0.90f) satCount++;
        }
        return (float)satCount / (float)HIDDEN_NODES;
    }

    /**
     * @brief Generates a Gaussian mutation value.
     * 
     * @param sigma Standard deviation of the normal distribution (mutation step size).
     * @return float A mutated value based on a normal distribution (mean 0, standard deviation sigma).
     */
    float MutateGaussian(float sigma = 0.1f) {
        thread_local static std::random_device rd; 
        thread_local static std::mt19937 generator(rd()); 
        std::normal_distribution<float> distribution(0.0f, sigma);
        return distribution(generator);
    }
};

#endif // BRAIN_H