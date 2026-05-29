#ifndef BRAIN_H
#define BRAIN_H

#include <random>
#include <algorithm>

const int HIDDEN_NODES = 8;
const int INPUT_NODES = 6;
const int OUTPUT_NODES = 2;

/**
 * @brief Structure representing the brain (Neural Network) of a car.
 * 
 * Implements a simple feedforward neural network with one hidden layer.
 * Used to evaluate sensor readings and decide acceleration and turn.
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
     * @brief Evaluates sensor inputs via feedforward to obtain control outputs.
     * 
     * @param sensorDistances Array of sensor distances.
     * @param speed The current speed of the car.
     * @param outAccelerate Reference where the calculated acceleration value will be stored.
     * @param outTurn Reference where the calculated turn value will be stored.
     */
    void Evaluate(float sensorDistances[5], float speed, float &outAccelerate, float &outTurn) {
        float input[6];
        for(int i=0; i<5; i++){
            input[i] = sensorDistances[i] / 400.0f; // 400 = Config::CAR_MAX_SENSOR_DIST
        }
        input[5] = std::clamp(speed / 20.0f, -1.0f, 1.0f); // Normalize speed (max approx 20)

        for(int i=0; i<INPUT_NODES; i++) last_input[i] = input[i];
        
        float hidden_values[HIDDEN_NODES];
        for(int i=0; i<HIDDEN_NODES; i++) {
            hidden_values[i] = 0.0f;
            for(int j=0; j<INPUT_NODES; j++) {
                hidden_values[i] += input[j] * weights_input_hidden[i][j];
            }
            hidden_values[i] += biases_hidden[i];
            
            // Use hyperbolic tangent to normalize values between -1 and 1
            // since the car requires negative ranges (e.g. reverse or turning left)
            hidden_values[i] = tanh(hidden_values[i]);
            last_hidden[i] = hidden_values[i];
        }

        float output_values[OUTPUT_NODES];
        for(int i=0; i<OUTPUT_NODES; i++) {
            output_values[i] = 0.0f;
            for(int j=0; j<HIDDEN_NODES; j++) {
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
     * @brief Generates a Gaussian mutation value.
     * 
     * @return float A mutated value based on a normal distribution (mean 0, standard deviation 0.1).
     */
    float MutateGaussian() {
        thread_local static std::random_device rd; 
        thread_local static std::mt19937 generator(rd()); 
        std::normal_distribution<float> distribution(0.0f, 0.1f);
        return distribution(generator);
    }
};

#endif // BRAIN_H