#ifndef BRAIN_H
#define BRAIN_H

#include <random>
#include <algorithm>
#include <cmath>
#include "Config.h"

#if defined(__AVX2__) && defined(__FMA__)
#include <immintrin.h>

/**
 * @brief Computes the horizontal sum of an 8-lane 32-bit single-precision float vector.
 */
inline float HorizontalSum256(__m256 v) noexcept {
    __m128 vlow = _mm256_castps256_ps128(v);
    __m128 vhigh = _mm256_extractf128_ps(v, 1);
    __m128 sum128 = _mm_add_ps(vlow, vhigh);
    __m128 shuf = _mm_movehdup_ps(sum128);
    __m128 sums = _mm_add_ps(sum128, shuf);
    shuf = _mm_movehl_ps(shuf, sums);
    sums = _mm_add_ss(sums, shuf);
    return _mm_cvtss_f32(sums);
}

/**
 * @brief Vectorized Padé [5/4] rational polynomial approximation of tanh(x) across 8 float lanes.
 * 
 * Computes tanh(x) ≈ x * (945 + 105*x^2 + x^4) / (945 + 420*x^2 + 15*x^4) clamped to [-1.0, 1.0].
 * Replaces expensive transcendental function calls with direct hardware FMA pipeline operations.
 */
inline __m256 FastTanhAVX2(__m256 x) noexcept {
    const __m256 c945 = _mm256_set1_ps(945.0f);
    const __m256 c105 = _mm256_set1_ps(105.0f);
    const __m256 c420 = _mm256_set1_ps(420.0f);
    const __m256 c15  = _mm256_set1_ps(15.0f);
    const __m256 one  = _mm256_set1_ps(1.0f);
    const __m256 neg1 = _mm256_set1_ps(-1.0f);

    __m256 x2 = _mm256_mul_ps(x, x);
    __m256 x4 = _mm256_mul_ps(x2, x2);

    __m256 num_poly = _mm256_fmadd_ps(c105, x2, c945);
    num_poly = _mm256_add_ps(num_poly, x4);
    __m256 num = _mm256_mul_ps(x, num_poly);

    __m256 den = _mm256_fmadd_ps(c420, x2, c945);
    den = _mm256_fmadd_ps(c15, x4, den);

    __m256 result = _mm256_div_ps(num, den);
    result = _mm256_min_ps(result, one);
    result = _mm256_max_ps(result, neg1);
    return result;
}
#endif

/**
 * @brief High-precision Padé [5/4] rational polynomial approximation of tanh(x).
 */
inline float FastTanh(float x) noexcept {
    if (x >= 4.5f) return 1.0f;
    if (x <= -4.5f) return -1.0f;
    float x2 = x * x;
    float x4 = x2 * x2;
    float num = x * (945.0f + 105.0f * x2 + x4);
    float den = 945.0f + 420.0f * x2 + 15.0f * x4;
    return std::clamp(num / den, -1.0f, 1.0f);
}

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
#if defined(__AVX2__) && defined(__FMA__)
        // Vectorized SIMD AVX2 + FMA Matrix-Vector Forward Pass (Input 9 -> Hidden 10)
        __m256 in0_7 = _mm256_loadu_ps(&input[0]);
        float in8 = input[8];

        for(int i = 0; i < HIDDEN_NODES; i++) {
            __m256 w0_7 = _mm256_loadu_ps(&weights_input_hidden[i][0]);
            __m256 prod = _mm256_mul_ps(in0_7, w0_7);
            float dot = HorizontalSum256(prod);
            float z = std::fma(in8, weights_input_hidden[i][8], dot + biases_hidden[i]);
            hidden_values[i] = FastTanh(z);
            last_hidden[i] = hidden_values[i];
        }

        // Vectorized SIMD AVX2 + FMA Matrix-Vector Forward Pass (Hidden 10 -> Output 2)
        __m256 h0_7 = _mm256_loadu_ps(&hidden_values[0]);
        float h8 = hidden_values[8];
        float h9 = hidden_values[9];
        float output_values[OUTPUT_NODES];

        for(int i = 0; i < OUTPUT_NODES; i++) {
            __m256 wo0_7 = _mm256_loadu_ps(&weights_hidden_output[i][0]);
            __m256 prod = _mm256_mul_ps(h0_7, wo0_7);
            float dot = HorizontalSum256(prod);
            float tail = (h8 * weights_hidden_output[i][8]) + (h9 * weights_hidden_output[i][9]) + biases_output[i];
            output_values[i] = FastTanh(dot + tail);
            last_output[i] = output_values[i];
        }
#else
        for(int i = 0; i < HIDDEN_NODES; i++) {
            hidden_values[i] = 0.0f;
            for(int j = 0; j < INPUT_NODES; j++) {
                hidden_values[i] += input[j] * weights_input_hidden[i][j];
            }
            hidden_values[i] += biases_hidden[i];
            hidden_values[i] = FastTanh(hidden_values[i]);
            last_hidden[i] = hidden_values[i];
        }

        float output_values[OUTPUT_NODES];
        for(int i = 0; i < OUTPUT_NODES; i++) {
            output_values[i] = 0.0f;
            for(int j = 0; j < HIDDEN_NODES; j++) {
                output_values[i] += hidden_values[j] * weights_hidden_output[i][j];
            }
            output_values[i] += biases_output[i];
            output_values[i] = FastTanh(output_values[i]);
            last_output[i] = output_values[i];
        }
#endif

        outAccelerate = output_values[0];
        outTurn = output_values[1]; 
    }

    /**
     * @brief Performs simultaneous batched inference across 8 agent state vectors using SIMD registers.
     * 
     * @param batchInput Matrix of shape [8][9]: inputs for 8 cars.
     * @param outAccelerate Array of 8 calculated acceleration values.
     * @param outTurn Array of 8 calculated turn values.
     */
    void EvaluateBatch8(const float batchInput[8][INPUT_NODES], float outAccelerate[8], float outTurn[8]) const {
#if defined(__AVX2__) && defined(__FMA__)
        __m256 inFeat[INPUT_NODES];
        for (int f = 0; f < INPUT_NODES; ++f) {
            inFeat[f] = _mm256_set_ps(
                batchInput[7][f], batchInput[6][f], batchInput[5][f], batchInput[4][f],
                batchInput[3][f], batchInput[2][f], batchInput[1][f], batchInput[0][f]
            );
        }

        __m256 hiddenAct[HIDDEN_NODES];
        for (int h = 0; h < HIDDEN_NODES; ++h) {
            __m256 accum = _mm256_set1_ps(biases_hidden[h]);
            for (int f = 0; f < INPUT_NODES; ++f) {
                __m256 weight = _mm256_set1_ps(weights_input_hidden[h][f]);
                accum = _mm256_fmadd_ps(weight, inFeat[f], accum);
            }
            hiddenAct[h] = FastTanhAVX2(accum);
        }

        for (int o = 0; o < OUTPUT_NODES; ++o) {
            __m256 accum = _mm256_set1_ps(biases_output[o]);
            for (int h = 0; h < HIDDEN_NODES; ++h) {
                __m256 weight = _mm256_set1_ps(weights_hidden_output[o][h]);
                accum = _mm256_fmadd_ps(weight, hiddenAct[h], accum);
            }
            __m256 out_act = FastTanhAVX2(accum);
            alignas(32) float temp[8];
            _mm256_store_ps(temp, out_act);
            for (int c = 0; c < 8; ++c) {
                if (o == 0) outAccelerate[c] = temp[c];
                else outTurn[c] = temp[c];
            }
        }
#else
        for (int c = 0; c < 8; ++c) {
            float hidden[HIDDEN_NODES];
            for (int h = 0; h < HIDDEN_NODES; ++h) {
                float sum = biases_hidden[h];
                for (int f = 0; f < INPUT_NODES; ++f) sum += batchInput[c][f] * weights_input_hidden[h][f];
                hidden[h] = std::tanh(sum);
            }
            float out0 = biases_output[0];
            float out1 = biases_output[1];
            for (int h = 0; h < HIDDEN_NODES; ++h) {
                out0 += hidden[h] * weights_hidden_output[0][h];
                out1 += hidden[h] * weights_hidden_output[1][h];
            }
            outAccelerate[c] = std::tanh(out0);
            outTurn[c] = std::tanh(out1);
        }
#endif
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