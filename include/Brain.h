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

/**
 * @brief Vectorized logistic sigmoid function sigma(x) = 0.5 * (tanh(0.5 * x) + 1.0) across 8 float lanes.
 */
inline __m256 FastSigmoidAVX2(__m256 x) noexcept {
    const __m256 half = _mm256_set1_ps(0.5f);
    __m256 half_x = _mm256_mul_ps(half, x);
    __m256 th = FastTanhAVX2(half_x);
    return _mm256_fmadd_ps(half, th, half);
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

/**
 * @brief High-performance rational approximation of the logistic sigmoid function sigma(x).
 * 
 * Maps real-valued activation to (0, 1) gate probabilities:
 *   sigma(x) = 0.5 * (tanh(0.5 * x) + 1.0)
 */
inline float FastSigmoid(float x) noexcept {
    return 0.5f * (FastTanh(0.5f * x) + 1.0f);
}

const int HIDDEN_NODES = 10;
const int INPUT_NODES = 12;
const int OUTPUT_NODES = 2;
const int RECURRENT_NODES = 8;

/**
 * @brief Structure representing the brain (Neural Network) of a car.
 * 
 * Supports both feedforward 12-10-2 evaluation and 8-cell Minimal Gated Unit (MGU)
 * temporal recurrent working memory for multi-agent anticipation and curve sequencing.
 */
struct Brain {
    alignas(32) float weights_input_hidden[HIDDEN_NODES][INPUT_NODES];
    alignas(32) float biases_hidden[HIDDEN_NODES];
    alignas(32) float weights_hidden_output[OUTPUT_NODES][HIDDEN_NODES];
    alignas(32) float biases_output[OUTPUT_NODES];
    
    // Minimal Gated Unit (MGU / GRU) Recurrent Topology Matrices
    alignas(32) float W_f[RECURRENT_NODES][INPUT_NODES];
    alignas(32) float U_f[RECURRENT_NODES][RECURRENT_NODES];
    alignas(32) float b_f[RECURRENT_NODES];

    alignas(32) float W_h[RECURRENT_NODES][INPUT_NODES];
    alignas(32) float U_h[RECURRENT_NODES][RECURRENT_NODES];
    alignas(32) float b_h[RECURRENT_NODES];

    alignas(32) float W_out[OUTPUT_NODES][RECURRENT_NODES];
    alignas(32) float b_out[OUTPUT_NODES];

    // Temporal recurrent hidden state vector h_t
    alignas(32) float h_state[RECURRENT_NODES];
    alignas(32) float last_f_gate[RECURRENT_NODES];
    bool useRecurrent;

    // Store the last activations to draw them in the graphical interface
    alignas(32) float last_input[INPUT_NODES];
    alignas(32) float last_hidden[HIDDEN_NODES];
    alignas(32) float last_output[OUTPUT_NODES];
    
    /**
     * @brief Default constructor that initializes weights and biases randomly.
     * 
     * Uses a random number generator to assign initial values between -1.0 and 1.0.
     */
    Brain() : useRecurrent(false) {
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

        // Initialize MGU Recurrent Cell Parameters
        for (int i = 0; i < RECURRENT_NODES; ++i) {
            b_f[i] = dist(generator);
            b_h[i] = dist(generator);
            for (int j = 0; j < INPUT_NODES; ++j) {
                W_f[i][j] = dist(generator);
                W_h[i][j] = dist(generator);
            }
            for (int j = 0; j < RECURRENT_NODES; ++j) {
                U_f[i][j] = dist(generator);
                U_h[i][j] = dist(generator);
            }
        }
        for (int o = 0; o < OUTPUT_NODES; ++o) {
            b_out[o] = dist(generator);
            for (int j = 0; j < RECURRENT_NODES; ++j) {
                W_out[o][j] = dist(generator);
            }
        }

        ResetState();
        for(int i = 0; i < INPUT_NODES; i++) last_input[i] = 0.0f;
        for(int i = 0; i < HIDDEN_NODES; i++) last_hidden[i] = 0.0f;
        for(int i = 0; i < OUTPUT_NODES; i++) last_output[i] = 0.0f;
    }

    /**
     * @brief Resets recurrent hidden state vector h_t to baseline zeroes.
     */
    void ResetState() noexcept {
        for (int i = 0; i < RECURRENT_NODES; ++i) {
            h_state[i] = 0.0f;
            last_f_gate[i] = 0.0f;
        }
    }

    /**
     * @brief Steps the Minimal Gated Unit (MGU) recurrent cell forward one discrete tick.
     * 
     * Formulations:
     *   f_t = sigma(W_f x_t + U_f h_{t-1} + b_f)
     *   h_tilde = tanh(W_h x_t + U_h (f_t (*) h_{t-1}) + b_h)
     *   h_t = (1 - f_t) (*) h_{t-1} + f_t (*) h_tilde
     *   y_t = tanh(W_out h_t + b_out)
     */
    void StepMGU(const float x[INPUT_NODES], float &outAccelerate, float &outTurn) noexcept {
#if defined(__AVX2__) && defined(__FMA__)
        __m256 h_prev = _mm256_load_ps(h_state);

        alignas(32) float sum_f[RECURRENT_NODES];
        alignas(32) float sum_h[RECURRENT_NODES];

        for (int i = 0; i < RECURRENT_NODES; ++i) {
            float sf = b_f[i];
            float sh = b_h[i];
            for (int j = 0; j < INPUT_NODES; ++j) {
                sf += W_f[i][j] * x[j];
                sh += W_h[i][j] * x[j];
            }
            sum_f[i] = sf;
            sum_h[i] = sh;
        }

        alignas(32) float gate_f_in[RECURRENT_NODES];
        for (int i = 0; i < RECURRENT_NODES; ++i) {
            __m256 u_row = _mm256_load_ps(U_f[i]);
            __m256 prod = _mm256_mul_ps(u_row, h_prev);
            gate_f_in[i] = sum_f[i] + HorizontalSum256(prod);
        }

        __m256 f_gate_reg = FastSigmoidAVX2(_mm256_load_ps(gate_f_in));
        _mm256_store_ps(last_f_gate, f_gate_reg);

        __m256 gated_h = _mm256_mul_ps(f_gate_reg, h_prev);

        alignas(32) float cand_h_in[RECURRENT_NODES];
        for (int i = 0; i < RECURRENT_NODES; ++i) {
            __m256 uh_row = _mm256_load_ps(U_h[i]);
            __m256 prod = _mm256_mul_ps(uh_row, gated_h);
            cand_h_in[i] = sum_h[i] + HorizontalSum256(prod);
        }

        __m256 h_cand_reg = FastTanhAVX2(_mm256_load_ps(cand_h_in));

        const __m256 one = _mm256_set1_ps(1.0f);
        __m256 one_minus_f = _mm256_sub_ps(one, f_gate_reg);
        __m256 h_next = _mm256_fmadd_ps(f_gate_reg, h_cand_reg, _mm256_mul_ps(one_minus_f, h_prev));
        _mm256_store_ps(h_state, h_next);

        __m256 w_out0 = _mm256_load_ps(W_out[0]);
        __m256 w_out1 = _mm256_load_ps(W_out[1]);
        float dot0 = HorizontalSum256(_mm256_mul_ps(w_out0, h_next)) + b_out[0];
        float dot1 = HorizontalSum256(_mm256_mul_ps(w_out1, h_next)) + b_out[1];

        outAccelerate = FastTanh(dot0);
        outTurn = FastTanh(dot1);
        last_output[0] = outAccelerate;
        last_output[1] = outTurn;
#else
        float f_gate[RECURRENT_NODES];
        float h_cand[RECURRENT_NODES];

        // 1. Forget / Update gate
        for (int i = 0; i < RECURRENT_NODES; ++i) {
            float sum = b_f[i];
            for (int j = 0; j < INPUT_NODES; ++j) {
                sum += W_f[i][j] * x[j];
            }
            for (int j = 0; j < RECURRENT_NODES; ++j) {
                sum += U_f[i][j] * h_state[j];
            }
            f_gate[i] = FastSigmoid(sum);
            last_f_gate[i] = f_gate[i];
        }

        // 2. Candidate state with gated memory
        for (int i = 0; i < RECURRENT_NODES; ++i) {
            float sum = b_h[i];
            for (int j = 0; j < INPUT_NODES; ++j) {
                sum += W_h[i][j] * x[j];
            }
            for (int j = 0; j < RECURRENT_NODES; ++j) {
                sum += U_h[i][j] * (f_gate[j] * h_state[j]);
            }
            h_cand[i] = FastTanh(sum);
        }

        // 3. Convex linear interpolation of temporal hidden state
        for (int i = 0; i < RECURRENT_NODES; ++i) {
            h_state[i] = (1.0f - f_gate[i]) * h_state[i] + f_gate[i] * h_cand[i];
        }

        // 4. Output projection to control actions
        float outA = b_out[0];
        float outT = b_out[1];
        for (int j = 0; j < RECURRENT_NODES; ++j) {
            outA += W_out[0][j] * h_state[j];
            outT += W_out[1][j] * h_state[j];
        }
        outAccelerate = FastTanh(outA);
        outTurn = FastTanh(outT);
        last_output[0] = outAccelerate;
        last_output[1] = outTurn;
#endif
    }

    /**
     * @brief Evaluates normalized inputs via feedforward to obtain control outputs.
     * 
     * @param sensorDistances Array of 5 sensor distances.
     * @param frontSensorDelta Temporal delta of front sensor (Time-to-Collision).
     * @param vLong Longitudinal velocity.
     * @param vLat Lateral velocity (slip).
     * @param angularVelocity Angular velocity (yaw rate).
     * @param competitorClosingVel Array of 5 competitor relative closing velocities.
     * @param outAccelerate Reference where the calculated acceleration value will be stored.
     * @param outTurn Reference where the calculated turn value will be stored.
     */
    void Evaluate(const float sensorDistances[5], float frontSensorDelta, float vLong, float vLat, float angularVelocity, const float competitorClosingVel[5], float &outAccelerate, float &outTurn) {
        float input[INPUT_NODES];
        for(int i = 0; i < 5; i++) {
            float maxDist = (i == 2) ? Config::CAR_FRONT_SENSOR_DIST : Config::CAR_LATERAL_SENSOR_MAX;
            input[i] = std::clamp(sensorDistances[i] / maxDist, 0.0f, 1.0f);
        }
        // Dynamic front sensor delta (rate of wall closure) normalized to [-1.0, 1.0] across typical 8-tick delta window
        input[5] = std::clamp(frontSensorDelta / 25.0f, -1.0f, 1.0f);
        input[6] = std::clamp(vLong / 4.0f, -1.0f, 1.0f);
        input[7] = std::clamp(vLat / 1.5f, -1.0f, 1.0f);
        input[8] = std::clamp(angularVelocity / Config::CAR_TURN_SPEED, -1.0f, 1.0f);
        // Multi-Agent competitor relative closing velocities
        input[9] = std::clamp(competitorClosingVel[2] / 10.0f, -1.0f, 1.0f);  // Front closing velocity
        input[10] = std::clamp(competitorClosingVel[1] / 10.0f, -1.0f, 1.0f); // Left-front closing velocity
        input[11] = std::clamp(competitorClosingVel[3] / 10.0f, -1.0f, 1.0f); // Right-front closing velocity

        for(int i = 0; i < INPUT_NODES; i++) last_input[i] = input[i];
        
        if (useRecurrent) {
            StepMGU(input, outAccelerate, outTurn);
            return;
        }

        float hidden_values[HIDDEN_NODES];
#if defined(__AVX2__) && defined(__FMA__)
        // Vectorized SIMD AVX2 + FMA Matrix-Vector Forward Pass (Input 12 -> Hidden 10)
        __m256 in0_7 = _mm256_loadu_ps(&input[0]);

        for(int i = 0; i < HIDDEN_NODES; i++) {
            __m256 w0_7 = _mm256_loadu_ps(&weights_input_hidden[i][0]);
            __m256 prod = _mm256_mul_ps(in0_7, w0_7);
            float dot = HorizontalSum256(prod);
            float tail = (input[8] * weights_input_hidden[i][8]) + 
                         (input[9] * weights_input_hidden[i][9]) + 
                         (input[10] * weights_input_hidden[i][10]) + 
                         (input[11] * weights_input_hidden[i][11]) + biases_hidden[i];
            hidden_values[i] = FastTanh(dot + tail);
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
     * @brief Backward-compatible overload without explicit competitor closing velocities.
     */
    void Evaluate(const float sensorDistances[5], float frontSensorDelta, float vLong, float vLat, float angularVelocity, float &outAccelerate, float &outTurn) {
        float defaultCompVel[5] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        Evaluate(sensorDistances, frontSensorDelta, vLong, vLat, angularVelocity, defaultCompVel, outAccelerate, outTurn);
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