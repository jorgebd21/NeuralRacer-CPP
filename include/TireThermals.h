#pragma once

#include <cmath>
#include <algorithm>

/**
 * @brief Lumped-parameter thermodynamic state for vehicle tires.
 * 
 * Simulates tread surface and carcass bulk thermal evolution based on
 * mechanical work, frictional slip dissipation, rolling deflection hysteresis,
 * and convective cooling.
 * 
 * Differential equation:
 *   dT/dt = (Q_slip + Q_deflection - Q_cooling) / thermal_mass
 */
struct TireThermalState {
    float surfaceTemp = 75.0f;       ///< Tread surface temperature in degrees Celsius
    float bulkTemp = 75.0f;          ///< Carcass core bulk temperature in degrees Celsius
    float thermalGripFactor = 1.0f;  ///< Instantaneous thermodynamic grip degradation scalar [0.0 - 1.0]

    static constexpr float AMBIENT_TEMP = 25.0f;     ///< Ambient track environment temperature (deg C)
    static constexpr float OPTIMAL_TEMP_MIN = 85.0f;  ///< Lower boundary of optimal operating window (deg C)
    static constexpr float OPTIMAL_TEMP_MAX = 100.0f; ///< Upper boundary of optimal operating window (deg C)
    static constexpr float CRITICAL_TEMP = 115.0f;    ///< Thermal blistering threshold (deg C)

    /**
     * @brief Computes thermodynamic grip efficiency factor eta_thermal(T).
     * 
     * Peaks at 1.00 in [85C, 100C], diminishing moderately when cold (<60C)
     * and degrading sharply when blistered (>115C).
     */
    static float ComputeThermalGripFactor(float temp) noexcept {
        if (temp >= OPTIMAL_TEMP_MIN && temp <= OPTIMAL_TEMP_MAX) {
            return 1.00f;
        } else if (temp < OPTIMAL_TEMP_MIN) {
            float delta = OPTIMAL_TEMP_MIN - temp;
            return std::clamp(1.00f - 0.0035f * delta, 0.75f, 1.00f);
        } else {
            float delta = temp - OPTIMAL_TEMP_MAX;
            if (temp <= CRITICAL_TEMP) {
                return std::clamp(1.00f - 0.008f * delta, 0.88f, 1.00f);
            } else {
                float blisterDelta = temp - CRITICAL_TEMP;
                return std::clamp(0.88f - 0.022f * blisterDelta, 0.55f, 0.88f);
            }
        }
    }

    /**
     * @brief Integrates dynamic tire surface and bulk temperature equations.
     * 
     * @param vLong Chassis forward velocity (px/tick)
     * @param vLat Chassis lateral sliding velocity (px/tick)
     * @param slipAngleDeg Calculated chassis slip angle in degrees
     * @param normalLoadRatio Dynamic axle normal load ratio from weight transfer
     * @param dt Simulation tick step (seconds, default 1/60 s)
     */
    void Update(float vLong, float vLat, float slipAngleDeg, float normalLoadRatio, float dt = 1.0f / 60.0f) noexcept {
        float speed = std::abs(vLong);
        float slipSpeed = std::abs(vLat);
        float absSlipAngle = std::abs(slipAngleDeg);

        // 1. Q_slip: Dissipative frictional work from lateral tire sliding
        float qSlip = (slipSpeed * 42.0f + absSlipAngle * slipSpeed * 1.8f) * normalLoadRatio;

        // 2. Q_deflection: Carcass structural hysteresis and cyclic deflection under load
        float qDeflection = (speed * 1.25f) * normalLoadRatio;

        // 3. Convective cooling to ambient air (speed-dependent airflow)
        float coolingAirSpeedFactor = 1.0f + 0.18f * speed;
        float qCoolingSurface = 1.45f * coolingAirSpeedFactor * (surfaceTemp - AMBIENT_TEMP);
        float qCoolingBulk = 0.35f * coolingAirSpeedFactor * (bulkTemp - AMBIENT_TEMP);

        // 4. Conduction heat transfer between tread surface and inner carcass
        float qConduction = 2.2f * (surfaceTemp - bulkTemp);

        constexpr float INV_THERMAL_MASS_SURF = 0.45f;
        constexpr float INV_THERMAL_MASS_BULK = 0.15f;

        float dSurf = (qSlip + 0.4f * qDeflection - qCoolingSurface - qConduction) * INV_THERMAL_MASS_SURF * dt;
        float dBulk = (0.6f * qDeflection + qConduction - qCoolingBulk) * INV_THERMAL_MASS_BULK * dt;

        surfaceTemp = std::clamp(surfaceTemp + dSurf, AMBIENT_TEMP, 160.0f);
        bulkTemp = std::clamp(bulkTemp + dBulk, AMBIENT_TEMP, 140.0f);

        thermalGripFactor = ComputeThermalGripFactor(surfaceTemp);
    }

    /**
     * @brief Resets tire temperature to initial race starting baseline.
     */
    void Reset(float initialTemp = 75.0f) noexcept {
        surfaceTemp = initialTemp;
        bulkTemp = initialTemp;
        thermalGripFactor = ComputeThermalGripFactor(surfaceTemp);
    }
};
