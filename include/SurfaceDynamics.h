#pragma once

#include <cstdint>

/**
 * @brief Categorization of track surface zones with distinct friction and mechanical characteristics.
 */
enum class SurfaceType : uint8_t {
    Asphalt = 0,   ///< Optimal racing line and track tarmac (\mu = 1.00)
    ApexCurbs,     ///< FIA apex rumble strips with induced chassis oscillation (\mu = 0.85)
    RunoffTurf,    ///< Astroturf and painted run-off margins (\mu = 0.65)
    GravelTrap     ///< High-drag deceleration gravel arrestor bed (\mu = 0.25)
};

/**
 * @brief Mechanics and physical parameter lookups for surface dynamics.
 */
namespace SurfaceDynamics {
    /**
     * @brief Returns the instantaneous Coulomb surface adhesion friction coefficient (\mu).
     */
    constexpr float GetFrictionCoefficient(SurfaceType type) noexcept {
        switch (type) {
            case SurfaceType::Asphalt:    return 1.00f;
            case SurfaceType::ApexCurbs:  return 0.85f;
            case SurfaceType::RunoffTurf: return 0.65f;
            case SurfaceType::GravelTrap: return 0.25f;
            default:                      return 1.00f;
        }
    }

    /**
     * @brief Multiplier applied to longitudinal rolling resistance depending on surface roughness.
     */
    constexpr float GetRollingResistanceFactor(SurfaceType type) noexcept {
        switch (type) {
            case SurfaceType::Asphalt:    return 1.0f;
            case SurfaceType::ApexCurbs:  return 1.25f;
            case SurfaceType::RunoffTurf: return 2.0f;
            case SurfaceType::GravelTrap: return 8.0f;
            default:                      return 1.0f;
        }
    }

    /**
     * @brief Returns high-frequency chassis oscillation impulse magnitude (e.g. curb rumble).
     */
    constexpr float GetCurbOscillationImpulse(SurfaceType type) noexcept {
        return (type == SurfaceType::ApexCurbs) ? 1.5f : 0.0f;
    }
}
