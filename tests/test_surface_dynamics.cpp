#include <cassert>
#include <iostream>
#include "SurfaceDynamics.h"
#include "Config.h"

int main() {
    std::cout << "[TEST] Starting SurfaceDynamics verification test suite..." << std::endl;

    // Verify friction coefficients match roadmap specifications
    assert(SurfaceDynamics::GetFrictionCoefficient(SurfaceType::Asphalt) == Config::FRICTION_ASPHALT);
    assert(SurfaceDynamics::GetFrictionCoefficient(SurfaceType::ApexCurbs) == Config::FRICTION_APEX_CURBS);
    assert(SurfaceDynamics::GetFrictionCoefficient(SurfaceType::RunoffTurf) == Config::FRICTION_RUNOFF_TURF);
    assert(SurfaceDynamics::GetFrictionCoefficient(SurfaceType::GravelTrap) == Config::FRICTION_GRAVEL_TRAP);

    assert(SurfaceDynamics::GetFrictionCoefficient(SurfaceType::Asphalt) == 1.00f);
    assert(SurfaceDynamics::GetFrictionCoefficient(SurfaceType::ApexCurbs) == 0.85f);
    assert(SurfaceDynamics::GetFrictionCoefficient(SurfaceType::RunoffTurf) == 0.65f);
    assert(SurfaceDynamics::GetFrictionCoefficient(SurfaceType::GravelTrap) == 0.25f);

    // Verify rolling resistance increases significantly off-track
    assert(SurfaceDynamics::GetRollingResistanceFactor(SurfaceType::Asphalt) == 1.0f);
    assert(SurfaceDynamics::GetRollingResistanceFactor(SurfaceType::GravelTrap) >= 8.0f);

    // Verify curb impulse
    assert(SurfaceDynamics::GetCurbOscillationImpulse(SurfaceType::ApexCurbs) > 0.0f);
    assert(SurfaceDynamics::GetCurbOscillationImpulse(SurfaceType::Asphalt) == 0.0f);

    std::cout << "[TEST] SurfaceDynamics: ALL ASSERTIONS PASSED SUCCESSFULLY!" << std::endl;
    return 0;
}
