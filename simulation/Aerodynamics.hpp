#pragma once

#include <memory>
#include <chrono/physics/ChBody.h>
#include "parameters/AerodynamicParametersBase.hpp"

namespace Aerodynamics {

/**
 * Applies aerodynamic forces (drag and Magnus) to a body based on its velocity
 * and angular velocity relative to the wind.
 *
 * @param body The body to apply forces to
 * @param acc_idx The force accumulator index
 * @param p Aerodynamic parameters (drag coefficient, air density, etc.)
 * @param enableDrag Whether to apply drag force
 * @param enableMagnus Whether to apply Magnus force
 */
template <class AeroT>
void ApplyToBody(
    const std::shared_ptr<chrono::ChBody>& body,
    const unsigned int acc_idx,
    const std::shared_ptr<AerodynamicParametersBase<AeroT>> p,
    const bool enableDrag = true,
    const bool enableMagnus = true)
{
    const chrono::ChVector3d v_W = body->GetLinVel();        // world linear velocity
    const chrono::ChVector3d w_W = body->GetAngVelParent();  // world angular velocity
    const chrono::ChVector3d v_rel = v_W - p->WindVeloctiy;

    const double speed = v_rel.Length();
    if (speed < 1e-9)
        return;

    // Drag: Fd = -0.5 * rho * Cd * A * |v| * v
    const chrono::ChVector3d F_drag =
        (-0.5 * p->AirDensity * p->DragCoefficient * p->getArea() * speed) * v_rel;

    // Magnus: Fm = 0.5 * rho * A * Cm * (omega x v)
    const chrono::ChVector3d F_magnus =
        (0.5 * p->AirDensity * p->getArea() * p->MagnusCoefficient) * (w_W.Cross(v_rel));

    // Apply at COM in world coordinates
    const auto com_W = body->GetPos();
    body->AccumulateForce(acc_idx, enableDrag ? F_drag : chrono::ChVector3d(), com_W, /*local=*/false);
    body->AccumulateForce(acc_idx, enableMagnus ? F_magnus : chrono::ChVector3d(), com_W, /*local=*/false);
}

} // namespace Aerodynamics
