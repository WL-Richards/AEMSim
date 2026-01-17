#pragma once
#include <memory>
#include <chrono/physics/ChBody.h>

#include "../simulation/parameters/AerodynamicParametersBase.hpp"

namespace chrono
{
    class ChBody;
}

class PhysicsHelper
{
public:
    /**
     * Freezes a given body in place
     * @param b Body to freeze in place
     */
    static void FreezeBody(const std::shared_ptr<chrono::ChBody>& b);

    template <class AeroT>
    static void ApplyAerodynamicsToArea(
                                            const std::shared_ptr<chrono::ChBody>& body,
                                            const unsigned int acc_idx,
                                            const std::shared_ptr<AerodynamicParametersBase<AeroT>> p,
                                            const bool enableDrag = true,
                                            const bool enableMagnus = true
                                        );

    static bool HitGroundSphere(const std::shared_ptr<chrono::ChBody>& ball, double radius, double ground_z = 0.0);
    
};

template <class AeroT>
void PhysicsHelper::ApplyAerodynamicsToArea(
    const std::shared_ptr<chrono::ChBody>& body,
    const unsigned int acc_idx,
    const std::shared_ptr<AerodynamicParametersBase<AeroT>> p,
    const bool enableDrag,
    const bool enableMagnus)
{
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
        body->AccumulateForce(acc_idx, enableDrag ? F_drag : chrono::ChVector3d(),   chrono::VNULL, /*local=*/false);
        body->AccumulateForce(acc_idx, enableMagnus ? F_magnus : chrono::ChVector3d(), chrono::VNULL, /*local=*/false);
    }
}


