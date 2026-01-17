#include "PhysicsHelper.h"

#include <chrono/core/ChVector3.h>
#include <chrono/physics/ChBody.h>

void PhysicsHelper::FreezeBody(const std::shared_ptr<chrono::ChBody>& b)
{
    if (!b) return;
    b->SetFixed(true);
    b->SetLinVel(chrono::VNULL);
    b->SetAngVelParent(chrono::VNULL);
}

bool PhysicsHelper::HitGroundSphere(const std::shared_ptr<chrono::ChBody>& ball, double radius, double ground_z)
{
    return ball && (ball->GetPos().z() <= ground_z + radius);
}

bool PhysicsHelper::WithinBox(const std::shared_ptr<chrono::ChBody>& ball, double radius, const chrono::ChVector3d& center, const chrono::ChVector3d& extent)
{
    if (!ball) {
        return false;
    }

    const auto p = ball->GetPos() - center;

    const double ex = extent.x() - radius;
    const double ey = extent.y() - radius;
    const double ez = extent.z() - radius;

    if (ex < 0 || ey < 0 || ez < 0) {
        return false;
    }

    return (std::abs(p.x()) <= ex) &&
           (std::abs(p.y()) <= ey) &&
           (std::abs(p.z()) <= ez);
}




