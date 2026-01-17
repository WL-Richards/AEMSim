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



