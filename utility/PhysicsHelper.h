#pragma once

#include <memory>
#include <chrono/physics/ChBody.h>

namespace chrono {
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

    static bool HitGroundSphere(const std::shared_ptr<chrono::ChBody>& ball, double radius, double ground_z = 0.0);

    static bool WithinBox(const std::shared_ptr<chrono::ChBody>& ball,
                          double radius,
                          const chrono::ChVector3d& center,
                          const chrono::ChVector3d& extent);
};
