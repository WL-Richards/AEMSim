#pragma once
#include <chrono/core/ChTypes.h>
#include <chrono/core/ChVector3.h>

namespace chrono
{
    class ChBody;
    class ChSystem;
}

class WorldHelper
{
public:

    static std::shared_ptr<chrono::ChBody> MakeInfiteishFloor(const std::shared_ptr<chrono::ChSystem>& sys,
                          double z = 0.0,
                          double half_x = 100.0,   // 1000 m wide
                          double half_y = 100.0,
                          double thickness = 0.2);

    static std::shared_ptr<chrono::ChBody> CreateHub(const std::shared_ptr<chrono::ChSystem>& sys, const chrono::ChVector3d& hubLocation);

    static std::shared_ptr<chrono::ChBody> CreateTestCube(
        const std::shared_ptr<chrono::ChSystem>& sys,
        const chrono::ChVector3d& hubLocation
    );

    static std::shared_ptr<chrono::ChBody> CreateTriangleMesh(
        const std::shared_ptr<chrono::ChSystem>& sys,
        const std::string& filePath,
        const chrono::ChVector3d& position
    );
    
};
