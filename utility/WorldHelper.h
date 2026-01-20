#pragma once
#include <cstddef>
#include <vector>
#include <chrono/assets/ChColor.h>
#include <chrono/core/ChQuaternion.h>
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

    static std::shared_ptr<chrono::ChBody> CreateHub(
        const std::shared_ptr<chrono::ChSystem>& sys,
        const chrono::ChVector3d& hubLocation,
        const chrono::ChQuaternion<double>& rotation = chrono::QUNIT,
        std::vector<std::shared_ptr<chrono::ChBody>>* funnel_bodies_out = nullptr);

    static std::shared_ptr<chrono::ChBody> CreateTestCube(
        const std::shared_ptr<chrono::ChSystem>& sys,
        const chrono::ChVector3d& hubLocation
    );

    static std::shared_ptr<chrono::ChBody> CreateTriangleMesh(
        const std::shared_ptr<chrono::ChSystem>& sys,
        const std::string& filePath,
        const chrono::ChVector3d& position,
        const chrono::ChQuaternion<double>& rotation = chrono::QUNIT
    );

    static std::shared_ptr<chrono::ChBody> CreateTriangleMeshApproxBoxes(
        const std::shared_ptr<chrono::ChSystem>& sys,
        const std::string& filePath,
        const chrono::ChVector3d& position,
        const chrono::ChQuaternion<double>& rotation = chrono::QUNIT,
        double cell_size = 0.25,
        bool add_visual = false,
        size_t max_cells = 200000
    );

    static std::shared_ptr<chrono::ChBody> CreateTrapezoidalPrism(
        const std::shared_ptr<chrono::ChSystem>& sys,
        const chrono::ChVector3d& position,
        const chrono::ChQuaternion<double>& rotation,
        double bottom_width,
        double bottom_depth,
        double top_width,
        double top_depth,
        double height,
        double density = 1000.0,
        bool fixed = true,
        const chrono::ChColor& color = chrono::ChColor(0.7f, 0.7f, 0.7f)
    );

    static std::vector<std::shared_ptr<chrono::ChBody>> CreateTrapezoidalPrismRing(
        const std::shared_ptr<chrono::ChSystem>& sys,
        const chrono::ChVector3d& center,
        double bottom_x,
        double bottom_y,
        double top_x,
        double top_y,
        double height,
        double tilt_x_radians,
        const chrono::ChQuaternion<double>& rotation = chrono::QUNIT,
        double ring_radius_scale = 1.0,
        double density = 1000.0,
        bool fixed = true,
        const chrono::ChColor& color = chrono::ChColor(0.7f, 0.7f, 0.7f)
    );

};
