#include "WorldHelper.h"

#include <array>
#include <limits>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <vector>

#include <chrono/assets/ChVisualShapeBox.h>
#include <chrono/collision/ChCollisionShapeBox.h>
#include <chrono/collision/ChCollisionShapeTriangleMesh.h>
#include <chrono/geometry/ChTriangleMeshConnected.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChContactMaterialNSC.h>
#include <chrono/physics/ChSystem.h>

#include "AMath.h"

std::shared_ptr<chrono::ChBody> WorldHelper::MakeInfiteishFloor(const std::shared_ptr<chrono::ChSystem>& sys, double z,
                                                                double half_x, double half_y, double thickness)
{
    auto mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    mat->SetFriction(1.0f);
    mat->SetRestitution(0.08f);

    auto floor = chrono_types::make_shared<chrono::ChBodyEasyBox>(
        half_x * 2,
        half_y * 2,
        thickness,
        10000,
        true,
        true,
        mat
    );
    floor->SetFixed(true);
    floor->SetPos(chrono::ChVector3d(0, 0, z - thickness/2)); // top surface at z

    floor->EnableCollision(true);
    sys->AddBody(floor);
    return floor;
}

std::shared_ptr<chrono::ChBody> WorldHelper::CreateHub(
    const std::shared_ptr<chrono::ChSystem>& sys,
    const chrono::ChVector3d& hubLocation,
    const chrono::ChQuaternion<double>& rotation,
    std::vector<std::shared_ptr<chrono::ChBody>>* funnel_bodies_out)
{
    
    auto hub = chrono_types::make_shared<chrono::ChBody>();
    hub->SetPos(hubLocation);
    hub->SetRot(rotation);
    hub->SetFixed(true);

    // Contact material (NSC since you're using ChContactMaterialNSC)
    auto mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();\
    hub->EnableCollision(true);

    // HUB CENTER
    auto hubCenterVis = chrono_types::make_shared<chrono::ChVisualShapeBox>(
        0.1, 0.1, 0.1
    );
    hubCenterVis->SetColor(chrono::ChColor(0.f, 1.f, 0.f));

    hub->AddVisualShape(
        hubCenterVis,
        chrono::ChFrame<>(chrono::ChVector3d(0, 0, INCHES_TO_METERS(59.64)), chrono::QUNIT)
    );
    
    // --- HUB FRONT FACE ---
    const double front_length = INCHES_TO_METERS(47);
    const double side_length = INCHES_TO_METERS(46.52);
    const double wall_height = INCHES_TO_METERS(59.64) - INCHES_TO_METERS(9.27);
    const double wall_thickness = 0.1;
    const double front_center_y = -INCHES_TO_METERS(23.373);
    const double front_outer_y = front_center_y - (wall_thickness * 0.5);
    const double back_center_y = front_outer_y + side_length - (wall_thickness * 0.5);
    const double side_center_y = front_outer_y + (side_length * 0.5);
    const double side_span_y = side_length - (wall_thickness * 2.0);
    const double side_center_x = (front_length * 0.5) - (wall_thickness * 0.5);
    const double wall_z = wall_height / 2.0;

    auto hubFrontFaceCollision = chrono_types::make_shared<chrono::ChCollisionShapeBox>(
        mat,
        front_length, wall_thickness, wall_height   // box sizes (X,Y,Z) to match the visual below
    );
    
    hub->AddCollisionShape(
       hubFrontFaceCollision,
       chrono::ChFrame<>(
            chrono::ChVector3d(0, front_center_y, wall_z),
           chrono::QUNIT)
    );
    
    auto hubFrontFaceVis = chrono_types::make_shared<chrono::ChVisualShapeBox>(
        front_length, wall_thickness, wall_height
    );
    hubFrontFaceVis->SetColor(chrono::ChColor(0.f, 0.f, 1.f));

    hub->AddVisualShape(
        hubFrontFaceVis,
        chrono::ChFrame<>(
            chrono::ChVector3d(0, front_center_y, wall_z),
            chrono::QUNIT)
    );

    const double front_trap_thickness = INCHES_TO_METERS(0.125);
    const double front_trap_rise = INCHES_TO_METERS(9.27);
    const double front_trap_run = INCHES_TO_METERS(5.237);
    const double front_trap_bottom_x = INCHES_TO_METERS(46.369200);
    const double front_trap_top_x = INCHES_TO_METERS(36.088974);
    const double front_trap_angle_x = std::asin(-front_trap_run / front_trap_rise);
    const double front_inner_y = front_center_y + (wall_thickness);
    const double trap_half_t = front_trap_thickness * 0.5;
    const double trap_half_h = front_trap_rise * 0.5;
    const double c = std::cos(front_trap_angle_x);
    const double s = std::sin(front_trap_angle_x);

    const double min_y_offset = std::min(
        std::min((-trap_half_t) * c - (-trap_half_h) * s, (-trap_half_t) * c - ( trap_half_h) * s),
        std::min(( trap_half_t) * c - (-trap_half_h) * s, ( trap_half_t) * c - ( trap_half_h) * s)
    );
    const double max_y_offset = std::max(
        std::max((-trap_half_t) * c - (-trap_half_h) * s, (-trap_half_t) * c - ( trap_half_h) * s),
        std::max(( trap_half_t) * c - (-trap_half_h) * s, ( trap_half_t) * c - ( trap_half_h) * s)
    );
    const double min_z_offset = std::min(
        std::min((-trap_half_t) * s + (-trap_half_h) * c, (-trap_half_t) * s + ( trap_half_h) * c),
        std::min(( trap_half_t) * s + (-trap_half_h) * c, ( trap_half_t) * s + ( trap_half_h) * c)
    );

    const double front_trap_center_y = front_inner_y - max_y_offset;
    const double front_trap_center_z = wall_height - min_z_offset;

    WorldHelper::CreateTrapezoidalPrism(
        sys,
        hubLocation + rotation.Rotate(chrono::ChVector3d(0, front_trap_center_y, front_trap_center_z)),
        rotation * chrono::QuatFromAngleX(front_trap_angle_x),
        front_trap_bottom_x, front_trap_thickness,
        front_trap_top_x, front_trap_thickness,
        front_trap_rise,
        /*density=*/1000.0,
        /*fixed=*/true,
        chrono::ChColor(0.f, 0.f, 1.f)
    );

    auto hubBackFaceCollision = chrono_types::make_shared<chrono::ChCollisionShapeBox>(
        mat,
        front_length, wall_thickness, wall_height
    );
    hub->AddCollisionShape(
        hubBackFaceCollision,
        chrono::ChFrame<>(
            chrono::ChVector3d(0, back_center_y, wall_z),
            chrono::QUNIT)
    );

    auto hubBackFaceVis = chrono_types::make_shared<chrono::ChVisualShapeBox>(
        front_length, wall_thickness, wall_height
    );
    hubBackFaceVis->SetColor(chrono::ChColor(0.f, 0.f, 1.f));
    hub->AddVisualShape(
        hubBackFaceVis,
        chrono::ChFrame<>(
            chrono::ChVector3d(0, back_center_y, wall_z),
            chrono::QUNIT)
    );

    auto hubLeftWallCollision = chrono_types::make_shared<chrono::ChCollisionShapeBox>(
        mat,
        wall_thickness, side_span_y, wall_height
    );
    hub->AddCollisionShape(
        hubLeftWallCollision,
        chrono::ChFrame<>(
            chrono::ChVector3d(-side_center_x, side_center_y, wall_z),
            chrono::QUNIT)
    );

    auto hubLeftWallVis = chrono_types::make_shared<chrono::ChVisualShapeBox>(
        wall_thickness, side_span_y, wall_height
    );
    hubLeftWallVis->SetColor(chrono::ChColor(0.f, 0.f, 1.f));
    hub->AddVisualShape(
        hubLeftWallVis,
        chrono::ChFrame<>(
            chrono::ChVector3d(-side_center_x, side_center_y, wall_z),
            chrono::QUNIT)
    );

    auto hubRightWallCollision = chrono_types::make_shared<chrono::ChCollisionShapeBox>(
        mat,
        wall_thickness, side_span_y, wall_height
    );
    hub->AddCollisionShape(
        hubRightWallCollision,
        chrono::ChFrame<>(
            chrono::ChVector3d(side_center_x, side_center_y, wall_z),
            chrono::QUNIT)
    );

    auto hubRightWallVis = chrono_types::make_shared<chrono::ChVisualShapeBox>(
        wall_thickness, side_span_y, wall_height
    );
    hubRightWallVis->SetColor(chrono::ChColor(0.f, 0.f, 1.f));
    hub->AddVisualShape(
        hubRightWallVis,
        chrono::ChFrame<>(
            chrono::ChVector3d(side_center_x, side_center_y, wall_z),
            chrono::QUNIT)
    );
    
    auto funnel_bodies = WorldHelper::CreateTrapezoidalPrismRing(
        sys,
        hubLocation + rotation.Rotate(chrono::ChVector3d(0, 0, (INCHES_TO_METERS(59.64)+(INCHES_TO_METERS(17.90))/2)-INCHES_TO_METERS(0.963))),
        INCHES_TO_METERS(13.75),
        INCHES_TO_METERS(0.12),
        INCHES_TO_METERS(24.09),
        INCHES_TO_METERS(0.12),
        INCHES_TO_METERS(17.90),
        0.54,
        rotation,
        0.955
    );
    if (funnel_bodies_out) {
        *funnel_bodies_out = std::move(funnel_bodies);
    }

    sys->AddBody(hub);
    return hub;
}


std::shared_ptr<chrono::ChBody> WorldHelper::CreateTestCube(
    const std::shared_ptr<chrono::ChSystem>& sys,
    const chrono::ChVector3d& hubLocation)
{
    
    auto testCube = chrono_types::make_shared<chrono::ChBody>();
    testCube->SetPos(hubLocation);
    testCube->SetFixed(true);

    
    // --- Collision: actual box shape WITH dimensions ---

    // HUB CENTER
    auto testCubeVis = chrono_types::make_shared<chrono::ChVisualShapeBox>(
        0.1, 0.1, 0.1
    );
    testCubeVis->SetColor(chrono::ChColor(0.f, 1.f, 0.f));

    testCube->AddVisualShape(
        testCubeVis,
        chrono::ChFrame<>(chrono::ChVector3d(0, 0, 0), chrono::QUNIT)
    );
    
    sys->AddBody(testCube);
    return testCube;
}

std::shared_ptr<chrono::ChBody> WorldHelper::CreateTriangleMesh(
        const std::shared_ptr<chrono::ChSystem>& sys,
        const std::string& filePath,
        const chrono::ChVector3d& position,
        const chrono::ChQuaternion<double>& rotation
    )
{
    auto mesh = chrono_types::make_shared<chrono::ChTriangleMeshConnected>();
    mesh->LoadWavefrontMesh(filePath, /*load_normals=*/true, /*load_uv=*/true);

    mesh->RepairDuplicateVertexes(1e-9);

    auto body = chrono_types::make_shared<chrono::ChBody>();
    body->SetFixed(true);
    body->EnableCollision(true);
    body->SetPos(position);
    body->SetRot(rotation);

    auto mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    mat->SetFriction(0.8f);
    mat->SetRestitution(0.05f);

    auto col_shape =
        chrono_types::make_shared<chrono::ChCollisionShapeTriangleMesh>(
            mat,
            mesh,
            /*is_static=*/true,
            /*is_convex=*/false
        );

    body->AddCollisionShape(col_shape);
    
    auto vis = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
    vis->SetMesh(mesh);
    body->AddVisualShape(vis);

    sys->AddBody(body);
    return body;
}

std::shared_ptr<chrono::ChBody> WorldHelper::CreateTriangleMeshApproxBoxes(
        const std::shared_ptr<chrono::ChSystem>& sys,
        const std::string& filePath,
        const chrono::ChVector3d& position,
        const chrono::ChQuaternion<double>& rotation,
        double cell_size,
        bool add_visual,
        size_t max_cells
    )
{
    if (!sys) return nullptr;
    if (cell_size <= 0.0) return nullptr;
    if (max_cells == 0) return nullptr;

    auto mesh = chrono_types::make_shared<chrono::ChTriangleMeshConnected>();
    mesh->LoadWavefrontMesh(filePath, /*load_normals=*/true, /*load_uv=*/true);
    mesh->RepairDuplicateVertexes(1e-9);

    if (mesh->m_vertices.empty() || mesh->GetNumTriangles() <= 0) return nullptr;

    chrono::ChVector3d min_v = mesh->m_vertices[0];
    chrono::ChVector3d max_v = mesh->m_vertices[0];
    for (const auto& v : mesh->m_vertices) {
        min_v.x() = std::min(min_v.x(), v.x());
        min_v.y() = std::min(min_v.y(), v.y());
        min_v.z() = std::min(min_v.z(), v.z());
        max_v.x() = std::max(max_v.x(), v.x());
        max_v.y() = std::max(max_v.y(), v.y());
        max_v.z() = std::max(max_v.z(), v.z());
    }

    double cell = cell_size;
    size_t nx = 1;
    size_t ny = 1;
    size_t nz = 1;

    auto update_dims = [&]() {
        nx = (size_t)std::max(1.0, std::ceil((max_v.x() - min_v.x()) / cell));
        ny = (size_t)std::max(1.0, std::ceil((max_v.y() - min_v.y()) / cell));
        nz = (size_t)std::max(1.0, std::ceil((max_v.z() - min_v.z()) / cell));
    };

    update_dims();
    while (nx * ny * nz > max_cells) {
        cell *= 1.25;
        update_dims();
    }

    const size_t total_cells = nx * ny * nz;
    std::vector<uint8_t> occupied(total_cells, 0);

    auto clamp_index = [](long value, size_t max_val) {
        if (value < 0) return size_t(0);
        if ((size_t)value >= max_val) return max_val - 1;
        return (size_t)value;
    };

    for (unsigned int i = 0; i < mesh->GetNumTriangles(); i++)
    {
        auto tri = mesh->GetTriangle(i);
        
        const chrono::ChVector3d& v0 = mesh->m_vertices[0];
        const chrono::ChVector3d& v1 = mesh->m_vertices[1];
        const chrono::ChVector3d& v2 = mesh->m_vertices[2];

        chrono::ChVector3d tri_min(
            std::min(v0.x(), std::min(v1.x(), v2.x())),
            std::min(v0.y(), std::min(v1.y(), v2.y())),
            std::min(v0.z(), std::min(v1.z(), v2.z()))
        );
        chrono::ChVector3d tri_max(
            std::max(v0.x(), std::max(v1.x(), v2.x())),
            std::max(v0.y(), std::max(v1.y(), v2.y())),
            std::max(v0.z(), std::max(v1.z(), v2.z()))
        );

        const long ix0 = (long)std::floor((tri_min.x() - min_v.x()) / cell);
        const long iy0 = (long)std::floor((tri_min.y() - min_v.y()) / cell);
        const long iz0 = (long)std::floor((tri_min.z() - min_v.z()) / cell);
        const long ix1 = (long)std::floor((tri_max.x() - min_v.x()) / cell);
        const long iy1 = (long)std::floor((tri_max.y() - min_v.y()) / cell);
        const long iz1 = (long)std::floor((tri_max.z() - min_v.z()) / cell);

        const size_t cx0 = clamp_index(ix0, nx);
        const size_t cy0 = clamp_index(iy0, ny);
        const size_t cz0 = clamp_index(iz0, nz);
        const size_t cx1 = clamp_index(ix1, nx);
        const size_t cy1 = clamp_index(iy1, ny);
        const size_t cz1 = clamp_index(iz1, nz);

        for (size_t ix = cx0; ix <= cx1; ++ix) {
            for (size_t iy = cy0; iy <= cy1; ++iy) {
                for (size_t iz = cz0; iz <= cz1; ++iz) {
                    const size_t idx = (ix * ny * nz) + (iy * nz) + iz;
                    occupied[idx] = 1;
                }
            }
        }
    }
    

    auto body = chrono_types::make_shared<chrono::ChBody>();
    body->SetFixed(true);
    body->EnableCollision(true);
    body->SetPos(position);

    auto mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    mat->SetFriction(0.8f);
    mat->SetRestitution(0.05f);

    for (size_t ix = 0; ix < nx; ++ix) {
        for (size_t iy = 0; iy < ny; ++iy) {
            for (size_t iz = 0; iz < nz; ++iz) {
                const size_t idx = (ix * ny * nz) + (iy * nz) + iz;
                if (!occupied[idx]) continue;

                const chrono::ChVector3d center(
                    min_v.x() + (ix + 0.5) * cell,
                    min_v.y() + (iy + 0.5) * cell,
                    min_v.z() + (iz + 0.5) * cell
                );

                auto col_shape = chrono_types::make_shared<chrono::ChCollisionShapeBox>(
                    mat,
                    cell,
                    cell,
                    cell
                );
                body->AddCollisionShape(col_shape, chrono::ChFrame<>(center, chrono::QUNIT));

                if (add_visual) {
                    auto vis = chrono_types::make_shared<chrono::ChVisualShapeBox>(cell, cell, cell);
                    body->AddVisualShape(vis, chrono::ChFrame<>(center, chrono::QUNIT));
                }
            }
        }
    }

    sys->AddBody(body);
    return body;
}
std::shared_ptr<chrono::ChBody> WorldHelper::CreateTrapezoidalPrism(
        const std::shared_ptr<chrono::ChSystem>& sys,
        const chrono::ChVector3d& position,
        const chrono::ChQuaternion<double>& rotation,
        double bottom_width,
        double bottom_depth,
        double top_width,
        double top_depth,
        double height,
        double density,
        bool fixed,
        const chrono::ChColor& color
    )
{
    if (!sys) return nullptr;
    if (bottom_width <= 0.0 || bottom_depth <= 0.0 || top_width <= 0.0 || top_depth <= 0.0 || height <= 0.0) return nullptr;

    const double hbz = height * 0.5;
    const double hx0 = bottom_width * 0.5;
    const double hx1 = top_width * 0.5;
    const double depth = 0.5 * (bottom_depth + top_depth);
    const double hy = depth * 0.5;

    std::vector<chrono::ChVector3d> points = {
        chrono::ChVector3d(-hx0, -hy, -hbz),
        chrono::ChVector3d( hx0, -hy, -hbz),
        chrono::ChVector3d( hx0,  hy, -hbz),
        chrono::ChVector3d(-hx0,  hy, -hbz),
        chrono::ChVector3d(-hx1, -hy,  hbz),
        chrono::ChVector3d( hx1, -hy,  hbz),
        chrono::ChVector3d( hx1,  hy,  hbz),
        chrono::ChVector3d(-hx1,  hy,  hbz)
    };

    auto mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    mat->SetFriction(0.8f);
    mat->SetRestitution(0.05f);

    auto body = chrono_types::make_shared<chrono::ChBodyEasyConvexHull>(
        points,
        density,
        /*create_visualization=*/true,
        /*create_collision=*/true,
        mat
    );

    body->SetPos(position);
    body->SetRot(rotation);
    body->SetFixed(fixed);
    body->EnableCollision(true);

    if (auto vm = body->GetVisualModel()) {
        for (size_t i = 0; i < vm->GetNumShapes(); ++i) {
            vm->GetShape(i)->SetColor(color);
        }
    }

    sys->AddBody(body);
    return body;
}
std::vector<std::shared_ptr<chrono::ChBody>> WorldHelper::CreateTrapezoidalPrismRing(
        const std::shared_ptr<chrono::ChSystem>& sys,
        const chrono::ChVector3d& center,
        double bottom_x,
        double bottom_y,
        double top_x,
        double top_y,
        double height,
        double tilt_x_radians,
        const chrono::ChQuaternion<double>& rotation,
        double ring_radius_scale,
        double density,
        bool fixed,
        const chrono::ChColor& color
    )
{
    std::vector<std::shared_ptr<chrono::ChBody>> bodies;
    if (!sys) return bodies;
    if (bottom_x <= 0.0 || bottom_y <= 0.0 || top_x <= 0.0 || top_y <= 0.0 || height <= 0.0) return bodies;

    const double hbz = height * 0.5;
    const double hx0 = bottom_x * 0.5;
    const double hx1 = top_x * 0.5;
    const double depth = 0.5 * (bottom_y + top_y);
    const double hy = depth * 0.5;

    const std::array<chrono::ChVector3d, 8> points = {
        chrono::ChVector3d(-hx0, -hy, -hbz),
        chrono::ChVector3d( hx0, -hy, -hbz),
        chrono::ChVector3d( hx0,  hy, -hbz),
        chrono::ChVector3d(-hx0,  hy, -hbz),
        chrono::ChVector3d(-hx1, -hy,  hbz),
        chrono::ChVector3d( hx1, -hy,  hbz),
        chrono::ChVector3d( hx1,  hy,  hbz),
        chrono::ChVector3d(-hx1,  hy,  hbz)
    };

    auto support_xy = [&](const chrono::ChQuaternion<double>& rotation, const chrono::ChVector3d& dir_xy) {
        double max_dot = -std::numeric_limits<double>::infinity();
        for (const auto& p : points) {
            const auto wp = rotation.Rotate(p);
            const double dot = wp.x() * dir_xy.x() + wp.y() * dir_xy.y();
            if (dot > max_dot) max_dot = dot;
        }
        return max_dot;
    };

    const double step = chrono::CH_PI / 3.0;
    const double z_theta_offset = chrono::CH_PI / 2.0;
    const double neighbor_dir_angle = step * 0.5;
    const chrono::ChVector3d dir(std::cos(neighbor_dir_angle), std::sin(neighbor_dir_angle), 0.0);

    const auto base_rot = chrono::QuatFromAngleX(tilt_x_radians);
    const auto rot0 = chrono::QuatFromAngleZ(z_theta_offset) * base_rot;
    const auto rot1 = chrono::QuatFromAngleZ(step + z_theta_offset) * base_rot;

    const double s0 = support_xy(rot0, dir);
    const double s1 = support_xy(rot1, chrono::ChVector3d(-dir.x(), -dir.y(), 0.0));
    const double ring_radius = (s0 + s1) * ring_radius_scale;

    bodies.reserve(6);
    for (int i = 0; i < 6; ++i) {
        const double theta = i * step;
        const auto local_rot = chrono::QuatFromAngleZ(theta + z_theta_offset) * base_rot;
        const auto rot = rotation * local_rot;
        const chrono::ChVector3d local_pos(
            ring_radius * std::cos(theta),
            ring_radius * std::sin(theta),
            0.0
        );
        const chrono::ChVector3d pos = center + rotation.Rotate(local_pos);

        auto body = WorldHelper::CreateTrapezoidalPrism(
            sys,
            pos,
            rot,
            bottom_x, bottom_y,
            top_x, top_y,
            height,
            density,
            fixed,
            color
        );
        if (body) bodies.push_back(body);
    }

    return bodies;
}
