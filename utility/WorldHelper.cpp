#include "WorldHelper.h"

#include <chrono/assets/ChVisualShapeBox.h>
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
    const chrono::ChVector3d& hubLocation)
{
    
    auto hub = chrono_types::make_shared<chrono::ChBody>();
    hub->SetPos(hubLocation);
    hub->SetFixed(true);

    // Contact material (NSC since you're using ChContactMaterialNSC)
    auto mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

    // --- Collision: actual box shape WITH dimensions ---

    // --- HUB Center ---
    
    // HUB Center Collision
    // NOTE: keep the pose at the origin unless you want an offset.
    // auto hubCenterCollision = chrono_types::make_shared<chrono::ChCollisionShapeBox>(
    //     mat,
    //     0.1, 0.1, 0.1   // box sizes (X,Y,Z) to match the visual below
    // );
    //
    //
    // hub->AddCollisionShape(
    //     hubCenterCollision,
    //     chrono::ChFrame<>(
    //         chrono::ChVector3d(0, 0, INCHES_TO_METERS(59.64)),
    //         chrono::QUNIT)
    // );
    //
    // hub->EnableCollision(false);

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
    // auto hubFrontFaceCollision = chrono_types::make_shared<chrono::ChCollisionShapeBox>(
    //     mat,
    //     0.1, 0.1, 0.1   // box sizes (X,Y,Z) to match the visual below
    // );
    //
    // hub->AddCollisionShape(
    //    hubFrontFaceCollision,
    //    chrono::ChFrame<>(
    //        chrono::ChVector3d(0, 0, INCHES_TO_METERS(59.64)),
    //        chrono::QUNIT)
    // );
    
    auto hubFrontFaceVis = chrono_types::make_shared<chrono::ChVisualShapeBox>(
        0.1, 0.1, (INCHES_TO_METERS(59.64) - INCHES_TO_METERS(9.27))
    );
    hubFrontFaceVis->SetColor(chrono::ChColor(0.f, 0.f, 1.f));

    hub->AddVisualShape(
        hubFrontFaceVis,
        chrono::ChFrame<>(
            chrono::ChVector3d(0, -INCHES_TO_METERS(23.373), (INCHES_TO_METERS(59.64) - INCHES_TO_METERS(9.27))/2),
            chrono::QUNIT)
    );

    // --- Hub Front Face Slant ---
    // auto hubFrontFaceSlantCollision = chrono_types::make_shared<chrono::ChCollisionShapeBox>(
    //     mat,
    //     0.1, 0.1, 0.1   // box sizes (X,Y,Z) to match the visual below
    // );
    //
    // hub->AddCollisionShape(
    //    hubFrontFaceSlantCollision,
    //    chrono::ChFrame<>(
    //        chrono::ChVector3d(0, 0, INCHES_TO_METERS(59.64)),
    //        chrono::QUNIT)
    // );
    
    auto hubFrontFaceSlantVis = chrono_types::make_shared<chrono::ChVisualShapeBox>(
        0.1, INCHES_TO_METERS(10.648), 0.1
    );
    hubFrontFaceSlantVis->SetColor(chrono::ChColor(0.f, 0.f, 1.f));

    hub->AddVisualShape(
        hubFrontFaceSlantVis,
        chrono::ChFrame<>(chrono::ChVector3d(0, -(INCHES_TO_METERS(18.133) + INCHES_TO_METERS(5.24/2)), INCHES_TO_METERS(59.64)-(INCHES_TO_METERS(9.27)/2)),
            chrono::QuatFromAngleAxis(1.0472, chrono::ChVector3d(1,0,0)))
    );

    auto hubFrontTop = chrono_types::make_shared<chrono::ChVisualShapeBox>(
        0.1, INCHES_TO_METERS(18.133)-INCHES_TO_METERS(11.915), 0.1
    );
    hubFrontTop->SetColor(chrono::ChColor(0.f, 0.f, 1.f));

    hub->AddVisualShape(
        hubFrontTop,
        chrono::ChFrame<>(
            chrono::ChVector3d(0,
                -(((INCHES_TO_METERS(18.133)-INCHES_TO_METERS(11.915))/2)+INCHES_TO_METERS(11.915)),
                    INCHES_TO_METERS(59.64)
                    ),
            chrono::QUNIT)
    );

    auto hubBackTop = chrono_types::make_shared<chrono::ChVisualShapeBox>(
        0.1, INCHES_TO_METERS(18.133)-INCHES_TO_METERS(11.915), 0.1
    );
    hubBackTop->SetColor(chrono::ChColor(0.f, 0.f, 1.f));

    hub->AddVisualShape(
        hubBackTop,
        chrono::ChFrame<>(
            chrono::ChVector3d(0,
                (((INCHES_TO_METERS(18.133)-INCHES_TO_METERS(11.915))/2)+INCHES_TO_METERS(11.915)),
                    INCHES_TO_METERS(59.64)
                    ),
            chrono::QUNIT)
    );

    auto hubFrontFunnel = chrono_types::make_shared<chrono::ChVisualShapeBox>(
        0.1, INCHES_TO_METERS(13.632), 0.1
    );
    hubFrontFunnel->SetColor(chrono::ChColor(0.f, 0.f, 1.f));

    hub->AddVisualShape(
        hubFrontFunnel,
        chrono::ChFrame<>(
            chrono::ChVector3d(0,
                -(INCHES_TO_METERS(18.133)-((INCHES_TO_METERS(2.731)/2))),
                    INCHES_TO_METERS(59.64) + (INCHES_TO_METERS(13.356)/2)
                    ),
            chrono::QuatFromAngleAxis(1.78024, chrono::ChVector3d(1,0,0)))
    );

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
        const chrono::ChVector3d& position
    )
{
    auto mesh = chrono_types::make_shared<chrono::ChTriangleMeshConnected>();
    mesh->LoadWavefrontMesh(filePath, /*load_normals=*/true, /*load_uv=*/true);

    mesh->RepairDuplicateVertexes(1e-9);

    auto body = chrono_types::make_shared<chrono::ChBody>();
    body->SetFixed(true);
    body->EnableCollision(true);
    body->SetPos(position);

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

