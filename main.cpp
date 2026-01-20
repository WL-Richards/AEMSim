// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// A simple Chrono + Irrlicht demo:
//  - Sphere "ball" with drag + Magnus force (custom force accumulator)
//  - Step mode: press ENTER to advance one physics step (only after launch)
//  - SPACE launches (unfreezes) the ball + sets initial lin/ang velocity
//  - R resets ball + marker + re-initializes the mate constraint (teleport-safe)
//  - Debug grid + meter labels on XY plane
//
// Notes:
//  - Chrono default "up" here is +Z (we set gravity along -Z).
//  - The marker is a small cylinder rigidly fixed to the ball so rotation is visible.
// =============================================================================

#include <chrono/physics/ChSystem.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChContactMaterialNSC.h>
#include <chrono/assets/ChTexture.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#include <irrlicht.h>

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <chrono>

#include "simulation/PhysicalSystemFactory.hpp"
#include "simulation/projectile/SphericalProjectileSpawner.h"
#include "simulation/projectile/data/SphericalProjectileData.hpp"
#include "utility/AMath.h"
#include "utility/DebugDrawer.h"
#include "utility/WorldHelper.h"
#include "utility/ui/SimTelemetryHUD.hpp"
#include "visualization/cameras/OrbitFieldCameraController.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;


// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    SetChronoDataPath(CHRONO_DATA_DIR);
    
    const double TIMESTEP = 1e-3;
    const bool use_visual = true;
    const int steps_no_visual = 1733;

    // 0 - Create a Chrono physical system
    const std::shared_ptr<chrono::ChSystem> sys = PhysicalSystemFactory::createNonSmoothContactSystemMulticore_Realworld(20);
    const std::shared_ptr<ChVisualSystemIrrlicht> vis = std::make_shared<ChVisualSystemIrrlicht>();
    

    // 1 - Create a projectile spawner and a projectile data object
    const std::shared_ptr<SphericalProjectileData> projectileData = CREATE_FUEL_PROJECTILE_DATA();
    const std::shared_ptr<SphericalProjectileSpawner> projectileSpawner = std::make_shared<SphericalProjectileSpawner>(sys, projectileData);

    // 2 - Spawn Projectiles
    // const ChVector3d p0(0, (1 - INCHES_TO_METERS(23.373)) - 0.8, FEET_TO_METERS(2));      // 1m above ground
    const ChVector3d p0(0,0,0.1);
    const ChVector3d robot_velocity(0,0,0);
    
    //THIS IS 70-30 angle 4 - 10.5 m/s
    //projectileSpawner->SweepShots(p0, 30.f, 70.f, 0.5f, 4.f, 10.5f, 0.5f, robot_velocity);
    
    projectileSpawner->SweepShots(p0, 55.f, 70.f, 0.5f, 4.f, 10.5f, 0.5f, robot_velocity);
    auto floor = WorldHelper::MakeInfiteishFloor(sys, -0.1);
    const ChVector3d hub_location(INCHES_TO_METERS(80), 0, 0.0);
    
    // FARTHEST WE CAN BE FROM HUB:
    // const ChVector3d hub_location(INCHES_TO_METERS(245), 0, 0.0);
    std::vector<std::shared_ptr<chrono::ChBody>> funnel_bodies;
    WorldHelper::CreateHub(sys, hub_location, chrono::QuatFromAngleZ(-CH_PI/2), &funnel_bodies);

    

    const double funnel_center_z =
        INCHES_TO_METERS(59.64 + (17.90 / 2.0) - 0.963);
    const double funnel_height = INCHES_TO_METERS(17.90);
    const double funnel_depth = INCHES_TO_METERS(0.12);
    const double funnel_tilt = 0.54;
    const double trigger_thickness = INCHES_TO_METERS(0.5);
    const double trigger_size = INCHES_TO_METERS(13.75);
    const double min_z_offset = -(funnel_height * 0.5) * std::cos(funnel_tilt) -
                                (funnel_depth * 0.5) * std::abs(std::sin(funnel_tilt));
    const double trigger_z = funnel_center_z + min_z_offset + (trigger_thickness * 0.5) - INCHES_TO_METERS(3);

    auto trigger_mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    trigger_mat->SetFriction(0.0f);
    trigger_mat->SetRestitution(0.0f);

    auto funnel_trigger = chrono_types::make_shared<chrono::ChBodyEasyBox>(
        trigger_size,
        trigger_size,
        trigger_thickness,
        1000.0,
        /*create_visualization=*/true,
        /*create_collision=*/true,
        trigger_mat
    );
    if (auto vm = funnel_trigger->GetVisualModel()) {
        if (vm->GetNumShapes() > 0) {
            vm->GetShape(0)->SetColor(chrono::ChColor(0.2f, 0.9f, 0.2f));
        }
    }
    funnel_trigger->SetFixed(true);
    funnel_trigger->SetPos(hub_location + chrono::ChVector3d(0, 0, trigger_z));
    funnel_trigger->EnableCollision(true);
    sys->AddBody(funnel_trigger);
    projectileSpawner->AddTriggerBody(funnel_trigger);
    for (const auto& funnel_body : funnel_bodies) {
        projectileSpawner->AddFunnelBody(funnel_body, hub_location);
    }

    //WorldHelper::CreateTriangleMesh(sys, "C:\\Users\\Will\\Documents\\FRC\\AEMSim\\AEMSim\\2026Hub.obj", ChVector3d(INCHES_TO_METERS(182.11), 0, 0.), chrono::QuatFromAngleZ(CH_PI/2));
    

    
    // ---- Solver tuning (optional) ----
    if (auto nsc_sys = std::dynamic_pointer_cast<chrono::ChSystemNSC>(sys)) {
        if (auto solver = nsc_sys->GetSolver()) {
            nsc_sys->SetSolverType(ChSolver::Type::PSOR);
            solver->AsIterative()->SetMaxIterations(100);
            solver->AsIterative()->SetTolerance(1e-6);
        }
    }

    if (use_visual) {
        // ---- Irrlicht visualization system ----
        vis->AttachSystem(sys.get());

        vis->SetCameraVertical(CameraVerticalDir::Z);
        vis->SetWindowSize(1280, 720);
        vis->SetBackgroundColor(ChColor(0.31f, 0.31f, 0.31f));
        vis->Initialize();
        
        auto telemetryHud = std::make_unique<SimTelemetryHUD>(vis, sys, TIMESTEP);
        auto camera = std::make_unique<OrbitFieldCameraController>(
            vis,
            hub_location, // target
            8.0,                                // distance
            0.7,                                // yaw
            0.55                                // pitch
        );
        
        vis->AddUserEventReceiver(camera.get());

        // Add lights / camera
        vis->AddLightDirectional(
            /*elevation*/ 60,
            /*azimuth*/ 45,
            /*ambient*/  ChColor(0.67f, 0.67f, 0.67f),
            /*specular*/ ChColor(0.05f, 0.05f, 0.05f),
            /*diffuse*/  ChColor(0.67f, 0.67f, 0.67f)
        );
        
        // Grid labels (one-time creation)
        irr::scene::ISceneManager* smgr = vis->GetSceneManager();
        irr::IrrlichtDevice* device = vis->GetDevice();
        DrawDebugMeterLabelsXY(device, smgr,
                         /*z_plane=*/0.05f,
                         /*half_extent_m=*/100.0f,
                         /*step_m=*/1.0f,
                         /*text_height_m=*/0.35f,
                         /*color=*/irr::video::SColor(255, 255, 255, 255),
                         /*label_x_axis=*/true,
                         /*label_y_axis=*/true);

        // ---- Main loop ----
        while (vis->Run()) {
            camera->Update(TIMESTEP);
            
            vis->BeginScene();
            vis->Render();

            // Draw grid every frame
            irr::video::IVideoDriver* drv = vis->GetVideoDriver();
            
            DrawDebugGridXY(drv,
                       /*z_plane=*/0.0f,
                       /*half_extent_m=*/100.0f,
                       /*minor_step_m=*/0.5f,
                       /*major_every=*/5,
                       /*minor_color=*/irr::video::SColor(255, 80, 80, 80),
                       /*major_color=*/irr::video::SColor(255, 140, 140, 140),
                       /*x_axis_color=*/irr::video::SColor(255, 255, 60, 60),
                       /*y_axis_color=*/irr::video::SColor(255, 60, 255, 60));

            // DrawDebugVector3DColor(drv, shooter_initial, irr::video::SColor(255,0,0,255));
            // DrawDebugVector3DColor(drv, robot_velocity, irr::video::SColor(255,255,0,0));
            // DrawDebugVector3D(drv, v0);

            projectileSpawner->DoRenderStep(drv);
            telemetryHud->DoRenderStep(drv);
            
            vis->EndScene();

            projectileSpawner->DoPhysicsStep();
            telemetryHud->DoPhysicsStep();
            
            sys->DoStepDynamics(TIMESTEP);
        }
    } else {
        const auto start_time = std::chrono::high_resolution_clock::now();
        for (int step = 1; step <= steps_no_visual; ++step) {
            const auto step_start = std::chrono::high_resolution_clock::now();
            projectileSpawner->DoPhysicsStep();
            sys->DoStepDynamics(TIMESTEP);
            const auto step_end = std::chrono::high_resolution_clock::now();
            const std::chrono::duration<double, std::milli> step_ms = step_end - step_start;
            std::cout << "Step: " << step << " (" << step_ms.count() << " ms)\n";
        }
        const auto end_time = std::chrono::high_resolution_clock::now();
        const std::chrono::duration<double, std::milli> total_ms = end_time - start_time;
        std::cout << "Total: " << total_ms.count() << " ms\n";
    }

    return 0;
}
