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
#include <chrono/assets/ChTexture.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#include <irrlicht.h>

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>

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
// Main
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    SetChronoDataPath(CHRONO_DATA_DIR);
    
    const double TIMESTEP = 1e-3;

    // 0 - Create a Chrono physical system
    const std::shared_ptr<chrono::ChSystem> sys = PhysicalSystemFactory::createNonSmoothContactSystemMulticore_Realworld();
    const std::shared_ptr<ChVisualSystemIrrlicht> vis = std::make_shared<ChVisualSystemIrrlicht>();
    

    // 1 - Create a projectile spawner and a projectile data object
    const std::shared_ptr<SphericalProjectileData> projectileData = CREATE_FUEL_PROJECTILE_DATA();
    const std::shared_ptr<SphericalProjectileSpawner> projectileSpawner = std::make_shared<SphericalProjectileSpawner>(sys, projectileData);

    // 2 - Spawn Projectiles
    //const ChVector3d p0(0, (1 - INCHES_TO_METERS(23.373)) - 0.8, FEET_TO_METERS(2));      // 1m above ground
    const ChVector3d p0(0, 0, 0.1);      // 1m above ground

    // const float hoodAngle = 70 * (CH_PI/180);
    // const float xComponent = cos(hoodAngle);
    // const float yComponent = sin(hoodAngle);
    // const float shooterVelocity = 8.7f; // m/s
    // const ChVector3d shooter_initial(0, xComponent * shooterVelocity, yComponent * shooterVelocity); // m/s
    const ChVector3d shooter_initial(5.26585, -3.94938, 10.8601); // m/s

    const ChVector3d robot_velocity(0,0,0); // m/s
    ///const ChVector3d w0((-0.8)*200,(-0.6)*200,0);  // rad/s (spin)
    //const ChVector3d w0((0.8)*25,(0.6)*25,0);  // rad/s (spin)
    const ChVector3d w0(0.0, 0.0, 0.0);  // rad/s (spin)
    const ChVector3d v0 = shooter_initial + robot_velocity;
    //std::cout << "Hood Angle: " << hoodAngle * (180/CH_PI) << "\n";


    projectileSpawner->Spawn(p0, v0, w0, true);
    
    
    auto floor = WorldHelper::MakeInfiteishFloor(sys, -0.1);
    //WorldHelper::CreateTriangleMesh(sys, "/home/will/Documents/frc/AEMSim/2026Hub.obj", ChVector3d(0, 1, 0.));
    WorldHelper::CreateTestCube(sys, ChVector3d(4, -3, 5));
    WorldHelper::CreateTestCube(sys, ChVector3d(8, -6, 2));
    
    // ---- Irrlicht visualization system ----
    vis->AttachSystem(sys.get());

    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowSize(1280, 720);
    vis->SetBackgroundColor(ChColor(0.31f, 0.31f, 0.31f));
    vis->Initialize();
    
    auto telemetryHud = std::make_unique<SimTelemetryHUD>(vis, sys, TIMESTEP);
    
    auto camera = std::make_unique<OrbitFieldCameraController>(
        vis,
        OrbitFieldCameraController::Params(),
        chrono::ChVector3d(0, 0, 0), // target
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
                     /*label_y_axis=*/true)

    // ---- Solver tuning (optional) ----
    if (auto nsc_sys = std::dynamic_pointer_cast<chrono::ChSystemNSC>(sys)) {
        if (auto solver = nsc_sys->GetSolver()) {
            nsc_sys->SetSolverType(ChSolver::Type::PSOR);
            solver->AsIterative()->SetMaxIterations(100);
            solver->AsIterative()->SetTolerance(1e-6);
        }
    }

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

        projectileSpawner->DoRenderStep(drv);
        telemetryHud->DoRenderStep(drv);
        
        vis->EndScene();

        projectileSpawner->DoPhysicsStep();
        telemetryHud->DoPhysicsStep();
        
        sys->DoStepDynamics(TIMESTEP);
    }

    return 0;
}
