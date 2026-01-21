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
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChContactMaterialNSC.h>
#include <chrono/assets/ChTexture.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#include <irrlicht.h>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <deque>

#include "core/SimulationConfig.hpp"
#include "scene/SceneState.hpp"
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

// Helper function to load projectile spawn data from CSV file
// CSV format (no header): p0X, p0Y, v0X, v0Y, v0Z
// p0Z is set to FEET_TO_METERS(2)
void SpawnProjectilesFromCSV(const std::string& filepath,
                              std::shared_ptr<SphericalProjectileSpawner> spawner) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Warning: Could not open CSV file: " << filepath << "\n";
        return;
    }

    std::string line;
    int count = 0;
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;

        while (std::getline(ss, token, ',')) {
            // Trim whitespace
            size_t start = token.find_first_not_of(" \t");
            size_t end = token.find_last_not_of(" \t");
            if (start != std::string::npos) {
                token = token.substr(start, end - start + 1);
            }
            values.push_back(std::stod(token));
        }

        if (values.size() >= 5) {
            const ChVector3d p0(values[0], values[1], FEET_TO_METERS(2));
            const ChVector3d v0(values[2], values[3], values[4]);
            spawner->Spawn(p0, v0, ChVector3d(0,0,0), true, chrono::ChColor(1,1,0), true);
            ++count;
        }
    }

    std::cout << "Spawned " << count << " projectiles from " << filepath << "\n";
}

// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    SetChronoDataPath(CHRONO_DATA_DIR);

    const std::shared_ptr<chrono::ChSystem> sys = PhysicalSystemFactory::createNonSmoothContactSystem_Realworld();
    const std::shared_ptr<ChVisualSystemIrrlicht> vis = std::make_shared<ChVisualSystemIrrlicht>();

    // 2 - Spawn Projectiles
    // const ChVector3d p0(0, (1 - INCHES_TO_METERS(23.373)) - 0.8, FEET_TO_METERS(2));      // 1m above ground

    const ChVector3d p0(2, 3,FEET_TO_METERS(2));
    const ChVector3d v0(2.56632, 1.01743, 2.97807);
    //const ChVector3d v0(0, 1, 0);
    const ChVector3d robot_velocity(0,0,0);
    
    //THIS IS 70-30 angle 4 - 10.5 m/s
    //projectileSpawner->SweepShots(p0, 30.f, 70.f, 0.5f, 4.f, 10.5f, 0.5f, robot_velocity);
    
    //projectileSpawner->SweepShots(p0, 60.f, 70.f, 0.5f, 8.f, 10.5f, 0.25f, robot_velocity, v0);
 
    // FARTHEST WE CAN BE FROM HUB:
    const ChVector3d hub_location(4.6101, 4.03479, FEET_TO_METERS(2));

    // Funnel geometry constants for debug visualization
    const double debug_funnel_center_z = INCHES_TO_METERS(59.64 + (17.90 / 2.0) - 0.963);
    const double debug_funnel_height = INCHES_TO_METERS(17.90);
    // Set halfway_z to the TOP of the funnel (center + half height)
    double debug_funnel_halfway_z = debug_funnel_center_z + (debug_funnel_height * 0.5);
    double debug_funnel_goal_z = debug_funnel_center_z;

    auto build_scene = [&](const std::shared_ptr<chrono::ChSystem>& sys_in) {
        SceneState scene;
        scene.sys = sys_in;
        const std::shared_ptr<SphericalProjectileData> projectileData = CREATE_FUEL_PROJECTILE_DATA();
        scene.spawner = std::make_shared<SphericalProjectileSpawner>(scene.sys, projectileData);

        scene.spawner->ConfigureProximityMonitor(
            chrono::ChVector3d(0, 3, 0),  // center
            0.2,                          // radius (m)
            1.5,                            // seconds inside radius
            SimConfig::TIMESTEP                      // timestep
        );

        //scene.spawner->SweepShots(p0, 50.f, 70.f, 0.5f, 6.f, 10.5f, 0.25f, robot_velocity, v0);
        // Option 2: Single hardcoded spawn
        //scene.spawner->Spawn(p0, v0, ChVector3d(0,0,0), true, chrono::ChColor(1,1,0), true);
        SpawnProjectilesFromCSV("C:\\Users\\Will\\Downloads\\out(1).txt", scene.spawner);
        //scene.spawner->Spawn(p1, v1, ChVector3d(0,0,0), true, chrono::ChColor(1,1,0), true);
        //scene.spawner->Spawn(p0, -v0, ChVector3d(0,0,0), true, chrono::ChColor(1,0,0), true);
        //scene.spawner->SetEnableFreezeOnContact(false);

        scene.floor = WorldHelper::MakeInfiteishFloor(scene.sys, 0);
        //scene.spawner->AddGroundBody(scene.floor);
        // WorldHelper::CreateWalls(scene.sys, ChVector3d(0, 0, 0), 5, 5, 2);
        // WorldHelper::CreateTestCube(scene.sys, ChVector3d(0,4,0));

        if (SimConfig::HAS_HUB) {
            WorldHelper::CreateHub(scene.sys,
                                   hub_location,
                                   chrono::QuatFromAngleZ(-CH_PI/2),
                                   &scene.funnel_bodies, 23, 26);

            const double funnel_center_z =
                INCHES_TO_METERS(59.64 + (17.90 / 2.0) - 0.963);
            const double funnel_height = INCHES_TO_METERS(17.90);
            const double funnel_depth = INCHES_TO_METERS(0.12);
            const double funnel_tilt = 0.54;

            // Set halfway_z to the TOP of the funnel
            const double funnel_halfway_z = funnel_center_z + (funnel_height * 0.34);
            const double funnel_halfway_z_world = hub_location.z() + funnel_halfway_z;
            debug_funnel_halfway_z = funnel_halfway_z;

            const double trigger_thickness = INCHES_TO_METERS(0.5);
            const double min_z_offset = -(funnel_height * 0.5) * std::cos(funnel_tilt) -
                                        (funnel_depth * 0.5) * std::abs(std::sin(funnel_tilt));
            const double trigger_z = funnel_center_z + min_z_offset + (trigger_thickness * 0.5) - INCHES_TO_METERS(3);
            const double trigger_z_world = hub_location.z() + trigger_z;
            scene.spawner->SetFunnelGoalZ(trigger_z_world);
            
            debug_funnel_goal_z = trigger_z;
            for (const auto& funnel_body : scene.funnel_bodies) {
                scene.spawner->AddFunnelBody(funnel_body, hub_location, funnel_halfway_z_world);
            }
        }

        if (auto nsc_sys = std::dynamic_pointer_cast<chrono::ChSystemNSC>(scene.sys)) {
            if (auto solver = nsc_sys->GetSolver()) {
                nsc_sys->SetSolverType(ChSolver::Type::PSOR);
                solver->AsIterative()->SetMaxIterations(40);
                solver->AsIterative()->SetTolerance(1e-6);
            }
        }

        if (auto coll = scene.sys->GetCollisionSystem()) {
            coll->BindAll();
        }

        return scene;
    };

    auto reset_system = [&](const std::shared_ptr<chrono::ChSystem>& sys_to_reset) {
        if (!sys_to_reset) {
            return;
        }

        const auto bodies = sys_to_reset->GetBodies();
        for (const auto& body : bodies) {
            if (body) {
                sys_to_reset->RemoveBody(body);
            }
        }
        const auto links = sys_to_reset->GetLinks();
        for (const auto& link : links) {
            if (link) {
                sys_to_reset->RemoveLink(link);
            }
        }
        const auto meshes = sys_to_reset->GetMeshes();
        for (const auto& mesh : meshes) {
            if (mesh) {
                sys_to_reset->RemoveMesh(mesh);
            }
        }
        const auto others = sys_to_reset->GetOtherPhysicsItems();
        for (const auto& item : others) {
            if (item) {
                sys_to_reset->RemoveOtherPhysicsItem(item);
            }
        }

        sys_to_reset->Clear();

        if (auto contacts = sys_to_reset->GetContactContainer()) {
            contacts->RemoveAllContacts();
        }
        if (auto coll = sys_to_reset->GetCollisionSystem()) {
            coll->Clear();
        }
        sys_to_reset->SetChTime(0.0);
        sys_to_reset->ResetNumSteps();
    };

    if (SimConfig::USE_VISUAL) {
        reset_system(sys);
        SceneState scene = build_scene(sys);
        std::shared_ptr<SphericalProjectileSpawner> projectileSpawner = scene.spawner;

        // ---- Irrlicht visualization system ----
        vis->AttachSystem(sys.get());

            vis->SetCameraVertical(CameraVerticalDir::Z);
            vis->SetWindowSize(1280, 720);
            vis->SetBackgroundColor(ChColor(0.31f, 0.31f, 0.31f));
            vis->Initialize();
            
        std::unique_ptr<SimTelemetryHUD> telemetryHud = std::make_unique<SimTelemetryHUD>(vis, sys, SimConfig::TIMESTEP);
        auto camera = std::make_unique<OrbitFieldCameraController>(
            vis,
            OrbitFieldCameraController::Params(),
            hub_location, // target
            8.0,                                // distance
            0.7,                                // yaw
            -1                                // pitch
        );
        camera->SetStepMode(true);
        
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
                         /*z_plane=*/0.1f,
                         /*half_extent_m=*/100.0f,
                         /*step_m=*/1.0f,
                         /*scale*/.3f,
                         /*color=*/irr::video::SColor(255, 0, 0, 0),
                         /*label_x_axis=*/true,
                         /*label_y_axis=*/true);

            // ---- Main loop ----
            struct SystemSnapshot {
                struct BodyState {
                    chrono::ChBody* body = nullptr;
                    chrono::ChVector3d pos;
                    chrono::ChQuaternion<> rot;
                    chrono::ChVector3d lin_vel;
                    chrono::ChVector3d ang_vel;
                    bool fixed = false;
                };

                double time = 0.0;
                std::vector<BodyState> bodies;
                SphericalProjectileSpawner::Snapshot projectiles;
            };
            auto capture_snapshot = [&](SystemSnapshot& snapshot) {
                snapshot.time = sys->GetChTime();
                snapshot.bodies.clear();
                if (auto& body_list = sys->GetBodies(); !body_list.empty()) {
                    snapshot.bodies.reserve(body_list.size());
                    for (const auto& body : body_list) {
                        if (!body) continue;
                        SystemSnapshot::BodyState state;
                        state.body = body.get();
                        state.pos = body->GetPos();
                        state.rot = body->GetRot();
                        state.lin_vel = body->GetLinVel();
                        state.ang_vel = body->GetAngVelParent();
                        state.fixed = body->IsFixed();
                        snapshot.bodies.push_back(state);
                    }
                }
                projectileSpawner->CaptureSnapshot(snapshot.projectiles);
            };

            auto restore_snapshot = [&](const SystemSnapshot& snapshot) {
                for (const auto& state : snapshot.bodies) {
                    if (!state.body) continue;
                    state.body->SetPos(state.pos);
                    state.body->SetRot(state.rot);
                    state.body->SetLinVel(state.lin_vel);
                    state.body->SetAngVelParent(state.ang_vel);
                    state.body->SetFixed(state.fixed);
                }
                sys->Update(UpdateFlags::UPDATE_ALL);
                projectileSpawner->RestoreSnapshot(snapshot.projectiles);
            };

            SystemSnapshot initial_snapshot;
            capture_snapshot(initial_snapshot);
            std::deque<SystemSnapshot> snapshot_history;

            int timing_frame = 0;
            double timing_accum_camera_ms = 0.0;
            double timing_accum_render_ms = 0.0;
            double timing_accum_grid_ms = 0.0;
            double timing_accum_telemetry_ms = 0.0;
            double timing_accum_projectile_render_ms = 0.0;
            double timing_accum_physics_step_ms = 0.0;
            double timing_accum_sys_step_ms = 0.0;

            while (vis->Run()) {
                const auto frame_start = std::chrono::high_resolution_clock::now();

                const auto camera_start = std::chrono::high_resolution_clock::now();
                camera->Update(SimConfig::TIMESTEP);
                const auto camera_end = std::chrono::high_resolution_clock::now();
                
                const auto render_start = std::chrono::high_resolution_clock::now();
                vis->BeginScene();
                vis->Render();

                // Draw grid every frame
                irr::video::IVideoDriver* drv = vis->GetVideoDriver();
                
                const auto grid_start = std::chrono::high_resolution_clock::now();
                DrawDebugGridXY(drv,
                           /*z_plane=*/0.0f,
                           /*half_extent_m=*/100.0f,
                           /*minor_step_m=*/0.5f,
                           /*major_every=*/5,
                           /*minor_color=*/irr::video::SColor(255, 80, 80, 80),
                           /*major_color=*/irr::video::SColor(255, 140, 140, 140),
                           /*x_axis_color=*/irr::video::SColor(255, 255, 60, 60),
                           /*y_axis_color=*/irr::video::SColor(255, 60, 255, 60));
                const auto grid_end = std::chrono::high_resolution_clock::now();

                // Debug visualization: funnel_halfway_z plane (magenta)
                if (SimConfig::HAS_HUB) {
                    DrawDebugHorizontalPlane(drv,
                        hub_location + ChVector3d(0, 0, debug_funnel_halfway_z),
                        0.5,  // half_size in meters
                        irr::video::SColor(255, 255, 0, 255),  // magenta
                        false);  // no depth test - always visible
                    DrawDebugHorizontalPlane(drv,
                        hub_location + ChVector3d(0, 0, debug_funnel_goal_z),
                        0.5,  // half_size in meters
                        irr::video::SColor(255, 60, 255, 60),  // green
                        false);  // no depth test - always visible
                }

                // DrawDebugVector3DColor(drv, shooter_initial, irr::video::SColor(255,0,0,255));
                //DrawDebugVector3DAtOriginColor(drv, v0, p0, irr::video::SColor(255,255,0,0));
                // DrawDebugVector3D(drv, v0);
                //DrawDebugYardStick(drv, ChVector3d(0,0,0), ChVector3d(0,0,1), irr::video::SColor(255, 255, 0, 255), false);

                const auto proj_render_start = std::chrono::high_resolution_clock::now();
                projectileSpawner->DoRenderStep(drv);
                const auto proj_render_end = std::chrono::high_resolution_clock::now();

                const auto telemetry_render_start = std::chrono::high_resolution_clock::now();
                telemetryHud->DoRenderStep(drv);
                const auto telemetry_render_end = std::chrono::high_resolution_clock::now();
                
                vis->EndScene();
                const auto render_end = std::chrono::high_resolution_clock::now();

                const auto physics_step_start = std::chrono::high_resolution_clock::now();
                const bool step_mode = camera->IsStepMode();
                const int step_request = camera->ConsumeStepRequest();
                const bool reset_requested = camera->ConsumeResetRequest();
                bool step_forward = !step_mode || step_request > 0;
                bool step_backward = step_mode && step_request < 0;

            if (reset_requested) {
                reset_system(sys);
                scene = build_scene(sys);
                projectileSpawner = scene.spawner;
                vis->BindAll();
                telemetryHud = std::make_unique<SimTelemetryHUD>(vis, sys, SimConfig::TIMESTEP);
                camera->ResetCamera();
                camera->SetStepMode(false);
                snapshot_history.clear();
                step_forward = false;
                step_backward = false;
            } else if (step_forward) {
                    SystemSnapshot snapshot;
                    capture_snapshot(snapshot);
                    snapshot_history.push_back(std::move(snapshot));
                    if (snapshot_history.size() > SimConfig::MAX_SNAPSHOT_HISTORY) {
                        snapshot_history.pop_front();
                    }

                    projectileSpawner->DoPhysicsStep();
                    telemetryHud->DoPhysicsStep();
                } else if (step_backward) {
                    if (snapshot_history.size() >= 2) {
                        snapshot_history.pop_back();
                        const auto snapshot = snapshot_history.back();
                        restore_snapshot(snapshot);
                        telemetryHud->DoPhysicsStepBackward();
                    }
                }
                const auto physics_step_end = std::chrono::high_resolution_clock::now();
                
                const auto sys_step_start = std::chrono::high_resolution_clock::now();
                if (step_forward) {
                    sys->DoStepDynamics(SimConfig::TIMESTEP);
                }
                const auto sys_step_end = std::chrono::high_resolution_clock::now();

                timing_accum_camera_ms += std::chrono::duration<double, std::milli>(camera_end - camera_start).count();
                timing_accum_render_ms += std::chrono::duration<double, std::milli>(render_end - render_start).count();
                timing_accum_grid_ms += std::chrono::duration<double, std::milli>(grid_end - grid_start).count();
                timing_accum_projectile_render_ms +=
                    std::chrono::duration<double, std::milli>(proj_render_end - proj_render_start).count();
                timing_accum_telemetry_ms +=
                    std::chrono::duration<double, std::milli>(telemetry_render_end - telemetry_render_start).count();
                timing_accum_physics_step_ms +=
                    std::chrono::duration<double, std::milli>(physics_step_end - physics_step_start).count();
                timing_accum_sys_step_ms += std::chrono::duration<double, std::milli>(sys_step_end - sys_step_start).count();

                timing_frame++;
                if (timing_frame >= SimConfig::TIMING_REPORT_EVERY) {
                    const double frames = static_cast<double>(timing_frame);
                    std::cout << std::fixed << std::setprecision(3)
                              << "Timing (ms/frame): camera=" << (timing_accum_camera_ms / frames)
                              << " render_total=" << (timing_accum_render_ms / frames)
                              << " grid=" << (timing_accum_grid_ms / frames)
                              << " proj_render=" << (timing_accum_projectile_render_ms / frames)
                              << " hud_render=" << (timing_accum_telemetry_ms / frames)
                              << " physics_step=" << (timing_accum_physics_step_ms / frames)
                              << " sys_step=" << (timing_accum_sys_step_ms / frames)
                              << "\n";

                    timing_frame = 0;
                    timing_accum_camera_ms = 0.0;
                    timing_accum_render_ms = 0.0;
                    timing_accum_grid_ms = 0.0;
                    timing_accum_projectile_render_ms = 0.0;
                    timing_accum_telemetry_ms = 0.0;
                    timing_accum_physics_step_ms = 0.0;
                    timing_accum_sys_step_ms = 0.0;
                }
            }
    } else {
        reset_system(sys);
        SceneState scene = build_scene(sys);
        std::shared_ptr<SphericalProjectileSpawner> projectileSpawner = scene.spawner;
        const auto start_time = std::chrono::high_resolution_clock::now();
        for (int step = 1; step <= SimConfig::STEPS_NO_VISUAL; ++step) {
            const auto step_start = std::chrono::high_resolution_clock::now();
            projectileSpawner->DoPhysicsStep();
            sys->DoStepDynamics(SimConfig::TIMESTEP);
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
