#include "SphericalProjectileSpawner.h"

#include <irrlicht.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChContactContainer.h>
#include <chrono/physics/ChBody.h>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <iostream>

#include "SphericalProjectile.h"
#include "../Aerodynamics.hpp"
#include "../../utility/DebugDrawer.h"
#include "../../utility/PhysicsHelper.h"
#include "data/SphericalProjectileData.hpp"


SphericalProjectileSpawner::SphericalProjectileSpawner(std::shared_ptr<chrono::ChSystem> physicalSystem,
                                                       std::shared_ptr<SphericalProjectileData> projectileData)
{
    this->projectileData = projectileData;
    this->physicalSystem = physicalSystem;
}

bool SphericalProjectileSpawner::Spawn(
    const chrono::ChVector3d location,
    const chrono::ChVector3d initialVelocity,
    const chrono::ChVector3d initialSpin,
    bool enableCollision,
    const chrono::ChColor projectileColor,
    bool logTrajectory
)
{
    auto mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

    mat->SetFriction(static_cast<float>(projectileData->PhysicalParameters->FrictionCoefficient));
    mat->SetRestitution(static_cast<float>(projectileData->PhysicalParameters->RestitutionCoefficient));

    auto ball = chrono_types::make_shared<chrono::ChBodyEasySphere>(
        projectileData->AerodynamicParameters->RadiusM,
        projectileData->PhysicalParameters->Density,
        /*create_visualization=*/true,
        /*create_collision=*/enableCollision,
        mat
    );

    // Give the ball a visible appearance
    if (auto vm = ball->GetVisualModel())
    {
        if (vm->GetNumShapes() > 0)
        {
            vm->GetShape(0)->SetColor(projectileColor); // yellow tint
        }
    }

    physicalSystem->Add(ball);

    ball->SetPos(location);
    ball->SetRot(chrono::ChQuaternion<>(1, 0, 0, 0));
    ball->SetLinVel(initialVelocity);
    ball->SetAngVelParent(initialSpin);
    ball->SetFixed(false);
    ball->EnableCollision(enableCollision);
    if (ball->GetCollisionModel())
    {
        ball->GetCollisionModel()->SetFamily(1);
        //ball->GetCollisionModel()->AllowCollisionsWith(0);
        ball->GetCollisionModel()->DisallowCollisionsWith(1);
    }


    auto projectile = std::make_unique<SphericalProjectile>(projectileData, ball, initialVelocity,
                                                            ball->AddAccumulator(), logTrajectory);
    if (projectile->ShouldLogTrajectory) {
        projectile->TrajectoryPoints.clear();
        projectile->TrajectoryPoints.push_back(ball->GetPos());
    }
    projectiles.push_back(std::move(projectile));
    return true;
}

void SphericalProjectileSpawner::AddTriggerBody(const std::shared_ptr<chrono::ChBody>& body)
{
    if (!body) return;
    trigger_bodies.insert(body.get());
}

void SphericalProjectileSpawner::AddNonRemovalBody(const std::shared_ptr<chrono::ChBody>& body)
{
    if (!body) return;
    non_removal_bodies.insert(body.get());
}

void SphericalProjectileSpawner::AddFunnelBody(const std::shared_ptr<chrono::ChBody>& body,
                                               const chrono::ChVector3d& hub_center,
                                               double funnel_halfway_z)
{
    if (!body) return;
    const auto diff = body->GetPos() - hub_center;
    const double len = std::sqrt(diff.x() * diff.x() + diff.y() * diff.y());
    if (len <= 1e-9) return;
    funnel_infos[body.get()] = FunnelInfo{hub_center, len, funnel_halfway_z};
}

void SphericalProjectileSpawner::CaptureSnapshot(Snapshot& out_snapshot) const
{
    out_snapshot.trajectory_step_counter = trajectory_step_counter;
    out_snapshot.projectiles.clear();
    out_snapshot.projectiles.reserve(projectiles.size());
    for (const auto& projectile : projectiles)
    {
        Snapshot::ProjectileSnapshot snap;
        snap.removed = projectile->Removed;
        snap.in_goal = projectile->InGoal;
        snap.fixed = projectile->Sphere->IsFixed();
        snap.trajectory_size = projectile->TrajectoryPoints.size();
        out_snapshot.projectiles.push_back(snap);
    }
}

void SphericalProjectileSpawner::RestoreSnapshot(const Snapshot& snapshot)
{
    trajectory_step_counter = snapshot.trajectory_step_counter;
    const size_t count = std::min(snapshot.projectiles.size(), projectiles.size());
    for (size_t i = 0; i < count; ++i)
    {
        const auto& snap = snapshot.projectiles[i];
        auto& projectile = projectiles[i];
        projectile->Removed = snap.removed;
        projectile->InGoal = snap.in_goal;
        projectile->Sphere->SetFixed(snap.fixed);
        if (snap.fixed) {
            projectile->Sphere->SetLinVel(chrono::ChVector3d(0, 0, 0));
            projectile->Sphere->SetAngVelParent(chrono::ChVector3d(0, 0, 0));
        }
        if (projectile->TrajectoryPoints.size() > snap.trajectory_size) {
            projectile->TrajectoryPoints.resize(snap.trajectory_size);
        }
    }
}

void SphericalProjectileSpawner::SetEnableFreezeOnContact(bool enabled)
{
    enable_freeze_on_contact = enabled;
}

void SphericalProjectileSpawner::AddGroundBody(const std::shared_ptr<chrono::ChBody>& body)
{
    if (!body) return;
    ground_bodies.insert(body.get());
}

void SphericalProjectileSpawner::SetFunnelGoalZ(double goal_z)
{
    funnel_goal_z = goal_z;
    funnel_goal_enabled = true;
}

void SphericalProjectileSpawner::ConfigureProximityMonitor(const chrono::ChVector3d& point,
                                                           double radius,
                                                           double time_threshold,
                                                           double timestep)
{
    monitor_point = point;
    monitor_radius = radius;
    monitor_time_threshold = time_threshold;
    monitor_timestep = timestep;
    monitor_enabled = (radius > 0.0 && time_threshold > 0.0 && timestep > 0.0);
    monitor_fired = false;
    monitor_time_in_zone.clear();
    if (monitor_enabled) {
        monitor_time_in_zone.reserve(projectiles.size());
        for (const auto& projectile : projectiles) {
            monitor_time_in_zone[projectile->Sphere.get()] = 0.0;
        }
    }
}

void SphericalProjectileSpawner::DoPhysicsStep()
{
    const bool debug_freeze_contacts = true;
    std::unordered_set<chrono::ChBody*> projectiles_in_contact;
    std::unordered_set<chrono::ChBody*> projectiles_in_goal;
    std::unordered_set<chrono::ChBody*> projectiles_on_ground;
    std::unordered_set<chrono::ChBody*> projectile_bodies;
    struct ContactDebugInfo {
        chrono::ChBody* other_body = nullptr;
        chrono::ChVector3d point = chrono::ChVector3d(0, 0, 0);
        bool is_trigger = false;
        bool is_ground = false;
        bool is_funnel = false;
    };
    std::unordered_map<chrono::ChBody*, ContactDebugInfo> debug_contacts;
    projectile_bodies.reserve(projectiles.size());
    for (const auto& projectile : projectiles)
    {
        if (projectile->Removed) {
            continue;
        }
        projectile_bodies.insert(projectile->Sphere.get());
    }

    if (projectile_bodies.empty()) {
        return;
    }

    if (physicalSystem && physicalSystem->GetContactContainer())
    {
        struct ContactReporter : chrono::ChContactContainer::ReportContactCallback
        {
            const std::unordered_set<chrono::ChBody*>& targets;
            const std::unordered_set<chrono::ChBody*>& triggers;
            const std::unordered_set<chrono::ChBody*>& non_removal;
            const std::unordered_map<chrono::ChBody*, FunnelInfo>& funnel_infos;
            const std::unordered_set<chrono::ChBody*>& ground;
            const double projectile_radius;
            std::unordered_set<chrono::ChBody*>& hit_any;
            std::unordered_set<chrono::ChBody*>& hit_goal;
            std::unordered_set<chrono::ChBody*>& hit_ground;
            std::unordered_map<chrono::ChBody*, ContactDebugInfo>& debug_contacts;

            ContactReporter(const std::unordered_set<chrono::ChBody*>& targets_in,
                            const std::unordered_set<chrono::ChBody*>& triggers_in,
                            const std::unordered_set<chrono::ChBody*>& non_removal_in,
                            const std::unordered_map<chrono::ChBody*, FunnelInfo>& funnel_infos_in,
                            const std::unordered_set<chrono::ChBody*>& ground_in,
                            double projectile_radius_in,
                            std::unordered_set<chrono::ChBody*>& hit_any_in,
                            std::unordered_set<chrono::ChBody*>& hit_goal_in,
                            std::unordered_set<chrono::ChBody*>& hit_ground_in,
                            std::unordered_map<chrono::ChBody*, ContactDebugInfo>& debug_contacts_in)
                : targets(targets_in),
                  triggers(triggers_in),
                  non_removal(non_removal_in),
                  funnel_infos(funnel_infos_in),
                  ground(ground_in),
                  projectile_radius(projectile_radius_in),
                  hit_any(hit_any_in),
                  hit_goal(hit_goal_in),
                  hit_ground(hit_ground_in),
                  debug_contacts(debug_contacts_in)
            {
            }

            bool ShouldCountContact(chrono::ChBody* projectile_body,
                                    chrono::ChBody* other_body,
                                    const chrono::ChVector3d& contact_point) const
            {
                if (!projectile_body || !other_body) return false;
                if (non_removal.count(other_body)) return false;
                const auto it = funnel_infos.find(other_body);
                if (it == funnel_infos.end()) return true;

                // Freeze only if the entire ball is above the rim height; otherwise allow bounce.
                const double epsilon = 1e-4;
                const double z = projectile_body->GetPos().z();
                return (z - projectile_radius) >= (it->second.halfway_z - epsilon);
            }

            void RecordDebugContact(chrono::ChBody* projectile_body,
                                    chrono::ChBody* other_body,
                                    const chrono::ChVector3d& contact_point) const
            {
                if (!projectile_body || !other_body) return;
                if (!targets.count(projectile_body)) return;
                if (debug_contacts.count(projectile_body) > 0) return;
                ContactDebugInfo info;
                info.other_body = other_body;
                info.point = contact_point;
                info.is_trigger = triggers.count(other_body) > 0;
                info.is_ground = ground.count(other_body) > 0;
                info.is_funnel = funnel_infos.count(other_body) > 0;
                debug_contacts[projectile_body] = info;
            }

            bool OnReportContact(const chrono::ChVector3d& pA,
                                 const chrono::ChVector3d& pB,
                                 const chrono::ChMatrix33<>& plane_coord,
                                 double distance,
                                 double eff_radius,
                                 const chrono::ChVector3d& react_forces,
                                 const chrono::ChVector3d& react_torques,
                                 chrono::ChContactable* modA,
                                 chrono::ChContactable* modB,
                                 int constraint_offset) override
            {
                auto* bodyA = dynamic_cast<chrono::ChBody*>(modA);
                auto* bodyB = dynamic_cast<chrono::ChBody*>(modB);
                if (bodyA && targets.count(bodyA))
                {
                    if (bodyB) {
                        RecordDebugContact(bodyA, bodyB, pB);
                    }
                    if (bodyB && triggers.count(bodyB))
                    {
                        hit_goal.insert(bodyA);
                    }
                    if (bodyB && ground.count(bodyB))
                    {
                        hit_ground.insert(bodyA);
                    }
                    if (bodyB && !triggers.count(bodyB) && ShouldCountContact(bodyA, bodyB, pB))
                    {
                        hit_any.insert(bodyA);
                        // Record debug for the contact that ACTUALLY causes freeze
                        debug_contacts[bodyA] = ContactDebugInfo{
                            bodyB, pB, triggers.count(bodyB) > 0,
                            ground.count(bodyB) > 0, funnel_infos.count(bodyB) > 0
                        };
                    }
                }
                if (bodyB && targets.count(bodyB))
                {
                    if (bodyA && triggers.count(bodyA))
                    {
                        hit_goal.insert(bodyB);
                    }
                    if (bodyA && ground.count(bodyA))
                    {
                        hit_ground.insert(bodyB);
                    }
                    if (bodyA && !triggers.count(bodyA) && ShouldCountContact(bodyB, bodyA, pA))
                    {
                        hit_any.insert(bodyB);
                        // Record debug for the contact that ACTUALLY causes freeze
                        debug_contacts[bodyB] = ContactDebugInfo{
                            bodyA, pA, triggers.count(bodyA) > 0,
                            ground.count(bodyA) > 0, funnel_infos.count(bodyA) > 0
                        };
                    }
                }
                return true;
            }
        };

        const double projectile_radius = projectileData->AerodynamicParameters->RadiusM;
        auto reporter = chrono_types::make_shared<ContactReporter>(
            projectile_bodies, trigger_bodies, non_removal_bodies, funnel_infos, ground_bodies,
            projectile_radius,
            projectiles_in_contact, projectiles_in_goal, projectiles_on_ground, debug_contacts);
        physicalSystem->GetContactContainer()->ReportAllContacts(reporter);
    }

    const double projectile_radius = projectileData->AerodynamicParameters->RadiusM;
    for (size_t projectile_index = 0; projectile_index < projectiles.size(); ++projectile_index)
    {
        const auto& projectile = projectiles[projectile_index];
        if (monitor_enabled && !monitor_fired && !projectile->Removed) {
            const double dist = (projectile->Sphere->GetPos() - monitor_point).Length();
            double& t = monitor_time_in_zone[projectile->Sphere.get()];
            if (dist <= monitor_radius) {
                t += monitor_timestep;
                if (t >= monitor_time_threshold) {
                    std::cout << "Projectile initial velocity: "
                              << projectile->LaunchVector.x() << ", "
                              << projectile->LaunchVector.y() << ", "
                              << projectile->LaunchVector.z() << "\n";
                    monitor_fired = true;
                }
            } else {
                t = 0.0;
            }
        }

        if (!projectile->Removed &&
            projectiles_in_goal.count(projectile->Sphere.get()) > 0 &&
            projectile->Sphere->GetLinVel().z() < 0)
        {
            if (debug_freeze_contacts) {
                auto it = debug_contacts.find(projectile->Sphere.get());
                if (it != debug_contacts.end()) {
                    const auto& info = it->second;
                    std::cout << "Goal freeze projectile " << projectile_index
                              << " other=" << info.other_body
                              << " point=(" << info.point.x() << "," << info.point.y() << "," << info.point.z() << ")"
                              << " trigger=" << info.is_trigger
                              << " ground=" << info.is_ground
                              << " funnel=" << info.is_funnel
                              << "\n";
                } else {
                    std::cout << "Goal freeze projectile " << projectile_index << " with no contact info\n";
                }
            }
            projectile->Sphere->SetFixed(true);
            projectile->Sphere->SetLinVel(chrono::ChVector3d(0, 0, 0));
            projectile->Sphere->SetAngVelParent(chrono::ChVector3d(0, 0, 0));
            projectile->Removed = true;
            projectile->InGoal = true;
        }

        if (!projectile->Removed &&
            funnel_goal_enabled &&
            projectile->Sphere->GetLinVel().z() < 0)
        {
            const auto pos = projectile->Sphere->GetPos();
            bool inside_xy = false;
            for (const auto& entry : funnel_infos) {
                const auto& info = entry.second;
                const double dx = pos.x() - info.hub_center.x();
                const double dy = pos.y() - info.hub_center.y();
                const double dist_xy = std::sqrt(dx * dx + dy * dy);
                if (dist_xy <= (info.radius + projectile_radius)) {
                    inside_xy = true;
                    break;
                }
            }
            if (inside_xy && pos.z() <= (funnel_goal_z + projectile_radius)) {
                if (debug_freeze_contacts) {
                    std::cout << "Goal freeze (geom) projectile " << projectile_index
                              << " pos=(" << pos.x() << "," << pos.y() << "," << pos.z() << ")"
                              << " goal_z=" << funnel_goal_z
                              << "\n";
                }
                projectile->Sphere->SetFixed(true);
                projectile->Sphere->SetLinVel(chrono::ChVector3d(0, 0, 0));
                projectile->Sphere->SetAngVelParent(chrono::ChVector3d(0, 0, 0));
                projectile->Removed = true;
                projectile->InGoal = true;
            }
        }

        if (enable_freeze_on_contact &&
            !projectile->Removed &&
            projectiles_in_contact.count(projectile->Sphere.get()) > 0)
        {
            if (debug_freeze_contacts) {
                auto it = debug_contacts.find(projectile->Sphere.get());
                if (it != debug_contacts.end()) {
                    const auto& info = it->second;
                    std::cout << "Contact freeze projectile " << projectile_index
                              << " other=" << info.other_body
                              << " point=(" << info.point.x() << "," << info.point.y() << "," << info.point.z() << ")"
                              << " trigger=" << info.is_trigger
                              << " ground=" << info.is_ground
                              << " funnel=" << info.is_funnel
                              << "\n";
                } else {
                    std::cout << "Contact freeze projectile " << projectile_index << " with no contact info\n";
                }
            }
            projectile->Sphere->SetFixed(true);
            projectile->Sphere->SetLinVel(chrono::ChVector3d(0, 0, 0));
            projectile->Sphere->SetAngVelParent(chrono::ChVector3d(0, 0, 0));
            projectile->Removed = true;
        }

        // if (!projectile->Removed &&
        //     PhysicsHelper::HitGroundSphere(projectile->Sphere,
        //                                    projectile->Data->AerodynamicParameters->RadiusM))
        // {
        //     // Multicore collision removal isn't implemented; disable instead.
        //     //projectile->Sphere->EnableCollision(false);
        //     projectile->Sphere->SetFixed(true);
        //     projectile->Sphere->SetLinVel(chrono::ChVector3d(0, 0, 0));
        //     projectile->Sphere->SetAngVelParent(chrono::ChVector3d(0, 0, 0));
        //     projectile->Removed = true;
        // }

        if (projectile->Removed)
        {
            continue;
        }

        projectile->Sphere->EmptyAccumulator(projectile->ForceAccumulatorIndex);

        // Apply the effects of aerodynamic forces (Drag + Magnus Effect) to a shape given its area
        Aerodynamics::ApplyToBody(
            projectile->Sphere,
            projectile->ForceAccumulatorIndex,
            std::static_pointer_cast<AerodynamicParametersBase<SphericalAeroParameters>>(
                projectileData->AerodynamicParameters),
            true, true
        );
        if (projectiles_on_ground.count(projectile->Sphere.get()) > 0)
        {
            const auto v = projectile->Sphere->GetLinVel();
            const auto w = projectile->Sphere->GetAngVelParent();
            projectile->Sphere->SetLinVel(v * (1.0 - kGroundLinearDampPerStep));
            projectile->Sphere->SetAngVelParent(w * (1.0 - kGroundAngularDampPerStep));
        }
        if (projectile->ShouldLogTrajectory &&
            (trajectory_step_counter % kTrajectoryDecimate) == 0) {
            projectile->TrajectoryPoints.push_back(projectile->Sphere->GetPos());
            if (projectile->TrajectoryPoints.size() > kMaxTrajectoryPoints) {
                projectile->TrajectoryPoints.erase(projectile->TrajectoryPoints.begin());
            }
        }
    }

    trajectory_step_counter++;
}

void SphericalProjectileSpawner::DoRenderStep(irr::video::IVideoDriver* drv)
{
    for (const auto& projectile : projectiles)
    {
        if (!projectile->ShouldLogTrajectory) {
            continue;
        }
        const auto color = projectile->InGoal
                               ? irr::video::SColor(255, 60, 220, 60)
                               : (projectile->Removed
                                      ? irr::video::SColor(255, 255, 60, 60)
                                      : irr::video::SColor(255, 255, 200, 0));
            DrawDebugPolyline(drv, projectile->TrajectoryPoints, color, true);
    
    }
}

void SphericalProjectileSpawner::emptyAllAccumlators()
{
    for (const auto& projectile : projectiles)
    {
        projectile->Sphere->EmptyAccumulator(projectile->ForceAccumulatorIndex);
    }
}
void SphericalProjectileSpawner::SweepShots(const chrono::ChVector3d& p0,
                                           float minAngleDeg,
                                           float maxAngleDeg,
                                           float angleStepDeg,
                                           float minSpeed,
                                           float maxSpeed,
                                           float speedStep,
                                           const chrono::ChVector3d& robotVelocity,
                                           const chrono::ChVector3d& sweepAxis)
{
        const chrono::ChVector3d w0(0, 0, 0);

        chrono::ChVector3d forward = sweepAxis;
        if (forward.Length() <= 1e-9) {
            forward = chrono::ChVector3d(0, 1, 0);
        }
        forward.Normalize();

        chrono::ChVector3d up(0, 0, 1);
        if (std::abs(forward.Dot(up)) > 0.98) {
            up = chrono::ChVector3d(1, 0, 0);
        }
        chrono::ChVector3d right = Vcross(up, forward);
        if (right.Length() <= 1e-9) {
            right = chrono::ChVector3d(1, 0, 0);
        }
        right.Normalize();

        for (float speed = minSpeed; speed <= maxSpeed + 1e-6f; speed += speedStep)
        {
            for (float angleDeg = minAngleDeg; angleDeg <= maxAngleDeg + 1e-6f; angleDeg += angleStepDeg)
            {
                const float angleRad = angleDeg * chrono::CH_PI / 180.0f;

                const chrono::ChVector3d dir =
                    (forward * std::cos(angleRad)) + (up * std::sin(angleRad));
                const chrono::ChVector3d v0 = dir * speed;

                this->Spawn(p0, v0 + robotVelocity, w0, true);
            }
        }
    }
