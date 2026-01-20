#include "SphericalProjectileSpawner.h"

#include <irrlicht.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChContactContainer.h>
#include <chrono/physics/ChBody.h>
#include <cmath>
#include <unordered_set>

#include "SphericalProjectile.h"
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
    const chrono::ChColor projectileColor
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
                                                            ball->AddAccumulator());
    projectile->TrajectoryPoints.clear();
    projectile->TrajectoryPoints.push_back(ball->GetPos());
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
                                               const chrono::ChVector3d& hub_center)
{
    if (!body) return;
    const auto diff = body->GetPos() - hub_center;
    const double len = std::sqrt(diff.x() * diff.x() + diff.y() * diff.y() + diff.z() * diff.z());
    if (len <= 1e-9) return;
    funnel_infos[body.get()] = FunnelInfo{hub_center, len};
}

void SphericalProjectileSpawner::DoPhysicsStep()
{
    std::unordered_set<chrono::ChBody*> projectiles_in_contact;
    std::unordered_set<chrono::ChBody*> projectiles_in_goal;
    if (physicalSystem && physicalSystem->GetContactContainer())
    {
        struct ContactReporter : chrono::ChContactContainer::ReportContactCallback
        {
            const std::unordered_set<chrono::ChBody*>& targets;
            const std::unordered_set<chrono::ChBody*>& triggers;
            const std::unordered_set<chrono::ChBody*>& non_removal;
            const std::unordered_map<chrono::ChBody*, FunnelInfo>& funnel_infos;
            std::unordered_set<chrono::ChBody*>& hit_any;
            std::unordered_set<chrono::ChBody*>& hit_goal;

            ContactReporter(const std::unordered_set<chrono::ChBody*>& targets_in,
                            const std::unordered_set<chrono::ChBody*>& triggers_in,
                            const std::unordered_set<chrono::ChBody*>& non_removal_in,
                            const std::unordered_map<chrono::ChBody*, FunnelInfo>& funnel_infos_in,
                            std::unordered_set<chrono::ChBody*>& hit_any_in,
                            std::unordered_set<chrono::ChBody*>& hit_goal_in)
                : targets(targets_in),
                  triggers(triggers_in),
                  non_removal(non_removal_in),
                  funnel_infos(funnel_infos_in),
                  hit_any(hit_any_in),
                  hit_goal(hit_goal_in)
            {
            }

            bool ShouldCountContact(chrono::ChBody* other_body, const chrono::ChVector3d& contact_point) const
            {
                if (!other_body) return false;
                if (non_removal.count(other_body)) return false;
                const auto it = funnel_infos.find(other_body);
                if (it == funnel_infos.end()) return true;

                const auto rel = contact_point - it->second.hub_center;
                const double dist = std::sqrt(rel.x() * rel.x() + rel.y() * rel.y() + rel.z() * rel.z());
                const double epsilon = 1e-4;
                return dist >= (it->second.radius + epsilon);
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
                    if (bodyB && triggers.count(bodyB))
                    {
                        hit_goal.insert(bodyA);
                    }
                    if (bodyB && !triggers.count(bodyB) && ShouldCountContact(bodyB, pB))
                    {
                        hit_any.insert(bodyA);
                    }
                }
                if (bodyB && targets.count(bodyB))
                {
                    if (bodyA && triggers.count(bodyA))
                    {
                        hit_goal.insert(bodyB);
                    }
                    if (bodyA && !triggers.count(bodyA) && ShouldCountContact(bodyA, pA))
                    {
                        hit_any.insert(bodyB);
                    }
                }
                return true;
            }
        };

        std::unordered_set<chrono::ChBody*> projectile_bodies;
        projectile_bodies.reserve(projectiles.size());
        for (const auto& projectile : projectiles)
        {
            projectile_bodies.insert(projectile->Sphere.get());
        }

        auto reporter = chrono_types::make_shared<ContactReporter>(
            projectile_bodies, trigger_bodies, non_removal_bodies, funnel_infos,
            projectiles_in_contact, projectiles_in_goal);
        physicalSystem->GetContactContainer()->ReportAllContacts(reporter);
    }

    for (const auto& projectile : projectiles)
    {
        if (!projectile->Removed &&
            projectiles_in_goal.count(projectile->Sphere.get()) > 0 &&
            projectile->Sphere->GetLinVel().z() < 0)
        {
            projectile->Sphere->SetFixed(true);
            projectile->Sphere->SetLinVel(chrono::ChVector3d(0, 0, 0));
            projectile->Sphere->SetAngVelParent(chrono::ChVector3d(0, 0, 0));
            projectile->Removed = true;
            projectile->InGoal = true;
        }

        if (!projectile->Removed &&
            projectiles_in_contact.count(projectile->Sphere.get()) > 0)
        {
            projectile->Sphere->SetFixed(true);
            projectile->Sphere->SetLinVel(chrono::ChVector3d(0, 0, 0));
            projectile->Sphere->SetAngVelParent(chrono::ChVector3d(0, 0, 0));
            projectile->Removed = true;
        }

        if (!projectile->Removed &&
            PhysicsHelper::HitGroundSphere(projectile->Sphere,
                                           projectile->Data->AerodynamicParameters->RadiusM))
        {
            // Multicore collision removal isn't implemented; disable instead.
            //projectile->Sphere->EnableCollision(false);
            projectile->Sphere->SetFixed(true);
            projectile->Sphere->SetLinVel(chrono::ChVector3d(0, 0, 0));
            projectile->Sphere->SetAngVelParent(chrono::ChVector3d(0, 0, 0));
            projectile->Removed = true;
        }

        if (projectile->Removed)
        {
            continue;
        }

        projectile->Sphere->EmptyAccumulator(projectile->ForceAccumulatorIndex);

        // Apply the effects of aerodynamic forces (Drag + Magnus Effect) to a shape given its area
        PhysicsHelper::ApplyAerodynamicsToArea(
            projectile->Sphere,
            projectile->ForceAccumulatorIndex,
            std::static_pointer_cast<AerodynamicParametersBase<SphericalAeroParameters>>(
                projectileData->AerodynamicParameters),
            true, true
        );
        std::cout << projectile->Sphere->GetPos().y() << "," << projectile->Sphere->GetAngVelLocal().Length() << "\n";
        projectile->TrajectoryPoints.push_back(projectile->Sphere->GetPos());
    }
}

void SphericalProjectileSpawner::DoRenderStep(irr::video::IVideoDriver* drv)
{
    for (const auto& projectile : projectiles)
    {
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
                                           const chrono::ChVector3d& robotVelocity)
{
        const chrono::ChVector3d w0(0, 0, 0);
    
        for (float speed = minSpeed; speed <= maxSpeed + 1e-6f; speed += speedStep)
        {
            for (float angleDeg = minAngleDeg; angleDeg <= maxAngleDeg + 1e-6f; angleDeg += angleStepDeg)
            {
                const float angleRad = angleDeg * chrono::CH_PI / 180.0f;
    
                chrono::ChVector3d v0(
                    std::cos(angleRad) * speed,
                    0.0,
                    std::sin(angleRad) * speed
                );
    
                this->Spawn(p0, v0 + robotVelocity, w0, true);
            }
        }
    }
