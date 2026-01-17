#include "SphericalProjectileSpawner.h"

#include <irrlicht.h>
#include <chrono/physics/ChBodyEasy.h>

#include "SphericalProjectile.h"
#include "../../utility/DebugDrawer.h"
#include "../../utility/PhysicsHelper.h"
#include "data/SphericalProjectileData.hpp"


SphericalProjectileSpawner::SphericalProjectileSpawner(std::shared_ptr<chrono::ChSystem> physicalSystem, std::shared_ptr<SphericalProjectileData> projectileData)
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
    if (auto vm = ball->GetVisualModel()) {
        if (vm->GetNumShapes() > 0) {
            vm->GetShape(0)->SetColor(projectileColor);  // yellow tint
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
  

  

    auto projectile = std::make_unique<SphericalProjectile>(projectileData, ball, initialVelocity, ball->AddAccumulator());
    projectile->TrajectoryPoints.clear();
    projectile->TrajectoryPoints.push_back(ball->GetPos());
    projectiles.push_back(std::move(projectile));
    return true;
}

void SphericalProjectileSpawner::DoPhysicsStep()
{
    for(const auto& projectile : projectiles)
    {
        if (!projectile->Removed &&
            PhysicsHelper::HitGroundSphere(projectile->Sphere,
                                           projectile->Data->AerodynamicParameters->RadiusM)) {
            // Multicore collision removal isn't implemented; disable instead.
            projectile->Sphere->EnableCollision(false);
            //projectile->Sphere->SetFixed(true);
            projectile->Sphere->SetLinVel(chrono::ChVector3d(0, 0, 0));
            projectile->Sphere->SetAngVelParent(chrono::ChVector3d(0, 0, 0));
            projectile->Removed = true;
        }

        if (projectile->Removed) {
            continue;
        }

        projectile->Sphere->EmptyAccumulator(projectile->ForceAccumulatorIndex);
        
        // Apply the effects of aerodynamic forces (Drag + Magnus Effect) to a shape given its area
        PhysicsHelper::ApplyAerodynamicsToArea(
            projectile->Sphere,
            projectile->ForceAccumulatorIndex,
            std::static_pointer_cast<AerodynamicParametersBase<SphericalAeroParameters>>(projectileData->AerodynamicParameters),
            true, true
        );
        std::cout << projectile->Sphere->GetPos().y() << "," << projectile->Sphere->GetAngVelLocal().Length() << "\n";
        projectile->TrajectoryPoints.push_back(projectile->Sphere->GetPos());

        
    }
}

void SphericalProjectileSpawner::DoRenderStep(irr::video::IVideoDriver* drv)
{
    for(const auto& projectile : projectiles)
    {
        const auto color = projectile->Removed
            ? irr::video::SColor(255, 255, 60, 60)
            : irr::video::SColor(255, 255, 200, 0);
        DrawDebugPolyline(drv, projectile->TrajectoryPoints, color, true);
    }
}

void SphericalProjectileSpawner::emptyAllAccumlators()
{
    for(const auto& projectile : projectiles)
    {
        projectile->Sphere->EmptyAccumulator(projectile->ForceAccumulatorIndex);
    }
}
