#pragma once
#include <chrono/physics/ChSystemNSC.h>

#include "../../utility/interfaces/Stepable.h"

class SphericalProjectile;

namespace chrono
{
    class ChSystemNSC;
}

class SphericalProjectileData;

class SphericalProjectileSpawner : public Stepable
{
    
public:
    SphericalProjectileSpawner(
        std::shared_ptr<chrono::ChSystemNSC> physicalSystem,      // Physical system that the objects are being spawned within
        std::shared_ptr<SphericalProjectileData> projectileData   // Data about the projectile we are spawning
    );


    bool Spawn(
        chrono::ChVector3d location,
        chrono::ChVector3d initialVelocity,
        chrono::ChVector3d initialSpin,
        bool enableCollision = false,
        chrono::ChColor projectileColor = chrono::ChColor(1.0f, 1.0f, 0.0f)
    );

    void DoPhysicsStep() override;
    void DoRenderStep(irr::video::IVideoDriver* drv) override;


    virtual ~SphericalProjectileSpawner() = default;
    
private:
    std::shared_ptr<chrono::ChSystemNSC> physicalSystem;
    std::shared_ptr<SphericalProjectileData> projectileData;
    
    std::vector<std::unique_ptr<SphericalProjectile>> projectiles;

    void emptyAllAccumlators();
};
