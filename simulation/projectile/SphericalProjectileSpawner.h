#pragma once
#include <chrono/physics/ChSystemNSC.h>
#include <unordered_map>
#include <unordered_set>

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
        std::shared_ptr<chrono::ChSystem> physicalSystem,      // Physical system that the objects are being spawned within
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
    void AddTriggerBody(const std::shared_ptr<chrono::ChBody>& body);
    void AddNonRemovalBody(const std::shared_ptr<chrono::ChBody>& body);
    void AddFunnelBody(const std::shared_ptr<chrono::ChBody>& body, const chrono::ChVector3d& hub_center);


    virtual ~SphericalProjectileSpawner() = default;
    
    void SweepShots(
        const chrono::ChVector3d& p0,
        float minAngleDeg,
        float maxAngleDeg,
        float angleStepDeg,
        float minSpeed,
        float maxSpeed,
        float speedStep,
        const chrono::ChVector3d& robotVelocity
    );

    
private:
    std::shared_ptr<chrono::ChSystem> physicalSystem;
    std::shared_ptr<SphericalProjectileData> projectileData;
    
    std::vector<std::unique_ptr<SphericalProjectile>> projectiles;
    std::unordered_set<chrono::ChBody*> trigger_bodies;
    std::unordered_set<chrono::ChBody*> non_removal_bodies;
    struct FunnelInfo {
        chrono::ChVector3d hub_center;
        double radius;
    };
    std::unordered_map<chrono::ChBody*, FunnelInfo> funnel_infos;

    void emptyAllAccumlators();
};
