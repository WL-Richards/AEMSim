#pragma once
#include <chrono/physics/ChSystemNSC.h>
#include <unordered_map>
#include <unordered_set>
#include <limits>

#include "./SphericalProjectile.h"
#include "../../utility/interfaces/Stepable.h"


namespace chrono
{
    class ChSystemNSC;
}

class SphericalProjectileData;

class SphericalProjectileSpawner : public Stepable
{
    
public:
    struct Snapshot {
        struct ProjectileSnapshot {
            bool removed = false;
            bool in_goal = false;
            bool fixed = false;
            size_t trajectory_size = 0;
        };

        int trajectory_step_counter = 0;
        std::vector<ProjectileSnapshot> projectiles;
    };

    SphericalProjectileSpawner(
        std::shared_ptr<chrono::ChSystem> physicalSystem,      // Physical system that the objects are being spawned within
        std::shared_ptr<SphericalProjectileData> projectileData   // Data about the projectile we are spawning
    );


    bool Spawn(
        chrono::ChVector3d location,
        chrono::ChVector3d initialVelocity,
        chrono::ChVector3d initialSpin,
        bool enableCollision = false,
        chrono::ChColor projectileColor = chrono::ChColor(1.0f, 1.0f, 0.0f),
        bool logTrajectory = true
    );

    void DoPhysicsStep() override;
    void DoRenderStep(irr::video::IVideoDriver* drv) override;
    void AddTriggerBody(const std::shared_ptr<chrono::ChBody>& body);
    void AddNonRemovalBody(const std::shared_ptr<chrono::ChBody>& body);
    void AddFunnelBody(const std::shared_ptr<chrono::ChBody>& body,
                       const chrono::ChVector3d& hub_center,
                       double funnel_halfway_z);
    void CaptureSnapshot(Snapshot& out_snapshot) const;
    void RestoreSnapshot(const Snapshot& snapshot);
    void SetEnableFreezeOnContact(bool enabled);
    void AddGroundBody(const std::shared_ptr<chrono::ChBody>& body);
    void SetFunnelGoalZ(double goal_z);
    void ConfigureProximityMonitor(const chrono::ChVector3d& point,
                                   double radius,
                                   double time_threshold,
                                   double timestep);

    
    void SweepShots(
        const chrono::ChVector3d& p0,
        float minAngleDeg,
        float maxAngleDeg,
        float angleStepDeg,
        float minSpeed,
        float maxSpeed,
        float speedStep,
        const chrono::ChVector3d& robotVelocity,
        const chrono::ChVector3d& sweepAxis = chrono::ChVector3d(0, 1, 0)
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
        double halfway_z;
    };
    std::unordered_map<chrono::ChBody*, FunnelInfo> funnel_infos;
    std::unordered_set<chrono::ChBody*> ground_bodies;

    bool funnel_goal_enabled = false;
    double funnel_goal_z = std::numeric_limits<double>::infinity();

    bool enable_freeze_on_contact = true;

    bool monitor_enabled = false;
    bool monitor_fired = false;
    chrono::ChVector3d monitor_point;
    double monitor_radius = 0.0;
    double monitor_time_threshold = 0.0;
    double monitor_timestep = 0.0;
    std::unordered_map<chrono::ChBody*, double> monitor_time_in_zone;

    static constexpr double kGroundLinearDampPerStep = 0.0003;
    static constexpr double kGroundAngularDampPerStep = 0.001;

    int trajectory_step_counter = 0;
    static constexpr int kTrajectoryDecimate = 5;
    static constexpr size_t kMaxTrajectoryPoints = 1500;

    void emptyAllAccumlators();
};
