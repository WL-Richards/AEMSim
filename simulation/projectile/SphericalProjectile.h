#pragma once
#include <chrono/physics/ChBodyEasy.h>

#include "data/SphericalProjectileData.hpp"


class SphericalProjectile
{
public:

    // Points to some data about this projectile
    std::shared_ptr<SphericalProjectileData> Data;
    
    // The sphere object being used in the system
    std::shared_ptr<chrono::ChBodyEasySphere> Sphere;

    // The accumulator index that is associated with this projectile
    unsigned int ForceAccumulatorIndex;

    // The velocity at which this projectile was launched
    chrono::ChVector3d LaunchVector;

    // Points that make up this objects trajectory and should we log them for this trajectory
    std::vector<chrono::ChVector3d> TrajectoryPoints;
    bool ShouldLogTrajectory = false;
    
    SphericalProjectile(
        const std::shared_ptr<SphericalProjectileData>& data,
        const std::shared_ptr<chrono::ChBodyEasySphere>& sphere,
        const chrono::ChVector3d& launchVector,
        const unsigned int accumIndex,
        bool logTrajectory = false) : Data(data), Sphere(sphere), ForceAccumulatorIndex(accumIndex), LaunchVector(launchVector)
    {
        ShouldLogTrajectory = logTrajectory;

        if(ShouldLogTrajectory)
        {
            TrajectoryPoints.reserve(20000);
        }

    }

    virtual ~SphericalProjectile() = default;
    
};
