#pragma once
#include <chrono/physics/ChSystemNSC.h>
#include <chrono_multicore/physics/ChSystemMulticore.h>

class PhysicalSystemFactory
{
public:
    
    /**
     * Create a new physical system with preconfigured properties to match that of the realworld
     * @return Newly created physics system with 'Realworld' configuration
     */
    static std::shared_ptr<chrono::ChSystemNSC> createNonSmoothContactSystem_Realworld()
    {
        std::shared_ptr<chrono::ChSystemNSC> sys = chrono_types::make_shared<chrono::ChSystemNSC>();
        
        // 1. Explicitly enable the Bullet collision system
        sys->SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
        
        sys->SetGravitationalAcceleration(chrono::ChVector3d(0, 0, -9.981));
        return sys;
    }


    /**
     * Create a new physical system with preconfigured properties to match that of the realworld
     * @return Newly created physics system with 'Realworld' configuration
     */
    static std::shared_ptr<chrono::ChSystemMulticoreNSC> createNonSmoothContactSystemMulticore_Realworld()
    {
        std::shared_ptr<chrono::ChSystemMulticoreNSC> sys = chrono_types::make_shared<chrono::ChSystemMulticoreNSC>();
        
        double gravity = 9.81;
        double time_step = 1e-3;

        uint max_iteration = 30;
        chrono::real tolerance = 1e-3;
        
        sys->SetNumThreads(8);
        
        sys->SetCollisionSystemType(chrono::ChCollisionSystem::Type::MULTICORE);

        sys->GetSettings()->solver.solver_mode = chrono::SolverMode::SLIDING;
        sys->GetSettings()->solver.max_iteration_normal = max_iteration / 3;
        sys->GetSettings()->solver.max_iteration_sliding = max_iteration / 3;
        sys->GetSettings()->solver.max_iteration_spinning = 0;
        sys->GetSettings()->solver.max_iteration_bilateral = max_iteration / 3;
        sys->GetSettings()->solver.tolerance = tolerance;
        sys->GetSettings()->solver.alpha = 0;
        sys->GetSettings()->solver.contact_recovery_speed = 10000;
        sys->ChangeSolverType(chrono::SolverType::APGD);
        sys->GetSettings()->collision.narrowphase_algorithm = chrono::ChNarrowphase::Algorithm::HYBRID;

        sys->GetSettings()->collision.collision_envelope = 0.01;
        sys->GetSettings()->collision.bins_per_axis = chrono::vec3(10, 10, 10);
        
        sys->SetGravitationalAcceleration(chrono::ChVector3d(0, 0, -gravity));
        return sys;
    }
};
