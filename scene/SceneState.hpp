#pragma once

#include <memory>
#include <vector>

namespace chrono {
    class ChSystem;
    class ChBody;
}

class SphericalProjectileSpawner;

struct SceneState {
    std::shared_ptr<chrono::ChSystem> sys;
    std::shared_ptr<SphericalProjectileSpawner> spawner;
    std::shared_ptr<chrono::ChBody> floor;
    std::vector<std::shared_ptr<chrono::ChBody>> funnel_bodies;
};
