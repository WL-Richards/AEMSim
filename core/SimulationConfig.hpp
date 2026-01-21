#pragma once

namespace SimConfig {
    // Physics timestep (seconds)
    constexpr double TIMESTEP = 1e-3;

    // Visualization settings
    constexpr bool USE_VISUAL = true;
    constexpr int STEPS_NO_VISUAL = 5000;

    // Scene settings
    constexpr bool HAS_HUB = true;

    // Time-travel/snapshot settings
    constexpr size_t MAX_SNAPSHOT_HISTORY = 600;

    // Timing/profiling
    constexpr int TIMING_REPORT_EVERY = 120;
}
