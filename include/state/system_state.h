#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "state/robot_state.h"

namespace grrt {

    /// @brief Represents an entire system state of all robots, in each of their respective roadmaps.
    /// This is a lightweight wrapper around a simple vector of roadmap state IDs.
    class SystemState {
       public:
        typedef std::shared_ptr<SystemState> SharedPtr;

        SystemState(const uint64_t numRobots) : roadmapStates(numRobots) {}

        SystemState(const std::vector<RoadmapStateId>& roadmapStates) : roadmapStates(roadmapStates) {}

        std::vector<RoadmapStateId> roadmapStates;
    };
}  // namespace grrt