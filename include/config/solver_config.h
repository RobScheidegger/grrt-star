#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "config/solver_problem.h"
#include "graphs/roadmap.h"
#include "robots/robot.h"

namespace grrt {

    struct SolverConfig {
        typedef std::shared_ptr<SolverConfig> SharedPtr;

        std::unordered_map<std::string, Roadmap::SharedPtr> roadmaps;
        std::vector<IRobot::SharedPtr> robots;
        std::vector<SolverProblem> problems;
    };
}  // namespace grrt