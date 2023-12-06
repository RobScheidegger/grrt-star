#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "config/solver_problem.h"
#include "graphs/roadmap.h"
#include "robots/robot.h"
#include "robots/robot_factory.h"

namespace grrt {

    /// @brief Configuration for the solver.
    struct SolverConfig {
        typedef std::shared_ptr<SolverConfig> SharedPtr;

        std::unordered_map<std::string, Roadmap::SharedPtr> roadmaps;
        std::vector<IRobot::SharedPtr> robots;
        std::vector<SolverProblem> problems;

        RobotFactory::SharedPtr robotFactory;
        bool useMPI = false;
    };
}  // namespace grrt