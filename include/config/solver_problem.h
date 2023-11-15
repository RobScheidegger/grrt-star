#pragma once

#include <string>

#include "state/system_state.h"

namespace grrt {

    /// @brief Represents a specific motion planning problem to be solved, with a specified stard and end state.
    struct SolverProblem {
        /// @brief The semantic name of the problem, for logging purposes.
        std::string name;
        /// @brief The starting state of the problem.
        SystemState::SharedPtr start;
        /// @brief The goal state of the problem.
        SystemState::SharedPtr goal;
    };
}  // namespace grrt