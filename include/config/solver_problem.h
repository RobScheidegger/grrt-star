#pragma once

#include <string>

#include "graphs/search_vertex.h"

namespace grrt {

    /// @brief Represents a specific motion planning problem to be solved, with a specified stard and end state.
    struct SolverProblem {
        SolverProblem(const std::string& name, const SearchVertex::SharedPtr& start,
                      const SearchVertex::SharedPtr& goal)
            : name(name), start(start), goal(goal) {}

        /// @brief The semantic name of the problem, for logging purposes.
        std::string name;
        /// @brief The starting state of the problem.
        SearchVertex::SharedPtr start;
        /// @brief The goal state of the problem.
        SearchVertex::SharedPtr goal;
    };
}  // namespace grrt