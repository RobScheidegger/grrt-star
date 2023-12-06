#pragma once

#include <memory>
#include <vector>

#include "graphs/search_vertex.h"

namespace grrt {

    /// @brief The result of a solver, including:
    ///     - Whether the problem was solved successfully.
    ///     - The solution path, if any.
    struct SolverResult {
        typedef std::shared_ptr<SolverResult> SharedPtr;

        SolverResult(bool success) : success(success) {}

        /// @brief The solution path, if it was found.
        std::vector<SearchVertex::SharedPtr> path;
        /// @brief Whether the problem was solved successfully.
        bool success;
        /// @brief Cost of the solution, if found.
        double cost;

        static SolverResult fail() { return SolverResult(false); }
    };
}  // namespace grrt