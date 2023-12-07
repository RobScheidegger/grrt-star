#pragma once

#include "config/solver_config.h"
#include "pkgs/rapidyaml.hpp"
#include "result.h"
#include "solver/solver.h"

namespace grrt {

    /// @brief Static parser for the solver config.
    class SolverConfigParser {
       public:
        static SolverConfig::SharedPtr parse(const std::string& filePath);
        static Result printSolution(std::ostream& stream, const SolverConfig::SharedPtr& config,
                                    const SolverResult::SharedPtr& solution);
        static Result parseSolution(const std::string& fileName, SolverResult::SharedPtr& solution);
    };
}  // namespace grrt