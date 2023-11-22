#pragma once

#include "config/solver_config.h"
#include "pkgs/rapidyaml.hpp"

namespace grrt {

    /// @brief Static parser for the solver config.
    class SolverConfigParser {
       public:
        static SolverConfig::SharedPtr parse(const std::string& filePath);
    };
}  // namespace grrt