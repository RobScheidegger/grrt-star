#include "pkgs/rapidyaml.hpp"

namespace grrt {
    class SolverConfigParser {
        static SolverConfig::SharedPtr parse(const std::string& filePath);
    };
}  // namespace grrt