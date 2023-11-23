#pragma once

#include "result.h"

namespace grrt {
    /// @brief The possible robot types that we want to use in our system.
    enum RobotType { DRONE };

    static Result parseRobotType(const std::string& robotTypeStr, RobotType& robotType);
}  // namespace grrt