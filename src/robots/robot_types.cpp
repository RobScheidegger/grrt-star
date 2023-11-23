#include "robot_types.h"

using namespace grrt;

Result grrt::parseRobotType(const std::string& robotTypeStr, RobotType& robotType) {

    if (robotTypeStr == "drone") {
        robotType = RobotType::DRONE;
        return Result::Ok();
    }
    return Result::Error("Unknown robot type: " + robotTypeStr);
}