#include <iostream>

#include "gtest/gtest.h"

#include "config/solver_config.h"
#include "graphs/roadmap.h"

using namespace grrt;

/// This test checks that in the situation where a roadmap has two vertices, each drone starts on one of them
/// and ends at the other, that the solver says that there is no possible solution.
TEST(Solver, TestHeadOnCollision) {

    Roadmap::SharedPtr roadmap = std::make_shared<Roadmap>("Test Roadmap", RobotType::DRONE);
}