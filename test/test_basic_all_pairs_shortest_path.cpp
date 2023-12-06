#include <iostream>

#include "gtest/gtest.h"

#include "constants.h"
#include "graphs/roadmap.h"

using namespace grrt;

TEST() {
    Roadmap roadmap;
    roadmap.addVertex("A", std::make_shared(grrt::RobotState)
}