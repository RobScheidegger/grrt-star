#include <gtest/gtest.h>
#include <iostream>
#include <memory>

#include "config/solver_config.h"
#include "graphs/search_vertex.h"
#include "gtest/gtest.h"

#include "constants.h"
#include "graphs/roadmap.h"
#include "robots/drone/drone.hpp"
#include "robots/robot_factory.h"
#include "solver/solver.h"

using namespace grrt;

TEST(SolverExpansion, TestBasicRRTExpansion1) {
    //
    // Checks that two voxels intersecting each other actually intersect
    //

    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD);
    Roadmap::SharedPtr roadmap = factory->makeRoadmap("Test Drone Roadmap", RobotType::DRONE);
    auto v1 = roadmap->addVertex("v1", std::make_shared<DroneState>(Point(-1, 0, 0), 1.0));
    auto v2 = roadmap->addVertex("v2", std::make_shared<DroneState>(Point(1, 0, 0), 1.0));

    auto start_vertex = std::make_shared<SearchVertex>(std::vector{v1->m_id, v2->m_id});
    auto end_vertex = std::make_shared<SearchVertex>(std::vector{v1->m_id, v2->m_id});

    auto d1 = roadmap->addDart(v1, v2);
    auto d2 = roadmap->addDart(v2, v1);

    Drone drone1(1, "Drone 1", roadmap);
    Drone drone2(2, "Drone 2", roadmap);

    auto voxel1 = drone1.getSweptVoxel(d1);
    auto voxel2 = drone2.getSweptVoxel(d2);

    VoxelManager::SharedPtr voxel_manager = factory->makeVoxelManager();
    EXPECT_TRUE(voxel_manager->intersect(voxel1, voxel2));

    SolverConfig::SharedPtr config = std::make_shared<SolverConfig>();
    config->roadmaps["Test Drone Roadmap"] = roadmap;
    config->robots.push_back(std::make_shared<Drone>(1, "Drone 1", roadmap));
    config->robots.push_back(std::make_shared<Drone>(2, "Drone 2", roadmap));
    config->problems.push_back(SolverProblem("Test Drone Roadmap", start_vertex, end_vertex));

    Solver solver = Solver(config);
    auto solutions = solver.solve();

    EXPECT_FALSE(solutions->at("Test Drone Roadmap").success);
}

TEST(SolverExpansion, TestBasicRRTExpansion2) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD);
    Roadmap::SharedPtr roadmap = factory->makeRoadmap("Test Drone Roadmap", RobotType::DRONE);
    auto v1 = roadmap->addVertex("v1", std::make_shared<DroneState>(Point(-1, 0, 0), 1.0));
    auto v2 = roadmap->addVertex("v2", std::make_shared<DroneState>(Point(1, 0, 0), 1.0));
    auto v3 = roadmap->addVertex("v3", std::make_shared<DroneState>(Point(0, 1, 0), 1.0));

    auto start_vertex = std::make_shared<SearchVertex>(std::vector{v1->m_id, v2->m_id});
    auto end_vertex = std::make_shared<SearchVertex>(std::vector{v1->m_id, v2->m_id});

    auto d1 = roadmap->addDart(v1, v2);
    auto d2 = roadmap->addDart(v2, v1);
    auto d3 = roadmap->addDart(v2, v3);
    auto d4 = roadmap->addDart(v3, v2);
    auto d5 = roadmap->addDart(v1, v3);
    auto d6 = roadmap->addDart(v3, v1);

    Drone drone1(1, "Drone 1", roadmap);
    Drone drone2(2, "Drone 2", roadmap);

    SolverConfig::SharedPtr config = std::make_shared<SolverConfig>();
    config->roadmaps["Test Drone Roadmap"] = roadmap;
    config->robots.push_back(std::make_shared<Drone>(1, "Drone 1", roadmap));
    config->robots.push_back(std::make_shared<Drone>(2, "Drone 2", roadmap));
    config->problems.push_back(SolverProblem("Test Drone Roadmap", start_vertex, end_vertex));

    Solver solver = Solver(config);
    auto solutions = solver.solve();

    EXPECT_TRUE(solutions->at("Test Drone Roadmap").success);
}