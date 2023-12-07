#include <iostream>

#include "gtest/gtest.h"

#include "constants.h"
#include "graphs/roadmap.h"
#include "robots/drone/drone_gpu.h"
#include "robots/robot_factory.h"

using namespace grrt;

TEST(PointCloudVoxelsGPU, TestHeadOnCollision) {
    //
    // Checks that two voxels intersecting each other actually intersect
    //

    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD_GPU);
    Roadmap::SharedPtr roadmap = factory->makeRoadmap("Test Drone Roadmap", RobotType::DRONE);
    auto v1 = roadmap->addVertex("v1", std::make_shared<DroneGPUState>(Point(-1, 0, 0), 1.0));
    auto v2 = roadmap->addVertex("v2", std::make_shared<DroneGPUState>(Point(1, 0, 0), 1.0));

    auto d1 = roadmap->addDart(v1, v2);
    auto d2 = roadmap->addDart(v2, v1);

    DroneGPU drone1(1, "Drone 1", roadmap);
    DroneGPU drone2(2, "Drone 2", roadmap);

    auto voxel1 = drone1.getSweptVoxel(d1);
    auto voxel2 = drone2.getSweptVoxel(d2);

    VoxelManager::SharedPtr voxel_manager = factory->makeVoxelManager();
    EXPECT_TRUE(voxel_manager->intersect(voxel1, voxel2));
}

TEST(PointCloudVoxelsGPU, TestParallelNearbySameRoadmap) {
    //
    // Checks that two voxels that are parallel but nearby do not intersect
    //

    const float ROBOT_RADIUS = 1.0;

    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD_GPU);
    Roadmap::SharedPtr roadmap = factory->makeRoadmap("Test Drone Roadmap", RobotType::DRONE);

    auto v1_left = roadmap->addVertex("v1_left", std::make_shared<DroneGPUState>(Point(0, 0, -1), ROBOT_RADIUS));
    auto v1_right = roadmap->addVertex("v1_right", std::make_shared<DroneGPUState>(Point(0, 0, 1), ROBOT_RADIUS));
    // essentially 2.1
    auto v2_left = roadmap->addVertex(
        "v2_left",
        std::make_shared<DroneGPUState>(Point(ROBOT_RADIUS * 2 + VOXEL_RESOLUTION * 2, 0, -1), ROBOT_RADIUS));
    auto v2_right = roadmap->addVertex(
        "v2_right",
        std::make_shared<DroneGPUState>(Point(ROBOT_RADIUS * 2 + VOXEL_RESOLUTION * 2, 0, 1), ROBOT_RADIUS));

    auto d1 = roadmap->addDart(v1_left, v1_right);
    auto d2 = roadmap->addDart(v2_left, v2_right);

    DroneGPU drone1(1, "Drone 1", roadmap);
    DroneGPU drone2(2, "Drone 2", roadmap);

    auto voxel1 = drone1.getSweptVoxel(d1);
    auto voxel2 = drone2.getSweptVoxel(d2);

    VoxelManager::SharedPtr voxel_manager = factory->makeVoxelManager();
    EXPECT_FALSE(voxel_manager->intersect(voxel1, voxel2));
}