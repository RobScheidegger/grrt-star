#include <iostream>

#include "gtest/gtest.h"

#include "constants.h"
#include "graphs/roadmap.h"
#include "robots/drone/drone.hpp"
#include "robots/drone/drone_gpu.h"
#include "robots/robot_factory.h"

using namespace grrt;

template <typename TRobotType>
void testHeadOnCollision(const RobotFactory::SharedPtr& factory) {
    //
    // Checks that two voxels intersecting each other actually intersect
    //

    Roadmap::SharedPtr roadmap = factory->makeRoadmap("Test Drone Roadmap", RobotType::DRONE);
    auto v1 = roadmap->addVertex("v1", std::make_shared<DroneState>(Point(-1, 0, 0), 1.0));
    auto v2 = roadmap->addVertex("v2", std::make_shared<DroneState>(Point(1, 0, 0), 1.0));

    auto d1 = roadmap->addDart(v1, v2);
    auto d2 = roadmap->addDart(v2, v1);

    TRobotType drone1(1, "Drone 1", roadmap);
    TRobotType drone2(2, "Drone 2", roadmap);

    auto voxel1 = drone1.getSweptVoxel(d1);
    auto voxel2 = drone2.getSweptVoxel(d2);

    VoxelManager::SharedPtr voxel_manager = factory->makeVoxelManager();
    EXPECT_TRUE(voxel_manager->intersect(voxel1, voxel2));
}

template <typename TRobotType>
void testHeadOnCollisionStationary(const RobotFactory::SharedPtr& factory) {
    //
    // Checks that two voxels intersecting each other actually intersect
    //

    Roadmap::SharedPtr roadmap = factory->makeRoadmap("Test Drone Roadmap", RobotType::DRONE);
    auto v1 = roadmap->addVertex("v1", std::make_shared<DroneState>(Point(-1, 0, 0), 1.0));
    auto v2 = roadmap->addVertex("v2", std::make_shared<DroneState>(Point(1, 0, 0), 1.0));

    auto d1 = roadmap->addDart(v1, v1);
    auto d2 = roadmap->addDart(v2, v1);

    TRobotType drone1(1, "Drone 1", roadmap);
    TRobotType drone2(2, "Drone 2", roadmap);

    auto voxel1 = drone1.getSweptVoxel(d1);
    auto voxel2 = drone2.getSweptVoxel(d2);

    VoxelManager::SharedPtr voxel_manager = factory->makeVoxelManager();
    EXPECT_TRUE(voxel_manager->intersect(voxel1, voxel2));
}

template <typename TRobotType>
void testBarelyCollision(const RobotFactory::SharedPtr& factory) {
    //
    // Checks that two voxels that barely intersect each other actually intersect
    //

    Roadmap::SharedPtr roadmap = factory->makeRoadmap("Test Drone Roadmap", RobotType::DRONE);
    auto v1_left = roadmap->addVertex("v1_left", std::make_shared<DroneState>(Point(0, 0, 10), DRONE_RADIUS));
    auto v1_right = roadmap->addVertex("v1_right", std::make_shared<DroneState>(Point(0, 0, 1), DRONE_RADIUS));
    // essentially 2.1
    auto v2_left =
        roadmap->addVertex("v2_left", std::make_shared<DroneState>(Point(0, 0, PCL_VOXEL_RADIUS), DRONE_RADIUS));
    auto v2_right = roadmap->addVertex("v2_right", std::make_shared<DroneState>(Point(0, 0, -1), DRONE_RADIUS));

    auto d1 = roadmap->addDart(v1_left, v1_right);
    auto d2 = roadmap->addDart(v2_left, v2_right);

    TRobotType drone1(1, "Drone 1", roadmap);
    TRobotType drone2(2, "Drone 2", roadmap);

    auto voxel1 = drone1.getSweptVoxel(d1);
    auto voxel2 = drone2.getSweptVoxel(d2);

    VoxelManager::SharedPtr voxel_manager = factory->makeVoxelManager();
    EXPECT_TRUE(voxel_manager->intersect(voxel1, voxel2));
}

template <typename TRobotType>
void testNoCollision(const RobotFactory::SharedPtr& factory) {
    //
    // Checks that two voxels that barely intersect each other actually intersect
    //

    Roadmap::SharedPtr roadmap = factory->makeRoadmap("Test Drone Roadmap", RobotType::DRONE);
    auto v1_left = roadmap->addVertex(
        "v1_left", std::make_shared<DroneState>(Point(-10.000000, 0.000000, 0.000000), DRONE_RADIUS));
    auto v1_right = roadmap->addVertex(
        "v1_right", std::make_shared<DroneState>(Point(-5.000000, 5.000000, 0.000000), DRONE_RADIUS));
    // essentially 2.1
    auto v2_left =
        roadmap->addVertex("v2_left", std::make_shared<DroneState>(Point(10.000000, 0.000000, 0.000000), DRONE_RADIUS));
    auto v2_right =
        roadmap->addVertex("v2_right", std::make_shared<DroneState>(Point(0.000000, 0.000000, 0.000000), DRONE_RADIUS));

    auto d1 = roadmap->addDart(v1_left, v1_right);
    auto d2 = roadmap->addDart(v2_left, v2_right);

    TRobotType drone1(1, "Drone 1", roadmap);
    TRobotType drone2(2, "Drone 2", roadmap);

    auto voxel1 = drone1.getSweptVoxel(d1);
    auto voxel2 = drone2.getSweptVoxel(d2);

    VoxelManager::SharedPtr voxel_manager = factory->makeVoxelManager();
    EXPECT_FALSE(voxel_manager->intersect(voxel1, voxel2));
}

template <typename TRobotType>
void testParallelNearbySameRoadmap(const RobotFactory::SharedPtr& factory) {
    //
    // Checks that two voxels that are parallel but nearby do not intersect
    //

    Roadmap::SharedPtr roadmap = factory->makeRoadmap("Test Drone Roadmap", RobotType::DRONE);

    auto v1_left = roadmap->addVertex("v1_left", std::make_shared<DroneState>(Point(0, 0, -1), DRONE_RADIUS));
    auto v1_right = roadmap->addVertex("v1_right", std::make_shared<DroneState>(Point(0, 0, 1), DRONE_RADIUS));
    // essentially 2.1
    auto v2_left = roadmap->addVertex(
        "v2_left", std::make_shared<DroneState>(Point(DRONE_RADIUS * 2 + PCL_VOXEL_RADIUS * 2, 0, -1), DRONE_RADIUS));
    auto v2_right = roadmap->addVertex(
        "v2_right", std::make_shared<DroneState>(Point(DRONE_RADIUS * 2 + PCL_VOXEL_RADIUS * 2, 0, 1), DRONE_RADIUS));

    auto d1 = roadmap->addDart(v1_left, v1_right);
    auto d2 = roadmap->addDart(v2_left, v2_right);

    TRobotType drone1(1, "Drone 1", roadmap);
    TRobotType drone2(2, "Drone 2", roadmap);

    auto voxel1 = drone1.getSweptVoxel(d1);
    auto voxel2 = drone2.getSweptVoxel(d2);

    VoxelManager::SharedPtr voxel_manager = factory->makeVoxelManager();
    EXPECT_FALSE(voxel_manager->intersect(voxel1, voxel2));
}

template <typename TRobotType>
void testCollisionCommonPosition(const RobotFactory::SharedPtr& factory) {
    //
    // Checks that two voxels that barely intersect each other actually intersect
    //

    Roadmap::SharedPtr roadmap = factory->makeRoadmap("Test Drone Roadmap", RobotType::DRONE);
    auto v1_left = roadmap->addVertex(
        "v1_left", std::make_shared<DroneState>(Point(-10.000000, 0.000000, 0.000000), DRONE_RADIUS));
    auto v1_right = roadmap->addVertex(
        "v1_right", std::make_shared<DroneState>(Point(-5.000000, 5.000000, 0.000000), DRONE_RADIUS));
    // essentially 2.1
    auto v2_left =
        roadmap->addVertex("v2_left", std::make_shared<DroneState>(Point(-5.000000, 5.000000, 0.000000), DRONE_RADIUS));
    auto v2_right =
        roadmap->addVertex("v2_right", std::make_shared<DroneState>(Point(0.000000, 0.000000, 0.000000), DRONE_RADIUS));

    auto d1 = roadmap->addDart(v1_left, v1_right);
    auto d2 = roadmap->addDart(v2_left, v2_right);

    TRobotType drone1(1, "Drone 1", roadmap);
    TRobotType drone2(2, "Drone 2", roadmap);

    auto voxel1 = drone1.getSweptVoxel(d1);
    auto voxel2 = drone2.getSweptVoxel(d2);

    VoxelManager::SharedPtr voxel_manager = factory->makeVoxelManager();
    EXPECT_TRUE(voxel_manager->intersect(voxel1, voxel2));
}

template <typename TRobotType>
void testNoCollision2(const RobotFactory::SharedPtr& factory) {
    //
    // Checks that two voxels that barely intersect each other actually intersect
    //

    Roadmap::SharedPtr roadmap = factory->makeRoadmap("Test Drone Roadmap", RobotType::DRONE);
    auto v1_left = roadmap->addVertex(
        "v1_left", std::make_shared<DroneState>(Point(-10.000000, 0.000000, 0.000000), DRONE_RADIUS));
    auto v1_right = roadmap->addVertex(
        "v1_right", std::make_shared<DroneState>(Point(-5.000000, 5.000000, 0.000000), DRONE_RADIUS));
    // essentially 2.1
    auto v2_left =
        roadmap->addVertex("v2_left", std::make_shared<DroneState>(Point(10.000000, 0.000000, 0.000000), DRONE_RADIUS));
    auto v2_right =
        roadmap->addVertex("v2_right", std::make_shared<DroneState>(Point(0.000000, 0.000000, 0.000000), DRONE_RADIUS));

    auto d1 = roadmap->addDart(v1_left, v1_right);
    auto d2 = roadmap->addDart(v2_left, v2_right);

    TRobotType drone1(1, "Drone 1", roadmap);
    TRobotType drone2(2, "Drone 2", roadmap);

    auto voxel1 = drone1.getSweptVoxel(d1);
    auto voxel2 = drone2.getSweptVoxel(d2);

    VoxelManager::SharedPtr voxel_manager = factory->makeVoxelManager();
    EXPECT_FALSE(voxel_manager->intersect(voxel1, voxel2));
}

template <typename TRobotType>
void testNoCollision3(const RobotFactory::SharedPtr& factory) {
    //
    // Checks that two voxels that barely intersect each other actually intersect
    //

    Roadmap::SharedPtr roadmap = factory->makeRoadmap("Test Drone Roadmap", RobotType::DRONE);
    auto v1_left = roadmap->addVertex(
        "v1_left", std::make_shared<DroneState>(Point(-10.000000, 0.000000, 0.000000), DRONE_RADIUS));
    auto v1_right = roadmap->addVertex(
        "v1_right", std::make_shared<DroneState>(Point(-5.000000, 5.000000, 0.000000), DRONE_RADIUS));
    // essentially 2.1
    auto v2_left =
        roadmap->addVertex("v2_left", std::make_shared<DroneState>(Point(10.000000, 0.000000, 0.000000), DRONE_RADIUS));
    auto v2_right =
        roadmap->addVertex("v2_right", std::make_shared<DroneState>(Point(0.000000, 0.000000, 0.000000), DRONE_RADIUS));

    auto d1 = roadmap->addDart(v1_left, v1_right);
    auto d2 = roadmap->addDart(v2_left, v2_right);

    TRobotType drone1(1, "Drone 1", roadmap);
    TRobotType drone2(2, "Drone 2", roadmap);

    auto voxel1 = drone1.getSweptVoxel(d1);
    auto voxel2 = drone2.getSweptVoxel(d2);

    VoxelManager::SharedPtr voxel_manager = factory->makeVoxelManager();
    EXPECT_FALSE(voxel_manager->intersect(voxel1, voxel2));
}

template <typename TRobotType>
void testNoCollision4(const RobotFactory::SharedPtr& factory) {
    //
    // Checks that two voxels that barely intersect each other actually intersect
    //

    Roadmap::SharedPtr roadmap = factory->makeRoadmap("Test Drone Roadmap", RobotType::DRONE);
    auto v1_left = roadmap->addVertex(
        "v1_left", std::make_shared<DroneState>(Point(-10.000000, 0.000000, 0.000000), DRONE_RADIUS));
    auto v1_right = roadmap->addVertex(
        "v1_right", std::make_shared<DroneState>(Point(-5.000000, 5.000000, 0.000000), DRONE_RADIUS));
    // essentially 2.1
    auto v2_left =
        roadmap->addVertex("v2_left", std::make_shared<DroneState>(Point(10.000000, 0.000000, 0.000000), DRONE_RADIUS));
    auto v2_right =
        roadmap->addVertex("v2_right", std::make_shared<DroneState>(Point(0.000000, 0.000000, 0.000000), DRONE_RADIUS));

    auto d1 = roadmap->addDart(v1_left, v1_right);
    auto d2 = roadmap->addDart(v2_left, v2_right);

    TRobotType drone1(1, "Drone 1", roadmap);
    TRobotType drone2(2, "Drone 2", roadmap);

    auto voxel1 = drone1.getSweptVoxel(d1);
    auto voxel2 = drone2.getSweptVoxel(d2);

    VoxelManager::SharedPtr voxel_manager = factory->makeVoxelManager();
    EXPECT_FALSE(voxel_manager->intersect(voxel1, voxel2));
}

//////////////////////////////////////// TESTS FOR CPU ////////////////////////////////////////

TEST(PointCloudVoxels, TestHeadOnCollision) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD);
    testHeadOnCollision<Drone>(factory);
}

TEST(PointCloudVoxels, TestHeadOnCollisionStationary) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD);
    testHeadOnCollisionStationary<Drone>(factory);
}

TEST(PointCloudVoxels, TestBarelyCollision) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD);
    testBarelyCollision<Drone>(factory);
}

TEST(PointCloudVoxels, TestNoCollision) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD);
    testNoCollision<Drone>(factory);
}

TEST(PointCloudVoxels, TestParallelNearbySameRoadmap) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD);
    testParallelNearbySameRoadmap<Drone>(factory);
}

TEST(PointCloudVoxels, TestCollisionCommonPosition) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD);
    testCollisionCommonPosition<Drone>(factory);
}

TEST(PointCloudVoxels, TestNoCollision2) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD);
    testNoCollision2<Drone>(factory);
}

TEST(PointCloudVoxels, TestNoCollision3) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD);
    testNoCollision3<Drone>(factory);
}

TEST(PointCloudVoxels, TestNoCollision4) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD);
    testNoCollision4<Drone>(factory);
}

//////////////////////////////////////// TESTS FOR GPU ////////////////////////////////////////

TEST(PointCloudVoxelsGPU, TestHeadOnCollision) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD_GPU);
    testHeadOnCollision<DroneGPU>(factory);
}

TEST(PointCloudVoxelsGPU, TestHeadOnCollisionStationary) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD_GPU);
    testHeadOnCollisionStationary<DroneGPU>(factory);
}

TEST(PointCloudVoxelsGPU, TestBarelyCollision) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD_GPU);
    testBarelyCollision<DroneGPU>(factory);
}

TEST(PointCloudVoxelsGPU, TestNoCollision) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD_GPU);
    testNoCollision<DroneGPU>(factory);
}

TEST(PointCloudVoxelsGPU, TestParallelNearbySameRoadmap) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD_GPU);
    testParallelNearbySameRoadmap<DroneGPU>(factory);
}

TEST(PointCloudVoxelsGPU, TestCollisionCommonPosition) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD_GPU);
    testCollisionCommonPosition<DroneGPU>(factory);
}

TEST(PointCloudVoxelsGPU, TestNoCollision2) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD_GPU);
    testNoCollision2<DroneGPU>(factory);
}

TEST(PointCloudVoxelsGPU, TestNoCollision3) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD_GPU);
    testNoCollision3<DroneGPU>(factory);
}

TEST(PointCloudVoxelsGPU, TestNoCollision4) {
    RobotFactory::SharedPtr factory = std::make_shared<RobotFactory>(VoxelType::POINT_CLOUD_GPU);
    testNoCollision4<DroneGPU>(factory);
}
