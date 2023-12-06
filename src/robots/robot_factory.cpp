#include <cassert>
#include <functional>
#include <unordered_map>

#include "robots/drone/drone.hpp"
#include "robots/drone/drone_gpu.h"
#include "robots/robot_factory.h"
#include "voxels/point_cloud/point_cloud_voxel.hpp"

using namespace grrt;

typedef std::pair<VoxelType, RobotType> RobotTypeIdentifier;

IRobot::SharedPtr RobotFactory::makeRobot(const RobotId id, const std::string& name,
                                          const Roadmap::SharedPtr& roadmap) const {
    assert(this != nullptr);
    assert(roadmap != nullptr);
    switch (m_voxelType) {
        case VoxelType::POINT_CLOUD:
            switch (roadmap->robotType) {
                case RobotType::DRONE:
                    return std::make_shared<Drone>(id, name, roadmap);
                default:
                    throw std::runtime_error("Unknown robot type");
            }
        case VoxelType::POINT_CLOUD_GPU:
            switch (roadmap->robotType) {
                case RobotType::DRONE:
                    return std::make_shared<DroneGPU>(id, name, roadmap);
                default:
                    throw std::runtime_error("Unknown robot type");
            }
        default:
            throw std::runtime_error("Unknown voxel type");
    }
}

Roadmap::SharedPtr RobotFactory::makeRoadmap(const std::string& name, const RobotType type) const {
    return std::make_shared<Roadmap>(name, type);
}

VoxelManager::SharedPtr RobotFactory::makeVoxelManager() const {
    switch (m_voxelType) {
        case VoxelType::POINT_CLOUD:
            return std::make_shared<PointCloudVoxelManager>();
        default:
            throw std::runtime_error("Unknown voxel type");
    }
}