#pragma once

#include "robots/robot.h"
#include "robots/robot_types.h"
#include "voxels/voxel_manager.h"
#include "voxels/voxel_type.h"

namespace grrt {

    /// @brief Generic robot factory class to generate robots based on
    /// the given configuration parameters (mainly, the voxel)
    class RobotFactory {
       public:
        typedef std::shared_ptr<RobotFactory> SharedPtr;

        RobotFactory(const VoxelType voxelType) : m_voxelType(voxelType) {}

        IRobot::SharedPtr makeRobot(const RobotId id, const std::string& name, const Roadmap::SharedPtr& roadmap) const;

        VoxelManager::SharedPtr makeVoxelManager() const;

        Roadmap::SharedPtr makeRoadmap(const std::string& name, const RobotType type) const;

       private:
        VoxelType m_voxelType;
    };
}  // namespace grrt