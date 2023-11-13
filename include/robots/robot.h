#pragma once

#include "graphs/roadmap_edge.h"
#include "state/robot_state.h"
#include "voxels/voxel.h"

namespace grrt {

    /// @brief Generic robot interface to represent a
    template <IRobotState RobotStateType, IVoxel VoxelType>
    class IRobot {
        // virtual VoxelType sweepVoxel(const RobotStateType& start, const RobotStateType& end) = 0;
        virtual VoxelType getSweptVoxel(const RoadmapEdge<RobotStateType>& edge) = 0;

        /// @brief Gets a roadmap edge between to particular robot states.
        /// @param start
        /// @param end
        /// @return
        virtual RoadmapEdge<RobotStateType> getEdge(const RobotStateType& start, const RobotStateType& end) = 0;
    };
}  // namespace grrt