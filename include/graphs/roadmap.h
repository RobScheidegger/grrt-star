#pragma once

#include <concepts>

#include "graphs/iroadmap.h"
#include "state/robot_state.h"
#include "voxels/voxel.h"

namespace grrt {

    template <IRobotState RobotStateType, IVoxel VoxelType>
    class Roadmap : public IRoadmap {
       public:
        Roadmap() = default;
        virtual ~Roadmap() = default;

        virtual void addState(const RobotStateType& state) = 0;
        virtual void addEdge(const RobotStateType& state1, const RobotStateType& state2) = 0;
    };
}  // namespace grrt