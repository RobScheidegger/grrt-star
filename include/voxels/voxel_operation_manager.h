#pragma once

#include "voxel.h"

namespace grrt {

    template <IVoxel VoxelType>
    class VoxelManager {
       public:
        /// @brief Determines whether two voxels intersect or not.
        /// @param voxel_1
        /// @param voxel_2
        /// @return `true` if the voxels intersect, and `false` otherwise.
        virtual bool intersect(const VoxelType& voxel_1, const VoxelType& voxel_2) = 0;

        /// @brief Optimizes the representation of a particular voxel.
        /// @param voxel The voxel that we want to optimize.
        virtual void optimize(VoxelType& voxel) = 0;
    };
}  // namespace grrt