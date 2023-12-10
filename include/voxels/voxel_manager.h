#pragma once

#include "voxel.h"

namespace grrt {

    class VoxelManager {
       public:
        typedef std::shared_ptr<VoxelManager> SharedPtr;

        /// @brief Determines whether two voxels intersect or not.
        /// @param voxel_1 The first voxel.
        /// @param voxel_2 The second voxel.
        /// @return `true` if the voxels intersect, and `false` otherwise.
        virtual bool intersect(const Voxel::SharedPtr& voxel_1, const Voxel::SharedPtr& voxel_2) = 0;
    };
}  // namespace grrt