#pragma once

#include <string>

#include "result.h"

namespace grrt {
    /// @brief Enum for the different voxel type representations that we want to support.
    enum VoxelType { POINT_CLOUD, POINT_CLOUD_SORTED, POINT_CLOUD_GPU, POINT_CLOUD_GPU_SORTED };

}  // namespace grrt