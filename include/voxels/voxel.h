#pragma once

#include <concepts>
#include <memory>
#include <type_traits>

#include "point.h"

namespace grrt {

    /// @brief An interface for a voxel region, representing some 3D volume of space.
    class Voxel {
       public:
        typedef std::shared_ptr<Voxel> SharedPtr;

        /// @brief Check if a point is inside the voxel. This is mostly used for debugging/testing.
        /// @param point The point to check.
        /// @return True if the point is inside the voxel, false otherwise.
        virtual bool contains(const Point& point) const = 0;
    };
}  // namespace grrt