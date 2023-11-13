#pragma once

#include <concepts>
#include <type_traits>

#include "point.h"

namespace grrt {

    /// @brief An interface for a voxel region, representing some 3D volume of space.
    class Voxel {
       public:
        /// @brief Check if a point is inside the voxel. This is mostly used for debugging/testing.
        /// @param point The point to check.
        /// @return True if the point is inside the voxel, false otherwise.
        virtual bool contains(const Point& point) const = 0;
    };

    // Concept for a Voxel that can be implemented in other ways.
    template <typename T>
    concept IVoxel = std::is_base_of<Voxel, T>::value;
}  // namespace grrt