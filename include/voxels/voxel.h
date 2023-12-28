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

        virtual bool contains(const Point& point) const = 0;
    };
}  // namespace grrt