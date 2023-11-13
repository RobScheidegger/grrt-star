#pragma once

#include <iostream>
#include <string>

namespace grrt {

    /// @brief Generic point struct to represent a point in 3D space, with no radius.
    struct Point {
        /// @brief The `x`-coordinate of the point.
        float x;
        /// @brief The `y`-coordinate of the point.
        float y;
        /// @brief The `z`-coordinate of the point (up direction).
        float z;

        /// @brief Basic constructor for a point.
        /// @param x The `x`-coordinate of the point.
        /// @param y The `y`-coordinate of the point.
        /// @param z The `z`-coordinate of the point (up direction).
        Point(const float x, const float y, const float z) : x(x), y(y), z(z) {}

        std::string toString() const {
            return "Point(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
        }
    };
}  // namespace grrt