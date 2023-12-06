#pragma once

#include <math.h>
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

        inline Point operator+(const Point& other) const { return Point(x + other.x, y + other.y, z + other.z); }

        inline Point operator-(const Point& other) const { return Point(x - other.x, y - other.y, z - other.z); }

        inline Point operator*(const double& scalar) const { return Point(x * scalar, y * scalar, z * scalar); }

        inline Point operator/(const double& scalar) const { return Point(x / scalar, y / scalar, z / scalar); }

        inline double distance(const Point& other) const {
            return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y) +
                             (z - other.z) * (z - other.z));
        }

        inline double norm() const { return std::sqrt(x * x + y * y + z * z); }

        inline double dot(const Point& other) const { return x * other.x + y * other.y + z * other.z; }

        inline double angle(const Point& other) const { return std::acos(dot(other) / (norm() * other.norm())); }

        std::string toString() const {
            return "Point(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
        }
    };
}  // namespace grrt