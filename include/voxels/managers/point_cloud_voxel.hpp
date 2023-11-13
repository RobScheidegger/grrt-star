#pragma once

#include <vector>

#include "point.h"
#include "voxels/voxel.h"

#define PCL_VOXEL_RADIUS 0.1

namespace grrt {

    class PointCloudVoxel : public Voxel {
       public:
        PointCloudVoxel() {}

        /// @brief Gets the radius of each point in the point cloud, in meters.
        inline float getRadius() const { return m_radius; }

        void addPoint(const Point& point) { m_points.push_back(point); }

        bool contains(const Point& point) const override {
            // TODO
            return false;
        }

       private:
        const float m_radius = PCL_VOXEL_RADIUS;

        std::vector<Point> m_points;
    };
}  // namespace grrt