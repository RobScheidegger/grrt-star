#pragma once

#include <vector>

#include "point.h"
#include "voxels/voxel.h"
#include "voxels/voxel_manager.h"

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

    class PointCloudVoxelManager : public VoxelManager {
       public:
        typedef std::shared_ptr<PointCloudVoxelManager> SharedPtr;

        bool intersect(const Voxel::SharedPtr& voxel_1, const Voxel::SharedPtr& voxel_2) override {
            // TODO
            return false;
        }

        Voxel::SharedPtr optimize(Voxel::SharedPtr& voxel) override {
            // TODO
            return nullptr;
        }
    };
}  // namespace grrt