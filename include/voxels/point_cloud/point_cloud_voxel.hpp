#pragma once

#include <vector>

#include "point.h"
#include "voxels/voxel.h"
#include "voxels/voxel_manager.h"

// this is the size of each point in the point cloud (used as a threshold to see if any points are in the range of any points from a different voxelspread).
#define PCL_VOXEL_RADIUS 0.05

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
            PointCloudVoxel::SharedPtr pcl_voxel_1 = std::dynamic_pointer_cast<PointCloudVoxel>(voxel_1);
            PointCloudVoxel::SharedPtr pcl_voxel_2 = std::dynamic_pointer_cast<PointCloudVoxel>(voxel_2);

            // TODO: siddharth

            return false;
        }

        Voxel::SharedPtr optimize(Voxel::SharedPtr& voxel) override {
            // TODO
            return nullptr;
        }
    };

    // for the gpu variants, it will call the cuda kernel.

    // All the cuda kernal stuff needs to be in not hpp files.
}  // namespace grrt