#pragma once

#include <vector>

#include "point.h"
#include "voxels/voxel.h"
#include "voxels/voxel_manager.h"

#include "constants.h"

// this is the size of each point in the point cloud (used as a threshold to see if any points are in the range of any points from a different voxelspread).
// #ifndef PCL_VOXEL_RADIUS
// #define PCL_VOXEL_RADIUS 0.05
// #endif

namespace grrt {

    class PointCloudVoxel : public Voxel {
       public:
        typedef std::shared_ptr<PointCloudVoxel> SharedPtr;

        PointCloudVoxel() {}

        /// @brief Gets the radius of each point in the point cloud, in meters.
        inline float getRadius() const { return m_radius; }

        void addPoint(const Point& point) { m_points.push_back(point); }

        bool contains(const Point& point) const override {
            // TODO
            return false;
        }

        const float m_radius = PCL_VOXEL_RADIUS;

        Point start_point;
        Point end_point;

        std::vector<Point> m_points;
    };

    class PointCloudVoxelManager : public VoxelManager {
       public:
        typedef std::shared_ptr<PointCloudVoxelManager> SharedPtr;

        bool intersect(const Voxel::SharedPtr& voxel_1, const Voxel::SharedPtr& voxel_2) override {
            PointCloudVoxel::SharedPtr pcl_voxel_1 = std::dynamic_pointer_cast<PointCloudVoxel>(voxel_1);
            PointCloudVoxel::SharedPtr pcl_voxel_2 = std::dynamic_pointer_cast<PointCloudVoxel>(voxel_2);

            // printf("start_point 1: (%f, %f, %f)\n", pcl_voxel_1->start_point.x, pcl_voxel_1->start_point.y,
            //        pcl_voxel_1->start_point.z);
            // printf("end_point 1: (%f, %f, %f)\n", pcl_voxel_1->end_point.x, pcl_voxel_1->end_point.y,
            //        pcl_voxel_1->end_point.z);

            // printf("start_point 2: (%f, %f, %f)\n", pcl_voxel_2->start_point.x, pcl_voxel_2->start_point.y,
            //        pcl_voxel_2->start_point.z);
            // printf("end_point 2: (%f, %f, %f)\n", pcl_voxel_2->end_point.x, pcl_voxel_2->end_point.y,
            //        pcl_voxel_2->end_point.z);

            int i = 0;
            for (const Point& point_1 : pcl_voxel_1->m_points) {
                int j = 0;
                for (const Point& point_2 : pcl_voxel_2->m_points) {
                    float dist = point_1.distance(point_2);
                    if (dist < PCL_VOXEL_RADIUS) {
                        // printf("Point: (%f, %f, %f) at %d and Point: (%f, %f, %f) at %d with dist: %f\n", point_1.x,
                        //        point_1.y, point_1.z, i, point_2.x, point_2.y, point_2.z, j, dist);
                        // printf("collision!\n");
                        return true;
                    }
                    j += 3;
                }
                i += 3;
            }

            // printf("no collision!\n");
            return false;
        }

        Voxel::SharedPtr optimize(Voxel::SharedPtr& voxel) override {
            // TODO
            return nullptr;
        }
    };

}  // namespace grrt