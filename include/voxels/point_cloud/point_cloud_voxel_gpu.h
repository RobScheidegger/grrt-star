#pragma once

#include <cuda.h>
#include <vector>

#include "point.h"
#include "voxels/voxel.h"
#include "voxels/voxel_manager.h"

namespace grrt
{

    class PointCloudVoxelGPU : public Voxel
    {
    public:
        typedef std::shared_ptr<PointCloudVoxelGPU> SharedPtr;

        PointCloudVoxelGPU(const size_t num_points);

        ~PointCloudVoxelGPU();

        /// @brief Gets the radius of each point in the point cloud, in meters.
        inline float getRadius() const { return m_radius; }

        void addPoint(const Point &point);

        bool contains(const Point &point) const override;

        const size_t num_points = 0;
        float *points = nullptr;

        Point start_point;
        Point end_point;

    private:
        const float m_radius = PCL_VOXEL_RADIUS;

        size_t current_num_points = 0;
    };

    class PointCloudVoxelGPUManager : public VoxelManager
    {
    public:
        typedef std::shared_ptr<PointCloudVoxelGPUManager> SharedPtr;

        // defined in cu file.
        bool intersect(const Voxel::SharedPtr &voxel_1, const Voxel::SharedPtr &voxel_2) override;

        Voxel::SharedPtr optimize(Voxel::SharedPtr &voxel) override
        {
            // TODO
            return nullptr;
        }
    };

    // for the gpu variants, it will call the cuda kernel.

    // All the cuda kernal stuff needs to be in not hpp files.
} // namespace grrt