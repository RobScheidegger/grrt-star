#pragma once

#include <cuda.h>
#include <vector>

#include "point.h"
#include "voxels/voxel.h"
#include "voxels/voxel_manager.h"

#define PCL_VOXEL_RADIUS 0.1

namespace grrt {

    class PointCloudVoxelGPU : public Voxel {
       public:
        typedef std::shared_ptr<PointCloudVoxelGPU> SharedPtr;

        PointCloudVoxelGPU(const size_t num_points);

        ~PointCloudVoxelGPU();

        /// @brief Gets the radius of each point in the point cloud, in meters.
        inline float getRadius() const { return m_radius; }

        void addPoint(const Point& point);

        bool contains(const Point& point) const override;

<<<<<<< HEAD
        const size_t num_points = 0;
        float* points = nullptr;
=======
        ~PointCloudVoxelGPU() {
            // cudaError_t err = cudaFree(points);
            // if (err != cudaSuccess) {
            //     throw std::runtime_error("Failed to free points memory for point cloud voxel gpu");
            // }
        }
>>>>>>> f1fa55f1de2ad0b4d253a9d5c24b0a4c4ad00c4f

       private:
        const float m_radius = PCL_VOXEL_RADIUS;

        size_t current_num_points = 0;
    };

    class PointCloudVoxelGPUManager : public VoxelManager {
       public:
        typedef std::shared_ptr<PointCloudVoxelGPUManager> SharedPtr;

        // defined in cu file.
        bool intersect(const Voxel::SharedPtr& voxel_1, const Voxel::SharedPtr& voxel_2) override;

        Voxel::SharedPtr optimize(Voxel::SharedPtr& voxel) override {
            // TODO
            return nullptr;
        }
    };

    // for the gpu variants, it will call the cuda kernel.

    // All the cuda kernal stuff needs to be in not hpp files.
}  // namespace grrt