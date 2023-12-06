#include <cuda.h>
#include <cuda_device_runtime_api.h>
#include <driver_types.h>

#include "voxels/point_cloud/point_cloud_voxel_gpu.h"

using namespace grrt;

// TODO: siddharth: add cuda kernels

PointCloudVoxelGPU::PointCloudVoxelGPU(const size_t num_points) : num_points(num_points) {
    cudaError_t err = cudaHostAlloc(&points, num_points * sizeof(Point), cudaHostAllocDefault);
    if (err != cudaSuccess) {
        throw std::runtime_error("Failed to allocate memory for point cloud voxel");
    }
}

void PointCloudVoxelGPU::addPoint(const Point& point) {}

bool PointCloudVoxelGPU::contains(const Point& point) const {}
