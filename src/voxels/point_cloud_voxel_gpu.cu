#include <cuda.h>
#include <cuda_device_runtime_api.h>
#include <driver_types.h>


#include "voxels/point_cloud/point_cloud_voxel_gpu.h"
// Linker issue caused here
// #include "spdlog/spdlog.h"


using namespace grrt;

// TODO: siddharth: add cuda kernels

PointCloudVoxelGPU::PointCloudVoxelGPU(const size_t num_points) : num_points(num_points) {
    // TODO: add a destructor
    cudaError_t err = cudaHostAlloc(&points, num_points * sizeof(Point), cudaHostAllocDefault);
    if (err != cudaSuccess) {
        throw std::runtime_error("Failed to allocate memory for point cloud voxel");
    }
}

// TODO: create PointCloudVoxelManager but for gpu with the intersect function

void PointCloudVoxelGPU::addPoint(const Point& point) {
    if (this->current_num_points >= this->num_points) {
        // spdlog::error("Point cloud voxel GPU is full");
    }
    this->points[this->current_num_points] = point.x;
    this->points[this->current_num_points + 1] = point.y;
    this->points[this->current_num_points + 2] = point.z;
    this->current_num_points += 3;
}

bool PointCloudVoxelGPU::contains(const Point& point) const {
    // TODO
    return false;
}
