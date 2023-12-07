#include <cuda.h>
#include <cuda_device_runtime_api.h>
#include <driver_types.h>
#include <chrono>


#include "voxels/point_cloud/point_cloud_voxel_gpu.h"
// Linker issue caused here
// #include "spdlog/spdlog.h"

#define MIN(a, b) (a < b ? a : b)
#define MAX(a, b) (a < b ? b : a)
#define CEIL(x, y) ((x + y - 1) / y)

#define MAX_THREADS_PER_BLOCK 1024
#define THREADS_PER_WARP 32
#define WARPS_PER_BLOCK 32
#define WARP_SIZE 32
#define FLOATS_PER_POINT 3


using namespace grrt;

// TODO: siddharth: add cuda kernels

// each thread works on 3 floats from pcl_voxel_1 and 3 floats from pcl_voxel_2
__global__ void saxby_shuffle_single(float* pcl_voxel_1_pnts,  float* pcl_voxel_2_pnts, int pcl_voxel_1_count, int pcl_voxel_2_count, float* bool_sum) {

    int warp_id = threadIdx.x / THREADS_PER_WARP;
    int lane_id = threadIdx.x % THREADS_PER_WARP;

    // A thread acts on three floats
    int pcl_v1_i = blockIdx.x * THREADS_PER_WARP * FLOATS_PER_POINT + warp_id;
    int pcl_v2_i = blockIdx.y * FLOATS_PER_POINT;

    if (pcl_v1_i >= pcl_voxel_1_count || pcl_v2_i >= pcl_voxel_2_count) {
        return;
    }
    
    int sum = 0;

    float dist = std::sqrt(
        (pcl_voxel_1_pnts[pcl_v1_i] - pcl_voxel_2_pnts[pcl_v2_i]) * (pcl_voxel_1_pnts[pcl_v1_i] - pcl_voxel_2_pnts[pcl_v2_i]) 
        + (pcl_voxel_1_pnts[pcl_v1_i + 1] - pcl_voxel_2_pnts[pcl_v2_i + 1]) * (pcl_voxel_1_pnts[pcl_v1_i + 1] - pcl_voxel_2_pnts[pcl_v2_i + 1]) 
        + (pcl_voxel_1_pnts[pcl_v1_i + 2] - pcl_voxel_2_pnts[pcl_v2_i + 2]) * (pcl_voxel_1_pnts[pcl_v1_i + 2] - pcl_voxel_2_pnts[pcl_v2_i + 2]));

    if (dist < PCL_VOXEL_RADIUS) {
        sum = 1;
    }

    __syncthreads();

    for (int offset = WARP_SIZE / 2; offset > 0; offset >>= 1)
        sum += __shfl_down_sync(0xffffff, sum, offset);

    if (lane_id == 0) {
        bool_sum[CEIL(pcl_v1_i, FLOATS_PER_POINT)] += sum;
    }
}


PointCloudVoxelGPU::PointCloudVoxelGPU(const size_t num_points) : num_points(num_points) {
    cudaError_t err = cudaHostAlloc(&points, num_points * sizeof(Point), cudaHostAllocDefault);
    if (err != cudaSuccess) {
        throw std::runtime_error("Failed to allocate memory for point cloud voxel");
    }
}

PointCloudVoxelGPU::~PointCloudVoxelGPU(){
    cudaError_t err = cudaFreeHost(points);
    if (err != cudaSuccess) {
        throw std::runtime_error("Failed to free points memory for point cloud voxel gpu");
    }
}


void PointCloudVoxelGPU::addPoint(const Point& point) {
    if (this->current_num_points >= this->num_points) {
        throw std::runtime_error("Point cloud voxel GPU is full");
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

bool intersect(const Voxel::SharedPtr& voxel_1, const Voxel::SharedPtr& voxel_2)  {
    // will have to cast the vortex to a PointCloudVoxelGPU.
    PointCloudVoxelGPU::SharedPtr pcl_voxel_1 = std::dynamic_pointer_cast<PointCloudVoxelGPU>(voxel_1);
    PointCloudVoxelGPU::SharedPtr pcl_voxel_2 = std::dynamic_pointer_cast<PointCloudVoxelGPU>(voxel_2);

    float *bool_sum;

    // Each MAX_THREADS_PER_BLOCK Points in pcl_voxel_1 will be worked on by a block, but the shuffle operation only occurs within a warp which is 32 threads aka 32 points (through 32 * 3 floats) in pcv1.
    int num_blocks_pcv1 = CEIL(pcl_voxel_1->num_points, 3 * MAX_THREADS_PER_BLOCK);

    cudaError_t err = cudaHostAlloc(&bool_sum, num_blocks_pcv1 * WARP_SIZE * sizeof(float), cudaHostAllocDefault);
    if (err != cudaSuccess) {
        throw std::runtime_error("Failed to allocate memory for bool_sum");
    }

    auto start = std::chrono::high_resolution_clock::now();

    // 30 elements from voxel_1 and voxel_2 will be assigned to a block (we won't use the full 32 since we need to operate over 3 floats at a time)


    // One block runs over MAX_THREADS_PER_BLOCK Points in pcl_voxel_1 and one Point in pcl_voxel_2
    dim3 num_blocks(num_blocks_pcv1, CEIL(pcl_voxel_2->num_points, 3), 1);
    dim3 max_threads_per_block(MAX_THREADS_PER_BLOCK, 1, 1);
    saxby_shuffle_single<<<num_blocks, max_threads_per_block>>>(pcl_voxel_1->points, pcl_voxel_2->points, pcl_voxel_1->num_points, pcl_voxel_2->num_points, bool_sum);
    cudaDeviceSynchronize();

    auto end = std::chrono::high_resolution_clock::now();

    err = cudaFreeHost(bool_sum);
    if (err != cudaSuccess) {
        throw std::runtime_error("Failed to free bool_sum for point cloud voxel gpu");
    }

    return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}