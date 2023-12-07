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
#define NUM_PCV1_POINTS_PER_THREAD 1
#define NUM_PCV2_POINTS_PER_THREAD 512


using namespace grrt;

// TODO: siddharth: add cuda kernels

// each thread works on 3 floats from pcl_voxel_1 and 3 floats from pcl_voxel_2
__global__ void saxby_shuffle_single(float* pcl_voxel_1_pnts,  float* pcl_voxel_2_pnts, int pcl_voxel_1_count, int pcl_voxel_2_count, float* bool_sum, int bool_sum_size) {

    int warp_id = threadIdx.x / THREADS_PER_WARP;
    int lane_id = threadIdx.x % THREADS_PER_WARP;

    // A thread acts on three floats
    int pcl_v1_start_i = (blockIdx.x * MAX_THREADS_PER_BLOCK * NUM_PCV1_POINTS_PER_THREAD + lane_id * WARP_SIZE + warp_id) * FLOATS_PER_POINT;
    int pcl_v2_start_i = blockIdx.y * FLOATS_PER_POINT * NUM_PCV2_POINTS_PER_THREAD;

    // if (pcl_v1_start_i >= pcl_voxel_1_count || pcl_v2_start_i >= pcl_voxel_2_count) {
    //     return;
    // }

    if (pcl_v1_start_i + (NUM_PCV1_POINTS_PER_THREAD * FLOATS_PER_POINT) - 1 >= pcl_voxel_1_count 
    || pcl_v2_start_i + (NUM_PCV2_POINTS_PER_THREAD * FLOATS_PER_POINT) - 1 >=  pcl_voxel_2_count) {
        return;
    }

    // printf("pcl_v1 point: %d\n", pcl_v1_start_i);
    // printf("pcl_v2 point: %d\n", pcl_v2_i);

    int sum = 0;
    
    for (int pcl_v1_i = pcl_v1_start_i; pcl_v1_i < pcl_v1_start_i + NUM_PCV1_POINTS_PER_THREAD * FLOATS_PER_POINT; pcl_v1_i += 3) {
        // printf("Point: (%f, %f, %f) at %d and Point: (%f, %f, %f) at %d\n", pcl_voxel_1_pnts[pcl_v1_i], pcl_voxel_1_pnts[pcl_v1_i + 1], pcl_voxel_1_pnts[pcl_v1_i + 2], pcl_v1_i, pcl_voxel_2_pnts[pcl_v2_i], pcl_voxel_2_pnts[pcl_v2_i + 1], pcl_voxel_2_pnts[pcl_v2_i + 2], pcl_v2_i);

        for (int pcl_v2_i = pcl_v2_start_i; pcl_v2_i < pcl_v2_start_i + NUM_PCV2_POINTS_PER_THREAD * FLOATS_PER_POINT; pcl_v2_i += 3) {

        float dist = std::sqrt(
        (pcl_voxel_1_pnts[pcl_v1_i] - pcl_voxel_2_pnts[pcl_v2_i]) * (pcl_voxel_1_pnts[pcl_v1_i] - pcl_voxel_2_pnts[pcl_v2_i]) 
        + (pcl_voxel_1_pnts[pcl_v1_i + 1] - pcl_voxel_2_pnts[pcl_v2_i + 1]) * (pcl_voxel_1_pnts[pcl_v1_i + 1] - pcl_voxel_2_pnts[pcl_v2_i + 1]) 
        + (pcl_voxel_1_pnts[pcl_v1_i + 2] - pcl_voxel_2_pnts[pcl_v2_i + 2]) * (pcl_voxel_1_pnts[pcl_v1_i + 2] - pcl_voxel_2_pnts[pcl_v2_i + 2]));

        // printf("dist: %f\n", dist);

        // if (dist < 2) {
        //     printf("Point: (%f, %f, %f) and Point: (%f, %f, %f) are within 0.15\n", pcl_voxel_1_pnts[pcl_v1_i], pcl_voxel_1_pnts[pcl_v1_i + 1], pcl_voxel_1_pnts[pcl_v1_i + 2], pcl_voxel_2_pnts[pcl_v2_i], pcl_voxel_2_pnts[pcl_v2_i + 1], pcl_voxel_2_pnts[pcl_v2_i + 2]);
        // }

        if (dist < PCL_VOXEL_RADIUS) {
            sum += 1;
        }
        }
    }

    __syncthreads();

    for (int offset = WARP_SIZE / 2; offset > 0; offset >>= 1)
        sum += __shfl_down_sync(0xffffff, sum, offset);

    if (lane_id == 0) {
        int bool_i = blockIdx.x * WARPS_PER_BLOCK + warp_id;

        // printf("bool_i: %d\n", bool_i);

        if (bool_i >= bool_sum_size) {
            printf("invalid acccess int bool_sum_size\n");
        }
        bool_sum[bool_i] += sum;
    }
}


PointCloudVoxelGPU::PointCloudVoxelGPU(const size_t num_points) : num_points(num_points) {
    cudaError_t err = cudaHostAlloc(&points, num_points * sizeof(Point), cudaHostAllocDefault);
    if (err != cudaSuccess) {
        printf("Failed to allocate memory for point cloud voxel: %s", cudaGetErrorString(err));
    }
}

PointCloudVoxelGPU::~PointCloudVoxelGPU(){
    cudaError_t err = cudaFreeHost(this->points);
    if (err != cudaSuccess) {
        printf("Failed to free points memory for point cloud voxel gpu: %s", cudaGetErrorString(err));
    }
}


void PointCloudVoxelGPU::addPoint(const Point& point) {
    if (this->current_num_points >= this->num_points) {
        printf("Point cloud voxel GPU is full\n");
        return;
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

bool PointCloudVoxelGPUManager::intersect(const Voxel::SharedPtr& voxel_1, const Voxel::SharedPtr& voxel_2)  {
    // will have to cast the vortex to a PointCloudVoxelGPU.
    PointCloudVoxelGPU::SharedPtr pcl_voxel_1 = std::dynamic_pointer_cast<PointCloudVoxelGPU>(voxel_1);
    PointCloudVoxelGPU::SharedPtr pcl_voxel_2 = std::dynamic_pointer_cast<PointCloudVoxelGPU>(voxel_2);

    float *bool_sum;

    // Each MAX_THREADS_PER_BLOCK * NUM_PCV1_POINTS_PER_THREAD Points in pcl_voxel_1 will be worked on by a block, but the shuffle operation only occurs within a warp which is 32 threads aka 32 points (through 32 * 3 floats) in pcv1.
    printf("blah: %lu\n", pcl_voxel_1->num_points);
    int num_blocks_pcv1 = CEIL(pcl_voxel_1->num_points, (3 * MAX_THREADS_PER_BLOCK * NUM_PCV1_POINTS_PER_THREAD));

    // int bool_sum_size = CEIL(pcl_voxel_1->num_points, (3 * MAX_THREADS_PER_BLOCK * NUM_PCV1_POINTS_PER_THREAD / WARP_SIZE));
    int bool_sum_size = num_blocks_pcv1 * WARP_SIZE;

    cudaError_t err = cudaHostAlloc(&bool_sum, bool_sum_size * sizeof(float), cudaHostAllocDefault);
    if (err != cudaSuccess) {
        printf("Failed to allocate memory for bool_sum");
    }

    printf("bool_sum_size: %d\n", bool_sum_size);

    int num_blocks_pcv2 = CEIL(pcl_voxel_2->num_points, (3 * NUM_PCV2_POINTS_PER_THREAD));


    auto start = std::chrono::high_resolution_clock::now();

    // 30 elements from voxel_1 and voxel_2 will be assigned to a block (we won't use the full 32 since we need to operate over 3 floats at a time)


    // One block runs over MAX_THREADS_PER_BLOCK * NUM_PCV1_POINTS_PER_THREAD Points in pcl_voxel_1 and one Point in pcl_voxel_2
    dim3 num_blocks(num_blocks_pcv1, num_blocks_pcv2, 1);

    // printf("pcl_voxel_2 points: %f\n", pcl_voxel_2->points[0]);
    // printf("pcl_voxel_2 points: %f\n", pcl_voxel_2->points[1]);
    // printf("pcl_voxel_2 points: %f\n", pcl_voxel_2->points[2]);

    printf("a: %d\n", num_blocks_pcv1);
    printf("b: %d\n", num_blocks_pcv2);

    dim3 max_threads_per_block(MAX_THREADS_PER_BLOCK, 1, 1);
    saxby_shuffle_single<<<num_blocks, max_threads_per_block>>>(pcl_voxel_1->points, pcl_voxel_2->points, pcl_voxel_1->num_points, pcl_voxel_2->num_points, bool_sum, bool_sum_size);
    cudaDeviceSynchronize();

    auto end = std::chrono::high_resolution_clock::now();

    err = cudaFreeHost(bool_sum);
    if (err != cudaSuccess) {
        printf("Failed to free bool_sum for point cloud voxel gpu\n");
    }

    auto diff = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    for (int i = 0; i < bool_sum_size; i++) {
        if (bool_sum[i] > 0) {
            printf("collision detected\n");
            return true;
        }
    }

     return false;
}