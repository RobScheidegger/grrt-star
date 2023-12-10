#include <cuda.h>
#include <cuda_device_runtime_api.h>
#include <driver_types.h>
#include <chrono>

#include "constants.h"
#include "voxels/point_cloud/point_cloud_voxel_gpu.h"

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

#define checkCudaErr(ans) \
    { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char* file, int line, bool abort = true) {
    if (code != cudaSuccess) {
        fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
        if (abort)
            exit(code);
    }
}

// each thread works on 3 floats from pcl_voxel_1 and 3 floats from pcl_voxel_2
__global__ void intersect_shuffle(float* pcl_voxel_1_pnts, float* pcl_voxel_2_pnts, int pcl_voxel_1_count,
                                  int pcl_voxel_2_count, int* bool_sum, int bool_sum_size) {

    const int warp_id = threadIdx.x / THREADS_PER_WARP;
    const int lane_id = threadIdx.x % THREADS_PER_WARP;

    // A thread acts on three floats
    const int pcl_v1_start_i = (blockIdx.x * MAX_THREADS_PER_BLOCK + lane_id * WARP_SIZE + warp_id) * FLOATS_PER_POINT *
                               NUM_PCV1_POINTS_PER_THREAD;
    const int pcl_v2_start_i = blockIdx.y * FLOATS_PER_POINT * NUM_PCV2_POINTS_PER_THREAD;

    if (pcl_v1_start_i >= pcl_voxel_1_count || pcl_v2_start_i >= pcl_voxel_2_count) {
        return;
    }

    int sum = 0;

    for (int pcl_v1_i = pcl_v1_start_i; pcl_v1_i < pcl_v1_start_i + NUM_PCV1_POINTS_PER_THREAD * FLOATS_PER_POINT;
         pcl_v1_i += 3) {

        if (sum)
            break;

        if (pcl_v1_i >= pcl_voxel_1_count) {
            return;
        }

        for (int pcl_v2_i = pcl_v2_start_i; pcl_v2_i < pcl_v2_start_i + NUM_PCV2_POINTS_PER_THREAD * FLOATS_PER_POINT;
             pcl_v2_i += 3) {

            if (pcl_v2_i >= pcl_voxel_2_count) {
                break;
            }
            const float dist = (pcl_voxel_1_pnts[pcl_v1_i] - pcl_voxel_2_pnts[pcl_v2_i]) *
                                   (pcl_voxel_1_pnts[pcl_v1_i] - pcl_voxel_2_pnts[pcl_v2_i]) +
                               (pcl_voxel_1_pnts[pcl_v1_i + 1] - pcl_voxel_2_pnts[pcl_v2_i + 1]) *
                                   (pcl_voxel_1_pnts[pcl_v1_i + 1] - pcl_voxel_2_pnts[pcl_v2_i + 1]) +
                               (pcl_voxel_1_pnts[pcl_v1_i + 2] - pcl_voxel_2_pnts[pcl_v2_i + 2]) *
                                   (pcl_voxel_1_pnts[pcl_v1_i + 2] - pcl_voxel_2_pnts[pcl_v2_i + 2]);

            if (dist < PCL_VOXEL_RADIUS * PCL_VOXEL_RADIUS) {
                sum += 1;
                break;
            }
        }
    }

    __syncthreads();

    for (int offset = WARP_SIZE / 2; offset > 0; offset >>= 1)
        sum += __shfl_down_sync(0xffffff, sum, offset);

    if (lane_id == 0) {
        int bool_i = blockIdx.x * WARPS_PER_BLOCK + warp_id;

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

PointCloudVoxelGPU::~PointCloudVoxelGPU() {
    cudaError_t err = cudaFreeHost(this->points);
    if (err != cudaSuccess) {
        printf("Failed to free points memory for point cloud voxel gpu: %s", cudaGetErrorString(err));
    }
}

void PointCloudVoxelGPU::addPoint(const Point& point) {
    if (this->current_num_points >= this->num_points) {
        printf("Point cloud voxel GPU is full: %lu\n", num_points);
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

bool PointCloudVoxelGPUManager::intersect(const Voxel::SharedPtr& voxel_1, const Voxel::SharedPtr& voxel_2) {
    // will have to cast the vortex to a PointCloudVoxelGPU.
    PointCloudVoxelGPU::SharedPtr pcl_voxel_1 = std::dynamic_pointer_cast<PointCloudVoxelGPU>(voxel_1);
    PointCloudVoxelGPU::SharedPtr pcl_voxel_2 = std::dynamic_pointer_cast<PointCloudVoxelGPU>(voxel_2);

    int num_blocks_pcv1 = CEIL(pcl_voxel_1->num_points, (3 * MAX_THREADS_PER_BLOCK * NUM_PCV1_POINTS_PER_THREAD));

    int bool_sum_size = num_blocks_pcv1 * WARP_SIZE;

    if (bool_sum_size > BOOL_SUM_MAX_SIZE) {
        printf("bool_sum doesn't have any space for this intersection\n");
    }

    memset(bool_sum, 0, bool_sum_size * sizeof(float));

    int num_blocks_pcv2 = CEIL(pcl_voxel_2->num_points, (3 * NUM_PCV2_POINTS_PER_THREAD));
    dim3 num_blocks(num_blocks_pcv1, num_blocks_pcv2, 1);

    dim3 max_threads_per_block(MAX_THREADS_PER_BLOCK, 1, 1);
    intersect_shuffle<<<num_blocks, max_threads_per_block>>>(pcl_voxel_1->points, pcl_voxel_2->points,
                                                             pcl_voxel_1->num_points, pcl_voxel_2->num_points, bool_sum,
                                                             bool_sum_size);

    checkCudaErr(cudaPeekAtLastError());

    checkCudaErr(cudaDeviceSynchronize());

    bool res = false;

    for (int i = 0; i < bool_sum_size; i++) {
        if (bool_sum[i] > 0) {
            res = true;
            break;
        }
    }

    return res;
}

PointCloudVoxelGPUManager::PointCloudVoxelGPUManager() {
    cudaError_t err = cudaHostAlloc(&bool_sum, BOOL_SUM_MAX_SIZE * sizeof(int), cudaHostAllocDefault);
    if (err != cudaSuccess) {
        printf("Failed to allocate memory for bool_sum: %s", cudaGetErrorString(err));
    }
}

PointCloudVoxelGPUManager::~PointCloudVoxelGPUManager() {
    if (bool_sum == nullptr) {
        return;
    }
    cudaError_t err = cudaFreeHost(bool_sum);
    if (err != cudaSuccess) {
        printf("Failed to free bool_sum for point cloud voxel gpu\n");
    }
}