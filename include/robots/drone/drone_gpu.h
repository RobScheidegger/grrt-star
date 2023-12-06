#pragma once

#include "robots/robot.h"
#include "voxels/point_cloud/point_cloud_voxel_gpu.h"

#define VOXEL_SWEEP_STEP_SIZE 0.05
#define VOXEL_POINTS_SAMPLING_SIZE 1000

namespace grrt {
    class DroneGPU : public IRobot {
       public:
        DroneGPU(const RobotId id, const std::string& name, const Roadmap::SharedPtr& roadmap)
            : IRobot(id, name, roadmap) {}

        Voxel::SharedPtr getSweptVoxel(const RoadmapDart::SharedPtr& dart) override {
            DroneState::SharedPtr start_state = std::dynamic_pointer_cast<DroneState>(dart->getStartState());
            DroneState::SharedPtr end_state = std::dynamic_pointer_cast<DroneState>(dart->getEndState());

            Point start_point = start_state->position;
            Point end_point = end_state->position;

            float sweep_length = start_point.distance(end_point);

            // We can't use a Point* here, instead a float* wehre each point is represented as three consecutive floats in the points array.
            int cloud_point_size = VOXEL_POINTS_SAMPLING_SIZE * int(sweep_length / VOXEL_SWEEP_STEP_SIZE) * 3;

            PointCloudVoxelGPU::SharedPtr cloud = std::make_shared<PointCloudVoxelGPU>(cloud_point_size);

            // generate fibonacci sphere: https://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere
            float phi = M_PI * (std::sqrt(5.) - 1.);  // golden angle in radian

            for (float swept_distance = 0; swept_distance <= sweep_length; swept_distance += VOXEL_SWEEP_STEP_SIZE) {
                float a = swept_distance;
                float b = sweep_length - swept_distance;

                float x_delta = a / (a + b) * (end_point.x - start_point.x);
                float y_delta = a / (a + b) * (end_point.y - start_point.y);
                float z_delta = a / (a + b) * (end_point.z - start_point.z);

                Point offset = {start_point.x + x_delta, start_point.y + y_delta, start_point.z + z_delta};

                for (int i = 1; i <= VOXEL_POINTS_SAMPLING_SIZE; i++) {
                    float y = 1 - (i / float(VOXEL_POINTS_SAMPLING_SIZE - 1)) * 2;  // y goes from 1 to - 1

                    float radius = std::sqrt(1 - y * y);  // radius at y
                    float theta = phi * i;                // golden angle increment

                    float x = cos(theta) * radius;
                    float z = sin(theta) * radius;
                    cloud->addPoint(Point(x * start_state->radius + offset.x, y * start_state->radius + offset.y,
                                          z * start_state->radius + offset.z));
                }
            }

            return cloud;
        }

        // TODO
        Voxel::SharedPtr getVoxel(const RoadmapDart::SharedPtr& dart, const double time) override {
            PointCloudVoxel::SharedPtr cloud = std::make_shared<PointCloudVoxel>();

            DroneState::SharedPtr start_state = std::dynamic_pointer_cast<DroneState>(dart->getStartState());
            DroneState::SharedPtr end_state = std::dynamic_pointer_cast<DroneState>(dart->getEndState());

            return cloud;
        }
    };
}  // namespace grrt