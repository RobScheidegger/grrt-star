#pragma once

#include "constants.h"
#include "robots/robot.h"
#include "voxels/point_cloud/point_cloud_voxel_gpu.h"

#include <math.h>

#define VOXEL_SWEEP_ERROR 0.001

namespace grrt {
    class DroneGPUState : public RobotState {
       public:
        typedef std::shared_ptr<DroneGPUState> SharedPtr;

        DroneGPUState(const Point& point, const float radius) : position(point), radius(radius) {}

        std::string toString() const override { return "DroneGPUState: " + position.toString(); }

        double distance(const RobotState::SharedPtr& other) const override {
            DroneGPUState::SharedPtr droneGPUState = std::dynamic_pointer_cast<DroneGPUState>(other);
            return position.distance(droneGPUState->position);
        }

        Point getPosition() const override { return position; }

        Point position;
        float radius;
    };

    class DroneGPU : public IRobot {
       public:
        DroneGPU(const RobotId id, const std::string& name, const Roadmap::SharedPtr& roadmap)
            : IRobot(id, name, roadmap) {}

        Voxel::SharedPtr getSweptVoxel(const RoadmapDart::SharedPtr& dart) override {
            DroneGPUState::SharedPtr start_state = std::dynamic_pointer_cast<DroneGPUState>(dart->getStartState());
            DroneGPUState::SharedPtr end_state = std::dynamic_pointer_cast<DroneGPUState>(dart->getEndState());

            Point start_point = start_state->position;
            Point end_point = end_state->position;

            // printf("start_point: (%f, %f, %f)\n", start_point.x, start_point.y, start_point.z);
            // printf("end_point: (%f, %f, %f)\n", end_point.x, end_point.y, end_point.z);

            float sweep_length = start_point.distance(end_point);

            // We can't use a Point* here, instead a float* wehre each point is represented as three consecutive floats in the points array.
            // printf("divisions: %d\n", int(sweep_length / VOXEL_SWEEP_STEP_SIZE));
            // the +1 is beccause a voxel is also generated at swept_distance = 0

            // int exptected_num_iters = (sweep_length != 0) ? int(sweep_length / VOXEL_SWEEP_STEP_SIZE) : 1;
            int exptected_num_iters = int(sweep_length / VOXEL_SWEEP_STEP_SIZE) + 1;
            // 0, 5, 10, 15
            // 15 / 5 == 3

            // if (std::fmod(sweep_length, VOXEL_SWEEP_STEP_SIZE) == 0) {
            //     exptected_num_iters += 1;
            // }

            int cloud_point_size = VOXEL_POINTS_SAMPLING_SIZE * exptected_num_iters * 3;

            PointCloudVoxelGPU::SharedPtr cloud = std::make_shared<PointCloudVoxelGPU>(cloud_point_size);

            // int cloud_point_size = 0;

            cloud->start_point = start_point;
            cloud->end_point = end_point;

            // generate fibonacci sphere: https://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere
            float phi = M_PI * (std::sqrt(5.) - 1.);  // golden angle in radian

            int thing = 0;
            for (float swept_distance = 0; swept_distance <= sweep_length + VOXEL_SWEEP_ERROR;
                 swept_distance += VOXEL_SWEEP_STEP_SIZE) {
                float a = swept_distance;
                float b = sweep_length - swept_distance;

                float x_delta = ((a + b) != 0) ? a / (a + b) * (end_point.x - start_point.x) : 0;
                float y_delta = ((a + b) != 0) ? a / (a + b) * (end_point.y - start_point.y) : 0;
                float z_delta = ((a + b) != 0) ? a / (a + b) * (end_point.z - start_point.z) : 0;

                Point offset = {start_point.x + x_delta, start_point.y + y_delta, start_point.z + z_delta};

                for (int i = 1; i <= VOXEL_POINTS_SAMPLING_SIZE; i++) {
                    float y = 1 - (i / float(VOXEL_POINTS_SAMPLING_SIZE - 1)) * 2;  // y goes from 1 to - 1

                    float radius = std::sqrt(1 - y * y);  // radius at y
                    float theta = phi * i;                // golden angle increment

                    float x = cos(theta) * radius;
                    float z = sin(theta) * radius;
                    cloud->addPoint(Point(x * start_state->radius + offset.x, y * start_state->radius + offset.y,
                                          z * start_state->radius + offset.z));
                    thing += 3;
                }
            }

            if ((exptected_num_iters * 3) != thing) {
                printf("mismatch!!! %d, %d\n", (exptected_num_iters * 3), thing);
            }

            // printf("thing: %d\n", thing);
            // printf("cloud_point_size: %d\n", cloud_point_size);

            dart->voxel = cloud;

            return cloud;
        }

        // TODO: what should the size be? This is never called btw
        Voxel::SharedPtr getVoxel(const RoadmapDart::SharedPtr& dart, const double time) override {
            PointCloudVoxelGPU::SharedPtr cloud = std::make_shared<PointCloudVoxelGPU>(0);

            DroneGPUState::SharedPtr start_state = std::dynamic_pointer_cast<DroneGPUState>(dart->getStartState());
            DroneGPUState::SharedPtr end_state = std::dynamic_pointer_cast<DroneGPUState>(dart->getEndState());

            return cloud;
        }
    };
}  // namespace grrt