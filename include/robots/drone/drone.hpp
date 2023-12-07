#pragma once

#include "constants.h"
#include "robots/robot.h"
#include "voxels/point_cloud/point_cloud_voxel.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

namespace grrt {
    /// @brief State representing a simple drone.
    class DroneState : public RobotState {
       public:
        typedef std::shared_ptr<DroneState> SharedPtr;

        DroneState(const Point& point, const float radius) : position(point), radius(radius) {}

        std::string toString() const override { return "DroneState: " + position.toString(); }

        double distance(const RobotState::SharedPtr& other) const override {
            DroneState::SharedPtr droneState = std::dynamic_pointer_cast<DroneState>(other);
            return position.distance(droneState->position);
        }

        inline Point getPosition() const override { return position; }

        Point position;
        float radius;
    };

    // point cloud resolution is the effective radius of each point in the point cloud
    // and then there is also the step size wihich is on the same order.

    class Drone : public IRobot {
       public:
        Drone(const RobotId id, const std::string& name, const Roadmap::SharedPtr& roadmap)
            : IRobot(id, name, roadmap) {}

        Voxel::SharedPtr getSweptVoxel(const RoadmapDart::SharedPtr& dart) override {
            PointCloudVoxel::SharedPtr cloud = std::make_shared<PointCloudVoxel>();

            DroneState::SharedPtr start_state = std::dynamic_pointer_cast<DroneState>(dart->getStartState());
            DroneState::SharedPtr end_state = std::dynamic_pointer_cast<DroneState>(dart->getEndState());

            // generate fibonacci sphere: https://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere
            float phi = M_PI * (std::sqrt(5.) - 1.);  // golden angle in radian

            Point start_point = start_state->position;
            Point end_point = end_state->position;

            float sweep_length = start_point.distance(end_point);

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

            dart->voxel = cloud;

            return cloud;
        }

        Voxel::SharedPtr getVoxel(const RoadmapDart::SharedPtr& dart, const double time) override {
            PointCloudVoxel::SharedPtr cloud = std::make_shared<PointCloudVoxel>();

            DroneState::SharedPtr start_state = std::dynamic_pointer_cast<DroneState>(dart->getStartState());
            DroneState::SharedPtr end_state = std::dynamic_pointer_cast<DroneState>(dart->getEndState());

            return cloud;
        }

       private:
        void addPointsAtDistance(PointCloudVoxel::SharedPtr& cloud, const float sweptDistance) const {}
    };

    // we will make a drone gpu (another robot) and have
}  // namespace grrt