#pragma once

#include "robots/robot.h"
#include "voxels/point_cloud/point_cloud_voxel.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

// TODO: produce the swept voxel for the drone and making an intersection to check if there is an intersepction.

// need to check if an edge is valid. A voxel is just an edge.

#define VOXEL_SWEEP_STEP_SIZE 0.05
#define VOXEL_POINTS_SAMPLING_SIZE 1000

namespace grrt {
    /// @brief State representing a simple drone.
    class DroneState : public RobotState {
       public:
        DroneState(const Point& point, const float radius) : m_position(point), m_radius(radius) {}

        std::string toString() const override { return "DroneState: " + m_position.toString(); }

        Point getPosition() const { return m_position; }
        float getRadius() const { return m_radius; }

       private:
        Point m_position;
        float m_radius;
    };

    // point cloud resolution is the effective radius of each point in the point cloud
    // and then there is also the step size wihich is on the same order.

    class Drone : public IRobot {
       public:
        Drone(const RobotId id, const std::string& name, const Roadmap::SharedPtr& roadmap)
            : IRobot(id, name, roadmap) {}

        PointCloudVoxel::SharedPtr getSweptVoxel(const RoadmapDart::SharedPtr& dart) override {
            PointCloudVoxel::SharedPtr cloud = std::make_shared<PointCloudVoxel>();

            DroneState::SharedPtr start_state = std::dynamic_pointer_cast<DroneState>(dart->getStartState());
            DroneState::SharedPtr end_state = std::dynamic_pointer_cast<DroneState>(dart->getEndState());

            // generate fibonacci sphere: https://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere
            float phi = M_PI * (sqrt(5.) - 1.);  // golden angle in radian

            Point start_point = start_state->getPosition();
            Point end_point = end_state->getPosition();

            float sweep_length = sqrt(pow(end_point.x - start_point.x, 2) + pow(end_point.y - start_point.y, 2) +
                                      pow(end_point.z - start_point.z, 2));

            float swept_distance = 0;
            while (swept_distance < sweep_length) {
                float a = swept_distance;
                float b = sweep_length - swept_distance;

                float x_delta = a / (a + b) * (end_point.x - start_point.x);
                float y_delta = a / (a + b) * (end_point.y - start_point.y);
                float z_delta = a / (a + b) * (end_point.z - start_point.z);

                Point offset = {start_point.x + x_delta, start_point.y + y_delta, start_point.z + z_delta};

                for (int i = 1; i <= VOXEL_POINTS_SAMPLING_SIZE; i++) {
                    float y = 1 - (i / float(VOXEL_POINTS_SAMPLING_SIZE - 1)) * 2;  // y goes from 1 to - 1

                    float radius = sqrt(1 - y * y);  // radius at y
                    float theta = phi * i;           // golden angle increment

                    float x = cos(theta) * radius;
                    float z = sin(theta) * radius;
                    cloud->addPoint(Point(x * start_state->getRadius() + offset.x,
                                          y * start_state->getRadius() + offset.y,
                                          z * start_state->getRadius() + offset.z));
                }

                swept_distance += VOXEL_SWEEP_STEP_SIZE;
            }

            return cloud;
        }
    };

    // we will make a drone gpu (another robot) and have
}  // namespace grrt