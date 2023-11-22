#pragma once

#include "robots/robot.h"
#include "voxels/point_cloud/point_cloud_voxel.hpp"

namespace grrt {
    /// @brief State representing a simple drone.
    class DroneState : public RobotState {
       public:
        DroneState(const Point& point, const float radius) : m_position(point), m_radius(radius) {}

        std::string toString() const override { return "DroneState: " + m_position.toString(); }

       private:
        Point m_position;
        float m_radius;
    };

    class Drone : public IRobot {
        PointCloudVoxel::SharedPtr getSweptVoxel(const RoadmapDart::SharedPtr& dart) override {
            return std::make_shared<PointCloudVoxel>();
        }
    };
}  // namespace grrt