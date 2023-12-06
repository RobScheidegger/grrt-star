#pragma once

#include "robots/robot.h"
#include "voxels/point_cloud/point_cloud_voxel.hpp"

// TODO: produce the swept voxel for the drone and making an intersection to check if there is an intersepction.

// need to check if an edge is valid. A voxel is just an edge.

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
       public:
        Drone(const RobotId id, const std::string& name, const Roadmap::SharedPtr& roadmap)
            : IRobot(id, name, roadmap) {}

        PointCloudVoxel::SharedPtr getSweptVoxel(const RoadmapDart::SharedPtr& dart) override {
            return std::make_shared<PointCloudVoxel>();
            // TODO: siddharth
        }
    };
}  // namespace grrt