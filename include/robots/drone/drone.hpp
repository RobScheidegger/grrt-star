#pragma once

#include "robots/robot.h"
#include "voxels/point_cloud/point_cloud_voxel.hpp"

// TODO: produce the swept voxel for the drone and making an intersection to check if there is an intersepction.

// need to check if an edge is valid. A voxel is just an edge.

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

        Point position;
        float radius;
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