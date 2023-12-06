#pragma once

#include "robots/robot.h"
#include "voxels/point_cloud/point_cloud_voxel_gpu.hpp"

namespace grrt {
    class DroneGPU : public IRobot {
       public:
        Drone(const RobotId id, const std::string& name, const Roadmap::SharedPtr& roadmap)
            : IRobot(id, name, roadmap) {}

        PointCloudVoxel::SharedPtr getSweptVoxel(const RoadmapDart::SharedPtr& dart) override {
            return std::make_shared<PointCloudVoxel>();
            // TODO: siddharth
        }
    };
}  // namespace grrt