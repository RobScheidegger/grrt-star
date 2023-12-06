#pragma once

#include "robots/robot.h"
#include "voxels/point_cloud/point_cloud_voxel_gpu.h"

namespace grrt {
    class DroneGPU : public IRobot {
       public:
        DroneGPU(const RobotId id, const std::string& name, const Roadmap::SharedPtr& roadmap)
            : IRobot(id, name, roadmap) {}

        Voxel::SharedPtr getSweptVoxel(const RoadmapDart::SharedPtr& dart) override {
            return std::make_shared<PointCloudVoxelGPU>();
            // TODO: siddharth
        }
    };
}  // namespace grrt