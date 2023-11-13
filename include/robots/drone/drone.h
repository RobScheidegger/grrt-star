#pragma once

#include <type_traits>

#include "robots/drone/drone_state.hpp"
#include "robots/robot.h"
#include "voxels/managers/point_cloud_voxel.hpp"

namespace grrt {
    class Drone : public IRobot<DroneState, PointCloudVoxel> {
        PointCloudVoxel getSweptVoxel(const RoadmapEdge<DroneState>& edge) override { return PointCloudVoxel(); }

        RoadmapEdge<DroneState> getEdge(const DroneState& start, const DroneState& end) override {}
    };
}  // namespace grrt