#pragma once

#include "graphs/roadmap.h"
#include "graphs/roadmap_dart.h"
#include "state/robot_state.h"
#include "voxels/voxel.h"

namespace grrt {

    /// @brief A unique ID for a robot.
    typedef uint64_t RobotId;

    /// @brief Generic robot interface to represent a single robot in a scene.
    class IRobot {
       public:
        typedef std::shared_ptr<IRobot> SharedPtr;

        std::string name;
        RobotId id;
        Roadmap::SharedPtr roadmap;

        IRobot(const RobotId id, const std::string& name, const Roadmap::SharedPtr& roadmap)
            : name(name), id(id), roadmap(roadmap) {}

        /// @brief Gets the swept voxel region for a given roadmap edge.
        /// @param dart The roadmap dart edge to get the swept voxel region for.
        /// @return The swept voxel region for the given roadmap dart.
        virtual Voxel::SharedPtr getSweptVoxel(const RoadmapDart::SharedPtr& dart) = 0;
    };
}  // namespace grrt