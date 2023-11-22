#pragma once

#include <concepts>
#include <memory>
#include <vector>

#include "graphs/roadmap_dart.h"
#include "graphs/roadmap_vertex.h"
#include "state/robot_state.h"
#include "voxels/voxel.h"

namespace grrt {

    class Roadmap {
       public:
        typedef std::shared_ptr<Roadmap> SharedPtr;

        Roadmap(const std::string name) : m_name(name) {}

        RoadmapVertex::SharedPtr addVertex(const RobotState::SharedPtr& state);
        RoadmapDart::SharedPtr addDart(const RobotState::SharedPtr& state1, const RobotState::SharedPtr& state2,
                                       EdgeParameters& parameters);

        std::vector<RoadmapVertex::SharedPtr> vertices;
        std::vector<RoadmapDart::SharedPtr> darts;

       private:
        const std::string m_name;
    };
}  // namespace grrt