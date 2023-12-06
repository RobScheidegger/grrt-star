#pragma once

#include <memory>

#include "state/robot_state.h"

namespace grrt {

    typedef uint64_t RoadmapVertexId;

    class RoadmapVertex {
       public:
        typedef std::shared_ptr<RoadmapVertex> SharedPtr;

        RoadmapVertex(const RoadmapVertexId id, const RobotState::SharedPtr& state) : m_id(id), m_state(state) {}

        RobotState::SharedPtr getState() const { return m_state; }

        const RoadmapVertexId m_id;

       private:
        const RobotState::SharedPtr m_state;
    };
}  // namespace grrt