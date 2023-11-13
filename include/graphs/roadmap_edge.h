#pragma once

#include <memory>
#include <string>

#include "state/robot_state.h"

#define EMPTY_EDGE_ID -1ull

namespace grrt {

    /// @brief Representation of a roadmap edge that
    template <IRobotState RobotStateType>
    class RoadmapEdge {
       public:
        typedef std::shared_ptr<RoadmapEdge<RobotStateType>> SharedPtr;

        RoadmapEdge(const RobotStateType& start, const RobotStateType& end, const float t, const float dt = 0.02)
            : m_start(start), m_end(end), m_t(t), m_dt(dt) {}

        /// @brief Get a string representation of the robot state.
        /// @return A string representation of the robot state.
        std::string toString() const {}

       private:
        const RobotStateType m_start;
        const RobotStateType m_end;

        const float m_dt;
        const float m_t;
    };
}  // namespace grrt