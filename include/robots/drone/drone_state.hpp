#pragma once

#include "point.h"
#include "state/robot_state.h"

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
}  // namespace grrt