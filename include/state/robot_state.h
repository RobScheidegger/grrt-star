#pragma once

#include <memory>
#include <optional>
#include <string>
#include <type_traits>

#include "point.h"

namespace grrt {
    /// @brief Type definition for a an Id for a roadmap state.
    typedef uint64_t RoadmapStateId;

    /// @brief Generic base class representing a particular robot's state.
    class RobotState {
       public:
        typedef std::shared_ptr<RobotState> SharedPtr;
        /// @brief Get the distance between this robot state and another robot state.
        /// @brief Get a string representation of the robot state.
        /// @return A string representation of the robot state.
        virtual std::string toString() const = 0;

        virtual double distance(const RobotState::SharedPtr& other) const = 0;

        virtual Point getPosition() const = 0;

        /// @brief Construct a new Robot State object
        void setId(const RoadmapStateId id) { m_id = id; }

        /// @brief Gets the unique Id for this robot state (in the roadmap graph).
        /// @return The unique Id for this robot state (in the roadmap graph).
        RoadmapStateId getId() const { return m_id; }

       private:
        RoadmapStateId m_id = -1;
    };

}  // namespace grrt