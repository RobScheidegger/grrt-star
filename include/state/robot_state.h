#pragma once

#include <optional>
#include <string>
#include <type_traits>

namespace grrt {

    /// @brief Generic base class representing a particular robot's state.
    class RobotState {
       public:
        /// @brief Get a string representation of the robot state.
        /// @return A string representation of the robot state.
        virtual std::string toString() const = 0;

        /// @brief Construct a new Robot State object
        void setId(const uint64_t id) { m_id = id; }

        /// @brief Gets the unique Id for this robot state (in the roadmap graph).
        /// @return The unique Id for this robot state (in the roadmap graph).
        uint64_t getId() const { return m_id; }

       private:
        uint64_t m_id = -1;
    };

    /// @brief Concept for a Robot State.
    template <typename T>
    concept IRobotState = std::is_base_of<RobotState, T>::value;

}  // namespace grrt