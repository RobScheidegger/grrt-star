#pragma once

#include <cassert>
#include <memory>
#include <string>
#include <unordered_map>

#include "graphs/roadmap_vertex.h"
#include "state/robot_state.h"
#include "voxels/voxel.h"

namespace grrt {

    typedef int16_t RoadmapDartId;

    /// @brief Representation of a roadmap edge that
    class RoadmapDart {
       public:
        typedef std::shared_ptr<RoadmapDart> SharedPtr;

        RoadmapDart(const RoadmapDartId id, const RoadmapVertex::SharedPtr& start, const RoadmapVertex::SharedPtr& end,
                    const double cost)
            : m_start(start), m_end(end), cost(cost), m_id(id) {}

        /// @brief Get a string representation of the robot state.
        /// @return A string representation of the robot state.
        std::string toString() const {
            return m_start->getState()->toString() + " -> " + m_end->getState()->toString();
        }

        void setVoxel(const Voxel::SharedPtr& voxel) { m_voxel = voxel; }

        RobotState::SharedPtr getStartState() const { return m_start->getState(); }

        RobotState::SharedPtr getEndState() const { return m_end->getState(); }

        const RoadmapDartId getId() const { return m_id; }

        const RoadmapVertex::SharedPtr m_start;

        const RoadmapVertex::SharedPtr m_end;
        const double cost;

       private:
        const RoadmapDartId m_id;

        Voxel::SharedPtr m_voxel = nullptr;
    };
}  // namespace grrt