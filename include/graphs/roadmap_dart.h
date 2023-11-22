#pragma once

#include <cassert>
#include <memory>
#include <string>
#include <unordered_map>

#include "graphs/roadmap_vertex.h"
#include "state/robot_state.h"
#include "voxels/voxel.h"

#define EMPTY_EDGE_ID -1ull

namespace grrt {

    typedef std::unordered_map<std::string, std::string> EdgeParameters;
    typedef uint64_t RoadmapDartId;

    /// @brief Representation of a roadmap edge that
    class RoadmapDart {
       public:
        typedef std::shared_ptr<RoadmapDart> SharedPtr;

        RoadmapDart(const RoadmapDartId id, const RoadmapVertex::SharedPtr& start, const RoadmapVertex::SharedPtr& end,
                    const EdgeParameters& parameters)
            : m_start(start), m_end(end), m_parameters(parameters), m_id(id) {}

        /// @brief Get a string representation of the robot state.
        /// @return A string representation of the robot state.
        std::string toString() const {
            return m_start->getState()->toString() + " -> " + m_end->getState()->toString();
        }

        void setVoxel(const Voxel::SharedPtr& voxel) { m_voxel = voxel; }

        RobotState::SharedPtr getStartState() const { return m_start->getState(); }

        RobotState::SharedPtr getEndState() const { return m_end->getState(); }

        const EdgeParameters& getParameters() const { return m_parameters; }

        const RoadmapDartId getId() const { return m_id; }

       private:
        const RoadmapVertex::SharedPtr m_start;
        const RoadmapVertex::SharedPtr m_end;
        const EdgeParameters m_parameters;
        const RoadmapDartId m_id;

        Voxel::SharedPtr m_voxel = nullptr;
    };
}  // namespace grrt