#pragma once

#include <concepts>
#include <memory>
#include <vector>

#include "graphs/roadmap_dart.h"
#include "graphs/roadmap_vertex.h"
#include "robots/robot_types.h"
#include "state/robot_state.h"
#include "voxels/voxel.h"

namespace grrt {

    class Roadmap {
       public:
        typedef std::shared_ptr<Roadmap> SharedPtr;

        Roadmap(const std::string& name, const RobotType type) : robotType(type), name(name) {}

        RoadmapVertex::SharedPtr addVertex(const std::string& name, const RobotState::SharedPtr& state);
        RoadmapDart::SharedPtr addDart(const RoadmapVertex::SharedPtr& state1, const RoadmapVertex::SharedPtr& state2);

        std::vector<RoadmapDart::SharedPtr> getAdjacentDarts(const RoadmapVertex::SharedPtr& vertex) const {
            return adjacencyList[vertex->m_id];
        }

        void computeAllPairsShortestPath();

        RoadmapVertex::SharedPtr getVertex(const std::string& name) const {
            if (!m_verticesByName.contains(name))
                return nullptr;
            return m_verticesByName.at(name);
        }

        std::vector<RoadmapVertex::SharedPtr> vertices;
        std::vector<RoadmapDart::SharedPtr> darts;
        std::vector<std::vector<RoadmapDart::SharedPtr>> adjacencyList;
        std::vector<std::vector<double>> distances;
        RobotType robotType;
        std::string name;

       private:
        std::unordered_map<std::string, RoadmapVertex::SharedPtr> m_verticesByName;

        void dijkstra(const RoadmapVertex::SharedPtr& start);
    };

}  // namespace grrt