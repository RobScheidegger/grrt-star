#include "graphs/roadmap.h"
#include <cassert>
#include <limits>
#include <queue>

using namespace grrt;

RoadmapVertex::SharedPtr Roadmap::addVertex(const std::string& name, const RobotState::SharedPtr& state) {
    auto vertex = std::make_shared<RoadmapVertex>(vertices.size(), state);
    vertices.push_back(vertex);
    m_verticesByName[name] = vertex;
    adjacencyList.push_back(std::vector<RoadmapDart::SharedPtr>());
    return vertex;
}

RoadmapDart::SharedPtr Roadmap::addDart(const RoadmapVertex::SharedPtr& v1, const RoadmapVertex::SharedPtr& v2,
                                        const double cost) {
    auto dart = std::make_shared<RoadmapDart>(darts.size(), v1, v2, cost);
    darts.push_back(dart);
    assert(v1->m_id < adjacencyList.size());
    adjacencyList[v1->m_id].push_back(dart);
    return dart;
}

RoadmapDart::SharedPtr Roadmap::addDart(const RoadmapVertex::SharedPtr& v1, const RoadmapVertex::SharedPtr& v2) {
    return this->addDart(v1, v2, v1->getState()->distance(v2->getState()));
}

void Roadmap::computeAllPairsShortestPath() {
    this->distances = std::vector<std::vector<double>>(
        vertices.size(), std::vector<double>(vertices.size(), std::numeric_limits<double>::infinity()));
    for (auto& vertex : vertices) {
        dijkstra(vertex);
    }
}

void Roadmap::dijkstra(const RoadmapVertex::SharedPtr& start) {
    std::priority_queue<std::pair<double, RoadmapVertex::SharedPtr>,
                        std::vector<std::pair<double, RoadmapVertex::SharedPtr>>,
                        std::greater<std::pair<double, RoadmapVertex::SharedPtr>>>
        queue;

    this->distances[start->m_id][start->m_id] = 0.0;
    queue.push(std::make_pair(0.0, start));

    while (!queue.empty()) {
        auto current = queue.top().second;
        queue.pop();

        for (auto& dart : adjacencyList[current->m_id]) {
            assert(dart->m_start->m_id == current->m_id);
            auto neighbor = dart->m_end;
            auto distance = this->distances[start->m_id][current->m_id] + dart->cost;
            if (distance < this->distances[start->m_id][neighbor->m_id]) {
                this->distances[start->m_id][neighbor->m_id] = distance;
                queue.push(std::make_pair(distance, neighbor));
            }
        }
    }
}