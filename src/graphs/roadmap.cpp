#include "graphs/roadmap.h"

using namespace grrt;

RoadmapVertex::SharedPtr Roadmap::addVertex(const std::string& name, const RobotState::SharedPtr& state) {
    auto vertex = std::make_shared<RoadmapVertex>(vertices.size(), state);
    vertices.push_back(vertex);
    m_verticesByName[name] = vertex;
    return vertex;
}

RoadmapDart::SharedPtr Roadmap::addDart(const RoadmapVertex::SharedPtr& state1, const RoadmapVertex::SharedPtr& state2,
                                        const EdgeParameters& parameters) {
    auto dart = std::make_shared<RoadmapDart>(darts.size(), state1, state2, parameters);
    darts.push_back(dart);
    return dart;
}

void Roadmap::dijkstra(const RoadmapVertex::SharedPtr& start, const RoadmapVertex::SharedPtr& goal) {
    // Dijkstra's Algorithm
    // Initialize all distances to infinity
}