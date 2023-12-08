#include <spdlog/spdlog.h>

#include "graphs/roadmap_vertex.h"
#include "graphs/search_graph.h"
#include "graphs/search_vertex.h"

using namespace grrt;

SearchVertex::SharedPtr SearchGraph::getRandomVertex() const {
    // For each roadmap, sample a vertex and add it to the tensor product.
    std::vector<RoadmapVertexId> roadmapStates;
    for (const auto& roadmap : roadmaps) {
        size_t roadmapSize = roadmap->vertices.size();
        roadmapStates.push_back(rand() % roadmapSize);
    }
    return std::make_shared<SearchVertex>(roadmapStates);
}

SearchVertex::SharedPtr SearchGraph::sampleAdjacentVertex(const SearchVertex::SharedPtr& vertex,
                                                          const std::vector<bool>& mask) const {
    auto adjacent_vertices_id = std::vector<RoadmapVertexId>();

    for (int i = 0; i < roadmaps.size(); i++) {
        if (!mask[i]) {
            adjacent_vertices_id.push_back(vertex->roadmapStates[i]);
            continue;
        }

        auto roadmap = roadmaps[i];
        auto darts = roadmap->getAdjacentDarts(roadmap->vertices[vertex->roadmapStates[i]]);
        // randomly select a dart
        if (darts.size() > 0) {
            int dartIndex = rand() % darts.size();
            auto dart = darts[dartIndex];
            auto adjacentVertex = dart->m_end;
            auto adjacentRoadmapState = adjacentVertex->m_id;
            adjacent_vertices_id.push_back(adjacentRoadmapState);
        }
    }

    return std::make_shared<SearchVertex>(adjacent_vertices_id);
}

std::vector<RoadmapDart::SharedPtr> SearchGraph::getRoadmapDarts(const SearchVertex::SharedPtr& start,
                                                                 const SearchVertex::SharedPtr& end) {
    std::vector<RoadmapDart::SharedPtr> darts;

    const size_t num_robots = roadmaps.size();
    for (size_t i = 0; i < num_robots; i++) {
        auto roadmap = roadmaps[i];
        auto start_vertex = roadmap->vertices[start->roadmapStates[i]];
        auto end_vertex = roadmap->vertices[end->roadmapStates[i]];

        auto dart = roadmap->getDart(start_vertex, end_vertex);
        if (dart == nullptr) {
            spdlog::error("Failed to find dart from {} to {}", start_vertex->m_id, end_vertex->m_id);
            exit(1);
        }
        darts.push_back(dart);
    }

    return darts;
}