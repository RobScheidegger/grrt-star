#include "graphs/search_graph.h"
#include "graphs/roadmap_vertex.h"
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

SearchVertex::SharedPtr SearchGraph::sampleAdjacentVertex(const SearchVertex::SharedPtr& vertex) const {
    auto adjacent_vertices_id = std::vector<RoadmapVertexId>();

    for (int i = 0; i < roadmaps.size(); i++) {
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