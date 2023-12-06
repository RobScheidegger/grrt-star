#include "graphs/search_graph.h"

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
    // TODO
    return nullptr;
}