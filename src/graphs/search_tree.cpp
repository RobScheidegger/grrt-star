#include "graphs/search_tree.h"
#include <algorithm>
#include "graphs/search_vertex.h"

using namespace grrt;

SearchVertex::SharedPtr SearchTree::getNearestPoint(const SearchVertex::SharedPtr& vertex) const {
    double best_distance = std::numeric_limits<float>::infinity();
    SearchVertex::SharedPtr best_vertex = nullptr;
    auto& roadmaps = m_searchGraph->roadmaps;
    for (auto& [treeVertex, _] : m_vertexParents) {

        double vertex_distance = 0;
        for (uint32_t i = 0; i < roadmaps.size(); i++) {
            const auto& vertex_state_id = vertex->roadmapStates[i];
            const auto& tree_vertex_state_id = treeVertex->roadmapStates[i];

            const auto distance = roadmaps[i]->distances[tree_vertex_state_id][vertex_state_id];
            vertex_distance = std::max(vertex_distance, distance);
        }

        if (vertex_distance < best_distance) {
            best_distance = vertex_distance;
            best_vertex = treeVertex;
        }

        if (best_distance == 0)
            break;
    }
    assert(best_vertex != nullptr);
    return best_vertex;
}