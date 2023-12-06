#pragma once

#include <memory>
#include <vector>

#include "graphs/roadmap_vertex.h"

namespace grrt {

    struct SearchVertex {
        typedef std::shared_ptr<SearchVertex> SharedPtr;

        SearchVertex(const std::vector<RoadmapVertexId>& roadmapStates) : roadmapStates(roadmapStates) {}

        const std::vector<RoadmapVertexId> roadmapStates;
    };

    class SearchVertexHash {
       public:
        size_t operator()(const SearchVertex::SharedPtr& vertex) const {
            auto hasher = std::hash<RoadmapVertexId>();
            size_t hash = 0;
            for (const auto& roadmapState : vertex->roadmapStates) {
                hash ^= hasher(roadmapState);
            }
            return hash;
        }
    };
}  // namespace grrt