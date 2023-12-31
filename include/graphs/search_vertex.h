#pragma once

#include <memory>
#include <vector>

#include "graphs/roadmap_vertex.h"

namespace grrt {

    struct SearchVertex {
        typedef std::shared_ptr<SearchVertex> SharedPtr;

        SearchVertex(const std::vector<RoadmapVertexId>& roadmapStates) : roadmapStates(roadmapStates) {}

        inline std::string toString() const {
            std::string str = "[";
            for (const auto& roadmapState : roadmapStates) {
                str += std::to_string(roadmapState) + ", ";
            }
            str += "]";
            return str;
        }

        const std::vector<RoadmapVertexId> roadmapStates;
    };

    inline bool operator==(const SearchVertex::SharedPtr& lhs, const SearchVertex::SharedPtr& rhs) {
        return lhs->roadmapStates == rhs->roadmapStates;
    }

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