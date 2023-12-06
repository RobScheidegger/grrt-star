#pragma once

#include <memory>
#include <vector>

#include "graphs/roadmap.h"
#include "graphs/search_vertex.h"

namespace grrt {
    class SearchGraph {
       public:
        typedef std::shared_ptr<SearchGraph> SharedPtr;

        SearchGraph(const std::vector<Roadmap::SharedPtr>& roadmaps) : roadmaps(roadmaps) {}

        SearchVertex::SharedPtr getRandomVertex() const;

        SearchVertex::SharedPtr getNearestVertex() const;

        const std::vector<Roadmap::SharedPtr> roadmaps;
    };
}  // namespace grrt