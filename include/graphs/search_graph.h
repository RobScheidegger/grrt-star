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

        /// @brief Randomly samples a vertex from the search (tensor) graph.
        /// @return A randomly sampled vertex.
        SearchVertex::SharedPtr getRandomVertex() const;

        /// @brief Randomly samples a vertex from the search (tensor) graph that is adjacent to the given vertex.
        /// Note that this vertex is _not_ guarenteed to have a collision-free edge to the given vertex.
        /// @param vertex The vertex to sample an adjacent vertex to.
        /// @return A randomly sampled vertex adjacent to the given vertex.
        SearchVertex::SharedPtr sampleAdjacentVertex(const SearchVertex::SharedPtr& vertex) const;

        std::vector<RoadmapDart::SharedPtr> getRoadmapDarts(const SearchVertex::SharedPtr& start,
                                                            const SearchVertex::SharedPtr& end);

        const std::vector<Roadmap::SharedPtr> roadmaps;
    };
}  // namespace grrt