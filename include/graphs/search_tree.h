#pragma once

#include <memory>

#include "graphs/search_dart.h"
#include "graphs/search_graph.h"
#include "graphs/search_vertex.h"

namespace grrt {
    class SearchTree {
       public:
        typedef std::shared_ptr<SearchTree> SharedPtr;

        SearchTree(const SearchGraph::SharedPtr& searchGraph, const SearchVertex::SharedPtr& root)
            : m_searchGraph(searchGraph) {
            m_vertexParents[root] = nullptr;
        }

        /// @brief Gets the nearest point currently in the search tree to another given point.
        /// @param vertex The point to find the nearest point to.
        /// @return The nearest point currently in the tree to the given point.
        SearchVertex::SharedPtr getNearestPoint(const SearchVertex::SharedPtr& vertex) const;

        inline bool contains(const SearchVertex::SharedPtr& vertex) const {
            return m_vertexParents.find(vertex) != m_vertexParents.end();
        }

        inline void addVertex(const SearchVertex::SharedPtr& vertex, const SearchDart::SharedPtr& parentDart) {
            m_vertexParents[vertex] = parentDart;
        }

        inline SearchVertex::SharedPtr getParent(const SearchVertex::SharedPtr& vertex) const {
            return m_vertexParents.at(vertex)->tail;
        }

        inline SearchDart::SharedPtr getParentDart(const SearchVertex::SharedPtr& vertex) const {
            return m_vertexParents.at(vertex);
        }

        inline size_t size() const { return m_vertexParents.size(); }

       private:
        const SearchGraph::SharedPtr m_searchGraph;
        std::unordered_map<SearchVertex::SharedPtr, SearchDart::SharedPtr, SearchVertexHash> m_vertexParents;
    };
}  // namespace grrt