#pragma once

#include <memory>

#include "graphs/search_vertex.h"

namespace grrt {

    struct SearchDart {
        typedef std::shared_ptr<SearchDart> SharedPtr;

        SearchDart(const SearchVertex::SharedPtr& head, const SearchVertex::SharedPtr& tail, const double cost)
            : head(head), tail(tail), cost(cost) {}

        inline std::string toString() const { return head->toString() + " -> " + tail->toString(); }

        const SearchVertex::SharedPtr head;
        const SearchVertex::SharedPtr tail;
        const double cost;
    };

}  // namespace grrt