#pragma once

#include <atomic>
#include <memory>
#include <unordered_map>

#include "config/solver_config.h"
#include "graphs/search_graph.h"
#include "graphs/search_tree.h"
#include "solver/solver_result.h"

namespace grrt {

    enum SolverMode { DRRT };

    typedef std::unique_ptr<std::unordered_map<std::string, SolverResult::SharedPtr>> SolverSolutions;

    /// @brief The solver is responsible for solving a particular motion planning problem with dRRT or other solvers.
    class Solver {
       public:
        typedef std::shared_ptr<Solver> SharedPtr;

        Solver(const SolverConfig::SharedPtr& config);

        SolverSolutions solve();

        SearchVertex::SharedPtr distanceOracle(const SearchVertex::SharedPtr& nearVertex,
                                               const SearchVertex::SharedPtr& randomVertex) const;

       private:
        const SolverConfig::SharedPtr m_config;
        SearchGraph::SharedPtr m_searchGraph;
        VoxelManager::SharedPtr m_voxelManager;

        SolverResult::SharedPtr solveProblem(const SolverProblem& problem, std::atomic_bool& cancellationToken);

        bool expand(SearchTree::SharedPtr& searchTree, const SearchVertex::SharedPtr& goal);

        void computeVoxels();

        SolverResult::SharedPtr tracePath(const SearchTree::SharedPtr& searchTree, const SearchVertex::SharedPtr& start,
                               const SearchVertex::SharedPtr& goal) const;

        bool checkCollisionFreeDarts(const std::vector<RoadmapDart::SharedPtr> darts) const;
    };
}  // namespace grrt