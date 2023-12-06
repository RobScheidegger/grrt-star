#include "solver/solver.h"

#include "graphs/search_tree.h"

using namespace grrt;

Solver::Solver(const SolverConfig::SharedPtr& config) : m_config(config) {
    // Initialize the search graph (generic, will be shared between all problems)
    m_searchGraph = std::make_shared<SearchGraph>(m_config->roadmaps);
}

SolverResult Solver::tracePath(const SearchTree::SharedPtr& searchTree, const SearchVertex::SharedPtr& start,
                               const SearchVertex::SharedPtr& goal) const {
    // Start at the goal state, and trace back to the start state.
    SolverResult result(true);
    SearchVertex::SharedPtr current = goal;
    while (current != nullptr) {
        result.path.push_back(current);
        auto parent_dart = searchTree->getParentDart(current);
        if (parent_dart != nullptr) {
            result.cost += parent_dart->cost;
        }
        current = parent_dart->head;
    }

    return result;
}

void Solver::expand(SearchTree::SharedPtr& searchTree) {}

SolverResult Solver::solveProblem(const SolverProblem& problem, std::atomic_bool& cancellationToken) {
    SearchTree::SharedPtr T = std::make_shared<SearchTree>(m_searchGraph, problem.start);

    const int num_iterations = 100;

    while (!cancellationToken) {
        for (uint32_t i = 0; i < num_iterations; i++) {
            expand(T);
        }

        if (T->contains(problem.goal)) {
            return tracePath(T, problem.start, problem.goal);
        }
    }

    return SolverResult::fail();
}

SolverSolutions Solver::solve() {

    std::atomic_bool cancellationToken(false);

    SolverSolutions solutions = std::make_unique<std::unordered_map<std::string, SolverResult>>();

    for (const auto& problem : m_config->problems) {

        auto result = solveProblem(problem, cancellationToken);
        solutions->insert(std::make_pair(problem.name, result));
    }
}