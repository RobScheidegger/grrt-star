#include "solver/solver.h"

#include "graphs/search_tree.h"

using namespace grrt;

Solver::Solver(const SolverConfig::SharedPtr& config) : m_config(config) {
    // Initialize the search graph (generic, will be shared between all problems)
    std::vector<Roadmap::SharedPtr> roadmaps;
    for (const auto& pair : m_config->roadmaps) {
        roadmaps.push_back(pair.second);
    }
    m_searchGraph = std::make_shared<SearchGraph>(roadmaps);
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

void Solver::expand(SearchTree::SharedPtr& searchTree) {
    // Sample a random vertex.
    SearchVertex::SharedPtr q_rand = m_searchGraph->getRandomVertex();

    // Find the nearest vertex in the search tree.
    SearchVertex::SharedPtr v_near = searchTree->getNearestPoint(q_rand);

    // Sample a vertex adjacent to x_near.
    SearchVertex::SharedPtr v_new = m_searchGraph->sampleAdjacentVertex(v_near);

    // Add x_new to the search tree.
    if (v_new != nullptr && !searchTree->contains(v_new)) {
        searchTree->addVertex(v_new, std::make_shared<SearchDart>(v_near, v_new, 0));
    }
}

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

    return solutions;
}