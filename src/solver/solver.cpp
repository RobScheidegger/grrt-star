#include "solver/solver.h"
#include "spdlog/spdlog.h"

#include "graphs/search_tree.h"
#include "graphs/search_vertex.h"
#include "pkgs/rapidyaml.hpp"

#include <unordered_set>

using namespace grrt;

Solver::Solver(const SolverConfig::SharedPtr& config) : m_config(config) {
    // Initialize the search graph (generic, will be shared between all problems)
    std::vector<Roadmap::SharedPtr> roadmaps;
    for (const auto& robot : m_config->robots) {
        roadmaps.push_back(robot->roadmap);
    }
    m_searchGraph = std::make_shared<SearchGraph>(roadmaps);
    m_voxelManager = config->robotFactory->makeVoxelManager();
}

SolverResult Solver::tracePath(const SearchTree::SharedPtr& searchTree, const SearchVertex::SharedPtr& start,
                               const SearchVertex::SharedPtr& goal) const {
    // Start at the goal state, and trace back to the start state.
    SolverResult result(true);
    SearchVertex::SharedPtr current = goal;
    while (true) {
        result.path.push_back(current);
        auto parent_dart = searchTree->getParentDart(current);
        if (parent_dart == nullptr) {
            break;
        }

        result.cost += parent_dart->cost;
        current = parent_dart->head;
    }
    return result;
}

bool Solver::expand(SearchTree::SharedPtr& searchTree, const SearchVertex::SharedPtr& goal) {
    // Sample a random vertex.
    SearchVertex::SharedPtr q_rand = m_searchGraph->getRandomVertex();

    // Find the nearest vertex in the search tree.
    SearchVertex::SharedPtr v_near = searchTree->getNearestPoint(q_rand);

    // Sample a vertex adjacent to x_near.
    SearchVertex::SharedPtr v_new = distanceOracle(v_near, q_rand);

    // Add x_new to the search tree.
    if (v_new != nullptr && !searchTree->contains(v_new)) {
        std::cout << "adding a vertex to the tree" << std::endl;
        searchTree->addVertex(v_new, std::make_shared<SearchDart>(v_near, v_new, 1));

        if (v_new == goal) {
            return true;
        }
    }

    return false;
}

SolverResult Solver::solveProblem(const SolverProblem& problem, std::atomic_bool& cancellationToken) {
    SearchTree::SharedPtr T = std::make_shared<SearchTree>(m_searchGraph, problem.start);

    const int num_iterations = 10;

    while (!cancellationToken) {
        for (uint32_t i = 0; i < num_iterations; i++) {
            if (expand(T, problem.goal))
                break;
        }

        if (T->contains(problem.goal)) {
            auto result = tracePath(T, problem.start, problem.goal);
            return tracePath(T, problem.start, problem.goal);
        }
    }

    auto result = SolverResult::fail();
    // We failed, but find the closest node and output the path
    auto nearest = T->getNearestPoint(problem.goal);
    if (nearest != nullptr) {
        result = tracePath(T, problem.start, nearest);
        result.success = false;
        return result;
    }
    return result;
}

SolverSolutions Solver::solve() {

    SolverSolutions solutions = std::make_unique<std::unordered_map<std::string, SolverResult>>();
    this->computeVoxels();
    for (const auto& roadmap : m_searchGraph->roadmaps) {
        roadmap->computeAllPairsShortestPath();
    }

    // set cancellation token to true in 10 seconds
    for (const auto& problem : m_config->problems) {
        std::atomic_bool cancellationToken(false);
        std::thread([&cancellationToken]() {
            std::this_thread::sleep_for(std::chrono::seconds(10));
            cancellationToken = true;
        }).detach();

        auto result = solveProblem(problem, cancellationToken);
        solutions->insert(std::make_pair(problem.name, result));
    }

    return solutions;
}

void Solver::computeVoxels() {
    // make a set of roadmaps
    std::unordered_set<Roadmap::SharedPtr> roadmaps;
    for (auto& robot : m_config->robots) {
        if (!roadmaps.count(robot->roadmap)) {
            for (auto& dart : robot->roadmap->darts) {
                robot->getSweptVoxel(dart);
            }
        }
        roadmaps.insert(robot->roadmap);
    }
}

#define ORACLE_VERTEX_ATTEMPTS 10

SearchVertex::SharedPtr Solver::distanceOracle(const SearchVertex::SharedPtr& nearVertex,
                                               const SearchVertex::SharedPtr& randomVertex) const {

    // Find the vertex in the graph that minimizes the angle between nearVertex and randomVertex.
    // This is the vertex that is closest to randomVertex in the graph.

    size_t attempts = 0;
    while (attempts++ < ORACLE_VERTEX_ATTEMPTS) {
        auto newVertex = m_searchGraph->sampleAdjacentVertex(nearVertex);
        if (newVertex == nullptr) {
            // Help
            spdlog::warn("sampleAdjacentVertex returned nullptr");
            continue;
        }

        // Check that the edge between the near vertex and new vertex is collision free.
        auto newDart = std::make_shared<SearchDart>(nearVertex, newVertex, 1);
        auto darts = m_searchGraph->getRoadmapDarts(nearVertex, newVertex);
        if (checkCollisionFreeDarts(darts)) {
            return newVertex;
        }
    }

    return nullptr;
};

bool Solver::checkCollisionFreeDarts(const std::vector<RoadmapDart::SharedPtr> darts) const {
    for (int i = 0; i < darts.size(); i++) {
        for (int j = i + 1; j < darts.size(); j++) {
            if (m_voxelManager->intersect(darts[i]->voxel, darts[j]->voxel)) {
                return false;
            }
        }
    }
    return true;
};