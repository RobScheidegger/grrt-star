#include "solver/solver.h"
#include "spdlog/spdlog.h"

#include "graphs/search_tree.h"
#include "graphs/search_vertex.h"
#include "pkgs/rapidyaml.hpp"

#include <unordered_set>

#define ORACLE_VERTEX_ATTEMPTS 10

using namespace grrt;

Solver::Solver(const SolverConfig::SharedPtr& config) : m_config(config) {
    // Initialize the search graph (generic, will be shared between all problems)
    std::vector<Roadmap::SharedPtr> roadmaps;
    for (const auto& robot : m_config->robots) {
        roadmaps.push_back(robot->roadmap);
    }
    m_searchGraph = std::make_shared<SearchGraph>(roadmaps);
    m_voxelManager = config->robotFactory->makeVoxelManager();
    m_robotsToMove = 3;  // TODO std::max(std::sqrt(m_config->robots.size()), 3);
}

SolverResult::SharedPtr Solver::tracePath(const SearchTree::SharedPtr& searchTree, const SearchVertex::SharedPtr& start,
                                          const SearchVertex::SharedPtr& goal) const {
    // Start at the goal state, and trace back to the start state.
    assert(start != nullptr);
    assert(goal != nullptr);
    SolverResult::SharedPtr result = std::make_shared<SolverResult>(true);
    SearchVertex::SharedPtr current = goal;
    while (current != nullptr) {
        result->path.push_back(current);
        auto parent_dart = searchTree->getParentDart(current);
        if (parent_dart == nullptr) {
            break;
        }

        result->cost += parent_dart->cost;
        current = parent_dart->head;
    }

    // TEST: Check that none of the things are nullptr
    for (const auto& vertex : result->path) {
        assert(vertex != nullptr);
    }

    return result;
}

bool Solver::expand(SearchTree::SharedPtr& searchTree, const SearchVertex::SharedPtr& goal) {
    // Sample a random vertex.
    SearchVertex::SharedPtr q_rand = m_searchGraph->getRandomVertex();
    // Find the nearest vertex in the search tree.
    SearchVertex::SharedPtr v_near = searchTree->getNearestPoint(q_rand);
    // Sample a vertex adjacent to x_near.
    SearchVertex::SharedPtr v_new = distanceOracle(v_near, q_rand, goal);
    // Add x_new to the search tree.
    if (v_new != nullptr && !searchTree->contains(v_new)) {
        searchTree->addVertex(v_new, std::make_shared<SearchDart>(v_near, v_new, 1));

        if (v_new == goal) {
            return true;
        }
    }

    return false;
}

SolverResult::SharedPtr Solver::solveProblem(const SolverProblem& problem, std::atomic_bool& cancellationToken) {
    SearchTree::SharedPtr T = std::make_shared<SearchTree>(m_searchGraph, problem.start);

    const int num_iterations = 10;

    while (!m_cancelled) {
        for (uint32_t i = 0; i < num_iterations; i++) {
            if (m_cancelled)
                break;
            auto start = std::chrono::high_resolution_clock::now();
            if (expand(T, problem.goal))
                break;
            auto end = std::chrono::high_resolution_clock::now();
            auto expand_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            spdlog::info("Expanded in {} ms ({})", expand_time, T->size());
        }

        if (T->contains(problem.goal)) {
            spdlog::info("Found goal!");
            return tracePath(T, problem.start, problem.goal);
        }
    }

    auto result = SolverResult::fail();
    // We failed, but find the closest node and output the path
    auto nearest = T->getNearestPoint(problem.goal);
    if (nearest != nullptr) {
        result = tracePath(T, problem.start, nearest);
        result->success = false;
        return result;
    }
    spdlog::warn("Couldn't find a nearest point to the goal, returning empty result");
    return result;
}

SolverSolutions Solver::solve() {

    SolverSolutions solutions = std::make_unique<std::unordered_map<std::string, SolverResult::SharedPtr>>();

    auto start = std::chrono::high_resolution_clock::now();
    this->computeVoxels();

    for (const auto& roadmap : m_searchGraph->roadmaps) {
        roadmap->computeAllPairsShortestPath();
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto preprocess_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    spdlog::info("Preprocessing took {} ms", preprocess_time);

    if (m_config->preprocessOnly)
        return solutions;

    // set cancellation token to true in 10 seconds
    for (const auto& problem : m_config->problems) {
        std::atomic_bool cancellationToken(false);
        std::thread([this]() {
            // Sleep for 10 seconds
            std::this_thread::sleep_for(std::chrono::seconds(60));
            this->m_cancelled = true;
        }).detach();
        auto start = std::chrono::high_resolution_clock::now();
        auto result = solveProblem(problem, cancellationToken);
        auto end = std::chrono::high_resolution_clock::now();
        auto solve_time = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();

        spdlog::info("Solved problem {} in {} s with path size {}", problem.name, solve_time, result->path.size());

        solutions->insert(std::make_pair(problem.name, result));
    }

    return solutions;
}

void Solver::computeVoxels() {
    // make a set of roadmaps
    std::unordered_set<Roadmap::SharedPtr> roadmaps;
#pragma omp parallel for
    for (auto& robot : m_config->robots) {
        if (!roadmaps.count(robot->roadmap)) {
            for (auto& dart : robot->roadmap->darts) {
                robot->getSweptVoxel(dart);
            }
        }
        roadmaps.insert(robot->roadmap);
    }
}

inline std::vector<bool> generateRandomRobotMask(const std::vector<IRobot::SharedPtr>& robots,
                                                 const size_t num_robots) {
    std::vector<bool> mask(robots.size());

    for (size_t i = 0; i < num_robots && i < robots.size(); i++) {
        mask[i] = true;
    }

    std::random_shuffle(mask.begin(), mask.end());
    return mask;
}

SearchVertex::SharedPtr Solver::distanceOracleMPI(const SearchVertex::SharedPtr& nearVertex,
                                                  const SearchVertex::SharedPtr& goalVertex) {
    // This is the MPI main node for the computation.

    // Broadcast out the currrent near vertex and goal vertex to all other nodes.

    // Each node will sample a new vertex and check for collisions.

    // Each node will send back the vertex that is collision free.

    // The main node will receive all of the vertices and return the one that is closest to the goal vertex.

    return nullptr;  // TODO
}

SearchVertex::SharedPtr Solver::distanceOracle(const SearchVertex::SharedPtr& nearVertex,
                                               const SearchVertex::SharedPtr& randomVertex,
                                               const SearchVertex::SharedPtr& goalVertex) {

    // Find the vertex in the graph that minimizes the angle between nearVertex and randomVertex.
    // This is the vertex that is closest to randomVertex in the graph.

    if (m_config->useMPI) {
        return distanceOracleMPI(nearVertex, goalVertex);
    }

    size_t attempts = 0;
    while (attempts++ < ORACLE_VERTEX_ATTEMPTS) {
        m_pointsConsidered += 1;
        if (m_cancelled)
            break;
        auto mask = generateRandomRobotMask(m_config->robots, m_robotsToMove);
        auto newVertex = m_searchGraph->sampleAdjacentVertex(nearVertex, mask);
        if (newVertex == nullptr) {
            spdlog::warn("sampleAdjacentVertex returned nullptr");
            continue;
        }

        // Check that the edge between the near vertex and new vertex is collision free.
        auto newDart = std::make_shared<SearchDart>(nearVertex, newVertex, 1);
        auto darts = m_searchGraph->getRoadmapDarts(nearVertex, newVertex);
        auto start = std::chrono::high_resolution_clock::now();
        bool collisionFree = checkCollisionFreeDarts(darts);
        auto end = std::chrono::high_resolution_clock::now();
        auto collision_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        if (collisionFree) {
            return newVertex;
        }
    }

    return nullptr;
};

SearchVertex::SharedPtr Solver::sampleAdjacentCollisionFreeVertex(const SearchVertex::SharedPtr& nearVertex,
                                                                  const SearchVertex::SharedPtr& goalVertex) {

    m_pointsConsidered += 1;
    // Generate a random permutation of the robots
    std::vector<IRobot::SharedPtr> robots = m_config->robots;
    std::random_shuffle(robots.begin(), robots.end());

    std::vector<RoadmapStateId> roadmapStateIds(robots.size());
    std::vector<RoadmapDart::SharedPtr> currentDarts;

    for (const auto& robot : robots) {
        const RobotId id = robot->id;
        const auto current_roadmap_state = nearVertex->roadmapStates[id];
        const auto darts = robot->roadmap->getAdjacentDarts(robot->roadmap->vertices[current_roadmap_state]);

        // Generate a random permutation of the darts
        std::vector<RoadmapDart::SharedPtr> dartsVector(darts.begin(), darts.end());

        std::random_shuffle(dartsVector.begin(), dartsVector.end());

        if (rand() % 10 <= 7) {
            // Have the robot remain in its current position.
            // Check that that dart is collision-free with the current darts.
            bool collisionFree = true;

            RoadmapDart::SharedPtr currentDart = robot->roadmap->getDart(
                robot->roadmap->vertices[current_roadmap_state], robot->roadmap->vertices[current_roadmap_state]);

            if (currentDart != nullptr) {
                for (const auto& dart : currentDarts) {
                    if (m_voxelManager->intersect(currentDart->voxel, dart->voxel)) {
                        collisionFree = false;
                        break;
                    }
                }

                if (collisionFree) {
                    currentDarts.push_back(currentDart);
                    roadmapStateIds[id] = current_roadmap_state;
                    continue;
                }
            }
        }

        // Find a dart that is collision-free with our current darts and add it to the list of current darts
        for (const auto& dart : dartsVector) {
            bool collisionFree = true;

            for (const auto& currentDart : currentDarts) {
                if (m_voxelManager->intersect(dart->voxel, currentDart->voxel)) {
                    collisionFree = false;
                    break;
                }
            }

            if (collisionFree) {
                currentDarts.push_back(dart);
                roadmapStateIds[id] = dart->m_end->m_id;
                break;
            } else {
                // No collision-free dart could be found
                spdlog::warn("Failed to find collision free path.");
                return nullptr;
            }
        }
    }
    return nullptr;
}

bool Solver::checkCollisionFreeDarts(const std::vector<RoadmapDart::SharedPtr> darts) const {

    bool collisionFree = true;
#pragma omp parallel for
    for (int i = 0; i < darts.size(); i++) {
        if (!collisionFree) {
            continue;
        }
        for (int j = i + 1; j < darts.size(); j++) {
            if (m_voxelManager->intersect(darts[i]->voxel, darts[j]->voxel)) {
                collisionFree = false;
            }
        }
    }

    return collisionFree;
};

void Solver::launchMPIWorker() {
    // TODO: Rob + Hammad
}