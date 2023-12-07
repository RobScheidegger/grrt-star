#define RYML_SINGLE_HDR_DEFINE_NOW
#include "pkgs/rapidyaml.hpp"
#include "spdlog/spdlog.h"

#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>

#include "config/solver_config.h"
#include "config/solver_config_parser.h"
#include "graphs/roadmap.h"

#include "result.h"
#include "robots/drone/drone.hpp"
#include "robots/robot_types.h"

using namespace grrt;

static grrt::Result parseRobotType(const std::string& robotTypeStr, RobotType& robotType) {

    if (robotTypeStr == "drone") {
        robotType = RobotType::DRONE;
        return Result::Ok();
    }
    return Result::Error("Unknown robot type: " + robotTypeStr);
}

static Result parseVoxelType(const std::string& voxelTypeStr, VoxelType& voxelType) {
    if (voxelTypeStr == "PCL") {
        voxelType = VoxelType::POINT_CLOUD;
        return Result::Ok();
    }
    return Result::Error("Unknown voxel type: " + voxelTypeStr);
}

static Result getString(const ryml::ConstNodeRef& node, const std::string& key, std::string& value) {
    auto key_substr = c4::to_csubstr(key);
    if (!node.has_child(key_substr)) {
        return Result::Error("Missing key: " + key);
    }
    auto value_node = node[key_substr];

    value = std::string(value_node.val().begin(), value_node.val().end());
    return Result::Ok();
}

static Result getFloat(const ryml::ConstNodeRef& node, const std::string& key, float& value) {
    auto key_substr = c4::to_csubstr(key);
    if (!node.has_child(key_substr)) {
        return Result::Error("Missing key: " + key);
    }
    auto value_node = node[key_substr];
    value = strtof32(std::string(value_node.val().begin(), value_node.val().end()).c_str(), nullptr);

    return Result::Ok();
}

static Result parseDroneState(const ryml::ConstNodeRef& node, RobotState::SharedPtr& config) {
    // Just has an x, y, z position.
    float x, y, z;

    Result res = getFloat(node, "x", x);
    if (res.isError) {
        return res;
    }

    res = getFloat(node, "y", y);
    if (res.isError) {
        return res;
    }

    res = getFloat(node, "z", z);
    if (res.isError) {
        return res;
    }

    config = std::make_shared<DroneState>(Point(x, y, z), 1);

    return Result::Ok();
}

static Result parseRobotState(const ryml::ConstNodeRef& node, const RobotType robotType, RobotState::SharedPtr& state) {
    switch (robotType) {
        case RobotType::DRONE:
            return parseDroneState(node, state);
    }

    return Result::Ok();
}

Result parseRoadmaps(const ryml::NodeRef& node, SolverConfig::SharedPtr config) {
    if (!node.is_seq()) {
        return Result::Error("roadmaps must be a sequence");
    }

    for (auto roadmap : node) {
        if (!roadmap.is_map()) {
            return Result::Error("roadmap must be a map");
        }

        std::string name;
        Result res = getString(roadmap, "name", name);
        if (res.isError) {
            return res;
        }

        std::string type;
        res = getString(roadmap, "type", type);
        if (res.isError) {
            spdlog::error("Couldn't find type for roadmap: {}", name);
            return res;
        }

        RobotType robotType;
        res = parseRobotType(type, robotType);
        if (res.isError) {
            return res;
        }

        Roadmap::SharedPtr roadmap_ptr = config->robotFactory->makeRoadmap(name, robotType);

        if (!roadmap.has_child("states")) {
            return Result::Error("roadmap must have a states key.");
        }
        // Load all of the states in one pass.
        // In the second pass add all of the edges (darts) to the graph.
        auto states = roadmap["states"];
        if (!states.is_seq()) {
            return Result::Error("states must be a sequence");
        }

        // First Pass
        std::unordered_map<std::string, RoadmapVertex::SharedPtr> state_name_to_vertex;
        for (auto state : states) {
            if (!state.is_map()) {
                return Result::Error("state must be a map");
            }

            std::string state_name;
            res = getString(state, "name", state_name);
            if (res.isError) {
                return res;
            }

            RobotState::SharedPtr robotState = nullptr;
            res = parseRobotState(state, robotType, robotState);
            if (res.isError) {
                return res;
            }

            assert(robotState != nullptr);

            auto vertex = roadmap_ptr->addVertex(state_name, robotState);
            state_name_to_vertex[state_name] = vertex;
        }

        // Second pass - Adding edges
        for (auto state : states) {
            // This is where the roadmap connections are specified.
            // If no 'to' tag is present, it is assume that it connects  all
            std::string state_name;
            res = getString(state, "name", state_name);

            auto vertex = state_name_to_vertex[state_name];

            if (state.has_child("to")) {
                auto to = state["to"];
                if (!to.is_seq()) {
                    return Result::Error("to must be a sequence");
                }

                for (auto to_state : to) {
                    auto value = to_state.val();
                    std::string to_state_name = std::string(value.begin(), value.end());

                    if (state_name_to_vertex.find(to_state_name) == state_name_to_vertex.end()) {
                        return Result::Error("Unknown state: " + to_state_name);
                    }

                    roadmap_ptr->addDart(vertex, state_name_to_vertex[to_state_name]);
                }

            } else {
                // Add edges to all other vertices
                for (auto other_vertex : roadmap_ptr->vertices) {
                    roadmap_ptr->addDart(vertex, other_vertex);
                }
            }
        }

        config->roadmaps[name] = roadmap_ptr;
    }

    return Result::Ok();
}

Result parseRobots(const ryml::NodeRef& node, SolverConfig::SharedPtr config) {
    if (!node.is_seq()) {
        return Result::Error("robots must be a sequence");
    }

    RobotId robot_id = 0;

    for (auto robot : node) {
        if (!robot.is_map()) {
            return Result::Error("robot must be a map");
        }

        std::string name;
        Result res = getString(robot, "name", name);
        if (res.isError) {
            return res;
        }

        std::string type;
        res = getString(robot, "type", type);
        if (res.isError) {
            spdlog::error("Couldn't find type for robot: {}", name);
            return res;
        }

        RobotType robotType;
        res = parseRobotType(type, robotType);
        if (res.isError) {
            return res;
        }

        std::string roadmap_name;
        res = getString(robot, "roadmap", roadmap_name);
        if (res.isError) {
            return res;
        }

        if (config->roadmaps.find(roadmap_name) == config->roadmaps.end()) {
            return Result::Error("Unknown roadmap: " + roadmap_name);
        }

        Roadmap::SharedPtr roadmap = config->roadmaps[roadmap_name];

        IRobot::SharedPtr robot_ptr = config->robotFactory->makeRobot(robot_id, name, roadmap);
        config->robots.push_back(robot_ptr);

        robot_id++;
    }

    return Result::Ok();
}

Result parseProblems(const ryml::NodeRef& node, SolverConfig::SharedPtr config) {
    if (!node.is_seq()) {
        return Result::Error("problems must be a sequence");
    }

    const auto num_robots = config->robots.size();

    for (auto problem : node) {
        if (!problem.is_map()) {
            return Result::Error("problem must be a map");
        }

        std::string name;
        Result res = getString(problem, "name", name);
        if (res.isError) {
            return res;
        }

        // Parse the start and end keys, which are sequences of nodes of the form 'robot: state'
        if (!problem.has_child("start")) {
            return Result::Error("problem must have a start key.");
        }

        auto start = problem["start"];
        if (!start.is_map()) {
            return Result::Error("start must be a map");
        }

        std::vector<RoadmapVertexId> start_state_ids(num_robots);
        std::vector<RoadmapVertexId> end_state_ids(num_robots);

        std::unordered_map<std::string, RobotState::SharedPtr> robot_name_to_state;
        for (auto state : start) {

            auto robot_name = state.key();
            auto state_name = state.val();

            auto robot =
                std::find_if(config->robots.begin(), config->robots.end(),
                             [&robot_name](const IRobot::SharedPtr& robot) { return robot->name == robot_name; });

            if (robot == config->robots.end()) {
                return Result::Error("Unknown robot");
            }

            std::string state_name_str = std::string(state_name.begin(), state_name.end());
            auto robot_state = (*robot)->roadmap->getVertex(state_name_str);
            if (!robot_state) {
                return Result::Error("Unknown state: " + state_name_str);
            }

            start_state_ids[(*robot)->id] = robot_state->getState()->getId();
        }

        // Same for goal state
        if (!problem.has_child("goal")) {
            return Result::Error("problem must have a goal key.");
        }

        auto goal = problem["goal"];
        if (!goal.is_map()) {
            return Result::Error("goal must be a sequence");
        }

        for (auto state : goal) {
            auto robot_name = state.key();
            auto state_name = state.val();

            auto robot =
                std::find_if(config->robots.begin(), config->robots.end(),
                             [&robot_name](const IRobot::SharedPtr& robot) { return robot->name == robot_name; });

            if (robot == config->robots.end()) {
                return Result::Error("Unknown robot");
            }

            std::string state_name_str = std::string(state_name.begin(), state_name.end());
            auto robot_state = (*robot)->roadmap->getVertex(state_name_str);
            if (!robot_state) {
                return Result::Error("Unknown state: " + state_name_str);
            }

            end_state_ids[(*robot)->id] = robot_state->getState()->getId();
        }

        SearchVertex::SharedPtr start_state = std::make_shared<SearchVertex>(start_state_ids);
        SearchVertex::SharedPtr end_state = std::make_shared<SearchVertex>(end_state_ids);

        config->problems.push_back(SolverProblem(name, start_state, end_state));
    }

    return Result::Ok();
}

SolverConfig::SharedPtr SolverConfigParser::parse(const std::string& fileName) {
    SolverConfig::SharedPtr config = std::make_shared<SolverConfig>();

    std::ifstream file(fileName);
    if (!file.is_open()) {
        std::cerr << "Could not open file: " << fileName << std::endl;
        return nullptr;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();

    std::string yaml_string = buffer.str();
    ryml::Tree tree = ryml::parse_in_arena(ryml::to_csubstr(yaml_string));
    ryml::NodeRef root = tree.rootref();

    if (!root.is_map()) {
        std::cerr << "Root node must be a map" << std::endl;
        return nullptr;
    }
    // Voxel Type
    if (!root.has_child("voxels")) {
        std::cerr << "Missing voxels" << std::endl;
        return nullptr;
    }

    auto voxels = root["voxels"];
    if (!voxels.is_map()) {
        std::cerr << "Voxels must be a map" << std::endl;
        return nullptr;
    }

    std::string voxel_type_str;
    Result result = getString(voxels, "type", voxel_type_str);
    if (result.isError) {
        spdlog::error("Error parsing voxel type: {}", result.msg);
        return nullptr;
    }

    VoxelType voxel_type;
    result = parseVoxelType(voxel_type_str, voxel_type);
    if (result.isError) {
        spdlog::error("Error parsing voxel type: {}", result.msg);
        return nullptr;
    }

    config->robotFactory = std::make_shared<RobotFactory>(voxel_type);

    // Roadmaps
    if (!root.has_child("roadmaps")) {
        std::cerr << "Missing roadmaps" << std::endl;
        return nullptr;
    }

    auto roadmaps = root["roadmaps"];
    result = parseRoadmaps(roadmaps, config);
    if (result.isError) {
        spdlog::error("Error parsing roadmap: {}", result.msg);
        return nullptr;
    }

    if (!root.has_child("robots")) {
        std::cerr << "Missing robots" << std::endl;
        return nullptr;
    }

    auto robots = root["robots"];
    result = parseRobots(robots, config);
    if (result.isError) {
        spdlog::error("Error parsing robots: {}", result.msg);
        return nullptr;
    }

    if (!root.has_child("problems")) {
        std::cerr << "Missing problems" << std::endl;
        return nullptr;
    }

    auto problems = root["problems"];
    result = parseProblems(problems, config);
    if (result.isError) {
        spdlog::error("Error parsing problems: {}", result.msg);
        return nullptr;
    }

    return config;
}

Result SolverConfigParser::printSolution(std::ostream& stream, const SolverConfig::SharedPtr& config,
                                         const SolverResult& solution) {
    stream << solution.cost << std::endl;
    stream << solution.time << std::endl;

    // Print the path as a CSV line-by-line
    const auto num_robots = config->robots.size();
    for (const auto& state : solution.path) {
        for (uint32_t i = 0; i < num_robots; i++) {
            stream << state->roadmapStates[i];
            if (i != num_robots - 1) {
                stream << ",";
            }
        }
        stream << std::endl;
    }
}

Result SolverConfigParser::parseSolution(const std::string& fileName, SolverResult& solution) {
    std::ifstream file(fileName);
    if (!file.is_open()) {
        std::cerr << "Could not open file: " << fileName << std::endl;
        return Result::Error("Could not open file: " + fileName);
    }

    std::string line;
    std::getline(file, line);
    solution.cost = std::stof(line);

    std::getline(file, line);
    solution.time = std::stof(line);

    while (std::getline(file, line)) {
        std::stringstream line_stream(line);
        std::string cell;
        std::vector<RoadmapVertexId> roadmapStates;
        while (std::getline(line_stream, cell, ',')) {
            roadmapStates.push_back(std::stoi(cell));
        }
        solution.path.push_back(std::make_shared<SearchVertex>(roadmapStates));
    }

    return Result::Ok();
}