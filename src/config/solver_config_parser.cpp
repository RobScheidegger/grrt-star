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

    config = std::make_shared<DroneState>(Point(x, y, z), 0.5f);

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

            auto vertex = roadmap_ptr->addVertex(robotState);
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

                    roadmap_ptr->addDart(vertex, state_name_to_vertex[to_state_name], EdgeParameters());
                }

            } else {
                // Add edges to all other vertices
                for (auto other_vertex : roadmap_ptr->vertices) {
                    if (other_vertex == vertex) {
                        continue;
                    }

                    roadmap_ptr->addDart(vertex, other_vertex, EdgeParameters());
                }
            }
        }

        config->roadmaps[name] = roadmap_ptr;
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

    if (!root.has_child("roadmaps")) {
        std::cerr << "Missing roadmaps" << std::endl;
        return nullptr;
    }

    auto roadmaps = root["roadmaps"];
    Result result = parseRoadmaps(roadmaps, config);
    if (result.isError) {
        spdlog::error("Error parsing roadmap: {}", result.msg);
        return nullptr;
    }

    return config;
}
