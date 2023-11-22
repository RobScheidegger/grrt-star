#define RYML_SINGLE_HDR_DEFINE_NOW
#include "pkgs/rapidyaml.hpp"

#include "config/solver_config_parser.h"

#include <fstream>
#include <iostream>
#include <sstream>

#include "config/solver_config.h"
#include "graphs/roadmap.h"

#include "result.h"
#include "robots/robot_types.h"

using namespace grrt;

Result parseRoadmaps(const ryml::NodeRef& node, SolverConfig::SharedPtr config) {
    if (!node.is_seq()) {
        return Result::Error("roadmaps must be a sequence");
    }

    for (auto roadmap : node) {
        if (!roadmap.is_map()) {
            return Result::Error("roadmap must be a map");
        }

        if (!roadmap.has_child("name")) {
            return Result::Error("roadmap must have a name");
        }
        auto name_value = roadmap["name"].val();
        std::string name(name_value.begin(), name_value.end());

        std::cout << name << std::endl;
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
        std::cerr << result.msg << std::endl;
        return nullptr;
    }
}
